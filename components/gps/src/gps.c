#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "pins.h"
#include "settings.h"
#include "stratux_status.h"

static const char *TAG = "gps";

#define GPS_UART_PORT    UART_NUM_1
#define GPS_RX_BUF       (2 * 1024)   // 9600 is slow; 2KB headroom is plenty
#define GPS_LINE_MAX     128          // NMEA sentences are ~80 bytes max
#define GPS_INIT_DELAY   100          // ms to wait between config commands

static portMUX_TYPE s_ownship_mux = portMUX_INITIALIZER_UNLOCKED;
static gps_ownship_t s_ownship;       // zero-initialised (valid=false)
static bool s_time_synced = false;

gps_ownship_t gps_get_ownship(void)
{
    gps_ownship_t snap;
    taskENTER_CRITICAL(&s_ownship_mux);
    snap = s_ownship;
    taskEXIT_CRITICAL(&s_ownship_mux);

    // Check staleness: fix is valid only if < 5 sec old.
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (snap.valid && (now_ms - snap.fix_time_ms) > 5000) {
        snap.valid = false;
    }
    return snap;
}

// ---- NMEA checksum -----------------------------------------------------------

// Validate an NMEA sentence: "$<body>*HH" where HH is the hex XOR of every byte
// of <body>. Port of stratux validateNMEAChecksum(). `line` is the full
// sentence with leading '$' and no CR/LF. Rejects corrupted serial so a garbled
// position can never reach the EFB. Returns true if the checksum matches.
static bool nmea_checksum_ok(const char *line)
{
    if (line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || star[1] == '\0' || star[2] == '\0') return false;  // need 2 hex digits

    uint8_t cs = 0;
    for (const char *p = line + 1; p < star; p++) cs ^= (uint8_t)*p;

    char hex[3] = { star[1], star[2], '\0' };
    char *end;
    unsigned long want = strtoul(hex, &end, 16);
    if (end != hex + 2) return false;  // non-hex checksum digits
    return cs == (uint8_t)want;
}

// Days since the Unix epoch (1970-01-01) for a proleptic-Gregorian date.
// Howard Hinnant's days_from_civil — exact, branch-light, and TZ-independent
// (newlib here doesn't expose timegm()). m in [1,12], d in [1,31].
static int64_t days_from_civil(int y, int m, int d)
{
    y -= m <= 2;
    int era = (y >= 0 ? y : y - 399) / 400;
    int yoe = y - era * 400;                                   // [0, 399]
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;  // [0, 365]
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;           // [0, 146096]
    return (int64_t)era * 146097 + doe - 719468;
}

// NACp from estimated horizontal accuracy (meters), per AC 20-165A. Port of
// stratux calculateNACp(). We estimate accuracy as HDOP * 4 (generic u-blox).
static uint8_t nacp_from_hdop(uint8_t hdop_x10)
{
    float accuracy = (hdop_x10 / 10.0f) * 4.0f;
    if (accuracy < 3)     return 11;
    if (accuracy < 10)    return 10;
    if (accuracy < 30)    return 9;
    if (accuracy < 92.6)  return 8;
    if (accuracy < 185.2) return 7;
    if (accuracy < 555.6) return 6;
    return 0;
}

// Parse NMEA GGA sentence: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
// Extract: lat, lng, MSL alt (m -> ft), num_sats, hdop, fix_quality, geoid_sep.
// Returns true if parse succeeded.
static bool parse_gga(const char *line, double *lat, double *lng, int32_t *alt_msl_ft,
                      uint8_t *num_sats, uint8_t *hdop_x10, uint8_t *fix_quality, float *geoid_sep_ft)
{
    // Fields: 0=type, 1=time, 2=lat, 3=N/S, 4=lng, 5=E/W, 6=fix, 7=sats, 8=hdop, 9=alt(m),
    //         10=alt_unit, 11=geoid_sep(m), 12=geoid_unit, 13-14=unused
    char buf[GPS_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    int field = 0;
    char *saveptr, *token = strtok_r(buf, ",", &saveptr);

    double lat_deg = 0, lat_min = 0, lng_deg = 0, lng_min = 0;
    char lat_ns = 'N', lng_ew = 'E';
    int fix = 0;
    double alt_m = 0, geoid_m = 0;
    double hdop = 0;

    while (token != NULL && field <= 12) {
        if (field == 2) {
            // Latitude: DDMM.MMMM
            double val = strtod(token, NULL);
            lat_deg = floor(val / 100.0);
            lat_min = val - lat_deg * 100.0;
        } else if (field == 3) {
            lat_ns = token[0];
        } else if (field == 4) {
            // Longitude: DDDMM.MMMM
            double val = strtod(token, NULL);
            lng_deg = floor(val / 100.0);
            lng_min = val - lng_deg * 100.0;
        } else if (field == 5) {
            lng_ew = token[0];
        } else if (field == 6) {
            fix = atoi(token);
            *fix_quality = (uint8_t)fix;  // 0=invalid, 1=GPS, 2=DGPS
        } else if (field == 7) {
            *num_sats = (uint8_t)atoi(token);
        } else if (field == 8) {
            hdop = strtod(token, NULL);
            int h = (int)(hdop * 10 + 0.5);  // round to nearest 0.1
            *hdop_x10 = (uint8_t)(h > 255 ? 255 : h);
        } else if (field == 9) {
            alt_m = strtod(token, NULL);
        } else if (field == 11) {
            geoid_m = strtod(token, NULL);
        }

        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    if (fix == 0 || *num_sats == 0) return false;  // no fix

    *lat = lat_deg + (lat_min / 60.0);
    if (lat_ns == 'S') *lat = -*lat;

    *lng = lng_deg + (lng_min / 60.0);
    if (lng_ew == 'W') *lng = -*lng;

    *alt_msl_ft = (int32_t)(alt_m * 3.28084 + (alt_m < 0 ? -0.5 : 0.5));  // MSL, m -> ft
    *geoid_sep_ft = (float)(geoid_m * 3.28084);                          // geoid sep, ft

    return true;
}

// Parse NMEA RMC sentence: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
// Extract: track, speed. Sync the system clock (UTC) on the first valid fix.
// Returns true if the sentence reports a valid fix.
static bool parse_rmc(const char *line, uint16_t *track_deg, uint16_t *speed_kt)
{
    char buf[GPS_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    int field = 0;
    char *saveptr, *token = strtok_r(buf, ",", &saveptr);

    char status = 'V';
    char time_str[10] = "", date_str[10] = "";

    while (token != NULL && field <= 9) {
        if (field == 1) {
            strncpy(time_str, token, 9);
            time_str[9] = '\0';
        } else if (field == 2) {
            status = token[0];  // A = valid, V = invalid
        } else if (field == 7) {
            *speed_kt = (uint16_t)(strtod(token, NULL) + 0.5);  // knots, round
        } else if (field == 8) {
            double track = strtod(token, NULL);
            if (track < 0.0 || track > 360.0) return false;     // invalid track
            *track_deg = (uint16_t)(track + 0.5) % 360;          // round; 360 -> 0
        } else if (field == 9) {
            strncpy(date_str, token, 9);
            date_str[9] = '\0';
        }

        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    if (status != 'A') return false;  // no valid fix

    // Sync the clock once, on the first valid fix with both date and time present.
    if (!s_time_synced && strlen(time_str) >= 6 && strlen(date_str) >= 6) {
        // time_str: HHMMSS, date_str: DDMMYY
        int hh = ((time_str[0] - '0') * 10) + (time_str[1] - '0');
        int mm = ((time_str[2] - '0') * 10) + (time_str[3] - '0');
        int ss = ((time_str[4] - '0') * 10) + (time_str[5] - '0');

        int dd = ((date_str[0] - '0') * 10) + (date_str[1] - '0');
        int mo = ((date_str[2] - '0') * 10) + (date_str[3] - '0');
        int yy = ((date_str[4] - '0') * 10) + (date_str[5] - '0');

        // Convert UTC calendar time -> Unix epoch directly (no mktime/$TZ).
        time_t unix_time = (time_t)(days_from_civil(2000 + yy, mo, dd) * 86400
                                    + hh * 3600 + mm * 60 + ss);

        struct timeval tv = { .tv_sec = unix_time, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        s_time_synced = true;
        g_status.utc_ok = true;  // GDL90 heartbeat may now advertise valid time
        ESP_LOGI(TAG, "Synced system time from GPS: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 yy + 2000, mo, dd, hh, mm, ss);
    }

    return true;
}

// Parse NMEA GSA sentence: $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*30
// Extract: satellites used in solution, PDOP, HDOP, VDOP. Returns true on success.
static bool parse_gsa(const char *line, uint8_t *num_sats_in_solution,
                      uint8_t *pdop_x10, uint8_t *hdop_x10, uint8_t *vdop_x10)
{
    // Fields: 0=type, 1=mode(A/M), 2=fix(1/2/3), 3-14=sat IDs, 15=PDOP, 16=HDOP, 17=VDOP
    char buf[GPS_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    int field = 0;
    char *saveptr, *token = strtok_r(buf, ",", &saveptr);

    uint8_t sat_count = 0;

    while (token != NULL && field <= 17) {
        if (field >= 3 && field <= 14) {
            if (strlen(token) > 0) sat_count++;   // non-empty = satellite ID
        } else if (field == 15) {
            int v = (int)(strtod(token, NULL) * 10 + 0.5);
            *pdop_x10 = (uint8_t)(v > 255 ? 255 : v);
        } else if (field == 16) {
            int v = (int)(strtod(token, NULL) * 10 + 0.5);
            *hdop_x10 = (uint8_t)(v > 255 ? 255 : v);
        } else if (field == 17) {
            // VDOP field may carry a trailing "*HH" checksum on the last token;
            // strtod stops at '*', so this is safe.
            int v = (int)(strtod(token, NULL) * 10 + 0.5);
            *vdop_x10 = (uint8_t)(v > 255 ? 255 : v);
        }

        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    *num_sats_in_solution = sat_count;
    return sat_count > 0;
}

// ---- GPS module configuration ------------------------------------------------

// Send an NMEA command (e.g. MTK "PMTK..."): "$<cmd>*HH\r\n" with the XOR
// checksum. Used by MediaTek/GlobalTop modules (BN-220 etc.).
static void gps_send_nmea_cmd(uart_port_t port, const char *cmd)
{
    uint8_t cksum = 0;
    for (int i = 0; cmd[i] != '\0'; i++) cksum ^= (uint8_t)cmd[i];

    char buf[64];
    int n = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", cmd, cksum);
    uart_write_bytes(port, buf, n);
    vTaskDelay(pdMS_TO_TICKS(GPS_INIT_DELAY));
}

// Build + send a u-blox UBX frame with a runtime-computed Fletcher checksum.
// Port of stratux makeUBXCFG()/chksumUBX(): B5 62 | class id | len(LE) |
// payload | CK_A CK_B, checksum over [class .. last payload byte].
static void ubx_send(uart_port_t port, uint8_t cls, uint8_t id,
                     const uint8_t *payload, uint16_t len)
{
    uint8_t buf[64];
    if ((size_t)len + 8 > sizeof(buf)) return;   // CFG-NAV5 (44) / CFG-PRT (28) fit

    buf[0] = 0xB5; buf[1] = 0x62;
    buf[2] = cls;  buf[3] = id;
    buf[4] = (uint8_t)(len & 0xFF);
    buf[5] = (uint8_t)((len >> 8) & 0xFF);
    for (uint16_t i = 0; i < len; i++) buf[6 + i] = payload[i];

    uint8_t a = 0, b = 0;
    for (uint16_t i = 2; i < 6 + len; i++) { a = (uint8_t)(a + buf[i]); b = (uint8_t)(b + a); }
    buf[6 + len] = a;
    buf[7 + len] = b;

    uart_write_bytes(port, (const char *)buf, 6 + len + 2);
    vTaskDelay(pdMS_TO_TICKS(GPS_INIT_DELAY));
}

// Configure the module to emit GGA + RMC + GSA at 1 Hz (and quiet the rest).
// Sends both MTK (PMTK) and u-blox (UBX) so it works on either family. Called
// while the UART is at the module's *current* baud.
static void gps_configure_messages(uart_port_t port)
{
    uart_flush_input(port);

    // ---- MTK / GlobalTop (BN-220 and similar) ----
    // PMTK314: GLL=0 RMC=1 VTG=0 GGA=1 GSA=1, rest off.
    gps_send_nmea_cmd(port, "PMTK314,0,1,0,1,1,0,0,0,0");
    // PMTK220: 1000 ms (1 Hz) fix interval.
    gps_send_nmea_cmd(port, "PMTK220,1000");

    // ---- u-blox UBX (correct checksums, computed at runtime) ----
    // CFG-NAV5: airborne <2g dynamic platform model (verbatim from stratux gps.go).
    static const uint8_t nav5[36] = {
        0x01, 0x00, 0x07, 0x00, 0,0,0,0, 0,0,0,0, 0,0,0,0,
        0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
    ubx_send(port, 0x06, 0x24, nav5, sizeof nav5);

    // CFG-RATE: 1000 ms (1 Hz). Payload little-endian.
    static const uint8_t rate1hz[6] = { 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00 };
    ubx_send(port, 0x06, 0x08, rate1hz, sizeof rate1hz);

    // CFG-MSG: rate per port {I2C, UART1, UART2, USB, SPI, res}. Enable
    // GGA/RMC/GSA on UART1 (and USB), disable the chatty defaults to keep the
    // 9600 link clean.
    static const uint8_t msg_gga[8] = { 0xF0, 0x00, 0, 1, 0, 1, 0, 0 };  // enable
    static const uint8_t msg_gll[8] = { 0xF0, 0x01, 0, 0, 0, 0, 0, 0 };  // disable
    static const uint8_t msg_gsa[8] = { 0xF0, 0x02, 0, 1, 0, 1, 0, 0 };  // enable
    static const uint8_t msg_gsv[8] = { 0xF0, 0x03, 0, 0, 0, 0, 0, 0 };  // disable
    static const uint8_t msg_rmc[8] = { 0xF0, 0x04, 0, 1, 0, 1, 0, 0 };  // enable
    static const uint8_t msg_vtg[8] = { 0xF0, 0x05, 0, 0, 0, 0, 0, 0 };  // disable
    ubx_send(port, 0x06, 0x01, msg_gga, 8);
    ubx_send(port, 0x06, 0x01, msg_gll, 8);
    ubx_send(port, 0x06, 0x01, msg_gsa, 8);
    ubx_send(port, 0x06, 0x01, msg_gsv, 8);
    ubx_send(port, 0x06, 0x01, msg_rmc, 8);
    ubx_send(port, 0x06, 0x01, msg_vtg, 8);
}

// Tell the module to switch its UART to `baud` (MTK PMTK251 + u-blox CFG-PRT).
// Must be sent while the host UART is still at the module's *current* baud.
static void gps_set_module_baud(uart_port_t port, uint32_t baud)
{
    char cmd[24];
    snprintf(cmd, sizeof cmd, "PMTK251,%u", (unsigned)baud);
    gps_send_nmea_cmd(port, cmd);

    // UBX-CFG-PRT (UART1): portID=1, mode=8N1 (0x000008C0), baud (LE),
    // inProtoMask=UBX+NMEA (0x0003), outProtoMask=NMEA (0x0002).
    uint8_t prt[20] = {0};
    prt[0]  = 0x01;
    prt[4]  = 0xC0; prt[5] = 0x08;
    prt[8]  = (uint8_t)(baud & 0xFF);
    prt[9]  = (uint8_t)((baud >> 8) & 0xFF);
    prt[10] = (uint8_t)((baud >> 16) & 0xFF);
    prt[11] = (uint8_t)((baud >> 24) & 0xFF);
    prt[12] = 0x03;
    prt[14] = 0x02;
    ubx_send(port, 0x06, 0x00, prt, sizeof prt);
}

// Listen for up to `timeout_ms` and return true if at least one checksum-valid
// NMEA sentence arrives — used to confirm the UART baud matches the module.
static bool gps_detect_nmea(uart_port_t port, uint32_t timeout_ms)
{
    char l[GPS_LINE_MAX];
    int ll = 0;
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        uint8_t b;
        if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(100)) <= 0) continue;
        if (b == '\n') {
            if (ll > 0 && l[ll - 1] == '\r') ll--;
            l[ll] = '\0';
            if (ll > 6 && l[0] == '$' && nmea_checksum_ok(l)) return true;
            ll = 0;
        } else if (b == '$') {
            ll = 0; l[ll++] = b;
        } else if (ll < GPS_LINE_MAX - 1) {
            l[ll++] = b;
        } else {
            ll = 0;
        }
    }
    return false;
}

// Detect the module's current baud, switch it to `target` if needed, then push
// the message/rate configuration. Leaves the host UART at the baud the module
// is actually talking on. Returns that baud.
static uint32_t gps_bring_up(uart_port_t port, uint32_t target)
{
    // Candidate rates, target first; common module defaults after. Skip dups.
    const uint32_t cands[] = { target, 9600, 38400, 57600, 115200 };
    uint32_t detected = 0;

    for (size_t i = 0; i < sizeof(cands) / sizeof(cands[0]); i++) {
        bool dup = false;
        for (size_t j = 0; j < i; j++) if (cands[j] == cands[i]) { dup = true; break; }
        if (dup) continue;

        uart_set_baudrate(port, cands[i]);
        uart_flush_input(port);
        ESP_LOGI(TAG, "Probing GPS at %u baud...", (unsigned)cands[i]);
        if (gps_detect_nmea(port, 1500)) {
            detected = cands[i];
            ESP_LOGI(TAG, "GPS detected at %u baud", (unsigned)detected);
            break;
        }
    }

    if (detected == 0) {
        // Nothing heard — module may be absent or silent until configured.
        // Proceed at the target baud as a best effort.
        ESP_LOGW(TAG, "No NMEA detected at any baud; assuming target %u", (unsigned)target);
        uart_set_baudrate(port, target);
        uart_flush_input(port);
        gps_configure_messages(port);
        return target;
    }

    // Apply message/rate config at the rate the module is currently using.
    gps_configure_messages(port);

    // If the module isn't at the desired baud, switch it and re-verify.
    if (detected != target) {
        ESP_LOGI(TAG, "Switching GPS %u -> %u baud", (unsigned)detected, (unsigned)target);
        gps_set_module_baud(port, target);
        vTaskDelay(pdMS_TO_TICKS(300));
        uart_set_baudrate(port, target);
        uart_flush_input(port);
        if (gps_detect_nmea(port, 2500)) {
            return target;
        }
        ESP_LOGW(TAG, "No NMEA after switch to %u; staying at %u",
                 (unsigned)target, (unsigned)detected);
        uart_set_baudrate(port, detected);
        uart_flush_input(port);
    }
    return detected;
}

// ---- FreeRTOS task -----------------------------------------------------------

void gps_rx_task(void *arg)
{
    (void)arg;
    uint32_t target_baud = g_settings.gps_baud;

    if (GPS_RX_GPIO == GPIO_NUM_NC || GPS_TX_GPIO == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "GPS disabled: board has no GPS UART pins configured");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Starting GPS task (UART%d, GPIO %u RX, GPIO %u TX, target %u baud)",
             GPS_UART_PORT, GPS_RX_GPIO, GPS_TX_GPIO, (unsigned)target_baud);

    uart_config_t uart_cfg = {
        .baud_rate = 9600,                 // bring-up scans from here
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_TX_GPIO, GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, GPS_RX_BUF, 0, 0, NULL, 0));

    uint32_t baud = gps_bring_up(GPS_UART_PORT, target_baud);
    ESP_LOGI(TAG, "GPS configured; reading NMEA at %u baud", (unsigned)baud);

    char line[GPS_LINE_MAX];
    int line_len = 0;

    while (1) {
        uint8_t byte;
        int n = uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        if (byte == '$') {
            // Start of a new sentence; reset buffer.
            line_len = 0;
            line[line_len++] = byte;
            continue;
        }
        if (byte != '\n') {
            if (line_len < GPS_LINE_MAX - 1) line[line_len++] = byte;
            else line_len = 0;  // overflow; drop
            continue;
        }

        // End of line.
        if (line_len > 0 && line[line_len - 1] == '\r') line_len--;
        line[line_len] = '\0';

        // Reject anything that isn't a checksum-valid NMEA sentence.
        if (line_len <= 6 || line[0] != '$' || !nmea_checksum_ok(line)) {
            line_len = 0;
            continue;
        }

        // Sentence type after the 2-char talker ID ("$GPGGA"->"GGA", "$GNRMC"->"RMC").
        char sent_type[4];
        strncpy(sent_type, line + 3, 3);
        sent_type[3] = '\0';

        double lat = 0, lng = 0;
        int32_t alt_msl_ft = 0;
        uint8_t num_sats = 0, num_sats_solution = 0;
        uint8_t hdop_x10 = 0, vdop_x10 = 0, pdop_x10 = 0;
        uint8_t fix_quality = 0;
        float geoid_sep_ft = 0;
        uint16_t track_deg = 0, speed_kt = 0;

        if (strcmp(sent_type, "GGA") == 0) {
            if (parse_gga(line, &lat, &lng, &alt_msl_ft, &num_sats,
                          &hdop_x10, &fix_quality, &geoid_sep_ft)) {
                // HAE (geometric) = MSL + geoid separation.
                int32_t alt_hae_ft = alt_msl_ft + (int32_t)(geoid_sep_ft + (geoid_sep_ft < 0 ? -0.5f : 0.5f));
                uint8_t nacp = nacp_from_hdop(hdop_x10);

                taskENTER_CRITICAL(&s_ownship_mux);
                s_ownship.lat = lat;
                s_ownship.lng = lng;
                s_ownship.alt_msl_ft = alt_msl_ft;
                s_ownship.alt_hae_ft = alt_hae_ft;
                s_ownship.num_sats = num_sats;
                s_ownship.hdop_x10 = hdop_x10;
                s_ownship.geoid_sep_ft = geoid_sep_ft;
                s_ownship.fix_quality = fix_quality;
                s_ownship.nacp = nacp;
                s_ownship.fix_time_ms = esp_timer_get_time() / 1000;
                s_ownship.valid = true;
                taskEXIT_CRITICAL(&s_ownship_mux);
                ESP_LOGD(TAG, "GGA: lat=%.6f lng=%.6f msl=%ld hae=%ld sats=%u hdop=%.1f nacp=%u",
                         lat, lng, (long)alt_msl_ft, (long)alt_hae_ft,
                         (unsigned)num_sats, hdop_x10 / 10.0, (unsigned)nacp);
            }
        } else if (strcmp(sent_type, "RMC") == 0) {
            if (parse_rmc(line, &track_deg, &speed_kt)) {
                taskENTER_CRITICAL(&s_ownship_mux);
                s_ownship.track_deg = track_deg;
                s_ownship.speed_kt = speed_kt;
                taskEXIT_CRITICAL(&s_ownship_mux);
                ESP_LOGD(TAG, "RMC: track=%u deg speed=%u kt",
                         (unsigned)track_deg, (unsigned)speed_kt);
            }
        } else if (strcmp(sent_type, "GSA") == 0) {
            if (parse_gsa(line, &num_sats_solution, &pdop_x10, &hdop_x10, &vdop_x10)) {
                taskENTER_CRITICAL(&s_ownship_mux);
                s_ownship.num_sats_solution = num_sats_solution;
                s_ownship.pdop_x10 = pdop_x10;
                s_ownship.vdop_x10 = vdop_x10;
                taskEXIT_CRITICAL(&s_ownship_mux);
                ESP_LOGD(TAG, "GSA: sats=%u pdop=%.1f hdop=%.1f vdop=%.1f",
                         (unsigned)num_sats_solution, pdop_x10 / 10.0,
                         hdop_x10 / 10.0, vdop_x10 / 10.0);
            }
        }

        line_len = 0;
    }
}
