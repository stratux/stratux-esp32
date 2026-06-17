#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "time.h"
#include "pins.h"
#include "settings.h"

static const char *TAG = "gps";

#define GPS_UART_PORT    UART_NUM_1
#define GPS_RX_GPIO     GPIO_NUM_34  // input-only
#define GPS_TX_GPIO     GPIO_NUM_4   // output for config commands
#define GPS_RX_BUF      (2 * 1024)   // 9600 is slow; 2KB headroom is plenty
#define GPS_LINE_MAX    256          // NMEA sentences are ~80 bytes typical
#define GPS_INIT_DELAY  100          // ms to wait between config commands
#define GPS_INIT_TIMEOUT 3000        // ms to wait for GPS to boot

static portMUX_TYPE s_ownship_mux = portMUX_INITIALIZER_UNLOCKED;
static gps_ownship_t s_ownship = {
    .valid = false,
    .lat = 0.0,
    .lng = 0.0,
    .alt_ft = 0,
    .alt_msl_ft = 0,
    .track_deg = 0,
    .speed_kt = 0,
    .vvel_fpm = 0,
    .fix_time_ms = 0,
    .num_sats = 0,
    .num_sats_tracked = 0,
    .hdop_x10 = 0,
    .vdop_x10 = 0,
    .pdop_x10 = 0,
    .geoid_sep_ft = 0.0,
    .fix_quality = 0,
};
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

// Parse NMEA GGA sentence: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
// Extract: lat, lng, alt (meters -> feet), num_sats, hdop, fix_quality, geoid_sep
// Returns true if parse succeeded.
static bool parse_gga(const char *line, double *lat, double *lng, int32_t *alt_ft,
                      uint8_t *num_sats, uint8_t *hdop_x10, uint8_t *fix_quality, float *geoid_sep_ft)
{
    // Fields: 0=type, 1=time, 2=lat, 3=N/S, 4=lng, 5=E/W, 6=fix, 7=sats, 8=hdop, 9=alt(m),
    //         10=alt_unit, 11=geoid_sep(m), 12=geoid_unit, 13-14=unused
    char buf[256];
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
            *hdop_x10 = (uint8_t)(hdop * 10 + 0.5);  // round to nearest 0.1
            if (*hdop_x10 > 999) *hdop_x10 = 999;
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
    
    *alt_ft = (int32_t)(alt_m * 3.28084 + 0.5);  // meters to feet, round
    *geoid_sep_ft = (float)(geoid_m * 3.28084);  // geoid separation in feet
    
    return true;
}

// Parse NMEA RMC sentence: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
// Extract: date, time (UTC), track, speed. Set system time on first fix.
// Returns true if parse and time sync succeeded.
static bool parse_rmc(const char *line, uint16_t *track_deg, uint16_t *speed_kt)
{
    char buf[256];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    
    int field = 0;
    char *saveptr, *token = strtok_r(buf, ",", &saveptr);
    
    char status = 'V';
    char time_str[10] = "", date_str[10] = "";
    
    while (token != NULL && field <= 9) {
        if (field == 2) {
            status = token[0];  // A = valid, V = invalid
        } else if (field == 1) {
            strncpy(time_str, token, 9);
            time_str[9] = '\0';
        } else if (field == 9) {
            strncpy(date_str, token, 9);
            date_str[9] = '\0';
        } else if (field == 8) {
            *track_deg = (uint16_t)atoi(token);
        } else if (field == 7) {
            *speed_kt = (uint16_t)(strtod(token, NULL) + 0.5);  // knots, round
        }
        
        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }
    
    if (status != 'A') return false;  // no valid fix
    
    // Sync time on first fix (if not already synced and both strings available).
    if (!s_time_synced && strlen(time_str) >= 6 && strlen(date_str) >= 6) {
        // time_str: HHMMSS, date_str: DDMMYY
        int hh = ((time_str[0] - '0') * 10) + (time_str[1] - '0');
        int mm = ((time_str[2] - '0') * 10) + (time_str[3] - '0');
        int ss = ((time_str[4] - '0') * 10) + (time_str[5] - '0');
        
        int dd = ((date_str[0] - '0') * 10) + (date_str[1] - '0');
        int mo = ((date_str[2] - '0') * 10) + (date_str[3] - '0');
        int yy = ((date_str[4] - '0') * 10) + (date_str[5] - '0');
        
        // Convert to Unix time (simplified: assume 20YY).
        struct tm t = {
            .tm_sec = ss,
            .tm_min = mm,
            .tm_hour = hh,
            .tm_mday = dd,
            .tm_mon = mo - 1,
            .tm_year = yy + 100,  // 1900-based
            .tm_isdst = 0,
        };
        time_t unix_time = mktime(&t);
        
        struct timeval tv = {
            .tv_sec = unix_time,
            .tv_usec = 0,
        };
        settimeofday(&tv, NULL);
        s_time_synced = true;
        ESP_LOGI(TAG, "Synced system time from GPS: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 yy + 2000, mo, dd, hh, mm, ss);
    }
    
    return true;
}

// Parse NMEA GSA sentence: $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*30
// Extract: satellites used in solution, PDOP, HDOP, VDOP
// Returns true if parse succeeded.
static bool parse_gsa(const char *line, uint8_t *num_sats_in_solution, 
                      uint8_t *pdop_x10, uint8_t *hdop_x10, uint8_t *vdop_x10)
{
    // Fields: 0=type, 1=mode(A/M), 2=fix(1/2/3), 3-14=sat IDs, 15=PDOP, 16=HDOP, 17=VDOP
    char buf[256];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    
    int field = 0;
    char *saveptr, *token = strtok_r(buf, ",", &saveptr);
    
    uint8_t sat_count = 0;
    double pdop = 0, hdop = 0, vdop = 0;
    
    while (token != NULL && field <= 17) {
        // Count satellites in solution (fields 3-14, non-empty = satellite ID)
        if (field >= 3 && field <= 14) {
            if (strlen(token) > 0) {
                sat_count++;
            }
        } else if (field == 15) {
            pdop = strtod(token, NULL);
            *pdop_x10 = (uint8_t)(pdop * 10 + 0.5);
            if (*pdop_x10 > 999) *pdop_x10 = 999;
        } else if (field == 16) {
            hdop = strtod(token, NULL);
            *hdop_x10 = (uint8_t)(hdop * 10 + 0.5);
            if (*hdop_x10 > 999) *hdop_x10 = 999;
        } else if (field == 17) {
            vdop = strtod(token, NULL);
            *vdop_x10 = (uint8_t)(vdop * 10 + 0.5);
            if (*vdop_x10 > 999) *vdop_x10 = 999;
        }
        
        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }
    
    *num_sats_in_solution = sat_count;
    return sat_count > 0;
}

// ---- GPS Module Configuration ----
// Send MTK (MediaTek) command: $PMTK<cmd>*hh\r\n
// Used by GlobalTop BN-220, u-Blox, and many cheap modules.
// Examples: PMTK251 = set baud, PMTK314 = set NMEA output.
static void gps_send_mtk_cmd(uart_port_t port, const char *cmd)
{
    // Calculate NMEA checksum (XOR of all chars between $ and *)
    uint8_t cksum = 0;
    for (int i = 0; cmd[i] != '\0'; i++) {
        cksum ^= (uint8_t)cmd[i];
    }
    
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", cmd, cksum);
    uart_write_bytes(port, (const char *)buf, n);
    vTaskDelay(pdMS_TO_TICKS(GPS_INIT_DELAY));
}

// Configure GPS module to output GGA, RMC, and GSA at 1 Hz.
// Tries MTK commands first (BN-220, most u-blox/GlobalTop), then u-blox UBX binary.
// Returns after sending config; module may take 1-5 sec to reboot.
static void gps_configure_module(uart_port_t port, uint32_t target_baud)
{
    ESP_LOGI(TAG, "Configuring GPS module: setting GGA+RMC+GSA at 1 Hz, baud %u", target_baud);
    
    // Flush any stale data
    uart_flush(port);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Try MTK commands first (BN-220, and most cheap/u-blox modules support these)
    // PMTK251 = set baud rate
    if (target_baud == 115200)
        gps_send_mtk_cmd(port, "PMTK251,115200");
    else if (target_baud == 38400)
        gps_send_mtk_cmd(port, "PMTK251,38400");
    else  // default 9600
        gps_send_mtk_cmd(port, "PMTK251,9600");
    
    // PMTK314 = set NMEA sentence output
    // Format: $PMTK314,<GLL>,<RMC>,<VTG>,<GGA>,<GSA>,<GSV>,<GRS>,<GST>,<1Hz>*hh
    // For GGA+RMC+GSA at 1 Hz: $PMTK314,0,1,0,1,1,0,0,0,0*28
    gps_send_mtk_cmd(port, "PMTK314,0,1,0,1,1,0,0,0,0");
    
    // PMTK220 = set update rate to 1000 ms (1 Hz)
    gps_send_mtk_cmd(port, "PMTK220,1000");
    
    // Also try u-blox UBX binary protocol configuration.
    // u-blox modules often respond to both MTK and UBX; send UBX to ensure coverage.
    // UBX-CFG-MSG: enable GGA (0xF0, 0x00), RMC (0xF0, 0x04), GSA (0xF0, 0x02) every 1 position
    ESP_LOGI(TAG, "Also sending u-blox UBX configuration (GGA, RMC, GSA)");
    
    // UBX binary message format: $sync(2) class(1) id(1) len_lo(1) len_hi(1) payload(...) cksum_a(1) cksum_b(1)\r\n
    // CFG-MSG: class=0x06, id=0x01, length=8
    // Format: [class] [id] [rate_DDM] [rate_UART1] [rate_UART2] [rate_USB] [rate_SPI] [reserved]
    
    uint8_t ubx_gga[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
    uart_write_bytes(port, (char *)ubx_gga, sizeof(ubx_gga));
    vTaskDelay(pdMS_TO_TICKS(GPS_INIT_DELAY));
    
    uint8_t ubx_rmc[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x29};
    uart_write_bytes(port, (char *)ubx_rmc, sizeof(ubx_rmc));
    vTaskDelay(pdMS_TO_TICKS(GPS_INIT_DELAY));
    
    uint8_t ubx_gsa[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x27};
    uart_write_bytes(port, (char *)ubx_gsa, sizeof(ubx_gsa));
    vTaskDelay(pdMS_TO_TICKS(GPS_INIT_DELAY));
    
    ESP_LOGI(TAG, "GPS module config sent (MTK); waiting for reboot...");
    vTaskDelay(pdMS_TO_TICKS(2000));  // Module reboots after config
}

// ---- FreeRTOS Task ----

void gps_rx_task(void *arg)
{
    uint32_t baud_rate = g_settings.gps_baud;
    
    ESP_LOGI(TAG, "Starting GPS task (UART%d @ %u baud, GPIO %u RX, GPIO %u TX)",
             GPS_UART_PORT, baud_rate, GPS_RX_GPIO, GPS_TX_GPIO);
    
    uart_config_t uart_cfg = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_TX_GPIO, GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, GPS_RX_BUF, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "GPS UART initialized; configuring module...");
    
    // Configure GPS module to output GGA+RMC at 1 Hz
    gps_configure_module(GPS_UART_PORT, baud_rate);
    
    ESP_LOGI(TAG, "Waiting for NMEA sentences...");
    
    char line[GPS_LINE_MAX];
    int line_len = 0;
    uint32_t no_fix_count = 0;
    uint32_t total_bytes = 0, total_sentences = 0;
    
    while (1) {
        uint8_t byte;
        int n = uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        
        if (n <= 0) {
            // Timeout: check if we should reset line buffer on prolonged silence.
            if (line_len > 0 && no_fix_count++ > 300) {
                line_len = 0;  // reset after ~30 sec of partial lines
                no_fix_count = 0;
            }
            continue;
        }
        
        total_bytes++;
        if (total_bytes % 1000 == 0) {
            ESP_LOGD(TAG, "Received %u bytes, %u complete sentences so far", 
                     total_bytes, total_sentences);
        }
        
        if (byte == '\n') {
            if (line_len > 0 && line[line_len - 1] == '\r') {
                line_len--;  // strip CR
            }
            line[line_len] = '\0';
            total_sentences++;
            
            // Parse NMEA sentence.
            if (line_len > 6 && line[0] == '$') {
                // Extract sentence type (e.g., "$GPGGA" -> "GGA", "$GPRMC" -> "RMC").
                char sent_type[6];
                strncpy(sent_type, line + 3, 3);
                sent_type[3] = '\0';
                
                double lat = 0, lng = 0;
                int32_t alt_ft = 0;
                uint8_t num_sats = 0, num_sats_solution = 0, hdop_x10 = 0, vdop_x10 = 0, pdop_x10 = 0;
                uint8_t fix_quality = 0;
                float geoid_sep_ft = 0;
                uint16_t track_deg = 0, speed_kt = 0;
                
                if (strcmp(sent_type, "GGA") == 0) {
                    if (parse_gga(line, &lat, &lng, &alt_ft, &num_sats, &hdop_x10, &fix_quality, &geoid_sep_ft)) {
                        // Calculate MSL altitude: HAE - geoid separation
                        int32_t alt_msl_ft = alt_ft - (int32_t)geoid_sep_ft;
                        
                        taskENTER_CRITICAL(&s_ownship_mux);
                        s_ownship.lat = lat;
                        s_ownship.lng = lng;
                        s_ownship.alt_ft = alt_ft;
                        s_ownship.alt_msl_ft = alt_msl_ft;
                        s_ownship.num_sats = num_sats;
                        s_ownship.hdop_x10 = hdop_x10;
                        s_ownship.geoid_sep_ft = geoid_sep_ft;
                        s_ownship.fix_quality = fix_quality;
                        s_ownship.fix_time_ms = esp_timer_get_time() / 1000;
                        s_ownship.valid = true;
                        taskEXIT_CRITICAL(&s_ownship_mux);
                        no_fix_count = 0;
                        ESP_LOGD(TAG, "GGA: lat=%.6f lng=%.6f alt=%ld ft (MSL %ld ft) sats=%u hdop=%.1f geoid=%.1f",
                                 lat, lng, alt_ft, alt_msl_ft, num_sats, hdop_x10 / 10.0, geoid_sep_ft);
                    }
                } else if (strcmp(sent_type, "RMC") == 0) {
                    if (parse_rmc(line, &track_deg, &speed_kt)) {
                        taskENTER_CRITICAL(&s_ownship_mux);
                        s_ownship.track_deg = track_deg;
                        s_ownship.speed_kt = speed_kt;
                        taskEXIT_CRITICAL(&s_ownship_mux);
                        ESP_LOGD(TAG, "RMC: track=%u deg speed=%u kt", track_deg, speed_kt);
                    }
                } else if (strcmp(sent_type, "GSA") == 0) {
                    if (parse_gsa(line, &num_sats_solution, &pdop_x10, &hdop_x10, &vdop_x10)) {
                        taskENTER_CRITICAL(&s_ownship_mux);
                        s_ownship.num_sats_tracked = num_sats_solution;
                        s_ownship.pdop_x10 = pdop_x10;
                        s_ownship.vdop_x10 = vdop_x10;
                        taskEXIT_CRITICAL(&s_ownship_mux);
                        ESP_LOGD(TAG, "GSA: sats_in_solution=%u pdop=%.1f hdop=%.1f vdop=%.1f",
                                 num_sats_solution, pdop_x10 / 10.0, hdop_x10 / 10.0, vdop_x10 / 10.0);
                    }
                }
            }
            
            line_len = 0;
        } else if (byte == '$') {
            // Start of new sentence; reset buffer.
            line_len = 0;
            line[line_len++] = byte;
        } else if (line_len < GPS_LINE_MAX - 1) {
            line[line_len++] = byte;
        } else {
            // Line overflow; reset and drop.
            line_len = 0;
        }
    }
}
