#pragma once
#include <stdbool.h>

// '$'-prefixed command channel on the console UART0 (USB serial — NOT the Pong
// link). Lets a host set the WiFi client (STA) credentials and the static
// GDL90 targets without joining the SoftAP; tools/wifi_config.py drives it.
//
// Grammar (one command per line, <=256 chars; values bare or double-quoted
// with \" and \\ escapes):
//   $WIFI GET                                    -> $OK sta_en=.. ssid=".." pass=<***|""> ip=.. gw=.. dns=.. state=..
//   $WIFI SET [sta_en=0|1] [ssid=".."] [pass=".."]  (PATCH semantics, >=1 key)
//                                                -> $OK saved (reboot to apply)
//   $DEST GET                                    -> $OK dest=<csv>
//   $DEST SET dest=<ip[,ip...]|"">  (applies live + saves)
//   $REBOOT                                      -> $OK rebooting
// Errors reply "$ERR <reason>". Replies are single full lines starting with
// '$' so a host can filter them out of the interleaved log stream.

// Start the command channel. Production (PONG_SOURCE_RADIO): spawns a task
// that owns UART0 RX. Replay builds (PONG_SOURCE_CONSOLE): no-op — pong_rx_task
// owns UART0 and routes '$' lines to console_cmd_handle_line() itself.
void console_cmd_start(void);

// Process one complete '$'-prefixed line; prints the $OK/$ERR reply to stdout.
// Safe to call from any task. Returns false if the line is not a '$' command.
bool console_cmd_handle_line(const char *line);
