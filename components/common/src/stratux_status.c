#include "stratux_status.h"

// Zero-initialized at boot: link down, no messages, UTC not OK — do not claim
// "UTC OK" before a real clock exists (see M0 / time source in AGENTS.md).
stratux_status_t g_status = {0};
