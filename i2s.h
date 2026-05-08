#pragma once

// Compatibility shim for ESP8266Audio on esp32 core 3.3.8.
// The library still includes <i2s.h>, but the core only ships the legacy
// compatibility header under driver/deprecated/driver/i2s.h.
#include <driver/deprecated/driver/i2s.h>
