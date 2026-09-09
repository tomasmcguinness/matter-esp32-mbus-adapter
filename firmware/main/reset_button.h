#pragma once

#include "esp_err.h"

// Factory-reset button.
//
// SW2 on the board is the ESP32-C6 BOOT button (net BOOT -> IO9), a plain
// push-to-ground with no external pull-up. Holding it for RESET_HOLD_MS while
// the device is running wipes the Matter credentials from NVS and reboots, so
// the adapter can be commissioned onto a different fabric. D3 (net LED -> IO15)
// blinks progressively faster during the hold so it is obvious the press has
// been registered.
//
// Note: IO9 is a strapping pin. If it is held low as the chip leaves reset the
// ROM enters serial-download mode and this application never starts, so the
// gesture is deliberately "hold while running" rather than "hold and power on".
//
// Starts a background task; returns ESP_OK once the GPIOs are configured and
// the task is running. Failure is not fatal to the application -- the meter can
// still be polled without a working button.
esp_err_t reset_button_init(void);
