#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Decoded subset of an EN 13757-3 heat/flow meter telegram.
typedef struct {
    bool   has_flow;         // volume flow
    bool   has_energy;       // heat energy
    bool   has_volume;
    bool   has_flow_temp;
    bool   has_return_temp;
    bool   has_power;

    float  flow_m3h;         // m^3/h
    double energy_wh;        // Wh
    double volume_m3;        // m^3
    float  flow_temp_c;      // deg C
    float  return_temp_c;    // deg C
    float  power_w;          // W
} heat_meter_data_t;

// Parse the user-data block of an M-Bus RSP_UD long frame. `user` must point at
// the C field (i.e. C, A, CI, <header>, <data records...>), `len` its length.
// Populates the `has_*` flags for every quantity recognised. Returns ESP_OK if
// the frame header was understood (individual records may still be absent).
esp_err_t mbus_parse(const uint8_t *user, size_t len, heat_meter_data_t *out);

#ifdef __cplusplus
}
#endif
