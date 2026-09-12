#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Decoded subset of an EN 13757-3 heat/flow meter telegram: the quantities the
// Heat Meter cluster publishes, and nothing else. A telegram carries plenty
// more -- energy, volume, hour counters, target-date copies -- which the parser
// walks past. See kAccept[] in mbus_parser.cpp for the accepted VIF codes.
typedef struct {
    bool   has_flow;         // volume flow
    bool   has_flow_temp;
    bool   has_return_temp;
    bool   has_power;

    float  flow_m3h;         // m^3/h
    float  flow_temp_c;      // deg C
    float  return_temp_c;    // deg C
    float  power_w;          // W
} heat_meter_data_t;

// Parse the user-data block of an M-Bus RSP_UD long frame. `user` must point at
// the C field (i.e. C, A, CI, <header>, <data records...>), `len` its length.
// Populates the `has_*` flags for every quantity recognised. Returns ESP_OK if
// the frame header was understood (individual records may still be absent).
esp_err_t mbus_parse(const uint8_t *user, size_t len, heat_meter_data_t *out);

// Bench/debug only: walk the data records of an M-Bus RSP_UD user block and log
// every one it finds (name, scaled value, unit, raw value). Unlike mbus_parse()
// this decodes the 0xFD/0xFB VIFE extension quantities (volts, amperes, digital
// I/O, firmware version, baud rate) and it does not care whether the quantity
// belongs to a heat meter. Returns nothing and touches no Matter state.
//
// `user` has the same layout as for mbus_parse(): C, A, CI, <header>, records.
// The offset at which the records start is probed rather than taken from CI,
// so this still works against a slave that emits bare records with no header.
void mbus_parse_test(const uint8_t *user, size_t len);

#ifdef __cplusplus
}
#endif
