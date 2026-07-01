#include "mbus_parser.h"

#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MBusParser";

// Number of data bytes encoded by the lower nibble of a DIF.
// Index 0x0..0xF; 0xD (LVAR) and 0xF (special) are handled separately.
static const int8_t kDifDataBytes[16] = {
    0, 1, 2, 3, 4, 4, 6, 8, /* 0x0..0x7 */
    0, 1, 2, 3, 4, -1, 6, -1 /* 0x8..0xF (0xD, 0xF => special) */
};

static bool dif_is_bcd(uint8_t coding)
{
    // BCD codings: 0x9 (2-digit) .. 0xC (8-digit) and 0xE (12-digit).
    return (coding >= 0x9 && coding <= 0xC) || coding == 0xE;
}

// Decode `n` little-endian bytes as a signed integer.
static double decode_int(const uint8_t *p, int n)
{
    int64_t v = 0;
    for (int i = n - 1; i >= 0; i--) {
        v = (v << 8) | p[i];
    }
    // Sign-extend from n bytes.
    if (n < 8 && (p[n - 1] & 0x80)) {
        int64_t mask = ((int64_t)-1) << (n * 8);
        v |= mask;
    }
    return (double)v;
}

// Decode `n` bytes of packed BCD (2 digits per byte, little-endian).
static double decode_bcd(const uint8_t *p, int n)
{
    double v = 0;
    double scale = 1;
    bool negative = false;
    for (int i = 0; i < n; i++) {
        uint8_t lo = p[i] & 0x0F;
        uint8_t hi = (p[i] >> 4) & 0x0F;
        // The most-significant nibble of the last byte holds 0xF for negatives.
        if (i == n - 1 && hi == 0xF) {
            negative = true;
            hi = 0;
        }
        v += lo * scale;
        scale *= 10;
        v += hi * scale;
        scale *= 10;
    }
    return negative ? -v : v;
}

static double pow10i(int exp)
{
    return pow(10.0, (double)exp);
}

// Map a primary-table VIF + decoded value into the output struct.
static void classify(uint8_t vif, double value, heat_meter_data_t *out)
{
    if (vif <= 0x07) {                       // Energy, Wh; 10^(n-3)
        out->energy_wh = value * pow10i((vif & 0x07) - 3);
        out->has_energy = true;
    } else if (vif <= 0x0F) {                // Energy, J; 10^(n) J -> Wh
        double j = value * pow10i(vif & 0x07);
        out->energy_wh = j / 3600.0;
        out->has_energy = true;
    } else if (vif <= 0x17) {                // Volume, m^3; 10^(n-6)
        out->volume_m3 = value * pow10i((vif & 0x07) - 6);
        out->has_volume = true;
    } else if (vif >= 0x28 && vif <= 0x2F) { // Power, W; 10^(n-3)
        out->power_w = (float)(value * pow10i((vif & 0x07) - 3));
        out->has_power = true;
    } else if (vif >= 0x30 && vif <= 0x37) { // Power, J/h; 10^(n) J/h -> W
        double jph = value * pow10i(vif & 0x07);
        out->power_w = (float)(jph / 3600.0);
        out->has_power = true;
    } else if (vif >= 0x38 && vif <= 0x3F) { // Volume flow, m^3/h; 10^(n-6)
        out->flow_m3h = (float)(value * pow10i((vif & 0x07) - 6));
        out->has_flow = true;
    } else if (vif >= 0x58 && vif <= 0x5B) { // Flow temperature, degC; 10^(n-3)
        out->flow_temp_c = (float)(value * pow10i((vif & 0x03) - 3));
        out->has_flow_temp = true;
    } else if (vif >= 0x5C && vif <= 0x5F) { // Return temperature, degC; 10^(n-3)
        out->return_temp_c = (float)(value * pow10i((vif & 0x03) - 3));
        out->has_return_temp = true;
    }
    // Other quantities (mass, temp difference, dates, etc.) are ignored.
}

esp_err_t mbus_parse(const uint8_t *user, size_t len, heat_meter_data_t *out)
{
    if (user == NULL || out == NULL || len < 3) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    // user = [C, A, CI, <header>, records...]
    uint8_t ci = user[2];
    size_t pos = 3;

    // Advance past the variable-data-block header depending on the CI field.
    switch (ci) {
    case 0x72: // 12-byte long header: id(4) man(2) ver(1) medium(1) acc(1) status(1) sig(2)
    case 0x76:
        pos += 12;
        break;
    case 0x7A: // 4-byte short header: acc(1) status(1) sig(2)
        pos += 4;
        break;
    default:
        ESP_LOGW(TAG, "Unsupported CI field 0x%02X", ci);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (pos > len) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Walk the data records.
    while (pos < len) {
        uint8_t dif = user[pos++];

        if (dif == 0x0F || dif == 0x1F) {
            // Start of manufacturer-specific data / more records in next frame.
            break;
        }
        if (dif == 0x2F) {
            continue; // idle filler byte
        }

        uint8_t coding = dif & 0x0F;

        // Skip DIFEs (bit 7 = extension).
        while ((dif & 0x80) && pos < len) {
            dif = user[pos++];
        }

        // VIF (+ optional VIFEs). We classify on the primary VIF only.
        if (pos >= len) {
            break;
        }
        uint8_t vif = user[pos++];
        uint8_t vif_primary = vif & 0x7F;
        while ((vif & 0x80) && pos < len) {
            vif = user[pos++]; // consume VIFEs
        }

        // Determine data length.
        int data_len;
        if (coding == 0x0D) {          // LVAR: next byte is the length
            if (pos >= len) break;
            data_len = user[pos++];
        } else if (coding == 0x0F) {   // special function
            break;
        } else {
            data_len = kDifDataBytes[coding];
            if (data_len < 0) break;   // unexpected
        }

        if (pos + (size_t)data_len > len) {
            ESP_LOGW(TAG, "Record overruns frame (need %d at %u/%u)", data_len, (unsigned)pos, (unsigned)len);
            break;
        }

        const uint8_t *data = &user[pos];
        pos += data_len;

        // Decode the value (integer / BCD / real). LVAR strings are skipped.
        double value = 0;
        bool decoded = false;
        if (coding == 0x05) {          // 32-bit IEEE-754 real
            if (data_len == 4) {
                float f;
                memcpy(&f, data, 4);
                value = f;
                decoded = true;
            }
        } else if (dif_is_bcd(coding)) {
            value = decode_bcd(data, data_len);
            decoded = true;
        } else if (coding >= 0x01 && coding <= 0x07) {
            value = decode_int(data, data_len);
            decoded = true;
        }

        if (decoded) {
            classify(vif_primary, value, out);
        }
    }

    return ESP_OK;
}
