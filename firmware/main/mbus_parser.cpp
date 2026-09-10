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

// ---------------------------------------------------------------------------
// Bench/debug decoder. Separate from mbus_parse() on purpose: it recognises far
// more quantities, it has no opinion about heat meters, and it only logs.
// ---------------------------------------------------------------------------

// A quantity occupies a run of consecutive VIF codes, one per decimal exponent.
// `base` is the code for the LOWEST exponent:
//   0x0000..0x007F  primary VIF table
//   0xFD00..0xFD7F  VIF 0xFD escape, key = 0xFD00 | (VIFE & 0x7F)
//   0xFB00..0xFB7F  VIF 0xFB escape, key = 0xFB00 | (VIFE & 0x7F)
// The run is `size` codes long, and exponent(key) = scalar + (key - base).
// This mirrors the vif_defs[] table in the AllWize MBUSPayload library that the
// bench slave HAT encodes with, so the two stay in step.
typedef struct {
    uint16_t    base;
    uint8_t     size;
    int8_t      scalar;
    const char *name;
    const char *unit;
} vif_test_def_t;

static const vif_test_def_t kVifTest[] = {
    // --- 0xFD extension table: what the bench slave HAT sends ---
    { 0xFD0E,  1,   0, "Firmware version",     ""      },
    { 0xFD1A,  1,   0, "Digital output",       ""      },
    { 0xFD1B,  1,   0, "Digital input",        ""      },
    { 0xFD1C,  1,   0, "Baud rate",            "bps"   },
    { 0xFD40, 16,  -9, "Voltage",              "V"     },
    { 0xFD50, 16, -12, "Current",              "A"     },
    { 0xFD17,  1,   0, "Error flags",          ""      },
    { 0xFD08,  1,   0, "Access number",        ""      },
    { 0xFD09,  1,   0, "Medium",               ""      },
    { 0xFD0A,  1,   0, "Manufacturer",         ""      },
    { 0xFD0B,  1,   0, "Parameter set id",     ""      },
    { 0xFD0C,  1,   0, "Model / version",      ""      },
    { 0xFD0D,  1,   0, "Hardware version",     ""      },
    { 0xFD0F,  1,   0, "Software version",     ""      },
    { 0xFD10,  1,   0, "Customer location",    ""      },
    { 0xFD11,  1,   0, "Customer",             ""      },
    { 0xFD18,  1,   0, "Error mask",           ""      },
    { 0xFD1D,  1,   0, "Response delay time",  ""      },
    { 0xFD1E,  1,   0, "Retry",                ""      },
    { 0xFD3C,  1,   0, "Dimensionless",        ""      },
    { 0xFD60,  1,   0, "Reset counter",        ""      },
    { 0xFD61,  1,   0, "Cumulation counter",   ""      },
    { 0xFD63,  1,   0, "Day of week",          ""      },
    { 0xFD64,  1,   0, "Week number",          ""      },
    // --- 0xFB extension table ---
    { 0xFB00,  2,  -1, "Energy",               "MWh"   },
    { 0xFB08,  2,  -1, "Energy",               "GJ"    },
    { 0xFB10,  2,   2, "Volume",               "m^3"   },
    { 0xFB18,  2,   2, "Mass",                 "t"     },
    { 0xFB21,  1,  -1, "Volume",               "ft^3"  },
    { 0xFB28,  2,  -1, "Power",                "MW"    },
    { 0xFB30,  2,  -1, "Power",                "GJ/h"  },
    { 0xFB58,  4,  -3, "Flow temperature",     "degF"  },
    { 0xFB5C,  4,  -3, "Return temperature",   "degF"  },
    { 0xFB60,  4,  -3, "Temperature diff",     "degF"  },
    { 0xFB64,  4,  -3, "External temperature", "degF"  },
    { 0xFB78,  8,  -3, "Max power",            "W"     },
    // --- 0xFF: manufacturer specific ---
    //
    // EN 13757-3 assigns 0xFF no meaning, and no public table decodes it -
    // libmbus stops at "manufacturer specific". These two were derived from a
    // live MULTICAL 403 telegram rather than looked up, so treat them as
    // evidence-backed rather than authoritative.
    //
    // They are Kamstrup's E8/E9: the volume-weighted temperature integrals the
    // meter keeps so it can report average T1/T2. (E8 - E9)/V is the lifetime
    // average dT, and V * dT * 1.163 kWh/m^3/K reproduced the meter's own
    // energy register to within +0.7% on BOTH the live records and the
    // target-date (storage 1) set - two independent checks on one telegram.
    //
    // FF 16, FF 17, FF 1A and FF 22 are also present on a 403 and remain
    // unmapped on purpose: guessing them would print confident nonsense, which
    // is the exact failure walk_is_clean() was written to avoid. They log raw,
    // which is what identifying them from the meter's own display needs.
    { 0xFF07,  1,   0, "Volume x flow temp",   "m^3*degC" },
    { 0xFF08,  1,   0, "Volume x return temp", "m^3*degC" },
    // --- primary table: the real heat meter ---
    { 0x00,    8,  -3, "Energy",               "Wh"    },
    { 0x08,    8,   0, "Energy",               "J"     },
    { 0x10,    8,  -6, "Volume",               "m^3"   },
    { 0x18,    8,  -3, "Mass",                 "kg"    },
    { 0x20,    4,   0, "On time",              ""      },
    { 0x24,    4,   0, "Operating time",       ""      },
    { 0x28,    8,  -3, "Power",                "W"     },
    { 0x30,    8,   0, "Power",                "J/h"   },
    { 0x38,    8,  -6, "Volume flow",          "m^3/h" },
    { 0x40,    8,  -7, "Volume flow ext",      "m^3/min" },
    { 0x48,    8,  -9, "Volume flow ext",      "m^3/s" },
    { 0x50,    8,  -3, "Mass flow",            "kg/h"  },
    { 0x58,    4,  -3, "Flow temperature",     "degC"  },
    { 0x5C,    4,  -3, "Return temperature",   "degC"  },
    { 0x60,    4,  -3, "Temperature diff",     "K"     },
    { 0x64,    4,  -3, "External temperature", "degC"  },
    { 0x68,    4,  -3, "Pressure",             "bar"   },
    { 0x6C,    1,   0, "Date",                 ""      },
    { 0x6D,    1,   0, "Date/time",            ""      },
    { 0x6E,    1,   0, "Units for H.C.A.",     ""      },
    { 0x70,    4,   0, "Averaging duration",   ""      },
    { 0x74,    4,   0, "Actuality duration",   ""      },
    { 0x78,    1,   0, "Fabrication number",   ""      },
    { 0x79,    1,   0, "Enhanced identification", ""   },
    { 0x7A,    1,   0, "Bus address",          ""      },
};

// Find the row covering `key`. On a hit `*exp_out` gets the decimal exponent.
static const vif_test_def_t *vif_test_lookup(uint16_t key, int8_t *exp_out)
{
    for (size_t i = 0; i < sizeof(kVifTest) / sizeof(kVifTest[0]); i++) {
        const vif_test_def_t *d = &kVifTest[i];
        if (key >= d->base && key < (uint16_t)(d->base + d->size)) {
            *exp_out = (int8_t)(d->scalar + (int)(key - d->base));
            return d;
        }
    }
    return NULL;
}

typedef struct {
    int    records;    // structurally valid records seen
    int    known;      // records whose VIF key hit the table
    int    unknown;    // decoded, but the VIF key is not in kVifTest
    int    nonnumeric; // coding we cannot turn into a number (LVAR, etc.)
    size_t consumed;   // where the walk stopped
    bool   overrun;    // a record ran off the end of the buffer
} walk_stats_t;

// Walk the data records starting at user[start]. With `log` set, every record
// is logged; otherwise this is a silent dry run used to score the candidate
// start offsets.
static walk_stats_t walk_records(const uint8_t *user, size_t len, size_t start, bool log)
{
    walk_stats_t st = { 0, 0, 0, 0, start, false };
    size_t pos = start;

    while (pos < len) {
        uint8_t dif0 = user[pos];
        uint8_t dif = user[pos++];

        if (dif == 0x0F || dif == 0x1F) {
            break; // manufacturer-specific data / more records in the next frame
        }
        if (dif == 0x2F) {
            continue; // idle filler byte
        }

        uint8_t coding = dif & 0x0F;

        // Skip DIFEs (bit 7 = extension).
        while ((dif & 0x80) && pos < len) {
            dif = user[pos++];
        }
        if (pos >= len) {
            st.overrun = true;
            break;
        }

        // VIF, plus the VIFE that carries the quantity when the VIF is an
        // extension escape. Test for the escape byte *exactly*: 0xFD/0xFB always
        // have bit 7 set (that is what makes them escapes), so masking with 0x7F
        // first -- as mbus_parse() does -- collapses every extension record to
        // 0x7D/0x7B and loses the quantity.
        uint8_t vif = user[pos++];
        uint16_t key;
        if (vif == 0xFD || vif == 0xFB) {
            if (pos >= len) {
                st.overrun = true;
                break;
            }
            uint8_t v = user[pos++];
            key = (uint16_t)(((uint16_t)vif << 8) | (uint16_t)(v & 0x7F));
            // Any further VIFEs chain off the FIRST VIFE's bit 7, not the escape's.
            while ((v & 0x80) && pos < len) {
                v = user[pos++];
            }
        } else {
            key = (uint16_t)(vif & 0x7F);
            uint8_t v = vif;
            while ((v & 0x80) && pos < len) {
                v = user[pos++];
            }
        }

        // Determine data length.
        int data_len;
        if (coding == 0x0D) {          // LVAR: next byte is the length
            if (pos >= len) {
                st.overrun = true;
                break;
            }
            data_len = user[pos++];
        } else if (coding == 0x0F) {   // special function
            break;
        } else {
            data_len = kDifDataBytes[coding];
            if (data_len < 0) break;   // unexpected
        }

        if (pos + (size_t)data_len > len) {
            st.overrun = true;
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

        int8_t exp = 0;
        const vif_test_def_t *def = vif_test_lookup(key, &exp);
        if (!decoded) {
            st.nonnumeric++;
        } else if (def) {
            st.known++;
        } else {
            st.unknown++;
        }

        if (log) {
            if (def && decoded) {
                ESP_LOGI(TAG, "  [%d] DIF=%02X VIF=%04X %-22s = %.6f %s  (raw=%.0f exp=%d)",
                         st.records, dif0, key, def->name, value * pow10i(exp), def->unit,
                         value, exp);
            } else if (decoded) {
                // Not in kVifTest -- add a row for it.
                ESP_LOGW(TAG, "  [%d] DIF=%02X VIF=%04X UNKNOWN raw=%.0f (%d data byte(s))",
                         st.records, dif0, key, value, data_len);
            } else {
                ESP_LOGW(TAG, "  [%d] DIF=%02X VIF=%04X non-numeric coding 0x%X, %d data byte(s)",
                         st.records, dif0, key, coding, data_len);
            }
        }

        st.records++;
    }

    st.consumed = pos;
    return st;
}

// A walk is "clean" only if every single record decoded to a recognised
// quantity and the walk landed exactly on the end of the buffer.
//
// Anything less is treated as a miss. This matters: the DIF/VIF grammar is
// self-synchronising enough that starting at the WRONG offset still yields a
// run of structurally valid records, some of which land in the table by chance
// and log as confident-looking nonsense ("Volume = 6718.34 m^3"). Counting good
// records and picking the highest total therefore reliably picks the wrong
// offset. Demanding zero junk is what actually discriminates.
static bool walk_is_clean(const walk_stats_t *st, size_t len)
{
    return !st->overrun
        && st->records > 0
        && st->unknown == 0
        && st->nonnumeric == 0
        && st->consumed == len;
}

// Weaker test: the walk never ran off the end and consumed the block exactly,
// but some records carry VIFs the table does not know.
//
// No real meter can pass walk_is_clean(). A MULTICAL 403 sends eight records
// under VIF 0xFF, which EN 13757-3 defines as manufacturer-specific and which
// no public table decodes, so demanding zero unknowns is satisfiable only by
// the bench simulator. That is why a genuine 205-byte telegram reported "no
// candidate offset produced a clean walk" while decoding perfectly.
//
// Act on this ONLY for the offset the CI field declares. On its own it does not
// discriminate: that same 403 telegram walks to exactly the end from offsets 3,
// 7 and 15, so picking the structural walk with the most records would choose
// +3 and print plausible-looking rubbish - precisely what walk_is_clean() was
// written to prevent. CI plus an exact walk is two independent confirmations;
// an exact walk alone is one.
static bool walk_is_structural(const walk_stats_t *st, size_t len)
{
    return !st->overrun
        && st->records > 0
        && st->consumed == len;
}

// Record offset implied by the CI field, or 0 if the CI is not one we know.
static size_t declared_record_offset(uint8_t ci)
{
    switch (ci) {
    case 0x72: // 12-byte long header
    case 0x76:
        return 3 + 12;
    case 0x7A: // 4-byte short header
        return 3 + 4;
    case 0x77:
    case 0x78: // no header
        return 3;
    default:
        return 0;
    }
}

void mbus_parse_test(const uint8_t *user, size_t len)
{
    if (user == NULL || len < 3) {
        ESP_LOGW(TAG, "mbus_parse_test: user block too short (%u)", (unsigned)len);
        return;
    }

    // Candidate offsets at which the data records might begin. The CI field is
    // only a hint: a slave built on a payload-encoder library may emit bare
    // records with no header at all, or declare a header it does not send.
    static const uint8_t kOffsets[] = {
        2,  // no CI at all:  [C, A, records...]
        3,  // CI, no header
        7,  // CI + 4-byte short header
        15, // CI + 12-byte long header
    };

    uint8_t ci = user[2];
    size_t declared = declared_record_offset(ci);

    ESP_LOGI(TAG, "user block %u bytes, C=%02X A=%02X CI=%02X (implies records at +%u)",
             (unsigned)len, user[0], user[1], ci, (unsigned)declared);

    // Prefer the offset the CI field declares, but only if it walks cleanly.
    // Otherwise take the clean walk that yields the most records: a clean walk
    // starting further in can only have skipped real records. Never fall back
    // to a "best effort" offset -- a mis-aligned walk produces plausible-looking
    // wrong values, which is worse than reporting nothing.
    bool declared_clean = false;
    bool declared_structural = false;
    int declared_unmapped = 0;
    size_t chosen = 0;
    int chosen_records = 0;
    int clean_count = 0;

    for (size_t i = 0; i < sizeof(kOffsets) / sizeof(kOffsets[0]); i++) {
        size_t off = kOffsets[i];
        if (off >= len) {
            continue;
        }
        walk_stats_t st = walk_records(user, len, off, false);
        bool clean = walk_is_clean(&st, len);

        if (st.overrun) {
            ESP_LOGI(TAG, "  offset %2u: overrun after %d record(s)", (unsigned)off, st.records);
        } else {
            ESP_LOGI(TAG, "  offset %2u: %d rec (%d known, %d unknown, %d non-numeric), "
                          "consumed %u/%u%s",
                     (unsigned)off, st.records, st.known, st.unknown, st.nonnumeric,
                     (unsigned)st.consumed, (unsigned)len, clean ? "  CLEAN" : "");
        }

        if (off == declared && walk_is_structural(&st, len)) {
            declared_structural = true;
            declared_unmapped = st.unknown + st.nonnumeric;
        }

        if (!clean) {
            continue;
        }
        clean_count++;
        if (off == declared) {
            declared_clean = true;
        }
        if (st.records > chosen_records) {
            chosen = off;
            chosen_records = st.records;
        }
    }

    // The CI field declared this offset and the walk consumed the block exactly.
    // Trust it even with unmapped records: they log raw, which is strictly more
    // information than refusing to decode the telegram at all.
    if (declared_structural && !declared_clean) {
        ESP_LOGI(TAG, "CI 0x%02X declares +%u and it walks the block exactly; %d record(s) "
                      "carry VIFs outside the table and will log raw",
                 ci, (unsigned)declared, declared_unmapped);
        ESP_LOGI(TAG, "Decoding from offset %u:", (unsigned)declared);
        walk_records(user, len, declared, true);
        return;
    }

    if (clean_count == 0) {
        ESP_LOGW(TAG, "No candidate offset produced a clean walk -- check the raw frame dump "
                      "for the real header layout, or add the missing VIF codes to kVifTest");
        return;
    }

    if (declared_clean) {
        chosen = declared;
    } else {
        ESP_LOGW(TAG, "CI 0x%02X implies records at +%u, but that offset does not decode "
                      "cleanly; using +%u instead -- the slave's header does not match its "
                      "CI field",
                 ci, (unsigned)declared, (unsigned)chosen);
    }

    if (clean_count > 1) {
        ESP_LOGW(TAG, "%d candidate offsets decoded cleanly -- picked +%u (most records). "
                      "Check the raw dump if the values below look wrong",
                 clean_count, (unsigned)chosen);
    }

    ESP_LOGI(TAG, "Decoding from offset %u:", (unsigned)chosen);
    walk_records(user, len, chosen, true);
}
