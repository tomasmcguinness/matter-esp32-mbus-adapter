# Matter ESP32 M-Bus Adapter

> ⚠️ **Work in progress.** This project is under active development and is not
> finished. The hardware is at the bring-up stage, the firmware currently runs in
> a bench-test mode that logs meter data rather than publishing it over Matter,
> and nothing here should be considered stable or production-ready. Expect
> breaking changes, incomplete features and rough edges.

A [Matter](https://csa-iot.org/all-solutions/matter/) accessory that reads a
wired [M-Bus](https://en.wikipedia.org/wiki/Meter-Bus) (EN 13757) heat meter and
exposes its readings — flow rate, flow/return temperatures and power — to a
Matter fabric over Thread.

It is built around an **ESP32-C6** (native 802.15.4 radio) and a custom PCB that
provides the M-Bus master side: the ~36 V/~24 V bus supply, the mark/space
modulator and the receive comparator, so a meter can be connected directly to the
board's M+/M- terminals.

## Repository layout

| Path | Contents |
|---|---|
| `firmware/` | The main ESP-IDF / esp-matter application for the ESP32-C6 |
| `hardware/PCB/` | KiCad schematic, PCB layout and BOM (`PCB.csv`, Mouser part numbers) |
| `tools/pcb-test/` | Standalone GPIO-only app for bringing up the M-Bus analogue circuit — see its own [README](tools/pcb-test/README.md) |

## Firmware

Source lives in `firmware/main`:

- `mbus.cpp` / `mbus.h` — the M-Bus link layer: UART setup (2400 8E1), `SND_NKE`
  link reset and `REQ_UD2` data requests, frame validation and checksums.
- `mbus_parser.cpp` / `mbus_parser.h` — decodes EN 13757-3 data records into a
  `heat_meter_data_t` struct, plus a verbose `mbus_parse_test()` used on the bench.
- `heat_meter_cluster.h` — IDs for a manufacturer-specific *Heat Meter* cluster,
  which carries the meter dataset at higher precision than the standard Flow
  Measurement cluster allows.
- `main.cpp` — brings up the Matter node and endpoints and runs the M-Bus poll task.

### Bring-up modes

`main.cpp` has an `MBUS_MODE` switch so each layer can be proven in turn. It
currently defaults to `MBUS_MODE_TEST`.

| Mode | Behaviour |
|---|---|
| `MBUS_MODE_NKE_ONLY` | Sends `SND_NKE` on a loop and reports whether the meter ACKs — checks wiring and polarity only |
| `MBUS_MODE_TEST` | Requests data and hex-dumps/decodes every record found. Nothing reaches Matter |
| `MBUS_MODE_NORMAL` | Parses into `heat_meter_data_t` and publishes to the Matter data model |

### Building

Requires ESP-IDF (with `esp_matter` pulled in as a managed component — see
`firmware/main/idf_component.yml`).

```sh
cd firmware
idf.py set-target esp32c6
idf.py build flash monitor
```

The device is Thread-only (Wi-Fi is disabled); BLE is used for commissioning.

## Known limitations

- Matter publishing is commented out in `main.cpp` while bench testing continues.
- The LDO is small and might not be able to provide WiFi.
- The custom cluster uses the Matter **test** vendor ID `0xFFF1`. A real product
  would need an allocated Vendor ID.
- Raw RX hex logging is enabled in `mbus.cpp` (`MBUS_LOG_RAW_RX`) and should be
  turned off for production.
- Only a subset of M-Bus data records is decoded, and it has so far been
  exercised against a test slave rather than a wide range of real meters.
