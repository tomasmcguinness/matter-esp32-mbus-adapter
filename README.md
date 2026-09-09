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

The radio is chosen by an explicit defaults overlay, so there is no single
`idf.py build` — each variant gets its own build directory and generated config,
and the two never collide. `set-target` is needed once per build directory.

```sh
cd firmware

# Thread — the supported configuration
idf.py -B build.thread -DSDKCONFIG=sdkconfig.thread \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.thread" \
  set-target esp32c6
idf.py -B build.thread -DSDKCONFIG=sdkconfig.thread \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.thread" \
  build flash monitor

# Wi-Fi — read the warning in sdkconfig.defaults.wifi first
idf.py -B build.wifi -DSDKCONFIG=sdkconfig.wifi \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.wifi" \
  set-target esp32c6
idf.py -B build.wifi -DSDKCONFIG=sdkconfig.wifi \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.wifi" \
  build flash monitor
```

`SDKCONFIG_DEFAULTS` must be passed to **`set-target` as well as `build`**.
`set-target` is what creates the config file, and defaults are only ever applied
when the file is created — pass it only to `build` and you silently get a config
with no radio overlay at all.

`sdkconfig.defaults.esp32c6` holds the settings common to both and selects no
radio. IDF appends `<file>.esp32c6` to every entry in `SDKCONFIG_DEFAULTS`, so it
is applied automatically and only the radio overlay has to be named.

Note that editing a defaults file does **not** update an existing generated
config — IDF only seeds one that does not yet exist. That is why each variant has
its own `-DSDKCONFIG`; if you do need to re-seed one, delete it first.

BLE is used for commissioning in both cases.

**Wi-Fi does not work on revision F hardware.** The 3V3 LDO (U3, `MCP1700T-3302E/TT`)
supplies 250 mA against a Wi-Fi TX peak north of 300 mA, so the board
brownout-resets at association and never finishes starting. Thread's 802.15.4 TX
peak fits the budget. See `firmware/sdkconfig.defaults.wifi` for the full
measurements and what a hardware fix would need.

### Factory reset

**SW2** is the ESP32-C6 BOOT button (net `BOOT` → IO9). Hold it for **five
seconds while the device is running** to wipe the Matter credentials from NVS and
reboot uncommissioned. The status LED (D3, net `LED` → IO15) blinks faster as the
hold progresses and goes solid at the moment the reset commits, so the button can
be released once it stops flashing.

Release before five seconds and nothing happens — the aborted hold is logged with
its elapsed time.

IO9 is a strapping pin, so holding SW2 *across* a power-up or an SW1 press puts
the chip into serial-download mode and the application never starts. That is
useful for flashing, but it is **not** the factory-reset gesture — the device has
to already be running.

SW1 pulls `EN` low and is a plain hardware reset; it does not touch NVS.

## Known limitations

- Matter publishing is commented out in `main.cpp` while bench testing continues.
- The LDO (U3, `MCP1700`, 250 mA) cannot supply Wi-Fi. Confirmed on the bench:
  10 brownout resets in 15 s, every one at Wi-Fi association. Thread works; Wi-Fi
  needs more bulk capacitance on 3V3 and/or a 500-600 mA regulator.
- The custom cluster uses the Matter **test** vendor ID `0xFFF1`. A real product
  would need an allocated Vendor ID.
- Raw RX hex logging is enabled in `mbus.cpp` (`MBUS_LOG_RAW_RX`) and should be
  turned off for production.
- Only a subset of M-Bus data records is decoded, and it has so far been
  exercised against a test slave rather than a wide range of real meters.
