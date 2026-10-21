# Matter Heat Meter - Kamstrup MULTICAL 403

> ⚠️ **Work in progress.** So far this has been exercised against the
> [simulator](../../../tools/kamstrup-403-sim) rather than a wide range of real
> meters. Expect breaking changes.

This example turns the M-Bus adapter into a Matter accessory for a
[Kamstrup MULTICAL 403](https://www.kamstrup.com/en-en/heat-solutions/meters-devices/meters/multical-403)
heat meter. The adapter polls the meter over wired M-Bus and exposes its
readings — flow rate, flow/return temperatures and power — to a Matter fabric
over Thread.

It is built with [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) and
[esp-matter](https://github.com/espressif/esp-matter), and targets the ESP32-C6
on the adapter board (see [`hardware/`](../../../hardware)).

## Source

Source lives in `main/`:

- `mbus.cpp` / `mbus.h` — the M-Bus link layer: UART setup (2400 8E1), `SND_NKE`
  link reset and `REQ_UD2` data requests, frame validation and checksums.
- `mbus_parser.cpp` / `mbus_parser.h` — decodes EN 13757-3 data records into a
  `heat_meter_data_t` struct, plus a verbose `mbus_parse_test()` used on the bench.
- `heat_meter_cluster.h` — IDs for a manufacturer-specific *Heat Meter* cluster,
  which carries the meter dataset at higher precision than the standard Flow
  Measurement cluster allows.
- `reset_button.cpp` / `reset_button.h` — the long-press factory reset on SW2.
- `main.cpp` — brings up the Matter node and endpoints and runs the M-Bus poll task.

## Bring-up modes

`main.cpp` has an `MBUS_MODE` switch so each layer can be proven in turn. It
defaults to `MBUS_MODE_NORMAL`.

| Mode | Behaviour |
|---|---|
| `MBUS_MODE_NKE_ONLY` | Sends `SND_NKE` on a loop and reports whether the meter ACKs — checks wiring and polarity only |
| `MBUS_MODE_TEST` | Requests data and hex-dumps/decodes every record found. Nothing reaches Matter |
| `MBUS_MODE_NORMAL` | Parses into `heat_meter_data_t` and publishes to the Matter data model |

## Building and flashing

Requires ESP-IDF (with `esp_matter` pulled in as a managed component — see
`main/idf_component.yml`).

The radio is chosen by an explicit defaults overlay, so there is no single
`idf.py build` — each variant gets its own build directory and generated config,
and the two never collide. `set-target` is needed once per build directory.

```sh
cd examples/matter/KamstrupMultical403

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

**Thread is the supported configuration.** On revision F hardware Wi-Fi
brownout-reset the board at association. Wi-Fi has not been re-checked on
revision G, where the 3V3 LDO (U4, AP2112K-3.3) shares the USB +5V rail with
the M-Bus boost converter. See the supply notes in `main.cpp`.

## Factory reset

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

## Testing without a meter

[`tools/kamstrup-403-sim`](../../../tools/kamstrup-403-sim) makes a Raspberry Pi
Pico answer M-Bus polls with the telegram a real MULTICAL 403 sends, so this
example can be exercised on the bench. For bringing up the analogue M-Bus circuit
on its own, start with [`tools/pcb-test`](../../../tools/pcb-test).

## Known limitations

- Wi-Fi is not supported (see above).
- The custom cluster uses the Matter **test** vendor ID `0xFFF1`. A real product
  would need an allocated Vendor ID.
- Raw RX hex logging is enabled in `mbus.cpp` (`MBUS_LOG_RAW_RX`) and should be
  turned off for production.
- Only a subset of M-Bus data records is decoded, and it has so far been
  exercised against a test slave rather than a wide range of real meters.
