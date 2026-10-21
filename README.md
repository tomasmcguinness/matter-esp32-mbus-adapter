# Matter ESP32 M-Bus Adapter

> ⚠️ **Work in progress.** This project is under active development and is not
> finished. Nothing here should be considered stable or production-ready. Expect
> breaking changes, incomplete features and rough edges.

An ESP32-C6 adapter board for wired [M-Bus](https://en.wikipedia.org/wiki/Meter-Bus)
(EN 13757) meters, with firmware examples that expose meter readings to a
[Matter](https://csa-iot.org/all-solutions/matter/) fabric over Thread.

It is built around an **ESP32-C6** (native 802.15.4 radio) and a custom PCB that
provides the M-Bus master side: the ~36 V/~24 V bus supply, the mark/space
modulator and the receive comparator, so a meter can be connected directly to the
board's M+/M- terminals.

## Repository layout

| Path | Contents |
|---|---|
| `examples/` | Firmware examples, each a standalone ESP-IDF project — see the [examples README](examples/README.md) |
| `hardware/PCB/` | KiCad schematic, PCB layout and BOM (`PCB.csv`, Mouser part numbers) |
| `tools/pcb-test/` | Standalone GPIO-only app for bringing up the M-Bus analogue circuit — see its own [README](tools/pcb-test/README.md) |
| `tools/kamstrup-403-sim/` | Raspberry Pi Pico sketch that simulates a Kamstrup MULTICAL 403 on the bus — see its own [README](tools/kamstrup-403-sim/README.md) |

## Examples

| Example | Device | Matter device type |
|---------|--------|--------------------|
| [KamstrupMultical403](examples/matter/KamstrupMultical403) | Kamstrup MULTICAL 403 heat meter | Custom Heat Meter cluster, with Flow Measurement |

Each example's README covers building, flashing, bring-up modes and the factory
reset gesture.
