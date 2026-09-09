# Kamstrup MULTICAL 403 M-Bus slave simulator

An Arduino sketch that makes a Raspberry Pi Pico answer M-Bus polls the way a
Kamstrup MULTICAL 403 heat meter does, so the ESP32-C6 adapter in `firmware/`
can be exercised without the real meter on the desk.

Unlike the generic bench slave that `mbus_parse_test()` was written for, this
emits a telegram a real 403 would send — Kamstrup identity fields, CI `0x72`
with a proper 12-byte header, and standard EN 13757-3 data records — so it
exercises the **production** decode path, `mbus_parse()`.

Modelled on [HWHardsoft/Arduino-MBUS-Meter](https://github.com/HWHardsoft/Arduino-MBUS-Meter)
(GPLv3, via OpenEnergyMonitor's HeatpumpMonitor), retargeted from its AEG
electricity-meter payload to a heat meter.

## Build

Either RP2040 core works:

- **Arduino Mbed OS RP2040 Boards** — in Boards Manager by default. Serial1's
  pins are fixed at construction, but they are already GP0/GP1.
- **[arduino-pico](https://github.com/earlephilhower/arduino-pico)**
  (earlephilhower) — add the Boards Manager URL
  `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`.
  Lets the sketch move the UART with `setTX`/`setRX`, so `MBUS_TX_PIN` /
  `MBUS_RX_PIN` in `mbusslave.h` can be changed.

The sketch detects which core it is on (`ARDUINO_ARCH_MBED`) and skips the pin
assignment where it is not supported — see `MBUS_SERIAL_PINS_FIXED` in
`mbusslave.h`. The startup banner prints which pins are actually in use. Both
cores honour `SERIAL_8E1`, which is the part M-Bus depends on.

1. Tools → Board → **Raspberry Pi Pico**
2. Open `kamstrup-403-sim.ino`, Upload
3. Serial Monitor at **115200** for the debug console

No libraries to install — the records are hand-rolled, so there is no
`MBUSPayload` dependency.

## Wiring

| Pico | Direction | M-Bus slave HAT |
|---|---|---|
| GP0 (UART0 TX, digital 0) | → | modulator in |
| GP1 (UART0 RX, digital 1) | ← | comparator out |
| GND | — | GND |

The HAT does the level conversion. On the wire, mark/idle is the high bus
voltage (~36 V) and a slave answers by modulating current, never by driving
voltage — see `tools/pcb-test/README.md` for the measured master-side numbers.

Link settings are **2400 baud, 8E1, primary address 0x05**, matching
`firmware/main/mbus.cpp:61-73` and `firmware/main/mbus.h:9`. A mismatch here is
the classic silent failure: the master just times out having received nothing
at all.

## Telegram

```
68 39 39 68
08            C  = RSP_UD
05            A  = primary address
72            CI = variable data, 12-byte header
78 56 34 12   identification 12345678, BCD, LSB first
2D 2C         manufacturer 0x2C2D = "KAM"
34            version
04            medium = heat (volume measured at return)
nn            access number, increments per response
00            status: no alarm, no error
00 00         signature (unused)
-- data records --
04 06 <i32>   Energy                kWh        VIF 0x06 -> 10^3 Wh
04 14 <i32>   Volume                0.01 m3    VIF 0x14 -> 10^-2 m3
04 22 <i32>   On time               hours      VIF 0x22
04 2D <i32>   Power                 0.1 kW     VIF 0x2D -> 10^2 W
04 3B <i32>   Volume flow           l/h        VIF 0x3B -> 10^-3 m3/h
02 59 <i16>   Flow temperature T1   0.01 degC  VIF 0x59 -> 10^-2 degC
02 5D <i16>   Return temperature T2 0.01 degC  VIF 0x5D -> 10^-2 degC
02 61 <i16>   Temperature diff      0.01 K     VIF 0x61 -> 10^-2 K
CS 16
```

`L = 3 + 12 + 42 = 57 (0x39)`, total 63 bytes. `CS` is the plain 8-bit sum of
the 57 bytes from `C` through the last record byte. All data is little-endian.

Every VIF above resolves identically in **both** master decoders — `classify()`
and the bench table `kVifTest[]` in `firmware/main/mbus_parser.cpp` — and the
records consume exactly `L` bytes, which is what `mbus_parse_test()`'s
"clean walk" check requires.

Power is quantised to 0.1 kW by VIF `0x2D`, so a modelled 4.07 kW is reported
as 4.1 kW. That is the real 403 coding, not a rounding bug.

## Golden frame

Printed by `setup()` at `t = 0`, with access number `0x00` (the first real
response carries `0x01`):

```
68 39 39 68 08 05 72 78 56 34 12 2D 2C 34 04 00 00 00 00
04 06 68 10 00 00  04 14 50 46 00 00  04 22 FC 53 00 00
04 2D 29 00 00 00  04 3B BC 02 00 00
02 59 CC 10  02 5D D8 0E  02 61 F4 01
F4 16
```

Decodes as: Kamstrup, medium heat, ID 12345678, energy 4200 kWh, volume
180.00 m³, on time 21500 h, power 4.1 kW, flow 700 l/h, T1 43.00 °C,
T2 38.00 °C, ΔT 5.00 K.

Paste it into an offline decoder (rSCADA `libmbus`, or `wmbusmeters --analyze`)
to check it independently of this repo's parser.

## Simulated values

Flow rate, temperature difference and return temperature are the independent
variables — slow sines on different periods, so the pattern does not look
canned. Everything else is derived from them, the way a real meter's readings
always are:

```
flow     700 +- 250 l/h   (180 s period)
dT         5 +-   2 K     ( 97 s period)
T2        38 +-   3 degC  (240 s period)
T1       = T2 + dT
power    = flow * dT * 1.163 kWh/m3/K
energy  += power * dt      seeded at 4200 kWh
volume  += flow  * dt      seeded at 180 m3
```

Energy and volume accumulate in `double` and only quantise at encode time, so
they rise monotonically. `power_kw` always agrees with `flow * dT * 1.163` — a
master that decodes all three can therefore self-check.

The model advances on a 1 s `millis()` timer independent of polling, so values
move whether or not the master is asking. To swap in fixed values instead, edit
`encode_records()` — nothing else in the sketch knows about scaling.

## Also supported

Beyond SND_NKE / REQ_UD2, the sketch answers the frames a real 403 does:

- **REQ_UD1** — ACK, no class 1 (alarm) data
- **Secondary address selection** (CI `0x52`) — matches on the four BCD ID
  bytes, honouring whole-byte `0xFF` wildcards
- **Change primary address** (CI `0x51`, `01 7A <new>`)
- **Change baud rate** (CI `0xB8`–`0xBD`) — ACKs at the old rate, then switches

Broadcast addresses `0xFD`/`0xFE` are accepted for SND_NKE; a REQ_UD2 on a
broadcast address is answered only after a successful secondary select, so
several slaves on one bus do not all talk at once.

The adapter's master re-sends SND_NKE before every REQ_UD2 and never toggles
the FCB bit (`mbus_request_data()` in `firmware/main/mbus.cpp`), so nothing here
depends on FCB state — but `FCB_BIT_MASK` is applied so a master that does
toggle it still works.

## Verifying against the adapter

```
cd firmware && idf.py -p <port> monitor
```

`MBUS_LOG_RAW_RX` is already `1`, so compare the master's raw RX hex
byte-for-byte against the `tx:` line the Pico prints. Then confirm
`mbus_parse()` — not just `mbus_parse_test()` — resolves all seven quantities,
and that the Matter heat-meter attributes follow.

Left running a few minutes, energy and volume must increase monotonically.
Power-cycling the Pico mid-poll should see the master recover on its next
SND_NKE; a corrupt frame is dropped and the sketch resyncs on the next start
byte rather than wedging.
