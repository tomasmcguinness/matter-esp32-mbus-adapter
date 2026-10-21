# PCB bring-up test

A small standalone ESP-IDF app for checking an adapter board from a serial console. No
Matter and no Thread, so a failure here points at the board rather than the network stack.

It tests at two levels:

- **Board check**: `check` flashes LED D3, checks the receiver's idle level, then talks
  to an M-Bus slave (the bench slave HAT, the [403 simulator](../kamstrup-403-sim) or a
  real meter) and prints PASS/FAIL for each step. This is the "is the board good?" test.
- **Analogue**: TX is driven as a plain GPIO so it can be parked at a fixed level and
  M+ / M- measured with a multimeter. Use this when the board check fails, to find out
  which part of the circuit is at fault.

The M-Bus link layer (`mbus.cpp`) and record decoder (`mbus_parser.cpp`) are compiled
straight from [`examples/matter/KamstrupMultical403/main`](../../examples/matter/KamstrupMultical403/main),
so this exercises the same code the firmware runs.

## Build and flash

```
cd tools/pcb-test
idf.py set-target esp32c6
idf.py build flash monitor
```

The `pcb>` prompt is on **UART0** at 115200 8N1: J3 pin 3 is the C6's TX (GPIO16) and
pin 2 its RX (GPIO17). Connect a USB-serial adapter to GND, TX and RX only. Set it to
3.3 V logic and power the board from USB-C, because J3 pin 4 is +5V. Log output is also
mirrored to the USB-C port. To move the prompt there instead, select
`ESP_CONSOLE_USB_SERIAL_JTAG` in menuconfig.

The console setting comes from `sdkconfig.defaults`, which only takes effect when
`sdkconfig` is created. If you built this tool before the console moved to UART0, delete
`sdkconfig` and run `idf.py fullclean` first.

## Board check

Connect a slave to M+ / M- and run:

```
pcb> check 5
Flashing LED D3 three times -- watch the board.
  ...decoded records from the slave...
  [PASS] rx-idle    GPIO22 = 1 with the bus idle at mark
  [PASS] mbus-ack   SND_NKE to 0x05 -> ESP_OK
  [PASS] mbus-data  REQ_UD2 to 0x05 -> ESP_OK
  [ -- ] led        confirm by eye that D3 flashed three times
PASS: 0 automatic check(s) failed
```

The address defaults to `0x01` (`MBUS_PRIMARY_ADDRESS` in the example's `mbus.h`). Pass
the slave's primary address if it differs, as decimal or `0x` hex, or `254` (`0xFE`) to
broadcast when it is the only slave on the bus.

| Check | Proves | If it fails |
|---|---|---|
| `led` | GPIO15 → R8 → D3 | Check D3's orientation and R8 |
| `rx-idle` | The receiver (Q1 → R15 → Q3 → GPIO22) is not stuck low with the bus idle | Too much idle current (a short across M+/M-, or more than about four unit loads), or a fault in the Q1/R15/Q3 path |
| `mbus-ack` | A SND_NKE got out and the 0xE5 ACK came back | Wrong address, or run the analogue steps below |
| `mbus-data` | A full REQ_UD2 telegram arrived with a valid checksum and stop byte | Framing problem. The raw frame and UART error counts are logged above the summary |

The decoded records are logged above the summary. The slave HAT sends voltage, current,
digital input/output, firmware version and baud rate. The 403 simulator sends a heat
meter telegram.

## Commands

| Command | Effect |
|---|---|
| `check [addr]` | Run the board check above |
| `mbus ping [addr]` | Send SND_NKE and wait for the ACK |
| `mbus read [addr]` | Send REQ_UD2, then hex-dump and decode every record in the reply |
| `led on` / `led off` | Turn LED D3 on or off |
| `led blink [ms]` | Blink D3, default 500 ms period |
| `hi` / `mark` | Drive TX high as a GPIO and hold it. Expect ~34 V |
| `lo` / `space` | Drive TX low as a GPIO and hold it. Expect ~22 V |
| `sq [ms]` | Square wave on TX, default 1000 ms period (50–10000) |
| `stop` | Stop the square wave, leave TX idle high |
| `rx` | Print the GPIO22 level and the edge count since boot |
| `status` | TX mode/level, LED and RX state |
| `help` | List commands |

TX is GPIO23 and RX is GPIO22. The `mbus` and `check` commands hand TX to UART1
(2400 8E1). `hi`, `lo` and `sq` take it back as a plain GPIO, so you can switch between
them freely. TX starts as a GPIO, high: an M-Bus bus idles at mark, and holding it low
looks like a break condition to some meters.

## Analogue bring-up

Use this when `check` fails, or on a freshly assembled board before connecting a slave.
Stop at the first step that fails.

Probe M+ (red) and M- (black) with the DMM in DC volts:

| TX (GPIO23) | M+ / M- |
|---|---|
| high (mark / idle) | ~34 V |
| low (space) | ~22 V |

M+ is the 35 V boost output. M- is the emitter of Q1, whose base Q2 either pulls to
ground (mark) or leaves on the R12/R19 divider at ~12.6 V (space). The ~12.6 V
difference between the two levels is the real result. If the two readings are the same,
the modulator isn't switching. If they're swapped, TX polarity is inverted on the board.

1. **Supply only.** Power the board with the C6 unflashed or held in reset and measure
   **M+ to GND**: ~35 V. If not, the boost converter (U2) is at fault and no firmware
   will help. Don't use M+ / M- for this step: with the C6 in reset nothing drives
   GPIO23, so the modulator sits at the space level or somewhere undefined.
2. **Flash** this app (above).
3. **Mark.** Type `hi`, probe M+/M-: ~34 V, stable.
4. **Space.** Type `lo`, same points: ~22 V.
5. **Under load.** Repeat 3 and 4 with a 22 kΩ (≥0.1 W) resistor across M+/M- to emulate
   one M-Bus unit load (~1.5 mA). The mark voltage should sag only slightly; a large sag
   means the supply can't source bus current. `rx` should still read 1.
6. **Toggling.** `sq 1000` and watch the DMM swing, or scope M+/M- for clean ~12 V steps.
   `stop` when done. This is far slower than 2400 baud, so it won't show slow edges.
7. **Receiver path.** `hi`, then `rx`: it should read 1. Put a **2.2 kΩ, 1 W** resistor
   across M+/M- (about 15 mA, emulating a slave's reply current) and run `rx` again: it
   should read 0. Q3 switches at roughly 7 mA through R15. Don't use a ¼ W part: at
   mark this resistor dissipates about 0.5 W.

Once 1–7 pass, the analogue side is good and anything still broken is UART framing or the
slave itself. Go back to `mbus read`, which logs the raw frame and any UART framing or
parity errors.
