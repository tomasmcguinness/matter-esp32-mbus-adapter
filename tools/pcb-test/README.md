# PCB bring-up test

A tiny standalone ESP-IDF app for testing the M-Bus master circuit on the adapter PCB
*before* trusting UART. It drives GPIO4 (the UART1 TX pin) as a plain GPIO from a serial
console so you can park it at a fixed level and measure M+ / M- with a multimeter, and
reads back GPIO5 (the receiver output).

No Matter, no Thread, no UART driver — just GPIO — so a failure here points squarely at
the hardware.

## Expected voltages

Probe M+ (red) and M- (black), DMM in DC volts:

| GPIO4 (TX) | M+ / M- |
|---|---|
| high (mark / idle) | ~36 V |
| low (space) | ~24 V |

The ~12 V drop is the real result. If the two readings are the same, the modulator isn't
switching. If they're swapped (~24 V idle, ~36 V when low), TX polarity is inverted on
the board — note it, since the real firmware will then need `uart_set_line_inverse()` or
a hardware fix.

## Build and flash

```
cd tools/pcb-test
idf.py set-target esp32c6
idf.py build flash monitor
```

The console appears over the C6's built-in USB Serial/JTAG on the same cable, with a
`pcb>` prompt.

## Commands

| Command | Effect |
|---|---|
| `hi` / `mark` | drive TX high and hold it — expect ~36 V, LED on |
| `lo` / `space` | drive TX low and hold it — expect ~24 V, LED off |
| `sq [ms]` | square wave on TX, default 1000 ms period (50–10000) |
| `stop` | stop the square wave, leave TX idle high |
| `rx` | print GPIO5 level and edge count since boot |
| `status` | TX mode/level plus RX state |
| `help` | list commands |

TX starts high at boot — an M-Bus bus idles at mark, and holding it low looks like a
break condition to some meters.

The LED on GPIO15 mirrors TX: lit = mark (~36 V), dark = space (~24 V), and it blinks
along with `sq`. It gives you a visual confirmation the command landed without looking
away from the multimeter. If your LED is wired active-low (cathode to the pin) it will
read inverted — set `LED_ACTIVE_HIGH` to 0 in `main/main.c`.

## Bring-up order

Stop at the first step that fails.

1. **Supply only.** Power the PCB with the C6 unflashed or held in reset. M+/M- should
   read ~36 V. If not, the boost converter is the fault and no firmware will help.
2. **Flash** this app (above).
3. **Mark.** Type `hi`, probe M+/M-: ~36 V, stable.
4. **Space.** Type `lo`, same points: ~24 V.
5. **Under load.** Repeat 3 and 4 with a 22 kΩ (≥0.1 W) resistor across M+/M- to emulate
   one M-Bus unit load (~1.6 mA at 36 V). The mark voltage should sag only slightly; a
   large sag means the supply can't source bus current, which would later show up as
   unreliable comms.
6. **Toggling.** `sq 1000` and watch the DMM swing, or scope M+ for clean ~12 V steps
   with no ringing or slow edges. `stop` when done.
7. **Receiver path.** Idle bus: `rx` should show a steady level and a static edge count.
   Briefly put a ~1 kΩ resistor across M+/M- (emulating a meter's current-mode reply) and
   run `rx` again — the level should have flipped and/or the edge count moved. That
   confirms the receive comparator reaches GPIO5.

Once 1–7 pass, the analogue side is good and anything still broken is UART framing or the
meter itself — try the main `firmware/` build next (2400 8E1, see
`firmware/main/mbus.cpp`).
