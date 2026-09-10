/*
 * Kamstrup MULTICAL 403 M-Bus slave simulator
 * -------------------------------------------
 * Makes a Raspberry Pi Pico answer M-Bus polls the way a MULTICAL 403 heat
 * meter does, so the ESP32-C6 adapter in firmware/ can be exercised without
 * the real meter on the desk. Identity fields and data records are the ones a
 * 403 actually sends; the values come from a small physical model, so energy,
 * volume, power, flow and the two temperatures stay mutually consistent as
 * they drift - which is the part worth testing the master against.
 *
 * Board:  Raspberry Pi Pico, arduino-pico core (earlephilhower)
 * Wiring: see mbusslave.h
 *
 * Structure derived from HWHardsoft/Arduino-MBUS-Meter (GPLv3, via
 * OpenEnergyMonitor's HeatpumpMonitor), retargeted from its AEG electricity
 * meter payload to a heat meter.
 *   https://github.com/HWHardsoft/Arduino-MBUS-Meter
 */

#include <math.h>

#include "mbusslave.h"

/* --------------------------------------------------------------------------
 * Data record coding (EN 13757-3)
 *
 * DIF gives length and coding, VIF gives unit and decimal exponent. Every
 * code below is understood by BOTH decoders on the master - classify() in
 * firmware/main/mbus_parser.cpp and the bench table kVifTest[] in the same
 * file - so the same telegram works in normal and test parse modes.
 * ------------------------------------------------------------------------ */
#define DIF_INT16 0x02 /* 16-bit signed integer, little-endian */
#define DIF_INT32 0x04 /* 32-bit signed integer, little-endian */

#define VIF_ENERGY_KWH  0x06 /* energy,      10^3 Wh    = kWh     */
#define VIF_VOLUME_10L  0x14 /* volume,      10^-2 m3   = 10 L    */
#define VIF_ON_TIME_H   0x22 /* on time,     hours                */
#define VIF_POWER_100W  0x2D /* power,       10^2 W     = 0.1 kW  */
#define VIF_FLOW_LPH    0x3B /* volume flow, 10^-3 m3/h = l/h     */
#define VIF_FLOW_TEMP   0x59 /* flow temp,   10^-2 degC           */
#define VIF_RETURN_TEMP 0x5D /* return temp, 10^-2 degC           */
#define VIF_TEMP_DIFF   0x61 /* temp diff,   10^-2 K              */

#define RECORDS_MAX 64

/* How many of the eight data records to actually send. Lowering this shortens
 * the telegram without changing anything else about it, which is how you tell
 * a time/energy-limited truncation on the bus from an index-limited one: send
 * fewer records and see whether the master's cut-off point follows the byte
 * count or stays put. Settable at runtime by typing 1..8 in the serial
 * monitor - see loop(). 8 = the full 63-byte MULTICAL 403 telegram. */
#define SIM_RECORDS_ALL 8
static uint8_t sim_records = SIM_RECORDS_ALL;

/* --------------------------------------------------------------------------
 * Simulation
 *
 * Flow rate, temperature difference and return temperature are the
 * independent variables (slow sines on co-prime-ish periods, so the pattern
 * does not look canned). Everything else is derived from them, the way a real
 * meter's readings always are.
 * ------------------------------------------------------------------------ */
#define WATER_KWH_PER_M3_K 1.163 /* specific heat of water, kWh per m3 per K */

#define SIM_TICK_MS 1000

/* Seeds, so the master is not decoding zeros on the very first poll. */
#define SEED_ENERGY_WH 4200000.0 /* 4200 kWh */
#define SEED_VOLUME_L   180000.0 /* 180 m3   */
#define SEED_ON_TIME_H   21500.0 /* ~2.5 y   */

struct MeterState {
  double energy_wh;
  double volume_l;
  double on_time_h;
  double flow_lph;
  double delta_k;
  double t_return_c;
  double t_flow_c;
  double power_w;
};

static MeterState meter;
static double sim_t_s = 0.0;
static uint32_t sim_last_ms = 0;

/* Recompute the derived quantities for the current simulation time. */
static void sim_derive(void) {
  meter.flow_lph   = 700.0 + 250.0 * sin(sim_t_s / 180.0); /* ~450..950 l/h */
  meter.delta_k    =   5.0 +   2.0 * sin(sim_t_s /  97.0); /* ~3..7 K       */
  meter.t_return_c =  38.0 +   3.0 * sin(sim_t_s / 240.0); /* ~35..41 degC  */
  meter.t_flow_c   = meter.t_return_c + meter.delta_k;
  meter.power_w    = (meter.flow_lph / 1000.0) * meter.delta_k *
                     WATER_KWH_PER_M3_K * 1000.0;
}

static void sim_init(void) {
  meter.energy_wh = SEED_ENERGY_WH;
  meter.volume_l  = SEED_VOLUME_L;
  meter.on_time_h = SEED_ON_TIME_H;
  sim_t_s = 0.0;
  sim_last_ms = millis();
  sim_derive();
}

/* Advance the model. Runs off millis(), independent of polling, so the values
 * move whether or not the master is asking. */
static void sim_tick(void) {
  uint32_t now = millis();
  uint32_t elapsed = (uint32_t)(now - sim_last_ms);
  if (elapsed < SIM_TICK_MS) return;

  double dt = (double)elapsed / 1000.0;
  sim_last_ms = now;
  sim_t_s += dt;

  sim_derive();
  meter.energy_wh += meter.power_w * dt / 3600.0;
  meter.volume_l  += meter.flow_lph * dt / 3600.0;
  meter.on_time_h += dt / 3600.0;
}

/* --------------------------------------------------------------------------
 * Record encoding
 * ------------------------------------------------------------------------ */
static uint8_t put_u32(uint8_t *p, uint8_t dif, uint8_t vif, uint32_t v) {
  if (mbus_fill_byte >= 0) v = 0x01010101UL * (uint8_t)mbus_fill_byte;
  p[0] = dif;
  p[1] = vif;
  p[2] = (uint8_t)(v);
  p[3] = (uint8_t)(v >> 8);
  p[4] = (uint8_t)(v >> 16);
  p[5] = (uint8_t)(v >> 24);
  return 6;
}

static uint8_t put_i16(uint8_t *p, uint8_t dif, uint8_t vif, int16_t v) {
  if (mbus_fill_byte >= 0)
    v = (int16_t)(uint16_t)(0x0101U * (uint8_t)mbus_fill_byte);
  p[0] = dif;
  p[1] = vif;
  p[2] = (uint8_t)((uint16_t)v);
  p[3] = (uint8_t)((uint16_t)v >> 8);
  return 4;
}

/* Convert the model to the integer units of the VIFs above. Nothing else in
 * the sketch knows about scaling, so swapping in fixed values is a one-
 * function edit. */
static uint8_t encode_records(uint8_t *r) {
  uint8_t n = 0;
  uint8_t k = 0; /* records emitted so far */

  if (k++ < sim_records)
    n += put_u32(r + n, DIF_INT32, VIF_ENERGY_KWH,
                 (uint32_t)(meter.energy_wh / 1000.0));
  if (k++ < sim_records)
    n += put_u32(r + n, DIF_INT32, VIF_VOLUME_10L,
                 (uint32_t)(meter.volume_l / 10.0));
  if (k++ < sim_records)
    n += put_u32(r + n, DIF_INT32, VIF_ON_TIME_H,
                 (uint32_t)meter.on_time_h);
  if (k++ < sim_records)
    n += put_u32(r + n, DIF_INT32, VIF_POWER_100W,
                 (uint32_t)lround(meter.power_w / 100.0));
  if (k++ < sim_records)
    n += put_u32(r + n, DIF_INT32, VIF_FLOW_LPH,
                 (uint32_t)lround(meter.flow_lph));
  if (k++ < sim_records)
    n += put_i16(r + n, DIF_INT16, VIF_FLOW_TEMP,
                 (int16_t)lround(meter.t_flow_c * 100.0));
  if (k++ < sim_records)
    n += put_i16(r + n, DIF_INT16, VIF_RETURN_TEMP,
                 (int16_t)lround(meter.t_return_c * 100.0));
  if (k++ < sim_records)
    n += put_i16(r + n, DIF_INT16, VIF_TEMP_DIFF,
                 (int16_t)lround(meter.delta_k * 100.0));
  return n;
}

/* Serial-console knobs, applied between polls so a bus fault can be bisected
 * without reflashing:
 *   1..8  how many data records to send  (varies frame LENGTH)
 *   fHH   fill values with the byte 0xHH (varies BIT PATTERN, same length)
 *   p     shorthand for f55
 *   r     back to real meter values
 *   s     park the line: off -> SPACE -> MARK -> off. Hands GP0 to plain GPIO
 *         so the bus sits under a steady load and a DMM can read it. The
 *         difference between the two readings is the slave's modulation
 *         current - see mbus_set_hold().
 *   t     transmit telegrams back-to-back until pressed again, for reading the
 *         bus under realistic traffic rather than a held level.
 *   g     step the inter-byte gap up: 0 -> 250 -> 500 -> 1000 -> 2000 -> 5000
 *         -> 10000 us -> 0. Varies how long the bus is left idling at mark
 *         between characters. 0 is what a real meter does; the stop bit alone
 *         gives the master 417 us at 2400 baud, and 10000 us is what the
 *         HWHardsoft reference sketch does.
 * Length and pattern are independent, which is the whole point - see
 * mbus_fill_byte in mbusslave.h. */
/* --------------------------------------------------------------------------
 * Runtime state
 * ------------------------------------------------------------------------ */
static uint8_t mbus_address = MBUS_ADDRESS_DEFAULT;
static uint32_t mbus_baud_rate = MBUS_BAUD_RATE_DEFAULT;
static bool device_selected = false;

/* Bring the M-Bus UART up. Also the restore path out of a parked line, which
 * is why it is a function rather than inline in setup(). */
static void mbus_serial_begin(void) {
#if !MBUS_SERIAL_PINS_FIXED
  MBUS_SERIAL.setTX(MBUS_TX_PIN);
  MBUS_SERIAL.setRX(MBUS_RX_PIN);
#endif
#if MBUS_SERIAL_HAS_FIFO_SIZE
  /* The whole echo arrives while write() is still blocking, so the RX buffer
   * has to hold a full telegram or the drain in mbus_tx_done() reports an
   * overflow as bus corruption. Must precede begin(). */
  MBUS_SERIAL.setFIFOSize(MBUS_FRAME_MAX + 16);
#endif
  MBUS_SERIAL.begin(mbus_baud_rate, MBUS_SERIAL_CONFIG);
}

/* --------------------------------------------------------------------------
 * Bench: parking the line, and transmitting without pause
 *
 * Both exist to put the bus under a load steady enough for a multimeter. One
 * telegram is ~289 ms at 2400 baud with 8 records, which no DMM can follow.
 * ------------------------------------------------------------------------ */
typedef enum { HOLD_OFF = 0, HOLD_SPACE, HOLD_MARK } hold_state_t;
static hold_state_t mbus_hold = HOLD_OFF;
static bool sim_continuous = false;

/* Park the line, WITHOUT taking GP0 away from the UART.
 *
 * The first version of this called MBUS_SERIAL.end() and drove GP0 with
 * pinMode/digitalWrite. Releasing it did not reliably hand the pad back, so the
 * sim went silent after any use of 's' - still logging "tx:" for telegrams that
 * never physically left - until the Pico was power-cycled. Do not reintroduce
 * that: SPACE is now a continuous stream of 0x00 characters instead, which at
 * 8E1 is ten space bit-times out of eleven (~91% space, against 100% for a true
 * hold). A DMM cannot tell the difference that matters, and the UART never
 * loses ownership of the pin.
 *
 * Mark is the idle level and costs the slave almost nothing; space is the slave
 * sinking its transmit current, and holding it is the worst case any telegram
 * can present. Reading M+/M- in both states gives the modulation current
 * without having to catch anything in flight: with the master's source
 * impedance R (open-circuit volts minus loaded volts, over the load current),
 *
 *     I_modulation = (V_mark - V_space) / R
 *
 * which is the 11-20 mA EN 13757-2 asks a slave to draw. If the HAT inverts,
 * the labels swap and nothing else changes - the lower reading is the space
 * state either way, and the difference is what matters. */
/* Takes no argument on purpose. The Arduino builder generates prototypes for
 * every function in a .ino and injects them ABOVE the file's own typedefs, so a
 * signature naming hold_state_t fails to compile there while building fine with
 * a plain C++ compiler. Cycling internally keeps the type out of the signature.
 * Same applies to any future helper here: pass built-in types only. */
static void mbus_hold_cycle(void) {
  mbus_hold = (mbus_hold == HOLD_OFF)   ? HOLD_SPACE
            : (mbus_hold == HOLD_SPACE) ? HOLD_MARK
                                        : HOLD_OFF;
  if (mbus_hold == HOLD_SPACE) {
    DEBUG_SERIAL.println(F("hold -> SPACE (streaming 0x00, ~91% space). Measure "
                           "M+/M- now:"));
    DEBUG_SERIAL.println(F("        the slave is sinking its transmit current. "
                           "'s' again for MARK."));
  } else if (mbus_hold == HOLD_MARK) {
    DEBUG_SERIAL.println(F("hold -> MARK (line idle). Measure M+/M- again. "
                           "(V_mark - V_space) / R"));
    DEBUG_SERIAL.println(F("        is the modulation current; EN 13757-2 asks "
                           "11-20 mA. 's' to release."));
  } else {
    DEBUG_SERIAL.println(F("hold -> off, answering requests again"));
  }
}

static int hex_nibble(int c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

/* Space bit-times in one 8E1 character, start and parity bits included. This
 * is how long the slave spends sinking current for that byte, and it is the
 * variable the master's corruption tracks - see mbus_fill_byte in
 * mbusslave.h. Even parity means popcount decides it outright. */
static uint8_t char_space_bits(uint8_t b) {
  uint8_t pc = 0;
  for (uint8_t i = 0; i < 8; i++) pc = (uint8_t)(pc + ((b >> i) & 1));
  return (uint8_t)(10 - pc - (pc & 1));
}

/* Longest unbroken run of space bits in the same character. Reported next to
 * the space count because the two come apart: 0x11 and 0x03 carry identical
 * charge over runs of 4 and 7. */
static uint8_t char_max_space_run(uint8_t b) {
  uint8_t pc = 0;
  for (uint8_t i = 0; i < 8; i++) pc = (uint8_t)(pc + ((b >> i) & 1));
  /* Transmission order, LSB first: start, d0..d7, even parity, stop. */
  uint16_t bits = (uint16_t)((uint16_t)b << 1);
  bits = (uint16_t)(bits | ((uint16_t)(pc & 1) << 9) | ((uint16_t)1 << 10));
  uint8_t run = 0, best = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if ((bits >> i) & 1) run = 0;
    else if (++run > best) best = run;
  }
  return best;
}

static void report_fill(void) {
  DEBUG_SERIAL.print(F("fill -> "));
  if (mbus_fill_byte < 0) {
    DEBUG_SERIAL.println(F("real meter values (zero-heavy)"));
    return;
  }
  uint8_t b = (uint8_t)mbus_fill_byte;
  DEBUG_SERIAL.print(F("0x"));
  if (b < 0x10) DEBUG_SERIAL.print('0');
  DEBUG_SERIAL.print(b, HEX);
  DEBUG_SERIAL.print(F(" ("));
  DEBUG_SERIAL.print(char_space_bits(b));
  DEBUG_SERIAL.print(F(" space bits of 11, longest run "));
  DEBUG_SERIAL.print(char_max_space_run(b));
  DEBUG_SERIAL.println(F(")"));
}

static void poll_console(void) {
  /* 'f' takes two hex digits. The partial value is kept across calls so the
   * console never blocks waiting for the rest of them. */
  static uint8_t fill_digits = 0;
  static uint8_t fill_acc = 0;

  while (DEBUG_SERIAL.available()) {
    int c = DEBUG_SERIAL.read();

    if (fill_digits) {
      int nib = hex_nibble(c);
      if (nib < 0) {
        fill_digits = 0;
        DEBUG_SERIAL.println(F("fill: expected two hex digits - cancelled"));
        continue;
      }
      fill_acc = (uint8_t)((fill_acc << 4) | (uint8_t)nib);
      if (--fill_digits == 0) {
        mbus_fill_byte = (int16_t)fill_acc;
        report_fill();
      }
      continue;
    }

    if (c == 'f') {
      fill_digits = 2;
      fill_acc = 0;
      continue;
    }
    if (c == 's') {
      mbus_hold_cycle();
      continue;
    }
    if (c == 't') {
      sim_continuous = !sim_continuous;
      if (sim_continuous) {
        DEBUG_SERIAL.println(F("continuous TX on - telegrams back-to-back, "
                               "per-telegram logging off."));
        DEBUG_SERIAL.println(F("        Measure M+/M- for the bus under real "
                               "traffic. 't' to stop."));
      } else {
        DEBUG_SERIAL.println(F("continuous TX off"));
      }
      continue;
    }
    if (c == 'g') {
      static const uint16_t steps[] = {0, 250, 500, 1000, 2000, 5000, 10000};
      uint8_t n = sizeof(steps) / sizeof(steps[0]);
      uint8_t i = 0;
      while (i < n && steps[i] != mbus_tx_gap_us) i++;
      mbus_tx_gap_us = steps[(i + 1) % n];
      DEBUG_SERIAL.print(F("inter-byte gap -> "));
      DEBUG_SERIAL.print(mbus_tx_gap_us);
      DEBUG_SERIAL.print(F(" us; master gets "));
      /* The stop bit is one bit time of mark and always precedes the gap. */
      DEBUG_SERIAL.print(mbus_tx_gap_us + 1000000.0 / MBUS_BAUD_RATE_DEFAULT, 0);
      DEBUG_SERIAL.print(F(" us of mark between characters ("));
      DEBUG_SERIAL.print(mbus_tx_gap_us ? F("padded") : F("back-to-back, like a real meter"));
      DEBUG_SERIAL.println(F(")"));
      continue;
    }
    if (c == 'p' || c == 'r') {
      mbus_fill_byte = (c == 'p') ? 0x55 : -1;
      report_fill();
      continue;
    }
    if (c < '1' || c > '0' + SIM_RECORDS_ALL) continue;
    sim_records = (uint8_t)(c - '0');
    /* L = C A CI + 12-byte header + records; frame = 4 + L + 2. */
    uint8_t records[RECORDS_MAX];
    uint8_t rl = encode_records(records);
    DEBUG_SERIAL.print(F("records -> "));
    DEBUG_SERIAL.print(sim_records);
    DEBUG_SERIAL.print(F(", telegram now "));
    DEBUG_SERIAL.print(4 + (3 + 12 + rl) + 2);
    DEBUG_SERIAL.print(F(" bytes ("));
    DEBUG_SERIAL.print((4 + (3 + 12 + rl) + 2) * 11.0 / MBUS_BAUD_RATE_DEFAULT * 1000.0, 0);
    DEBUG_SERIAL.println(F(" ms on the wire)"));
  }
}

static void print_bytes(const uint8_t *bytes, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (bytes[i] < 0x10) DEBUG_SERIAL.print('0');
    DEBUG_SERIAL.print(bytes[i], HEX);
    DEBUG_SERIAL.print(' ');
  }
  DEBUG_SERIAL.println();
}

/* Compare the HAT's loopback against what we asked it to transmit - the slave
 * half of the bus, measured on its own with no instruments. See mbus_echo in
 * mbusslave.h for what each outcome localises the fault to. The echo line is
 * printed in the same hex format the master logs, so it can go straight into
 * tools/mbus-align/mbus_align.py. */
static void mbus_report_echo(const uint8_t *sent, size_t len) {
  if (mbus_echo_len == 0) {
    DEBUG_SERIAL.println(F("echo: none - this HAT does not loop back, "
                           "so the slave half cannot be tested this way"));
    return;
  }

  size_t common = (mbus_echo_len < len) ? mbus_echo_len : len;
  size_t differ = 0;
  int first = -1;
  for (size_t i = 0; i < common; i++) {
    if (mbus_echo[i] != sent[i]) {
      if (first < 0) first = (int)i;
      differ++;
    }
  }

  DEBUG_SERIAL.print(F("echo: "));
  DEBUG_SERIAL.print(mbus_echo_len);
  DEBUG_SERIAL.print(F(" of "));
  DEBUG_SERIAL.print(len);
  DEBUG_SERIAL.print(F(" bytes"));
  if (mbus_echo_len == len && differ == 0) {
    DEBUG_SERIAL.println(F(" - CLEAN, the slave end put the telegram on the "
                           "bus intact"));
    return;
  }
  DEBUG_SERIAL.print(F(", "));
  DEBUG_SERIAL.print(differ);
  DEBUG_SERIAL.print(F(" differ"));
  if (first >= 0) {
    DEBUG_SERIAL.print(F(", first at index "));
    DEBUG_SERIAL.print(first);
    DEBUG_SERIAL.print(F(" (sent "));
    if (sent[first] < 0x10) DEBUG_SERIAL.print('0');
    DEBUG_SERIAL.print(sent[first], HEX);
    DEBUG_SERIAL.print(F(", echo "));
    if (mbus_echo[first] < 0x10) DEBUG_SERIAL.print('0');
    DEBUG_SERIAL.print(mbus_echo[first], HEX);
    DEBUG_SERIAL.print(')');
  }
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print(F("echo: "));
  print_bytes(mbus_echo, mbus_echo_len);
}

static void print_values(void) {
  DEBUG_SERIAL.print(F("  energy "));
  DEBUG_SERIAL.print(meter.energy_wh / 1000.0, 1);
  DEBUG_SERIAL.print(F(" kWh, volume "));
  DEBUG_SERIAL.print(meter.volume_l / 1000.0, 2);
  DEBUG_SERIAL.print(F(" m3, power "));
  DEBUG_SERIAL.print(meter.power_w / 1000.0, 2);
  DEBUG_SERIAL.print(F(" kW, flow "));
  DEBUG_SERIAL.print(meter.flow_lph, 0);
  DEBUG_SERIAL.print(F(" l/h, T1 "));
  DEBUG_SERIAL.print(meter.t_flow_c, 2);
  DEBUG_SERIAL.print(F(" C, T2 "));
  DEBUG_SERIAL.print(meter.t_return_c, 2);
  DEBUG_SERIAL.print(F(" C, dT "));
  DEBUG_SERIAL.print(meter.delta_k, 2);
  DEBUG_SERIAL.println(F(" K"));
}

/* Encode the current readings and answer with a RSP_UD long frame. Quiet when
 * `verbose` is false, which is how continuous transmit avoids drowning the
 * console in ~200 telegrams a minute. */
static void send_data_response(uint8_t address, bool verbose) {
  uint8_t records[RECORDS_MAX];
  uint8_t frame[MBUS_FRAME_MAX];

  uint8_t records_len = encode_records(records);
  int len = mbus_build_frame(MBUS_RSP_UD, address, records, records_len,
                             frame, sizeof(frame));
  if (len < 0) {
    if (DEBUG) DEBUG_SERIAL.println(F("mbus: response too large"));
    return;
  }
  mbus_transmit(frame, (size_t)len);

  digitalWrite(LED_BUILTIN, HIGH);
  if (DEBUG && verbose) {
    DEBUG_SERIAL.print(F("tx: "));
    print_bytes(frame, (size_t)len);
    mbus_report_echo(frame, (size_t)len);
    print_values();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  DEBUG_SERIAL.begin(115200);

  mbus_serial_begin();
  delay(1000); /* let the UART settle, or the first frame is garbage */

  sim_init();

  uint8_t id[4];
  mbus_id_bytes(id);

  DEBUG_SERIAL.println(F("Kamstrup MULTICAL 403 M-Bus slave simulator"));
  DEBUG_SERIAL.print(F("primary address: 0x"));
  DEBUG_SERIAL.println(mbus_address, HEX);
  DEBUG_SERIAL.print(F("baud rate: "));
  DEBUG_SERIAL.println(mbus_baud_rate);
#if MBUS_SERIAL_PINS_FIXED
  DEBUG_SERIAL.println(F("uart pins: core default for Serial1 (GP0 TX, GP1 RX)"));
#else
  DEBUG_SERIAL.print(F("uart pins: GP"));
  DEBUG_SERIAL.print(MBUS_TX_PIN);
  DEBUG_SERIAL.print(F(" TX, GP"));
  DEBUG_SERIAL.print(MBUS_RX_PIN);
  DEBUG_SERIAL.println(F(" RX"));
#endif
  DEBUG_SERIAL.print(F("serial (secondary address): "));
  DEBUG_SERIAL.println(KAM_SERIAL);
  DEBUG_SERIAL.print(F("id bytes: "));
  print_bytes(id, sizeof(id));

  /* Golden frame: the telegram as it stands at t = 0, for checking the length
   * and checksum by hand and for pasting into an offline decoder. It carries
   * access number 0x00; the first real response will carry 0x01. */
  uint8_t records[RECORDS_MAX];
  uint8_t frame[MBUS_FRAME_MAX];
  uint8_t records_len = encode_records(records);
  int len = mbus_build_frame(MBUS_RSP_UD, mbus_address, records, records_len,
                             frame, sizeof(frame));
  DEBUG_SERIAL.print(F("golden frame (t=0): "));
  print_bytes(frame, (size_t)len);
  print_values();
}

void loop() {
  sim_tick();
  poll_console();

  /* Line parked for a meter reading. MARK needs nothing done - an idle UART
   * already holds the line at mark. SPACE streams zeros; flush() so the bytes
   * are actually on the wire before the next console check, and bin the echo so
   * mbus_get_response() does not later parse it as a master command. */
  if (mbus_hold == HOLD_SPACE) {
    static const uint8_t zeros[16] = {0};
    MBUS_SERIAL.write(zeros, sizeof(zeros));
    MBUS_SERIAL.flush();
    while (MBUS_SERIAL.available()) MBUS_SERIAL.read();
    return;
  }
  if (mbus_hold == HOLD_MARK) return;

  /* Continuous transmit: ignore the bus and keep the line busy, so the load a
   * DMM sees is the one a real telegram presents rather than a held level. */
  if (sim_continuous) {
    send_data_response(mbus_address, false);
    return;
  }

  uint8_t rx[MBUS_DATA_SIZE];
  int n = mbus_get_response(rx, sizeof(rx));

  if (n == 0) return; /* bus idle */
  if (n < 0) {
    if (DEBUG) DEBUG_SERIAL.println(F("mbus: bad frame"));
    return;
  }
  if (n == 1) return; /* single-character ACK from the master, nothing to do */

  if (DEBUG) {
    DEBUG_SERIAL.print(F("rx: "));
    print_bytes(rx, (size_t)n);
  }

  /* ---- short frames: 10 C A CS 16 ---------------------------------------
   * This master re-sends SND_NKE before every REQ_UD2 and never toggles the
   * FCB bit (see mbus_request_data() in firmware/main/mbus.cpp), so nothing
   * below may depend on FCB state. */
  if (n == 5) {
    uint8_t c = rx[1];
    uint8_t a = rx[2];
    bool for_us = (a == mbus_address);
    bool broadcast = (a == MBUS_ADDR_BROADCAST_NOREPLY ||
                      a == MBUS_ADDR_BROADCAST_REPLY);

    if (!for_us && !broadcast) return;

    if (c == MBUS_SND_NKE) {
      if (DEBUG) DEBUG_SERIAL.println(F("  SND_NKE - init slave"));
      device_selected = false;
      mbus_send_ack();
      return;
    }

    if ((c & FCB_BIT_MASK) == MBUS_REQ_UD2) {
      /* On a broadcast address, answer only if we were selected by secondary
       * address - otherwise every slave on the bus would talk at once. */
      if (broadcast && !device_selected) return;
      if (DEBUG) DEBUG_SERIAL.println(F("  REQ_UD2 - class 2 data request"));
      send_data_response(mbus_address, true);
      return;
    }

    if ((c & FCB_BIT_MASK) == MBUS_REQ_UD1) {
      /* No class 1 (alarm) data to report. */
      if (DEBUG) DEBUG_SERIAL.println(F("  REQ_UD1 - no class 1 data"));
      mbus_send_ack();
      return;
    }
    return;
  }

  /* ---- secondary address selection: 68 0B 0B 68 53 FD 52 <id><mfr><ver><med> CS 16 */
  if (n == 17 && (rx[4] & FCB_BIT_MASK) == MBUS_SND_UD &&
      rx[5] == MBUS_ADDR_BROADCAST_NOREPLY && rx[6] == MBUS_CI_SELECT) {
    uint8_t id[4];
    mbus_id_bytes(id);
    /* A master may wildcard ID digits with 0xF. Only whole-byte wildcards
     * (0xFF) are honoured here - per-nibble wildcards would need a digit-wise
     * compare, and no master on this bench uses them. */
    bool match = true;
    for (int i = 0; i < 4; i++) {
      uint8_t want = rx[7 + i];
      if (want != id[i] && want != 0xFF) match = false;
    }
    device_selected = match;
    if (DEBUG) {
      DEBUG_SERIAL.print(F("  secondary select - "));
      DEBUG_SERIAL.println(match ? F("selected") : F("not ours"));
    }
    if (match) mbus_send_ack();
    return;
  }

  /* ---- change primary address: 68 06 06 68 53 A 51 01 7A <new> CS 16 ---- */
  if (n == 12 && (rx[4] & FCB_BIT_MASK) == MBUS_SND_UD &&
      rx[5] == mbus_address && rx[6] == MBUS_CI_DATA_SEND &&
      rx[7] == 0x01 && rx[8] == 0x7A) {
    mbus_address = rx[9];
    if (DEBUG) {
      DEBUG_SERIAL.print(F("  change address -> 0x"));
      DEBUG_SERIAL.println(mbus_address, HEX);
    }
    mbus_send_ack();
    return;
  }

  /* ---- change baud rate: 68 03 03 68 53 A <CI B8..BD> CS 16 -------------
   * The baud rate lives in the CI field here, not in a data record. */
  if (n == 9 && (rx[4] & FCB_BIT_MASK) == MBUS_SND_UD &&
      rx[5] == mbus_address && rx[6] >= 0xB8 && rx[6] <= 0xBD) {
    static const uint32_t rates[] = {300, 600, 1200, 2400, 4800, 9600};
    mbus_baud_rate = rates[rx[6] - 0xB8];
    if (DEBUG) {
      DEBUG_SERIAL.print(F("  change baud rate -> "));
      DEBUG_SERIAL.println(mbus_baud_rate);
    }
    mbus_send_ack(); /* acknowledge at the OLD rate, then switch */
    delay(100);
    MBUS_SERIAL.end();
    MBUS_SERIAL.begin(mbus_baud_rate, MBUS_SERIAL_CONFIG);
    return;
  }

  if (DEBUG) DEBUG_SERIAL.println(F("  unhandled frame"));
}
