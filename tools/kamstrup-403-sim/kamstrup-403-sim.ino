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
  p[0] = dif;
  p[1] = vif;
  p[2] = (uint8_t)(v);
  p[3] = (uint8_t)(v >> 8);
  p[4] = (uint8_t)(v >> 16);
  p[5] = (uint8_t)(v >> 24);
  return 6;
}

static uint8_t put_i16(uint8_t *p, uint8_t dif, uint8_t vif, int16_t v) {
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
  n += put_u32(r + n, DIF_INT32, VIF_ENERGY_KWH,
               (uint32_t)(meter.energy_wh / 1000.0));
  n += put_u32(r + n, DIF_INT32, VIF_VOLUME_10L,
               (uint32_t)(meter.volume_l / 10.0));
  n += put_u32(r + n, DIF_INT32, VIF_ON_TIME_H,
               (uint32_t)meter.on_time_h);
  n += put_u32(r + n, DIF_INT32, VIF_POWER_100W,
               (uint32_t)lround(meter.power_w / 100.0));
  n += put_u32(r + n, DIF_INT32, VIF_FLOW_LPH,
               (uint32_t)lround(meter.flow_lph));
  n += put_i16(r + n, DIF_INT16, VIF_FLOW_TEMP,
               (int16_t)lround(meter.t_flow_c * 100.0));
  n += put_i16(r + n, DIF_INT16, VIF_RETURN_TEMP,
               (int16_t)lround(meter.t_return_c * 100.0));
  n += put_i16(r + n, DIF_INT16, VIF_TEMP_DIFF,
               (int16_t)lround(meter.delta_k * 100.0));
  return n;
}

/* --------------------------------------------------------------------------
 * Runtime state
 * ------------------------------------------------------------------------ */
static uint8_t mbus_address = MBUS_ADDRESS_DEFAULT;
static uint32_t mbus_baud_rate = MBUS_BAUD_RATE_DEFAULT;
static bool device_selected = false;

static void print_bytes(const uint8_t *bytes, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (bytes[i] < 0x10) DEBUG_SERIAL.print('0');
    DEBUG_SERIAL.print(bytes[i], HEX);
    DEBUG_SERIAL.print(' ');
  }
  DEBUG_SERIAL.println();
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

/* Encode the current readings and answer with a RSP_UD long frame. */
static void send_data_response(uint8_t address) {
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
  if (DEBUG) {
    DEBUG_SERIAL.print(F("tx: "));
    print_bytes(frame, (size_t)len);
    print_values();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  DEBUG_SERIAL.begin(115200);

#if !MBUS_SERIAL_PINS_FIXED
  MBUS_SERIAL.setTX(MBUS_TX_PIN);
  MBUS_SERIAL.setRX(MBUS_RX_PIN);
#endif
  MBUS_SERIAL.begin(mbus_baud_rate, MBUS_SERIAL_CONFIG);
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
      send_data_response(mbus_address);
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
