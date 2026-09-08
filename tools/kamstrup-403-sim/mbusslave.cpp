/*
 * M-Bus slave link layer (EN 13757-2). See mbusslave.h for wiring and credits.
 */
#include "mbusslave.h"

#include <string.h>

/* Access number, incremented on every response frame so the master can tell
 * a fresh telegram from a repeat. */
static uint8_t access_no = 0;

void mbus_id_bytes(uint8_t out[4]) {
  uint32_t v = KAM_SERIAL;
  for (int i = 0; i < 4; i++) {
    out[i] = (uint8_t)((v % 10) | (((v / 10) % 10) << 4));
    v /= 100;
  }
}

/* Wait up to timeout_ms for one byte. Returns -1 on timeout. */
static int read_byte_timeout(uint32_t timeout_ms) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < timeout_ms) {
    if (MBUS_SERIAL.available()) return MBUS_SERIAL.read();
  }
  return -1;
}

/* After transmitting: wait for the last stop bit to leave the UART, let the
 * HAT's comparator settle, then bin whatever it echoed back. Without this the
 * next mbus_get_response() parses our own reply as a master command. */
static void mbus_tx_done(void) {
  MBUS_SERIAL.flush();
  delay(MBUS_ECHO_DRAIN_MS);
  while (MBUS_SERIAL.available()) MBUS_SERIAL.read();
}

int mbus_get_response(uint8_t *pdata, size_t cap) {
  if (cap < 6) return -1;

  /* Hunt for a start byte. Everything else is bus-turnaround noise. */
  int b;
  do {
    if (!MBUS_SERIAL.available()) return 0;
    b = MBUS_SERIAL.read();
  } while (b != MBUS_ACK &&
           b != MBUS_FRAME_SHORT_START &&
           b != MBUS_FRAME_LONG_START);

  size_t n = 0;
  pdata[n++] = (uint8_t)b;

  size_t expected;
  if (b == MBUS_ACK) {
    return 1; /* single character */
  } else if (b == MBUS_FRAME_SHORT_START) {
    expected = 5; /* 10 C A CS 16 */
  } else {
    /* Long/control frame: 68 L L 68 <L user bytes> CS 16 */
    int len = read_byte_timeout(MBUS_GAP_MS);
    int len2 = read_byte_timeout(MBUS_GAP_MS);
    int start2 = read_byte_timeout(MBUS_GAP_MS);
    if (len < 0 || len2 < 0 || start2 < 0) return -1;
    if (len != len2) return -1;
    if (start2 != MBUS_FRAME_LONG_START) return -1;
    pdata[n++] = (uint8_t)len;
    pdata[n++] = (uint8_t)len2;
    pdata[n++] = (uint8_t)start2;
    expected = 4 + (size_t)len + 2;
  }
  if (expected > cap) return -1;

  while (n < expected) {
    b = read_byte_timeout(MBUS_GAP_MS);
    if (b < 0) return -1; /* partial frame - resync on the next start byte */
    pdata[n++] = (uint8_t)b;
  }

  if (pdata[expected - 1] != MBUS_FRAME_STOP) return -1;

  /* Checksum is the plain 8-bit sum of the user-data bytes only:
   * from C up to (not including) the checksum itself. */
  size_t first = (pdata[0] == MBUS_FRAME_SHORT_START) ? 1 : 4;
  uint8_t cs = 0;
  for (size_t i = first; i < expected - 2; i++) cs += pdata[i];
  if (cs != pdata[expected - 2]) return -1;

  return (int)expected;
}

int mbus_build_frame(uint8_t c_field, uint8_t address,
                     const uint8_t *records, uint8_t records_len,
                     uint8_t *frame, size_t cap) {
  const uint8_t header_len = 3 + 12; /* C A CI + 12-byte fixed header */
  const uint8_t l = (uint8_t)(header_len + records_len);
  const size_t total = 4 + (size_t)l + 2;
  if (total > cap) return -1;

  uint8_t id[4];
  mbus_id_bytes(id);

  size_t i = 0;
  frame[i++] = MBUS_FRAME_LONG_START;
  frame[i++] = l;
  frame[i++] = l;
  frame[i++] = MBUS_FRAME_LONG_START;

  frame[i++] = c_field;
  frame[i++] = address;
  frame[i++] = MBUS_CI_RSP_LONG;

  frame[i++] = id[0]; /* identification number, BCD, LSB first */
  frame[i++] = id[1];
  frame[i++] = id[2];
  frame[i++] = id[3];
  frame[i++] = (uint8_t)(KAM_MANUFACTURER & 0xFF);
  frame[i++] = (uint8_t)(KAM_MANUFACTURER >> 8);
  frame[i++] = KAM_VERSION;
  frame[i++] = KAM_MEDIUM;
  frame[i++] = access_no++;
  frame[i++] = KAM_STATUS;
  frame[i++] = 0x00; /* signature, unused */
  frame[i++] = 0x00;

  memcpy(&frame[i], records, records_len);
  i += records_len;

  uint8_t cs = 0;
  for (size_t c = 4; c < i; c++) cs += frame[c];
  frame[i++] = cs;
  frame[i++] = MBUS_FRAME_STOP;

  return (int)i;
}

void mbus_transmit(const uint8_t *frame, size_t len) {
  MBUS_SERIAL.write(frame, len);
  mbus_tx_done();
}

void mbus_send_ack(void) {
  delay(random(1, 10)); /* small jitter, in case something else is talking */
  MBUS_SERIAL.write((uint8_t)MBUS_ACK);
  mbus_tx_done();
}
