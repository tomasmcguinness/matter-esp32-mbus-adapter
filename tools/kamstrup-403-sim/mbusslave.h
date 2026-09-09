/*
 * M-Bus slave link layer (EN 13757-2) for the Kamstrup MULTICAL 403 simulator.
 *
 * Structure derived from HWHardsoft/Arduino-MBUS-Meter, which in turn derives
 * from OpenEnergyMonitor's HeatpumpMonitor - GPLv3.
 *   https://github.com/HWHardsoft/Arduino-MBUS-Meter
 */
#ifndef MBUSSLAVE_H
#define MBUSSLAVE_H

#include <Arduino.h>

/* ---------------------------------------------------------------------------
 * Wiring - Raspberry Pi Pico (arduino-pico core)
 *
 *   GP0 (UART0 TX) --> M-Bus slave HAT modulator in
 *   GP1 (UART0 RX) <-- M-Bus slave HAT comparator out
 *   USB            <-> debug console @ 115200
 *
 * The HAT does the level conversion. On the wire, mark/idle is the high bus
 * voltage (~36 V) and a slave answers by modulating current, never by driving
 * voltage - see tools/pcb-test/README.md for the measured master-side numbers.
 * ------------------------------------------------------------------------- */
#define MBUS_TX_PIN 0
#define MBUS_RX_PIN 1

/* Neither RP2040 core gives Serial1 as a HardwareSerial, so the reference
 * sketch's `HardwareSerial *MBusSerial` will not compile on either. Plain
 * aliases sidestep the whole problem. */
#define MBUS_SERIAL  Serial1
#define DEBUG_SERIAL Serial

/* Two Arduino cores can target the Pico, and they differ on pin assignment:
 *
 *   earlephilhower's arduino-pico - Serial1 is a SerialUART whose TX and RX can
 *     be moved to another valid pin pair at runtime via setTX()/setRX().
 *
 *   Arduino's Mbed OS RP2040 core - Serial1 is an arduino::UART whose pins are
 *     fixed when it is constructed; there is no setTX/setRX. Its Serial1 is
 *     already on digital 0/1, which is GP0/GP1, so nothing needs moving and
 *     MBUS_TX_PIN/MBUS_RX_PIN below simply document the wiring.
 *
 * Both honour SERIAL_8E1, which is the part M-Bus actually depends on.
 * If you are on some third core whose Serial1 has no setTX/setRX, define
 * MBUS_SERIAL_PINS_FIXED=1 and wire the meter to that core's UART pins. */
#ifndef MBUS_SERIAL_PINS_FIXED
#if defined(ARDUINO_ARCH_MBED)
#define MBUS_SERIAL_PINS_FIXED 1
#else
#define MBUS_SERIAL_PINS_FIXED 0
#endif
#endif

/* Link settings. The source of truth is the master:
 *   firmware/main/mbus.cpp:61-73 - 2400 baud, 8 data bits, EVEN parity, 1 stop
 *   firmware/main/mbus.h:9       - primary address 0x05
 * A mismatch here is the classic silent failure: the master simply times out
 * having received no bytes at all. */
#define MBUS_BAUD_RATE_DEFAULT 2400
#define MBUS_ADDRESS_DEFAULT   0x05
#define MBUS_SERIAL_CONFIG     SERIAL_8E1

#define MBUS_DATA_SIZE      255
#define MBUS_FRAME_MAX      266
#define MBUS_GAP_MS          50  /* inter-byte gap that abandons a frame.
                                  * One byte at 2400 8E1 takes ~4.6 ms. */
#define MBUS_ECHO_DRAIN_MS    5  /* let the HAT comparator settle after TX */

#define DEBUG 1

/* --------------------------------------------------------------------------
 * C field (control): F = FCB, A = ACD, D = DFC
 *   SND_NKE  init slave           0100 0000  0x40        short frame
 *   SND_UD   send user data       01F1 0011  0x53 / 0x73 long frame
 *   REQ_UD2  request class 2 data 01F1 1011  0x5B / 0x7B short frame
 *   REQ_UD1  request class 1 data 01F1 1010  0x5A / 0x7A short frame
 *   RSP_UD   data from slave      00AD 1000  0x08 ...    long frame
 * Masking with FCB_BIT_MASK ignores the toggling FCB bit. This master never
 * toggles it (it re-sends SND_NKE before every poll instead), but a real
 * master will.
 * ------------------------------------------------------------------------ */
#define MBUS_SND_NKE 0x40
#define MBUS_SND_UD  0x53
#define MBUS_REQ_UD1 0x5A
#define MBUS_REQ_UD2 0x5B
#define MBUS_RSP_UD  0x08
#define FCB_BIT_MASK 0xDF

#define MBUS_FRAME_SHORT_START 0x10
#define MBUS_FRAME_LONG_START  0x68
#define MBUS_FRAME_STOP        0x16
#define MBUS_ACK               0xE5

/* CI field */
#define MBUS_CI_DATA_SEND  0x51 /* master -> slave, data send            */
#define MBUS_CI_SELECT     0x52 /* master -> slave, selection of slaves  */
#define MBUS_CI_RSP_LONG   0x72 /* slave -> master, variable data, 12-byte header */

/* Broadcast addresses: 0xFD (no reply) and 0xFE (reply). 0 is unconfigured. */
#define MBUS_ADDR_BROADCAST_NOREPLY 0xFD
#define MBUS_ADDR_BROADCAST_REPLY   0xFE

/* --------------------------------------------------------------------------
 * Kamstrup MULTICAL 403 identity, as it appears in the 12-byte fixed header.
 * ------------------------------------------------------------------------ */
#define KAM_MANUFACTURER 0x2C2D     /* 'K','A','M' packed per EN 13757-3;
                                     * transmitted little-endian as 2D 2C   */
#define KAM_VERSION      0x34       /* generation/version byte              */
#define KAM_MEDIUM       0x04       /* heat, volume measured at return      */
#define KAM_STATUS       0x00       /* no alarm, no error                   */
#define KAM_SERIAL       12345678UL /* 8 digits, encoded BCD LSB first      */

/* --------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------ */

/* Encode KAM_SERIAL as the four BCD identification bytes, LSB first. */
void mbus_id_bytes(uint8_t out[4]);

/* Read one frame from the bus.
 *   >0  frame length in bytes, checksum verified, stored in pdata
 *    0  nothing waiting - the caller should get on with other work
 *   -1  malformed frame (bad length, checksum, or stop byte) */
int mbus_get_response(uint8_t *pdata, size_t cap);

/* Build a RSP_UD long frame around `records`, returning its length.
 * Increments the access number. Does not transmit. */
int mbus_build_frame(uint8_t c_field, uint8_t address,
                     const uint8_t *records, uint8_t records_len,
                     uint8_t *frame, size_t cap);

void mbus_transmit(const uint8_t *frame, size_t len);
void mbus_send_ack(void);

#endif /* MBUSSLAVE_H */
