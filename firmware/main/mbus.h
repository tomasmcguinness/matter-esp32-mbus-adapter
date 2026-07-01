#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

// Primary (bus) address of the meter to poll.
// 0x01..0xFA = primary addressing; 0xFE = broadcast (single meter on the bus).
#define MBUS_PRIMARY_ADDRESS 0x01

// Initialise the UART used to talk to the DYBKRADIO M-Bus <-> TTL adapter.
void mbus_uart_init(void);

// Send REQ_UD2 to `primary_addr` and read the RSP_UD long frame into `buf`.
// On success `*out_len` receives the number of user-data bytes (CI + data,
// i.e. the payload between the second 0x68 and the checksum), and ESP_OK is
// returned. The user-data bytes are copied to the start of `buf`.
esp_err_t mbus_request_data(uint8_t primary_addr, uint8_t *buf, size_t buflen, size_t *out_len);
