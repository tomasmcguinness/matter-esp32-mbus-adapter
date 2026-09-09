#include "mbus.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

// --- Wiring / UART configuration -------------------------------------------
// The DYBKRADIO adapter does the M-Bus <-> TTL level conversion, so the ESP32
// just drives a plain UART. M-Bus uses 2400 baud, 8 data bits, EVEN parity,
// 1 stop bit. Pins below avoid the C6 strapping/USB-JTAG pins.
#define MBUS_UART   UART_NUM_1
#define MBUS_TXD_PIN ((gpio_num_t)23)  // ESP TX -> adapter RX
#define MBUS_RXD_PIN ((gpio_num_t)22)  // ESP RX <- adapter TX

// M-Bus frame markers / control fields (EN 13757-2)
#define MBUS_START_SHORT 0x10
#define MBUS_START_LONG  0x68
#define MBUS_STOP        0x16
#define MBUS_C_SND_NKE   0x40  // link reset
#define MBUS_C_REQ_UD2   0x5B  // request user data (class 2), FCB=0
#define MBUS_ACK         0xE5  // single-byte acknowledge

#define MBUS_RX_BUF_SIZE 512

// Timeouts. uart_read_bytes() does NOT decrement its ticks_to_wait between
// ring-buffer chunks (esp_driver_uart/src/uart.c), so one call bounds the gap
// between chunks, never the whole read. Treat it as a character-gap timer and
// do the framing here.
#define MBUS_RESP_TIMEOUT_MS 1500  // silence after REQ_UD2 before we give up
#define MBUS_CHAR_TIMEOUT_MS  250  // gap between characters *within* a frame.
                                   // One byte at 2400 8E1 is 4.6 ms and EN
                                   // 13757-2 allows 11 bit times between
                                   // characters, so this is ~50x generous -
                                   // but short enough that a slave that dies
                                   // mid-telegram is reported as a stall
                                   // rather than as a mystery short read.

// Hex-dump the whole RX burst before any framing checks. Without this every
// early return below (bad L/L, missing stop byte, checksum mismatch) throws
// away the only evidence of what the meter actually sent. Set to 0 in
// production.
#define MBUS_LOG_RAW_RX 1

static const char *TAG = "MBus";

static uint8_t mbus_checksum(const uint8_t *data, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) {
        cs = (uint8_t)(cs + data[i]);
    }
    return cs;
}

// Read exactly `want` bytes, tolerating the burst arriving in chunks.
// Returns the number actually read; < want means `timeout_ms` elapsed with the
// bus idle.
static int mbus_read_exact(uint8_t *dst, int want, int timeout_ms)
{
    int got = 0;
    while (got < want) {
        int n = uart_read_bytes(MBUS_UART, dst + got, want - got, pdMS_TO_TICKS(timeout_ms));
        if (n <= 0) {
            break;
        }
        got += n;
    }
    return got;
}

static void mbus_send_short_frame(uint8_t c, uint8_t addr)
{
    uint8_t frame[5];
    frame[0] = MBUS_START_SHORT;
    frame[1] = c;
    frame[2] = addr;
    frame[3] = (uint8_t)(c + addr); // checksum over C + A
    frame[4] = MBUS_STOP;

    uart_write_bytes(MBUS_UART, (const char *)frame, sizeof(frame));
    uart_wait_tx_done(MBUS_UART, pdMS_TO_TICKS(100));
}

void mbus_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_param_config(MBUS_UART, &uart_config);
    uart_set_pin(MBUS_UART, MBUS_TXD_PIN, MBUS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MBUS_UART, MBUS_RX_BUF_SIZE, 0, 0, NULL, 0);
}

esp_err_t mbus_send_nke(uint8_t primary_addr)
{
    uart_flush_input(MBUS_UART);

    ESP_LOGI(TAG, "TX SND_NKE: 10 %02X %02X %02X 16", MBUS_C_SND_NKE, primary_addr,
             (uint8_t)(MBUS_C_SND_NKE + primary_addr));
    mbus_send_short_frame(MBUS_C_SND_NKE, primary_addr);

    // A slave that accepts the reset answers with a single 0xE5 within the
    // 11-330 bit-time window; 1 s is generous at 2400 baud.
    uint8_t rx[16];
    int len = uart_read_bytes(MBUS_UART, rx, sizeof(rx), pdMS_TO_TICKS(1000));
    if (len <= 0) {
        ESP_LOGW(TAG, "No ACK from meter at 0x%02X", primary_addr);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx, len, ESP_LOG_INFO);

    for (int i = 0; i < len; i++) {
        if (rx[i] == MBUS_ACK) {
            ESP_LOGI(TAG, "ACK (0xE5) received from meter at 0x%02X", primary_addr);
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Got %d byte(s) but no ACK (0xE5)", len);
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t mbus_request_data(uint8_t primary_addr, uint8_t *buf, size_t buflen, size_t *out_len)
{
    if (buf == NULL || out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Reset the slave's frame-count bit and clear any stale RX bytes.
    uart_flush_input(MBUS_UART);
    mbus_send_short_frame(MBUS_C_SND_NKE, primary_addr);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_flush_input(MBUS_UART);

    // Ask for user data (class 2).
    mbus_send_short_frame(MBUS_C_REQ_UD2, primary_addr);

    // Hunt for "68 L L 68", one byte at a time. Leading bytes are bus
    // turnaround noise or an echo of our own request. A bare 0x68 is not a
    // reliable marker on its own - the 403 telegram carries 0x68 inside the
    // energy record (04 06 68 10 00 00) - so a failed header check has to
    // resync rather than give up, or a lost frame start is reported as a
    // corrupt one.
    static uint8_t rx[MBUS_RX_BUF_SIZE];
    int timeout_ms = MBUS_RESP_TIMEOUT_MS;
    int skipped = 0;
    uint8_t l1 = 0;
    while (true) {
        if (mbus_read_exact(&rx[0], 1, timeout_ms) != 1) {
            if (skipped == 0) {
                ESP_LOGW(TAG, "No response from meter at 0x%02X", primary_addr);
                return ESP_ERR_TIMEOUT;
            }
            ESP_LOGW(TAG, "%d byte(s) before timeout, never saw a long-frame header", skipped);
            return ESP_ERR_INVALID_RESPONSE;
        }
        // Once anything is coming in, the rest of the burst is close behind.
        timeout_ms = MBUS_CHAR_TIMEOUT_MS;

        if (rx[0] != MBUS_START_LONG) {
            if (rx[0] == MBUS_ACK && skipped == 0) {
                ESP_LOGW(TAG, "Meter ACKed REQ_UD2 - no class 2 data");
                return ESP_ERR_INVALID_RESPONSE;
            }
            ESP_LOGD(TAG, "skipping 0x%02X before frame start", rx[0]);
            skipped++;
            continue;
        }

        // Long frame layout: 68 L L 68 [C A CI ...data...] CS 16
        if (mbus_read_exact(&rx[1], 3, MBUS_CHAR_TIMEOUT_MS) != 3) {
            ESP_LOGW(TAG, "Frame stalled in header (after 0x68)");
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (rx[1] == rx[2] && rx[3] == MBUS_START_LONG) {
            l1 = rx[1];
            break;
        }
        ESP_LOGD(TAG, "not a header: 68 %02X %02X %02X - resyncing", rx[1], rx[2], rx[3]);
        skipped += 4;
    }

    // Remaining bytes for this frame: L user bytes + CS + STOP.
    int rest = (int)l1 + 2;
    int got = mbus_read_exact(&rx[4], rest, MBUS_CHAR_TIMEOUT_MS);
    int frame_len = 4 + rest;

#if MBUS_LOG_RAW_RX
    ESP_LOGI(TAG, "RX frame, %d of %d byte(s):", 4 + got, frame_len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, rx, 4 + got, ESP_LOG_INFO);
#endif

    if (got != rest) {
        // The bus went quiet part-way through the telegram. That is a
        // transmitter/bus problem, not a framing one: the master waited
        // MBUS_CHAR_TIMEOUT_MS with nothing arriving, where the remaining
        // bytes were due within a few ms of each other.
        ESP_LOGW(TAG, "Frame stalled: %d of %d bytes, then %d ms idle",
                 4 + got, frame_len, MBUS_CHAR_TIMEOUT_MS);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t *user = &rx[4];          // C A CI data...
    uint8_t rx_cs = rx[4 + l1];
    uint8_t rx_stop = rx[4 + l1 + 1];

    if (rx_stop != MBUS_STOP) {
        ESP_LOGW(TAG, "Missing stop byte (got 0x%02X)", rx_stop);
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t calc_cs = mbus_checksum(user, l1);
    if (calc_cs != rx_cs) {
        ESP_LOGW(TAG, "Checksum mismatch: got 0x%02X, calc 0x%02X", rx_cs, calc_cs);
        return ESP_ERR_INVALID_CRC;
    }

    if ((size_t)l1 > buflen) {
        ESP_LOGE(TAG, "User data (%u) exceeds buffer (%u)", l1, (unsigned)buflen);
        return ESP_ERR_NO_MEM;
    }

    memcpy(buf, user, l1);
    *out_len = l1;
    return ESP_OK;
}
