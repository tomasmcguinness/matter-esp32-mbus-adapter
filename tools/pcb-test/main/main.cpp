// PCB bring-up test for the M-Bus adapter.
//
// Two levels of test from one serial console:
//
//   Analogue  - TX is driven as a plain GPIO, with no UART involved, so it can
//               be parked at a fixed level and M+ / M- measured with a
//               multimeter. This proves the modulator before any framing.
//   Link      - TX is handed to UART1 and the M-Bus link layer from the
//               KamstrupMultical403 example talks to a real slave (the bench
//               slave HAT, a simulator or a meter). This proves the board end
//               to end at the application level.
//
// The pin is switched between the two on demand: GPIO commands take it back
// from the UART, M-Bus commands give it to the UART.
//
// Expected across M+ (red probe) / M- (black probe), DC volts:
//   TX high (mark  / idle)  ->  ~34 V
//   TX low  (space)         ->  ~22 V   (a ~12.6 V drop)

#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mbus.h"
#include "mbus_parser.h"

// Same pins as the production firmware: see
// examples/matter/KamstrupMultical403/main/mbus.cpp, which is the source of
// truth for the wiring.
#define TX_PIN ((gpio_num_t)23) // ESP TX -> M-Bus modulator (R16 -> Q2)
#define RX_PIN ((gpio_num_t)22) // ESP RX <- M-Bus receiver (Q3), input only

// The UART mbus.cpp installs its driver on. It keeps its own copy private, so
// this has to match MBUS_UART there.
#define MBUS_UART_NUM UART_NUM_1

// D3 (net LED) on IO15 -> R8 5.1K -> LED -> GND, so the LED is active high.
#define LED_PIN ((gpio_num_t)15)
#define LED_ACTIVE_HIGH 1

#define SQ_PERIOD_MIN_MS 50
#define SQ_PERIOD_MAX_MS 10000
#define SQ_PERIOD_DEFAULT_MS 1000

#define LED_BLINK_DEFAULT_MS 500

static const char *TAG = "pcb-test";

// Who owns the TX pin right now.
typedef enum {
    TX_MODE_GPIO,
    TX_MODE_UART,
} tx_mode_t;

static tx_mode_t tx_mode = TX_MODE_GPIO;

// Square-wave state. Written by console commands, read by sq_task. A period of
// 0 means "not running"; the task then leaves TX wherever a command put it.
static volatile uint32_t sq_period_ms = 0;
static volatile int tx_level = 1;

// LED blink state, same convention as the square wave.
static volatile uint32_t led_period_ms = 0;
static volatile int led_level = 0;

// Edge count on the receiver output, bumped from the GPIO ISR. In UART mode
// every received character adds to this too.
static volatile uint32_t rx_edges = 0;

static void IRAM_ATTR rx_isr_handler(void *arg)
{
    rx_edges = rx_edges + 1;
}

// --- Pin ownership ---------------------------------------------------------

static void set_tx(int level)
{
    tx_level = level;
    gpio_set_level(TX_PIN, level);
}

// Take TX back from the UART as a plain push-pull output. gpio_config()
// routes the pin's output back to the GPIO register, which disconnects the
// UART's TX signal without uninstalling the driver.
static void tx_use_gpio(void)
{
    if (tx_mode == TX_MODE_GPIO) {
        return;
    }
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << TX_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    set_tx(1);
    tx_mode = TX_MODE_GPIO;
}

// Hand TX to the UART. The first call installs the driver via
// mbus_uart_init(); after that only the pin routing needs restoring.
static void tx_use_uart(void)
{
    sq_period_ms = 0;
    if (tx_mode == TX_MODE_UART) {
        return;
    }
    if (!uart_is_driver_installed(MBUS_UART_NUM)) {
        mbus_uart_init();
    } else {
        uart_set_pin(MBUS_UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    tx_level = 1; // the UART idles at mark
    tx_mode = TX_MODE_UART;
}

static void set_led(int on)
{
    led_level = on;
    gpio_set_level(LED_PIN, LED_ACTIVE_HIGH ? on : !on);
}

// --- Background tasks ------------------------------------------------------

static void sq_task(void *arg)
{
    while (true) {
        uint32_t period = sq_period_ms;
        if (period == 0 || tx_mode != TX_MODE_GPIO) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        set_tx(!tx_level);
        vTaskDelay(pdMS_TO_TICKS(period / 2));
    }
}

static void led_task(void *arg)
{
    while (true) {
        uint32_t period = led_period_ms;
        if (period == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        set_led(!led_level);
        vTaskDelay(pdMS_TO_TICKS(period / 2));
    }
}

// --- Analogue commands -----------------------------------------------------

static void print_tx_state(void)
{
    if (tx_mode == TX_MODE_UART) {
        printf("TX (GPIO%d) = UART1, idling at mark -> expect ~34 V on M+/M-\n", TX_PIN);
        return;
    }
    printf("TX (GPIO%d) = %s  -> expect %s on M+/M-\n",
           TX_PIN, tx_level ? "HIGH (mark)" : "LOW (space)",
           tx_level ? "~34 V" : "~22 V");
}

static int cmd_hi(int argc, char **argv)
{
    tx_use_gpio();
    sq_period_ms = 0;
    set_tx(1);
    print_tx_state();
    return 0;
}

static int cmd_lo(int argc, char **argv)
{
    tx_use_gpio();
    sq_period_ms = 0;
    set_tx(0);
    print_tx_state();
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    sq_period_ms = 0;
    if (tx_mode == TX_MODE_GPIO) {
        set_tx(1); // leave the bus idling at mark
    }
    printf("Square wave stopped.\n");
    print_tx_state();
    return 0;
}

static struct {
    struct arg_int *period;
    struct arg_end *end;
} sq_args;

static int cmd_sq(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sq_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, sq_args.end, argv[0]);
        return 1;
    }

    int period = SQ_PERIOD_DEFAULT_MS;
    if (sq_args.period->count > 0) {
        period = sq_args.period->ival[0];
        if (period < SQ_PERIOD_MIN_MS || period > SQ_PERIOD_MAX_MS) {
            printf("Period must be %d..%d ms\n", SQ_PERIOD_MIN_MS, SQ_PERIOD_MAX_MS);
            return 1;
        }
    }

    tx_use_gpio();
    sq_period_ms = (uint32_t)period;
    printf("Square wave on GPIO%d: %d ms period (%d ms per level). 'stop' to end.\n",
           TX_PIN, period, period / 2);
    return 0;
}

static int cmd_rx(int argc, char **argv)
{
    printf("RX (GPIO%d) = %d, edges since boot = %lu\n",
           RX_PIN, gpio_get_level(RX_PIN), (unsigned long)rx_edges);
    return 0;
}

// --- LED -------------------------------------------------------------------

static struct {
    struct arg_str *action;
    struct arg_int *period;
    struct arg_end *end;
} led_args;

static int cmd_led(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&led_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, led_args.end, argv[0]);
        return 1;
    }

    const char *action = led_args.action->sval[0];
    if (strcmp(action, "on") == 0) {
        led_period_ms = 0;
        set_led(1);
    } else if (strcmp(action, "off") == 0) {
        led_period_ms = 0;
        set_led(0);
    } else if (strcmp(action, "blink") == 0) {
        int period = LED_BLINK_DEFAULT_MS;
        if (led_args.period->count > 0) {
            period = led_args.period->ival[0];
            if (period < SQ_PERIOD_MIN_MS || period > SQ_PERIOD_MAX_MS) {
                printf("Period must be %d..%d ms\n", SQ_PERIOD_MIN_MS, SQ_PERIOD_MAX_MS);
                return 1;
            }
        }
        led_period_ms = (uint32_t)period;
    } else {
        printf("Usage: led on|off|blink [ms]\n");
        return 1;
    }

    if (led_period_ms != 0) {
        printf("LED D3 (GPIO%d) blinking, %lu ms period\n", LED_PIN, (unsigned long)led_period_ms);
    } else {
        printf("LED D3 (GPIO%d) %s\n", LED_PIN, led_level ? "on" : "off");
    }
    return 0;
}

// --- M-Bus link ------------------------------------------------------------

static struct {
    struct arg_str *action;
    struct arg_int *addr;
    struct arg_end *end;
} mbus_args;

static bool parse_addr(struct arg_int *arg, uint8_t *out)
{
    int addr = MBUS_PRIMARY_ADDRESS;
    if (arg->count > 0) {
        addr = arg->ival[0];
    }
    // 0x01..0xFA primary, 0xFE broadcast (single slave on the bus).
    if (!((addr >= 0x01 && addr <= 0xFA) || addr == 0xFE)) {
        printf("Address must be 1..250 (0x01..0xFA) or 254 (0xFE, broadcast)\n");
        return false;
    }
    *out = (uint8_t)addr;
    return true;
}

static esp_err_t mbus_ping(uint8_t addr)
{
    tx_use_uart();
    return mbus_send_nke(addr);
}

// REQ_UD2 and decode every record. ESP_OK means a complete long frame arrived
// with a good checksum and stop byte -- the link works end to end.
static esp_err_t mbus_read(uint8_t addr)
{
    static uint8_t user[256];
    size_t user_len = 0;

    tx_use_uart();
    esp_err_t err = mbus_request_data(addr, user, sizeof(user), &user_len);
    if (err == ESP_OK) {
        mbus_parse_test(user, user_len);
    }
    return err;
}

static int cmd_mbus(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&mbus_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, mbus_args.end, argv[0]);
        return 1;
    }

    uint8_t addr;
    if (!parse_addr(mbus_args.addr, &addr)) {
        return 1;
    }

    const char *action = mbus_args.action->sval[0];
    esp_err_t err;
    if (strcmp(action, "ping") == 0) {
        err = mbus_ping(addr);
        printf("SND_NKE to 0x%02X -> %s\n", addr, esp_err_to_name(err));
    } else if (strcmp(action, "read") == 0) {
        err = mbus_read(addr);
        printf("REQ_UD2 to 0x%02X -> %s\n", addr, esp_err_to_name(err));
    } else {
        printf("Usage: mbus ping|read [addr]\n");
        return 1;
    }
    return err == ESP_OK ? 0 : 1;
}

// --- Board check -----------------------------------------------------------

static struct {
    struct arg_int *addr;
    struct arg_end *end;
} check_args;

typedef struct {
    const char *name;
    bool pass;
    char detail[64];
} check_result_t;

// Run the application-level checks in order and summarise. Each step only
// means something if the ones before it passed, but they all run so a single
// pass shows the whole picture.
static int cmd_check(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&check_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, check_args.end, argv[0]);
        return 1;
    }

    uint8_t addr;
    if (!parse_addr(check_args.addr, &addr)) {
        return 1;
    }

    // Results are printed together at the end: the M-Bus steps log the raw
    // frame and every decoded record, which would bury per-step lines.
    check_result_t results[3];

    // LED: nothing to read back, so flash it where the operator can see it.
    printf("Flashing LED D3 three times -- watch the board.\n");
    led_period_ms = 0;
    for (int i = 0; i < 3; i++) {
        set_led(1);
        vTaskDelay(pdMS_TO_TICKS(250));
        set_led(0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    // Receiver idle level. With TX at mark and nothing replying, the bus
    // current is a few unit loads at most, well under Q3's ~7 mA threshold, so
    // GPIO22 should sit high. Low here means too much idle current (a short,
    // or too many slaves) or a fault in the Q1/R15/Q3 path.
    tx_use_uart();
    vTaskDelay(pdMS_TO_TICKS(50));
    int idle = gpio_get_level(RX_PIN);
    results[0].name = "rx-idle";
    results[0].pass = idle == 1;
    snprintf(results[0].detail, sizeof(results[0].detail), "GPIO%d = %d with the bus idle at mark",
             RX_PIN, idle);

    // Link reset: a slave at this address ACKs with 0xE5.
    esp_err_t err = mbus_ping(addr);
    results[1].name = "mbus-ack";
    results[1].pass = err == ESP_OK;
    snprintf(results[1].detail, sizeof(results[1].detail), "SND_NKE to 0x%02X -> %s",
             addr, esp_err_to_name(err));

    // Data request: a full telegram with a valid checksum. The decoded records
    // are logged above the summary.
    err = mbus_read(addr);
    results[2].name = "mbus-data";
    results[2].pass = err == ESP_OK;
    snprintf(results[2].detail, sizeof(results[2].detail), "REQ_UD2 to 0x%02X -> %s",
             addr, esp_err_to_name(err));

    int failures = 0;
    printf("\n");
    for (size_t i = 0; i < sizeof(results) / sizeof(results[0]); i++) {
        printf("  [%s] %-10s %s\n", results[i].pass ? "PASS" : "FAIL", results[i].name, results[i].detail);
        failures += results[i].pass ? 0 : 1;
    }
    printf("  [ -- ] %-10s confirm by eye that D3 flashed three times\n", "led");
    printf("%s: %d automatic check(s) failed\n", failures == 0 ? "PASS" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}

// --- Status ----------------------------------------------------------------

static int cmd_status(int argc, char **argv)
{
    print_tx_state();
    if (tx_mode == TX_MODE_GPIO && sq_period_ms != 0) {
        printf("Square wave: running, %lu ms period\n", (unsigned long)sq_period_ms);
    } else {
        printf("Square wave: stopped\n");
    }
    if (led_period_ms != 0) {
        printf("LED D3: blinking, %lu ms period\n", (unsigned long)led_period_ms);
    } else {
        printf("LED D3: %s\n", led_level ? "on" : "off");
    }
    cmd_rx(argc, argv);
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "hi", .help = "Drive TX high (mark) as a GPIO - expect ~34 V on M+/M-", .func = &cmd_hi},
        {.command = "mark", .help = "Alias for 'hi'", .func = &cmd_hi},
        {.command = "lo", .help = "Drive TX low (space) as a GPIO - expect ~22 V on M+/M-", .func = &cmd_lo},
        {.command = "space", .help = "Alias for 'lo'", .func = &cmd_lo},
        {.command = "stop", .help = "Stop the square wave, leave TX idle high", .func = &cmd_stop},
        {.command = "rx", .help = "Read the receiver output level and edge count", .func = &cmd_rx},
        {.command = "status", .help = "Show TX mode/level, LED and RX state", .func = &cmd_status},
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    sq_args.period = arg_int0(NULL, NULL, "<ms>", "period in ms (default 1000)");
    sq_args.end = arg_end(2);
    const esp_console_cmd_t sq_cmd = {
        .command = "sq",
        .help = "Square-wave TX as a GPIO so the bus voltage swings between the two levels",
        .hint = NULL,
        .func = &cmd_sq,
        .argtable = &sq_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sq_cmd));

    led_args.action = arg_str1(NULL, NULL, "on|off|blink", "LED state");
    led_args.period = arg_int0(NULL, NULL, "<ms>", "blink period in ms (default 500)");
    led_args.end = arg_end(2);
    const esp_console_cmd_t led_cmd = {
        .command = "led",
        .help = "Turn LED D3 on, off or blinking",
        .hint = NULL,
        .func = &cmd_led,
        .argtable = &led_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&led_cmd));

    mbus_args.action = arg_str1(NULL, NULL, "ping|read", "SND_NKE, or REQ_UD2 and decode");
    mbus_args.addr = arg_int0(NULL, NULL, "<addr>", "primary address, e.g. 5 or 0x05 (default 0x01)");
    mbus_args.end = arg_end(2);
    const esp_console_cmd_t mbus_cmd = {
        .command = "mbus",
        .help = "Talk to an M-Bus slave over UART1 (2400 8E1)",
        .hint = NULL,
        .func = &cmd_mbus,
        .argtable = &mbus_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&mbus_cmd));

    check_args.addr = arg_int0(NULL, NULL, "<addr>", "slave primary address (default 0x01)");
    check_args.end = arg_end(2);
    const esp_console_cmd_t check_cmd = {
        .command = "check",
        .help = "Run the board checks: LED, receiver idle level, M-Bus ACK and data",
        .hint = NULL,
        .func = &cmd_check,
        .argtable = &check_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&check_cmd));
}

// --- Entry point -----------------------------------------------------------

static void init_pins(void)
{
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << TX_PIN) | (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    set_tx(1); // idle at mark
    set_led(0);

    // RX is driven by the bus receiver on the PCB - input only, no pulls.
    gpio_config_t rx_cfg = {
        .pin_bit_mask = 1ULL << RX_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&rx_cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RX_PIN, rx_isr_handler, NULL));
}

static void print_banner(void)
{
    printf("\n");
    printf("M-Bus adapter PCB test\n");
    printf("  TX = GPIO%d   RX = GPIO%d (read only)   LED D3 = GPIO%d\n", TX_PIN, RX_PIN, LED_PIN);
    printf("  Board check:  check [addr]\n");
    printf("  M-Bus:        mbus ping|read [addr]   (default addr 0x%02X)\n", MBUS_PRIMARY_ADDRESS);
    printf("  LED:          led on|off|blink [ms]\n");
    printf("  Analogue:     hi, lo, sq [ms], stop, rx   (DMM on M+/M-: mark ~34 V, space ~22 V)\n");
    printf("  TX starts as a GPIO, high (bus idle at mark).\n\n");
}

extern "C" void app_main(void)
{
    init_pins();
    ESP_LOGI(TAG, "Pins configured, TX idling high");

    xTaskCreate(sq_task, "sq", 2048, NULL, 5, NULL);
    xTaskCreate(led_task, "led", 2048, NULL, 5, NULL);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "pcb>";
    repl_config.max_cmdline_length = 64;
    // mbus_request_data() keeps its RX frame buffer static, but mbus_parse_test()
    // logs every record with a formatted line each -- give it headroom.
    repl_config.task_stack_size = 6144;

    // The prompt goes wherever the primary console is configured (UART0 by
    // default, see sdkconfig.defaults).
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t dev_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&dev_config, &repl_config, &repl));
#else
    esp_console_dev_uart_config_t dev_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&dev_config, &repl_config, &repl));
#endif

    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_commands();

    print_banner();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
