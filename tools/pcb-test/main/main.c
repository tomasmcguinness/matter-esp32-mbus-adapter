// PCB bring-up test for the M-Bus master circuit.
//
// This app deliberately does NOT install a UART driver. It drives the TX pin as
// a plain GPIO so you can park it at a fixed level and measure the resulting
// bus voltage on M+ / M- with a multimeter, ruling the analogue side in or out
// before any UART framing is involved.
//
// Expected on M+ (red probe) / M- (black probe), DC volts:
//   TX high (mark  / idle)  ->  ~36 V
//   TX low  (space)         ->  ~24 V   (a ~12 V drop)

#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Same pins as the production firmware: see firmware/main/mbus.cpp, which is
// the source of truth for the wiring.
#define TX_PIN ((gpio_num_t)23) // ESP TX -> M-Bus modulator
#define RX_PIN ((gpio_num_t)22) // ESP RX <- M-Bus receiver (input only)

// Status LED, mirrors the TX level so the bus state is visible at a glance:
// lit = mark (~36 V), dark = space (~24 V). Set to 0 if the LED is wired
// active-low (cathode to the pin).
#define LED_PIN ((gpio_num_t)15)
#define LED_ACTIVE_HIGH 1

#define SQ_PERIOD_MIN_MS 50
#define SQ_PERIOD_MAX_MS 10000
#define SQ_PERIOD_DEFAULT_MS 1000

static const char *TAG = "pcb-test";

// Square-wave state. Written by console commands, read by sq_task. A period of
// 0 means "not running"; the task then leaves TX wherever a command put it.
static volatile uint32_t sq_period_ms = 0;
static volatile int tx_level = 1;

// Edge count on the receiver output, bumped from the GPIO ISR.
static volatile uint32_t rx_edges = 0;

static void IRAM_ATTR rx_isr_handler(void *arg)
{
    rx_edges++;
}

static void set_tx(int level)
{
    tx_level = level;
    gpio_set_level(TX_PIN, level);
    gpio_set_level(LED_PIN, LED_ACTIVE_HIGH ? level : !level);
}

static void sq_task(void *arg)
{
    while (true) {
        uint32_t period = sq_period_ms;
        if (period == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        set_tx(!tx_level);
        vTaskDelay(pdMS_TO_TICKS(period / 2));
    }
}

// --- Console commands ------------------------------------------------------

static void print_tx_state(void)
{
    printf("TX (GPIO%d) = %s  -> expect %s on M+/M-, LED (GPIO%d) %s\n",
           TX_PIN, tx_level ? "HIGH (mark)" : "LOW (space)",
           tx_level ? "~36 V" : "~24 V",
           LED_PIN, tx_level ? "on" : "off");
}

static int cmd_hi(int argc, char **argv)
{
    sq_period_ms = 0;
    set_tx(1);
    print_tx_state();
    return 0;
}

static int cmd_lo(int argc, char **argv)
{
    sq_period_ms = 0;
    set_tx(0);
    print_tx_state();
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    sq_period_ms = 0;
    set_tx(1); // leave the bus idling at mark
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

static int cmd_status(int argc, char **argv)
{
    print_tx_state();
    if (sq_period_ms != 0) {
        printf("Square wave: running, %lu ms period\n", (unsigned long)sq_period_ms);
    } else {
        printf("Square wave: stopped\n");
    }
    cmd_rx(argc, argv);
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "hi", .help = "Drive TX high (mark) - expect ~36 V on M+/M-", .func = &cmd_hi},
        {.command = "mark", .help = "Alias for 'hi'", .func = &cmd_hi},
        {.command = "lo", .help = "Drive TX low (space) - expect ~24 V on M+/M-", .func = &cmd_lo},
        {.command = "space", .help = "Alias for 'lo'", .func = &cmd_lo},
        {.command = "stop", .help = "Stop the square wave, leave TX idle high", .func = &cmd_stop},
        {.command = "rx", .help = "Read the receiver output level and edge count", .func = &cmd_rx},
        {.command = "status", .help = "Show TX mode/level and RX state", .func = &cmd_status},
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    sq_args.period = arg_int0(NULL, NULL, "<ms>", "period in ms (default 1000)");
    sq_args.end = arg_end(2);
    const esp_console_cmd_t sq_cmd = {
        .command = "sq",
        .help = "Square-wave TX so the bus voltage swings between the two levels",
        .hint = NULL,
        .func = &cmd_sq,
        .argtable = &sq_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sq_cmd));
}

// --- Entry point -----------------------------------------------------------

static void init_pins(void)
{
    gpio_config_t tx_cfg = {
        .pin_bit_mask = (1ULL << TX_PIN) | (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&tx_cfg));
    set_tx(1); // idle at mark

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
    printf("M-Bus PCB bring-up test\n");
    printf("  TX = GPIO%d (driven)   RX = GPIO%d (read only)   LED = GPIO%d (follows TX)\n",
           TX_PIN, RX_PIN, LED_PIN);
    printf("  Probe M+ (red) / M- (black) with the DMM in DC volts:\n");
    printf("    TX high (mark)  -> ~36 V\n");
    printf("    TX low  (space) -> ~24 V\n");
    printf("  The GPIO%d LED is lit whenever TX is high (mark).\n", LED_PIN);
    printf("  Commands: hi, lo, sq [ms], stop, rx, status, help\n");
    printf("  TX starts high (bus idle at mark).\n\n");
}

void app_main(void)
{
    init_pins();
    ESP_LOGI(TAG, "Pins configured, TX idling high");

    xTaskCreate(sq_task, "sq", 2048, NULL, 5, NULL);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "pcb>";
    repl_config.max_cmdline_length = 64;

    esp_console_dev_usb_serial_jtag_config_t dev_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&dev_config, &repl_config, &repl));

    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_commands();

    print_banner();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
