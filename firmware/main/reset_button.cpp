#include "reset_button.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_matter.h>

#include <inttypes.h>

// --- Wiring -----------------------------------------------------------------
// SW2 (net BOOT) ties IO9 to ground when pressed. There is no external pull-up,
// so the internal one has to be enabled and the input reads low when held.
// D3 (net LED) sits on IO15 -> R8 5.1K -> LED -> GND, so the LED is active high.
#define RESET_BUTTON_PIN ((gpio_num_t)9)  // SW2 / BOOT, active low
#define RESET_LED_PIN    ((gpio_num_t)15) // D3 via R8, active high

// --- Timing -----------------------------------------------------------------
#define RESET_HOLD_MS    5000
#define RESET_POLL_MS    20
#define RESET_DEBOUNCE_N 3 // 3 x 20 ms = 60 ms of a steady level before we believe it

// Blink period as the hold progresses. Getting faster is the only feedback that
// distinguishes "held for 4 s" from "the button never made contact".
#define RESET_BLINK_SLOW_MS   500
#define RESET_BLINK_MEDIUM_MS 200
#define RESET_BLINK_FAST_MS   100
#define RESET_BLINK_MEDIUM_AT 2000
#define RESET_BLINK_FAST_AT   4000

static const char *TAG = "ResetBtn";

static uint32_t blink_period_for(uint32_t held_ms)
{
    if (held_ms >= RESET_BLINK_FAST_AT) {
        return RESET_BLINK_FAST_MS;
    }
    if (held_ms >= RESET_BLINK_MEDIUM_AT) {
        return RESET_BLINK_MEDIUM_MS;
    }
    return RESET_BLINK_SLOW_MS;
}

static void reset_button_task(void *arg)
{
    bool pressed = false;      // debounced state
    int stable_count = 0;      // consecutive samples disagreeing with `pressed`
    uint32_t held_ms = 0;

    while (true) {
        // Active low: the button pulls IO9 down against the internal pull-up.
        bool raw = (gpio_get_level(RESET_BUTTON_PIN) == 0);

        if (raw != pressed) {
            if (++stable_count >= RESET_DEBOUNCE_N) {
                pressed = raw;
                stable_count = 0;

                if (pressed) {
                    held_ms = 0;
                } else {
                    ESP_LOGI(TAG, "Button released after %" PRIu32 " ms, factory reset aborted", held_ms);
                    gpio_set_level(RESET_LED_PIN, 0);
                }
            }
        } else {
            stable_count = 0;
        }

        // Only count time while the raw level still agrees, so a release that
        // is still inside the debounce window cannot push the timer over the
        // threshold and fire a reset after the button was let go.
        if (pressed && raw) {
            held_ms += RESET_POLL_MS;

            if (held_ms >= RESET_HOLD_MS) {
                // Solid LED marks the moment of commit, so the button can be
                // let go knowing the reset is already under way.
                gpio_set_level(RESET_LED_PIN, 1);
                ESP_LOGW(TAG, "Factory reset triggered by button");

                esp_matter::factory_reset();

                // factory_reset() schedules the reboot itself; just idle here so
                // we cannot trigger it a second time on the way down.
                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }

            uint32_t period = blink_period_for(held_ms);
            gpio_set_level(RESET_LED_PIN, (held_ms % period) < (period / 2) ? 1 : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(RESET_POLL_MS));
    }
}

esp_err_t reset_button_init(void)
{
    gpio_config_t btn_config = {
        .pin_bit_mask = 1ULL << RESET_BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&btn_config);
    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << RESET_LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&led_config);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(RESET_LED_PIN, 0);

    if (xTaskCreate(reset_button_task, "reset_btn", 3072, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Reset button ready on GPIO%d, hold %d ms to factory reset",
             RESET_BUTTON_PIN, RESET_HOLD_MS);
    return ESP_OK;
}
