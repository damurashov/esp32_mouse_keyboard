/**
 * pwr.c
 *
 * Created on: November 09, 2026
 *     Author: Dmitry Murashov
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOGTAG "pwr"

/****************************************************************************
 * Included files
 ****************************************************************************/

#include "config.h"
#include "hmi.h"
#include "pinout.h"
#include "syswrap.h"
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_pm.h>
#include <esp_sleep.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct
{
    esp_pm_lock_handle_t sleep_lock;
    int locked;
} s_sleep = {};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_a_encoder_lvl(void)
{
    return gpio_get_level(THUMBWHEEL_PIN_ENCODER_A);
}

static int get_b_encoder_lvl(void)
{
    return gpio_get_level(THUMBWHEEL_PIN_ENCODER_B);
}

static void prevent_sleep(void)
{
    if (!s_sleep.locked)
    {
        esp_pm_lock_acquire(s_sleep.sleep_lock);
    }
}

static void allow_sleep(void)
{
    if (s_sleep.locked)
    {
        esp_pm_lock_release(s_sleep.sleep_lock);
    }
}

static inline void enter_deep_sleep(void)
{
    /* Enable wakeup from deep sleep on encoder action */

    esp_err_t err = esp_sleep_enable_ext1_wakeup(
        (1ULL << THUMBWHEEL_PIN_ENCODER_A) | (1ULL << THUMBWHEEL_PIN_ENCODER_B),
        ESP_EXT1_WAKEUP_ANY_LOW
    );
    if (err != ESP_OK)
    {
        ESP_LOGE(LOGTAG, "Failed to configure EXT1 wakeup. Will not transfer into deep sleep");
        return;
    }

    /* Enter sandman */

    esp_deep_sleep_start();
}

static inline int is_user_idle(void)
{
    const uint64_t now = syswrap_get_upttime_us();
    if (now < hmi_last_user_action_timestamp_us() + THUMBWHEEL_USER_INACTIVE_DEEP_SLEEP_TIMEOUT_US)
    {
        return 0;
    }
    return 1;
}

static inline void handle_deep_sleep_loop(void)
{
    /* Enter deep sleep when inactive */

    if (is_user_idle())
    {
        enter_deep_sleep();
    }
}

/**
 * TODO DM Will probably be replaced w/ a sleep lock of sorts: (1) light sleep
 * for brief inactivity (half a minute), (2) deep sleep for prolonged
 * inactivity, (3) no sleep for periods of action (several seconds)
 */
static inline void handle_encoder_pin_wakeup_loop(void)
{
    /* TODO DM This function causes the encoder to become unresponsive. Remove
     * "return", and get down to it */

    return;

    /* Configure encoder pins for wakeup */

    ESP_ERROR_CHECK(gpio_wakeup_enable(THUMBWHEEL_PIN_ENCODER_A,
        get_a_encoder_lvl() ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(THUMBWHEEL_PIN_ENCODER_B,
        get_b_encoder_lvl() ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void pwr_init(void)
{
    /* Enable PM */

    esp_pm_config_t config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 40,
        .light_sleep_enable = true
    };

    ESP_ERROR_CHECK(esp_pm_configure(&config));
    ESP_ERROR_CHECK(
        esp_pm_lock_create(
            ESP_PM_NO_LIGHT_SLEEP,
            0,
            "no_sleep",
            &s_sleep.sleep_lock
        )
    );
}

void pwr_loop(void)
{
    handle_deep_sleep_loop();
    handle_encoder_pin_wakeup_loop();
}