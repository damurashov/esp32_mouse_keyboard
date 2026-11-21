/**
 * pwr.c
 *
 * Created on: November 09, 2026
 *     Author: Dmitry Murashov
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Included files
 ****************************************************************************/

#include "pinout.h"
#include <driver/gpio.h>
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