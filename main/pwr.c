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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int get_a_encoder_lvl(void)
{
    return gpio_get_level(THUMBWHEEL_PIN_ENCODER_A);
}

int get_b_encoder_lvl(void)
{
    return gpio_get_level(THUMBWHEEL_PIN_ENCODER_B);
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
}

void pwr_loop(void)
{
    /* Configure encoder pins for wakeup */
    ESP_ERROR_CHECK(gpio_wakeup_enable(THUMBWHEEL_PIN_ENCODER_A,
        get_a_encoder_lvl() ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(THUMBWHEEL_PIN_ENCODER_B,
        get_b_encoder_lvl() ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
}