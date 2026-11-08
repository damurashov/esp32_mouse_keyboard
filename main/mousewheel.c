/**
 * mousewheel.c
 *
 * Created on: November 06, 2026
 *     Author: Dmitry Murashov
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOUSEWHEEL_TAG "mousewheel"
#define MOUSEWHEEL_LOGD(...) ESP_LOGD(MOUSEWHEEL_TAG , __VA_ARGS__)
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

/****************************************************************************
 * Included files
 ****************************************************************************/

#include "pinout.h"
#include "syswrap.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <stdint.h>

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
    int32_t acc_dir;
    int32_t rept_dir;
    int pending_pos;
    int pending_neg;
    int32_t dir;
} s_enc = {};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int get_a_lvl(void)
{
    return gpio_get_level(THUMBWHEEL_PIN_ENCODER_A);
}

int get_b_lvl(void)
{
    return gpio_get_level(THUMBWHEEL_PIN_ENCODER_B);
}

static void thumbwheel_isr(void *arg)
{

    /* 3 things must happen:
     * - A MUST trigger,
     * - B MUST trigger,
     * - A, and B MUST have the same signal level */

    int32_t dir = (int32_t)arg;

    /* Set the direction */

    if (!s_enc.dir)
    {
        s_enc.dir = dir;
    }

    /* Keep tabs on which of the encoder leads have triggered */

    if (dir > 0)
    {
        s_enc.pending_pos = 1;
    }
    else if (dir < 0)
    {
        s_enc.pending_neg = 1;
    }

    /* Check if a transition happened */

    int a = get_a_lvl();
    int b = get_b_lvl();
    if (a == b)
    {
        if (s_enc.pending_neg && s_enc.pending_pos)
        {
            s_enc.acc_dir += dir;
        }
        s_enc.dir = 0;
        s_enc.pending_pos = 0;
        s_enc.pending_neg = 0;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mousewheel_init(void)
{
    /* Init pins */

    gpio_config_t io_enc = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << THUMBWHEEL_PIN_ENCODER_A) | (1ULL << THUMBWHEEL_PIN_ENCODER_B)),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_enc);

    /* Install ISRs */

    gpio_install_isr_service(0);
    gpio_isr_handler_add(THUMBWHEEL_PIN_ENCODER_A, thumbwheel_isr, (void *)1);
    gpio_isr_handler_add(THUMBWHEEL_PIN_ENCODER_B, thumbwheel_isr, (void *)-1);

    esp_log_level_set(MOUSEWHEEL_TAG, ESP_LOG_VERBOSE);
}

int32_t mousewheel_loop(void)
{
    const int32_t ret = s_enc.acc_dir - s_enc.rept_dir;
    s_enc.rept_dir = s_enc.acc_dir;
    if (ret)
    {
        MOUSEWHEEL_LOGD("Wheel: %d", ret);
    }
    return ret;
}
