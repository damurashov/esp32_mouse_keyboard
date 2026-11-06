/**
 * mousewheel.c
 *
 * Created on: November 06, 2026
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
    int32_t curr_dir;
    int32_t acc_dir;
    int32_t rept_dir;
} s_enc = {};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void thumbwheel_isr(void *arg)
{
    if (!s_enc.curr_dir)
    {
        s_enc.curr_dir = (int32_t)arg;
    }
    else
    {
        s_enc.curr_dir = 0;
        s_enc.acc_dir += (int32_t arg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mousewheel_init(void)
{
    /* Init pins */

    gpio_config_t io_enc = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << THUMBWHEEL_PIN_ENCODER_A) | (1ULL << THUMBWHEEL_PIN_ENCODER_B)),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_pwrb);

    /* Install ISRs */

    gpio_isr_handler_add(THUMBWHEEL_PIN_ENCODER_A, thumbwheel_isr, (void*)(-1ULL));
    gpio_isr_handler_add(THUMBWHEEL_PIN_ENCODER_B, thumbwheel_isr, (void *)1ULL);
}

int32_t mousewheel_loop(void)
{
    const int32_t ret = s_enc.acc_dir - s_enc.rept_dir;
    s_enc.rept_dir = s_enc.acc_dir;
}
