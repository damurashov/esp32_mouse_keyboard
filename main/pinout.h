/**
 * pinout.h
 *
 * Created on: November 06, 2026
 *     Author: Dmitry Murashov
 */

#ifndef MAIN_PINOUT_H_
#define MAIN_PINOUT_H_

/****************************************************************************
* Included Files
****************************************************************************/

/****************************************************************************
* Pre-processor Definitions
****************************************************************************/

#if CONFIG_USE_AS_FLIPMOUSE_FABI
#else
	#define INDICATOR_LED_PIN    CONFIG_MODULE_LED_PIN
	#define EX_SERIAL_TXPIN      CONFIG_MODULE_TX_PIN
	#define EX_SERIAL_RXPIN      CONFIG_MODULE_RX_PIN
#endif

#define THUMBWHEEL_PIN_ENCODER_A (39ULL)
#define THUMBWHEEL_PIN_ENCODER_B (40ULL)

/****************************************************************************
* Public Types
****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
* Public Data
****************************************************************************/

#ifdef EXTERN
#error EXTERN is defined
#endif

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
* Public Function Prototypes
****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __ASSEMBLY__ */

#endif /* MAIN_PINOUT_H_ */

