/**
 * cli.h
 *
 * Created on: November 21, 2026
 *     Author: Dmitry Murashov
 */

#ifndef MAIN_CLI_H_
#define MAIN_CLI_H_

/****************************************************************************
* Included Files
****************************************************************************/

/****************************************************************************
* Pre-processor Definitions
****************************************************************************/

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

/** 2nd UART unit number for RX/TX of commands
 * Previously we used a define via KConfig to set the UART number.
 * Because we need to detect on the fly if we are on an Arduino RP2040 Connect
 * or on a Bluetooth addon, this value is set in a variable now. */
EXTERN int ext_uart_num;

/** 2nd UART unit RX pin */
EXTERN int ext_uart_rx;

/** 2nd UART unit TX pin */
EXTERN int ext_uart_tx;

/****************************************************************************
* Public Function Prototypes
****************************************************************************/

void cli_init(void);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __ASSEMBLY__ */

#endif /* MAIN_CLI_H_ */

