/**
 * mousewheel.h
 *
 * Created on: November 06, 2026
 *     Author: Dmitry Murashov
 */

#ifndef MAIN_MOUSEWHEEL_H_
#define MAIN_MOUSEWHEEL_H_

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

/****************************************************************************
* Public Function Prototypes
****************************************************************************/

void mousewheel_init(void);

/**
 * @returns next direction for the mouse (number of clicks)
 */
int32_t mousewheel_loop(void);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __ASSEMBLY__ */

#endif /* MAIN_MOUSEWHEEL_H_ */
