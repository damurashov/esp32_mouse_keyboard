/**
 * syswrap.h
 *
 * Created on: November 08, 2026
 *     Author: Dmitry Murashov
 */

#ifndef SYSWRAP_H_
#define SYSWRAP_H_

/****************************************************************************
* Included Files
****************************************************************************/

/* Always go first */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>
#include <stdio.h>

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

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint64_t syswrap_get_upttime_us(void)
{
	return esp_timer_get_time();
}

#undef EXTERN
#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __ASSEMBLY__ */

#endif /* SYSWRAP_H_ */

