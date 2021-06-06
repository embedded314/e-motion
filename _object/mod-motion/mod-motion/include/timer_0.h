/**
 * \file
 *
 * \brief Timer 0 driver.
 *
 *
 */

#ifndef TIMER_0_H_
#define TIMER_0_H_

#include <compiler.h>

#ifdef __cplusplus
extern "C" {
	#endif

	volatile uint8_t TC0_OVF_flag;
	int8_t TIMER_0_initialization();


	#ifdef __cplusplus
}
#endif


#endif /* TIMER_0_H_ */