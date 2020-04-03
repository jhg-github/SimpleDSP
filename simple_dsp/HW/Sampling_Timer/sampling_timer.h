/*
 * sampling_timer.h
 *
 *  Created on: Apr 3, 2020
 *      Author: javi
 */

#ifndef SAMPLING_TIMER_SAMPLING_TIMER_H_
#define SAMPLING_TIMER_SAMPLING_TIMER_H_


/* Includes */
#include "main.h"


/* Public functions */

/**
 * Starts the timer that sinchronizes all sampling related tasks like adc and dac conversions
 * The timer is setup to trigger update event at a frequency of 48KHz
 */
void sampling_timer_Start(void);


#endif /* SAMPLING_TIMER_SAMPLING_TIMER_H_ */
