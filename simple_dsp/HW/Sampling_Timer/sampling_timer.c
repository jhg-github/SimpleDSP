/*
 * sampling_timer.c
 *
 *  Created on: Apr 3, 2020
 *      Author: javi
 */


/* Includes */
#include "sampling_timer.h"


/* Public functions */

/**
 * Starts the timer that sinchronizes all sampling related tasks like adc and dac conversions
 * The timer is setup to trigger update event at a frequency of 48KHz
 */
void sampling_timer_Start(void) {
  LL_TIM_EnableCounter(TIM6);
}

