/*
 * filter_tests.h
 *
 *  Created on: Mar 30, 2020
 *      Author: javi
 */

#ifndef DSP_FILTER_TESTS_H_
#define DSP_FILTER_TESTS_H_


/* Includes */
#include "main.h"


/* Public defines */
#define FILTER_TESTS_SIGNAL_SIZE_N      (1024)
#define FILTER_TESTS_SINE_TABLE_SIZE_N  (256)


/* Public variables */

extern const float filter_tests_signal[FILTER_TESTS_SIGNAL_SIZE_N];

extern const uint16_t filter_tests_sine_table[];


#endif /* DSP_FILTER_TESTS_H_ */
