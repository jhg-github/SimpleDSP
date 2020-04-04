/*
 * adc_driver.h
 *
 *  Created on: Mar 27, 2020
 *      Author: javi
 */

#ifndef HW_ADC_ADC_DRIVER_H_
#define HW_ADC_ADC_DRIVER_H_


/* Includes */
#include "main.h"
#include "../../Modules/DSP/dsp.h"


/* Public functions */

/**
 * Initializes adc driver
 * @param adcBuffer, pointer to buffer where the adc writes
 * @param bufferSize, number of samples in buffer
 */
void adc_Init(uint16_t *const adcBuffer, const uint16_t bufferSize);

/**
 * Returns if the half buffer free and ready to be processed
 * @param bufferHalf
 * @return true if half buffer free and ready to be processed
 */
bool adc_IsHalfBufferFree(const buffer_half_t bufferHalf);

#endif /* HW_ADC_ADC_DRIVER_H_ */
