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

/* Public enums */
typedef enum {
  ADC_BUFFER_NUMBER_0, ADC_BUFFER_NUMBER_1,
  // keep last
  ADC_BUFFER_NUMBER_SIZE,
} adc_buffer_number_t;

/* Public functions */

/**
 * Initializes adc driver
 * @param adcBuffer, pointer to buffer where the adc writes
 * @param bufferSize, number of samples in buffer
 */
void adc_Init(uint16_t *const adcBuffer, const uint16_t bufferSize);

/**
 * Returns if a buffer is full and ready to be processed
 * @param bufferNumber
 * @return true if buffer is full and ready to be processed
 */
bool adc_IsBufferFull(const adc_buffer_number_t bufferNumber);

#endif /* HW_ADC_ADC_DRIVER_H_ */
