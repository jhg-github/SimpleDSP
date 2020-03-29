/*
 * adc_driver.c
 *
 *  Created on: Mar 27, 2020
 *      Author: javi
 */

/* Includes */
#include "adc_driver.h"

/* Private variables */

static struct adc_driver_tag { // adc driver structure
  uint16_t *adcBuffer; // adc data is written to this buffer
  uint16_t bufferSize; // number of samples in buffer
  bool isBufferFull[ADC_BUFFER_NUMBER_SIZE]; // flag to mark that a buffer is full and ready to be processed
} adc_driver_mod; //TODO check if this should be volatile

/* Public functions */

/**
 * Initializes adc driver
 * @param adcBuffer, pointer to buffer where the adc writes
 * @param bufferSize, number of samples in buffer
 */
void adc_Init(uint16_t *const adcBuffer, const uint16_t bufferSize) {
  // init driver variables
  adc_driver_mod.adcBuffer = adcBuffer;
  adc_driver_mod.bufferSize = bufferSize;
  adc_driver_mod.isBufferFull[ADC_BUFFER_NUMBER_0] = false;
  adc_driver_mod.isBufferFull[ADC_BUFFER_NUMBER_1] = false;
}

/**
 * Returns if a buffer is full and ready to be processed
 * @param bufferNumber
 * @return true if buffer is full and ready to be processed
 */
bool adc_IsBufferFull(const adc_buffer_number_t bufferNumber) {
#warning ONLY FOR TEST !!!
  return true;
#warning ONLY FOR TEST !!!
  // TODO add assert here
  return adc_driver_mod.isBufferFull[bufferNumber];
}
