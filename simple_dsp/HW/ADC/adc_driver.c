/*
 * adc_driver.c
 *
 *  Created on: Mar 27, 2020
 *      Author: javi
 */

/* Includes */
#include "adc_driver.h"


/* Private variables */

static struct adc_driver_mod_tag { // adc driver structure
  uint16_t *adcBuffer; // adc data is written to this buffer
  uint16_t bufferSize; // number of samples in buffer
  bool isHalfBufferFree[BUFFER_HALF_SIZE]; // flag to mark that the half buffer is free, not in use
} adc_driver_mod; //TODO check if this should be volatile


/* Public functions */

/**
 * Initializes adc driver
 * @param adcBuffer, pointer to buffer where the adc writes
 * @param bufferSize, number of samples in buffer
 */
void adc_Init(uint16_t *const adcBuffer, const uint16_t bufferSize) {
  //TODO assert(adcBuffer)
  //TODO assert(bufferSize)
  // init driver variables
  adc_driver_mod.adcBuffer = adcBuffer;
  adc_driver_mod.bufferSize = bufferSize;
  adc_driver_mod.isHalfBufferFree[BUFFER_HALF_FIRST] = false;
  adc_driver_mod.isHalfBufferFree[BUFFER_HALF_SECOND] = false;
}

/**
 * Returns if the half buffer free and ready to be processed
 * @param bufferHalf
 * @return true if half buffer free and ready to be processed
 */
bool adc_IsHalfBufferFree(const buffer_half_t bufferHalf) {
#warning ONLY FOR TEST !!!
  return true;
#warning ONLY FOR TEST !!!
  // TODO assert (bufferHalf)
  return adc_driver_mod.isHalfBufferFree[bufferHalf];
}
