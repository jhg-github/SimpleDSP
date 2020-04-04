/*
 * dac_driver.h
 *
 *  Created on: Apr 3, 2020
 *      Author: javi
 */

#ifndef DAC_DAC_DRIVER_H_
#define DAC_DAC_DRIVER_H_


/* Includes */
#include "main.h"


/* Public functions */

/**
 * Initializes DAC and DMA. The DAC is feed continuously by the DMA in circular buffer mode.
 * DAC conversions are triggered by sampling_timer's update event, therefore sampling_timer
 * must be also enabled
 * @param pdacBufferAddress, points to buffer to output via DAC
 * @param bufferSize, size of buffer [n samples]
 */
void dac_Init(uint16_t *pdacBufferAddress, uint16_t bufferSize) ;

/**
 * Returns if the half buffer free and ready to be processed
 * @param bufferHalf
 * @return true if half buffer free and ready to be processed
 */
bool dac_IsHalfBufferFree(const buffer_half_t bufferHalf);


#endif /* DAC_DAC_DRIVER_H_ */
