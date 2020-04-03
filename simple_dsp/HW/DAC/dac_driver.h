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
 * @param dacBufferAddress, address of the buffer with the data to be output by DAC
 * @param bufferSize, size of buffer [n samples]
 */
void dac_Init(uint32_t dacBufferAddress, uint16_t bufferSize);


#endif /* DAC_DAC_DRIVER_H_ */
