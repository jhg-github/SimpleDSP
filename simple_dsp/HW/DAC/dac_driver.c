/*
 * dac_driver.c
 *
 *  Created on: Apr 3, 2020
 *      Author: javi
 */


/* Includes */
#include "dac_driver.h"
#include "../../Modules/DSP/filter_tests.h"


/* Private defines */
#define DAC_DAC         DAC1
#define DAC_DAC_CHANNEL LL_DAC_CHANNEL_1
#define DAC_DMA         DMA1
#define DAC_DMA_CHANNEL LL_DMA_CHANNEL_3


/* Private variables */

static struct dac_driver_tag {  // dac driver structure
  uint32_t dacBufferAddress;    // buffer address to output via DAC
  uint16_t dacBufferSize;          // number of samples in buffer
} dac_driver_mod;


/* Private function prototypes */
static void dac_InitDMA(void);
static void dac_InitDAC(void);


/* Public functions */

/**
 * Initializes DAC and DMA. The DAC is feed continuously by the DMA in circular buffer mode.
 * DAC conversions are triggered by sampling_timer's update event, therefore sampling_timer
 * must be also enabled
 * @param dacBufferAddress, address of the buffer with the data to be output by DAC
 * @param bufferSize, size of buffer [n samples]
 */
void dac_Init(uint32_t dacBufferAddress, uint16_t bufferSize) {
  //TODO assert(dacBufferAddress)
  //TODO assert(bufferSize)
  dac_driver_mod.dacBufferAddress = dacBufferAddress;
  dac_driver_mod.dacBufferSize = bufferSize;
  dac_InitDMA();
  dac_InitDAC();
}


/* Private function */

/**
 * Initializes DMA to transfer the data from buffer to DAC
 */
static void dac_InitDMA(void){
  LL_DMA_ConfigTransfer(DAC_DMA, DAC_DMA_CHANNEL, LL_DMA_DIRECTION_MEMORY_TO_PERIPH
      | LL_DMA_MODE_CIRCULAR | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT
      | LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD | LL_DMA_PRIORITY_HIGH);
  LL_DMA_ConfigAddresses(DAC_DMA, DAC_DMA_CHANNEL, dac_driver_mod.dacBufferAddress
      , LL_DAC_DMA_GetRegAddr(DAC_DAC, DAC_DAC_CHANNEL, LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED)
      , LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DAC_DMA, DAC_DMA_CHANNEL, dac_driver_mod.dacBufferSize);
  LL_DMA_EnableChannel(DAC_DMA, DAC_DMA_CHANNEL);
}


/**
 * Initializes the DAC to trigger DMA request after each conversion
 */
static void dac_InitDAC(void){
  LL_DAC_EnableDMAReq(DAC_DAC, DAC_DAC_CHANNEL);
  LL_DAC_Enable(DAC_DAC, DAC_DAC_CHANNEL);
  LL_DAC_EnableTrigger(DAC_DAC, DAC_DAC_CHANNEL);
}
