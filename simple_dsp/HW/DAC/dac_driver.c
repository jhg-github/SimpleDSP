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
#define DAC_DAC                   DAC1
#define DAC_DAC_CHANNEL           LL_DAC_CHANNEL_1
#define DAC_DMA                   DMA1
#define DAC_DMA_CHANNEL           LL_DMA_CHANNEL_3
#define DAC_DMA_CHANNEL_IRQN      DMA1_Channel3_IRQn
#define DAC_DMA_CHANNEL_IRQN_PRIO (1)


/* Private variables */

static struct dac_driver_mod_tag {  // dac driver structure
  uint16_t *pdacBufferAddress;      // points to buffer to output via DAC
  uint16_t dacBufferSize;           // number of samples in buffer
  bool isHalfBufferFree[BUFFER_HALF_SIZE]; // flag to mark that the half buffer is free, not in use //TODO check if this should be volatile
} dac_driver_mod;


/* Private function prototypes */
static void dac_InitDMA(void);
static void dac_InitDAC(void);
static void dac_InitBuffer(void);


/* Public functions */

/**
 * Initializes DAC and DMA. The DAC is feed continuously by the DMA in circular buffer mode.
 * DAC conversions are triggered by sampling_timer's update event, therefore sampling_timer
 * must be also enabled
 * @param pdacBufferAddress, points to buffer to output via DAC
 * @param bufferSize, size of buffer [n samples]
 */
void dac_Init(uint16_t *pdacBufferAddress, uint16_t bufferSize) {
  //TODO assert(pdacBufferAddress)
  //TODO assert(bufferSize)
  dac_driver_mod.pdacBufferAddress = pdacBufferAddress;
  dac_driver_mod.dacBufferSize = bufferSize;
  dac_driver_mod.isHalfBufferFree[BUFFER_HALF_FIRST] = false;
  dac_driver_mod.isHalfBufferFree[BUFFER_HALF_SECOND] = false;
  dac_InitDMA();
  dac_InitDAC();
  dac_InitBuffer();
}

/**
 * Returns if the half buffer free and ready to be processed
 * @param bufferHalf
 * @return true if half buffer free and ready to be processed
 */
bool dac_IsHalfBufferFree(const buffer_half_t bufferHalf) {
#warning ONLY FOR TEST !!!
  //return true;
#warning ONLY FOR TEST !!!
  // TODO assert (bufferHalf)
  return dac_driver_mod.isHalfBufferFree[bufferHalf];
}


/* Private functions */

/**
 * Initializes DMA to transfer the data from buffer to DAC
 */
static void dac_InitDMA(void){
  // transfer
  LL_DMA_ConfigTransfer(DAC_DMA, DAC_DMA_CHANNEL, LL_DMA_DIRECTION_MEMORY_TO_PERIPH
      | LL_DMA_MODE_CIRCULAR | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT
      | LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD | LL_DMA_PRIORITY_HIGH);
  LL_DMA_ConfigAddresses(DAC_DMA, DAC_DMA_CHANNEL, (uint32_t)dac_driver_mod.pdacBufferAddress
      , LL_DAC_DMA_GetRegAddr(DAC_DAC, DAC_DAC_CHANNEL, LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED)
      , LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DAC_DMA, DAC_DMA_CHANNEL, dac_driver_mod.dacBufferSize);

  // interrupts
  NVIC_SetPriority(DAC_DMA_CHANNEL_IRQN, DAC_DMA_CHANNEL_IRQN_PRIO);
  NVIC_EnableIRQ(DAC_DMA_CHANNEL_IRQN);
  LL_DMA_EnableIT_TC(DAC_DMA, DAC_DMA_CHANNEL);
  LL_DMA_EnableIT_HT(DAC_DMA, DAC_DMA_CHANNEL);

  // enable dma
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

/**
 * Initializes the buffer to half supply
 */
static void dac_InitBuffer(void){
  uint16_t i;
  for(i=0;i<dac_driver_mod.dacBufferSize;i++){
    dac_driver_mod.pdacBufferAddress[i] = 2047;  // 2047 =  half supply
  }
}
