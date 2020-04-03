/*
 * dac_driver.c
 *
 *  Created on: Apr 3, 2020
 *      Author: javi
 */


/* Includes */
#include "dac_driver.h"
#include "../../Modules/DSP/filter_tests.h"


/* Public functions */


void dac_Init(void) {
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH
      | LL_DMA_MODE_CIRCULAR | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT
      | LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD | LL_DMA_PRIORITY_HIGH);

  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t) &filter_tests_sine_table
      , LL_DAC_DMA_GetRegAddr(DAC1, LL_DAC_CHANNEL_1, LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED)
      , LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, FILTER_TESTS_SINE_TABLE_SIZE_N);

  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

  /* Enable DAC channel DMA request */
  LL_DAC_EnableDMAReq(DAC1, LL_DAC_CHANNEL_1);

  /* Enable DAC channel */
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);

  HAL_Delay(1);

  /* Enable DAC channel trigger */
  /* Note: DAC channel conversion can start from trigger enable:              */
  /*       - if DAC channel trigger source is set to SW:                      */
  /*         DAC channel conversion will start after trig order               */
  /*         using function "LL_DAC_TrigSWConversion()".                      */
  /*       - if DAC channel trigger source is set to external trigger         */
  /*         (timer, ...):                                                    */
  /*         DAC channel conversion can start immediately                     */
  /*         (after next trig order from external trigger)                    */
  LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
}
