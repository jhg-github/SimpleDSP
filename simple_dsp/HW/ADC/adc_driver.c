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
  bool isHalfBufferFree[BUFFER_HALF_SIZE]; // flag to mark that the half buffer is free, not in use //TODO check if this should be volatile
} adc_driver_mod;


/* Public functions */


//static volatile uint16_t data;
//void test(void){
//  LL_ADC_EnableInternalRegulator(ADC1);
//  HAL_Delay(10);
//  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
//  HAL_Delay(10);
//  LL_ADC_Enable(ADC1);
//  HAL_Delay(10);
//  while(1){
//    LL_ADC_REG_StartConversion(ADC1);
//    HAL_Delay(10);
//    data = LL_ADC_REG_ReadConversionData12(ADC1);
//  }
//}

#define SIZE 10
static volatile uint16_t data[SIZE];
void test_adc_dma(void){

  //-----------------------------------------------------------
  LL_ADC_EnableInternalRegulator(ADC1);
  HAL_Delay(10);
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  HAL_Delay(10);

  //-----------------------------------------------------------
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&data,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, SIZE);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  //-----------------------------------------------------------
  LL_ADC_Enable(ADC1);
  HAL_Delay(10);
  LL_ADC_REG_StartConversion(ADC1);

  while(1){
    HAL_Delay(10);
  }
}



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

  test_adc_dma();
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
