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
  uint16_t *padcBuffer; // adc data is written to this buffer
  uint16_t bufferSize; // number of samples in buffer
  bool isHalfBufferFree[BUFFER_HALF_SIZE]; // flag to mark that the half buffer is free, not in use //TODO check if this should be volatile
} adc_driver_mod;


/* Private function prototypes */
static void adc_InitDMA(void);
static void adc_InitADC(void);


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

//#define SIZE 10
//static volatile uint16_t data[SIZE];
//void test_adc_dma(void){
//
//  //-----------------------------------------------------------
//  LL_ADC_EnableInternalRegulator(ADC1);
//  HAL_Delay(10);
//  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
//  HAL_Delay(10);
//
//  //-----------------------------------------------------------
//  LL_DMA_ConfigAddresses(DMA1,
//                         LL_DMA_CHANNEL_1,
//                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
//                         (uint32_t)&data,
//                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, SIZE);
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
//
//  //-----------------------------------------------------------
//  LL_ADC_Enable(ADC1);
//  HAL_Delay(10);
//  LL_ADC_REG_StartConversion(ADC1);
//
//  while(1){
//    HAL_Delay(10);
//  }
//}

//#define SIZE 10
//static volatile uint16_t data[SIZE];
//void test_adc_dma(void){
//
//  //-----------------------------------------------------------
//  LL_ADC_EnableInternalRegulator(ADC1);
//  HAL_Delay(10);
//  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
//  HAL_Delay(10);
//
//  //-----------------------------------------------------------
//  LL_DMA_ConfigAddresses(DMA1,
//                         LL_DMA_CHANNEL_1,
//                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
//                         (uint32_t)&data,
//                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, SIZE);
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
//
//  //-----------------------------------------------------------
//  LL_ADC_Enable(ADC1);
//  HAL_Delay(10);
//  LL_ADC_REG_StartConversion(ADC1);
//
//  while(1){
//    LL_ADC_REG_StartConversion(ADC1);
//    HAL_Delay(10);
//  }
//}

//#define SIZE 10
//static volatile uint16_t data[SIZE];
//void test_adc_dma_tim(void){
//
//  //-----------------------------------------------------------
//  LL_ADC_EnableInternalRegulator(ADC1);
//  HAL_Delay(10);
//  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
//  HAL_Delay(10);
//
//  //-----------------------------------------------------------
//  LL_DMA_ConfigAddresses(DMA1,
//                         LL_DMA_CHANNEL_1,
//                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
//                         (uint32_t)&data,
//                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, SIZE);
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
//
//  //-----------------------------------------------------------
//  LL_ADC_Enable(ADC1);
//  HAL_Delay(10);
//  LL_ADC_REG_StartConversion(ADC1);
//}


/**
 * Initializes adc driver
 * @param adcBuffer, pointer to buffer where the adc writes
 * @param bufferSize, number of samples in buffer
 */
void adc_Init(uint16_t *padcBuffer, uint16_t bufferSize) {
  //TODO assert(adcBuffer)
  //TODO assert(bufferSize)
  // init driver variables
  adc_driver_mod.padcBuffer = padcBuffer;
  adc_driver_mod.bufferSize = bufferSize;
  adc_driver_mod.isHalfBufferFree[BUFFER_HALF_FIRST] = false;
  adc_driver_mod.isHalfBufferFree[BUFFER_HALF_SECOND] = false;

  adc_InitDMA();
  adc_InitADC();
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


/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void){
  // half buffer transfer complete
  if(LL_DMA_IsActiveFlag_HT1(DMA1) == 1) {
    LL_DMA_ClearFlag_HT1(DMA1);
    adc_driver_mod.isHalfBufferFree[BUFFER_HALF_FIRST] = true;
    adc_driver_mod.isHalfBufferFree[BUFFER_HALF_SECOND] = false;
  }

  // buffer transfer complete
  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
    LL_DMA_ClearFlag_TC1(DMA1);
    adc_driver_mod.isHalfBufferFree[BUFFER_HALF_FIRST] = false ;
    adc_driver_mod.isHalfBufferFree[BUFFER_HALF_SECOND] = true ;
  }
}


/* Private functions */

/**
 * Initializes DMA to transfer the data from ADC to buffer
 */
static void adc_InitDMA(void){
  // transfer
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)adc_driver_mod.padcBuffer,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, adc_driver_mod.bufferSize);

  // interrupts
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

  // enable dma
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

/**
 * Initializes the ADC to trigger DMA request after each conversion
 */
static void adc_InitADC(void){
  LL_ADC_EnableInternalRegulator(ADC1);
  HAL_Delay(10);
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  HAL_Delay(10);
  LL_ADC_Enable(ADC1);
  HAL_Delay(10);
  LL_ADC_REG_StartConversion(ADC1);
}

