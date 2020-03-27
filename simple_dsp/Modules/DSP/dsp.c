/*
 * dsp.c
 *
 *  Created on: Mar 27, 2020
 *      Author: javi
 */

/* Includes */
#include "dsp.h"
#include "../../HW/ADC/adc_driver.h"

/* Private defines */
#define DSP_BLOCK_N_SAMPLES       (1024) // number of samples per block of processing
#define DSP_ADC_BUFFER_N_SAMPLES  (2*DSP_BLOCK_N_SAMPLES) // 2 times DSP_BLOCK_N_SAMPLES in order to process one block while the other is being written by the adc

/* Private variables */

static struct dsp_mod_tag {                     // dsp module structure
  // buffers
  uint16_t adcBuffer[DSP_ADC_BUFFER_N_SAMPLES]; //TODO check if this should be volatile, look at ST library // adc data is written to this buffer
  //
  adc_buffer_number_t nextBufferToProcess;   // number of next buffer to process
  dsp_mode_t mode;                              // type of filter to apply
} dsp_mod;

/* Public functions */

/**
 * Initializes dsp module
 */
void dsp_Init(void) {
  // init drivers
  adc_Init(&dsp_mod.adcBuffer[0], DSP_ADC_BUFFER_N_SAMPLES);
  // init module variables
  dsp_mod.nextBufferToProcess = ADC_BUFFER_NUMBER_0;
  dsp_mod.mode = DSP_MODE_BYPASS;
}

/**
 * Dsp module process
 */
void dsp_Process(void) {
  if (adc_IsBufferFull(dsp_mod.nextBufferToProcess)) {
    // process here !
    switch (dsp_mod.mode) {
    case DSP_MODE_BYPASS:
    default:
      break;
    case DSP_MODE_LOWPASS:
      break;
    case DSP_MODE_BANDPASS:
      break;
    case DSP_MODE_HIGHPASS:
      break;
    }
    // change to next buffer
    if (dsp_mod.nextBufferToProcess == ADC_BUFFER_NUMBER_0) {
      dsp_mod.nextBufferToProcess = ADC_BUFFER_NUMBER_1;
    } else {
      dsp_mod.nextBufferToProcess = ADC_BUFFER_NUMBER_0;
    }
  }
}
