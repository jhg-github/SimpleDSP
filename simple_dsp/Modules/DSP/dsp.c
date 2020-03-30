/*
 * dsp.c
 *
 *  Created on: Mar 27, 2020
 *      Author: javi
 */


/* Includes */
#include "dsp.h"
#include "filter_coeffs.h"
#include "filter_decimation.h"
#include "filter_tests.h"
#include "../../HW/ADC/adc_driver.h"
#include "arm_math.h"


/* Private defines */
#define DSP_DECIMATION_FACTOR     (8)                         // decimation factor
#define DSP_BLOCK_FS_N_SAMPLES    (1024)                      // number of samples per block of processing at sampling frequency
#define DSP_BLOCK_DEC_N_SAMPLES   (DSP_BLOCK_FS_N_SAMPLES/DSP_DECIMATION_FACTOR)  // number of samples per block of processing at decimated frequency
#define DSP_ADC_BUFFER_N_SAMPLES  (2*DSP_BLOCK_FS_N_SAMPLES)  // 2 times DSP_BLOCK_N_SAMPLES in order to process one block while the other is being written by the adc
#define DSP_DAC_BUFFER_N_SAMPLES  (2*DSP_BLOCK_FS_N_SAMPLES)  // 2 times DSP_BLOCK_N_SAMPLES in order to write one block while the other is being used by the dac


/* Private variables */
static struct dsp_mod_tag {                     // dsp module structure
  // buffers
  uint16_t adcBuffer[DSP_ADC_BUFFER_N_SAMPLES]; //TODO check if this should be volatile, look at ST library // adc data is written to this buffer
  float signal_fs[DSP_BLOCK_FS_N_SAMPLES];      // signal at sampling frequency. used before decimating and after interpolating */
  float signal_dec[DSP_BLOCK_DEC_N_SAMPLES];    // signal at decimated frequency
  float signal_filt[DSP_BLOCK_DEC_N_SAMPLES];   // signal filtered at decimated frequency
  uint16_t dacBuffer[DSP_ADC_BUFFER_N_SAMPLES]; // dac data is written to this buffer
  // control
  adc_buffer_number_t nextBufferToProcess;      // number of next buffer to process
  dsp_mode_t mode;                              // type of filter to apply
  // filters
  filter_dec_t *pfilter_dec;                    // points to decimation filter instance
} dsp_mod;


/* Private function prototypes */
static void dsp_InitFilters(void);
static void dsp_BypassSignal(void);
static void dsp_RemoveDcFromSignal(void);
static void dsp_ConvertSignalToFloat(void);
static void dsp_DecimateSignal(void);
static void dsp_LowPassSignal(void);
static void dsp_BandPassSignal(void);
static void dsp_HighPassSignal(void);
static void dsp_InterpolateSignal(void);
static void dsp_ConvertSignalToUint16(void);

/* Public functions */

/**
 * Initializes dsp module
 */
void dsp_Init(void) {
  // init drivers
  adc_Init(&dsp_mod.adcBuffer[0], DSP_ADC_BUFFER_N_SAMPLES);
  // init filters
  dsp_InitFilters();
  // init module variables
  dsp_mod.nextBufferToProcess = ADC_BUFFER_NUMBER_0;
  dsp_mod.mode = DSP_MODE_BYPASS;
#warning ONLY FOR TEST !!!
  dsp_mod.mode = DSP_MODE_LOWPASS;
#warning ONLY FOR TEST !!!

#warning ONLY FOR TEST !!!
  int i;
  for (i = 0; i < (sizeof(filter_tests_signal) / sizeof(filter_tests_signal[0])); i++) {
    dsp_mod.signal_fs[i] = filter_tests_signal[i];
  }
#warning ONLY FOR TEST !!!
}

/**
 * Dsp module process
 */
void dsp_Process(void) {
  LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin); // visual feedabck
  if (adc_IsBufferFull(dsp_mod.nextBufferToProcess)) { // adc has filled up the buffer and is ready to be processed
    if (dsp_mod.mode == DSP_MODE_BYPASS) {
      dsp_BypassSignal();
    } else {
      // pre-filter
      dsp_RemoveDcFromSignal();
      dsp_ConvertSignalToFloat();
      dsp_DecimateSignal();
      // filter
      switch (dsp_mod.mode) {
      case DSP_MODE_LOWPASS:
        dsp_LowPassSignal();
        break;
      case DSP_MODE_BANDPASS:
        dsp_BandPassSignal();
        break;
      case DSP_MODE_HIGHPASS:
        dsp_HighPassSignal();
        break;
      default:
        //TODO assert false here
        break;
      }
      // post-filter
      dsp_InterpolateSignal();
      dsp_ConvertSignalToUint16();
    }
    // change to next buffer
    if (dsp_mod.nextBufferToProcess == ADC_BUFFER_NUMBER_0) {
      dsp_mod.nextBufferToProcess = ADC_BUFFER_NUMBER_1;
    } else {
      dsp_mod.nextBufferToProcess = ADC_BUFFER_NUMBER_0;
    }
  }
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin); // visual feedback
}

/* Private functions */

/**
 * Initializes all filters
 */
static void dsp_InitFilters(void) {
  dsp_mod.pfilter_dec = (filter_dec_t *)filter_dec_Ctor(FILTER_COEFFS_DEC_NTAPS, filter_coeffs_dec, DSP_BLOCK_FS_N_SAMPLES, DSP_DECIMATION_FACTOR);
}


static void dsp_BypassSignal(void) {
}

static void dsp_RemoveDcFromSignal(void) {
}

static void dsp_ConvertSignalToFloat(void) {
}

/**
 * Decimates the signal from a sampling frequency of 48KHz to 6KHz
 */
static void dsp_DecimateSignal(void) {
  filter_dec_Decimate(dsp_mod.pfilter_dec, dsp_mod.signal_fs, dsp_mod.signal_dec);
}

static void dsp_LowPassSignal(void) {
}
static void dsp_BandPassSignal(void) {
}
static void dsp_HighPassSignal(void) {
}
static void dsp_InterpolateSignal(void) {
}
static void dsp_ConvertSignalToUint16(void) {
}
