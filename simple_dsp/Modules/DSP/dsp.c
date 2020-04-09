/*
 * dsp.c
 *
 *  Created on: Mar 27, 2020
 *      Author: javi
 */


/* Includes */
#include "dsp.h"
#include "filter_coeffs.h"
#include "filter_fir.h"
#include "filter_decimation.h"
#include "filter_interpolation.h"
#include "filter_tests.h"
#include "../../HW/ADC/adc_driver.h"
#include "../../HW/DAC/dac_driver.h"
#include "../../HW/Sampling_Timer/sampling_timer.h"
#include "arm_math.h"


/* Private defines */
#define DSP_DECIMATION_FACTOR     (8)                         // decimation factor
#define DSP_INTERPOLATION_FACTOR  (DSP_DECIMATION_FACTOR)     // interpolation factor
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
  uint16_t dacBuffer[DSP_DAC_BUFFER_N_SAMPLES]; // dac data is written to this buffer
  // control
  buffer_half_t activeHalfBuffer;               // half buffer to be processed
  dsp_mode_t mode;                              // type of filter to apply
  // filters
  filter_dec_t *pfilter_dec;                    // points to decimation filter instance
  filter_fir_t *pfilter_lowpass;                // points to lowpass fir filter instance
  filter_fir_t *pfilter_bandpass;               // points to bandpass fir filter instance
  filter_fir_t *pfilter_highpass;               // points to highass fir filter instance
  filter_int_t *pfilter_int;                    // points to interpolation filter instance
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
  dac_Init(&dsp_mod.dacBuffer[0], DSP_DAC_BUFFER_N_SAMPLES);
  // init filters
  dsp_InitFilters();
  // init module variables
  dsp_mod.activeHalfBuffer = BUFFER_HALF_FIRST;
  dsp_mod.mode = DSP_MODE_BYPASS;
#warning ONLY FOR TEST !!!
//  int i;
//  for (i = 0; i < (sizeof(filter_tests_signal) / sizeof(filter_tests_signal[0])); i++) {
//    dsp_mod.signal_fs[i] = filter_tests_signal[i];
//  }
//  for (i = 0; i < (DSP_DAC_BUFFER_N_SAMPLES/2); i++) {
//    dsp_mod.adcBuffer[i] = 1535 + i;
//  }
//  for (i = 0; i < (DSP_DAC_BUFFER_N_SAMPLES/2); i++) {
//    dsp_mod.adcBuffer[(DSP_DAC_BUFFER_N_SAMPLES/2)+i] = 2559 - i;
//  }
#warning ONLY FOR TEST !!!
  // start all by enabling sampling timer
  sampling_timer_Start();
}

/**
 * Dsp module process
 */
/*TODO implement with a event driveb state machine.
 * Events can be created:
 * - Simple. A function before stm like ReadEvents() or GenerateEvents() with a lot of ifs checking for conditions. ex. if flagx==1 then return EVENT_FLAGX
 * - Complex. Event manager/handler were the servers will publish events and the clients will subscribe to them. when an event is published event manger will transmit it to all subscribed clients
 */
void dsp_Process(void) {
  if (adc_IsHalfBufferFree(dsp_mod.activeHalfBuffer)
      && dac_IsHalfBufferFree(dsp_mod.activeHalfBuffer)) {  // check adc and dac half buffers are ready to be processed
    LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin);             // for debugging purposes
    if (dsp_mod.mode == DSP_MODE_BYPASS) {
      //TODO check dac buffer ready else fail, what then?
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
    if (dsp_mod.activeHalfBuffer == BUFFER_HALF_FIRST) {
      dsp_mod.activeHalfBuffer = BUFFER_HALF_SECOND;
    } else {
      dsp_mod.activeHalfBuffer = BUFFER_HALF_FIRST;
    }
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin); // for debugging purposes
  }
}

/* Private functions */

/**
 * Initializes all filters
 */
static void dsp_InitFilters(void) {
  dsp_mod.pfilter_dec = (filter_dec_t *)filter_dec_Ctor(FILTER_COEFFS_DEC_NTAPS, filter_coeffs_dec, DSP_BLOCK_FS_N_SAMPLES, DSP_DECIMATION_FACTOR);
  dsp_mod.pfilter_lowpass = (filter_fir_t *)filter_fir_Ctor(FILTER_COEFFS_LOW_NTAPS, filter_coeffs_low, DSP_BLOCK_DEC_N_SAMPLES);
  dsp_mod.pfilter_bandpass = (filter_fir_t *)filter_fir_Ctor(FILTER_COEFFS_BAND_NTAPS, filter_coeffs_band, DSP_BLOCK_DEC_N_SAMPLES);
  dsp_mod.pfilter_highpass = (filter_fir_t *)filter_fir_Ctor(FILTER_COEFFS_HIGH_NTAPS, filter_coeffs_high, DSP_BLOCK_DEC_N_SAMPLES);
  dsp_mod.pfilter_int = (filter_int_t *)filter_int_Ctor(FILTER_COEFFS_INT_NTAPS, filter_coeffs_int, DSP_BLOCK_DEC_N_SAMPLES, DSP_INTERPOLATION_FACTOR);
}

/**
 * Copies the input directly to the output (ADC -> DAC)
 */
static void dsp_BypassSignal(void) {
  uint32_t i;
  uint32_t offset;
  // offset for active buffer
  if (dsp_mod.activeHalfBuffer == BUFFER_HALF_FIRST) {
    offset = 0;
  } else {
    offset = DSP_DAC_BUFFER_N_SAMPLES/2;
  }
  // copy buffer
  for (i = 0; i < (DSP_DAC_BUFFER_N_SAMPLES/2); i++) {
    dsp_mod.dacBuffer[i+offset] = dsp_mod.adcBuffer[i+offset];
  }
}

static void dsp_RemoveDcFromSignal(void) {
}

static void dsp_ConvertSignalToFloat(void) {
}

/**
 * Decimates the signal from a sampling frequency of 48KHz to 6KHz with cutoff frequency of 2.2KHz
 */
static void dsp_DecimateSignal(void) {
  filter_dec_Decimate(dsp_mod.pfilter_dec, dsp_mod.signal_fs, dsp_mod.signal_dec);
}

/**
 * Lowpass filters the signal cutoff frequency of 400Hz
 */
static void dsp_LowPassSignal(void) {
  filter_fir_Filter(dsp_mod.pfilter_lowpass, dsp_mod.signal_dec, dsp_mod.signal_filt);
}

/**
 * Bandpass filters the signal cutoff frequencies of 800Hz and 1200Hz
 */
static void dsp_BandPassSignal(void) {
  filter_fir_Filter(dsp_mod.pfilter_bandpass, dsp_mod.signal_dec, dsp_mod.signal_filt);
}

/**
 * Highpass filters the signal cutoff frequency of 1600Hz
 */
static void dsp_HighPassSignal(void) {
  filter_fir_Filter(dsp_mod.pfilter_highpass, dsp_mod.signal_dec, dsp_mod.signal_filt);
}

/**
 * Interpolates the signal from a sampling frequency of 6KHz to 48KHz with cutoff freq 2.2KHz
 */
static void dsp_InterpolateSignal(void) {
  filter_int_Interpolate(dsp_mod.pfilter_int, dsp_mod.signal_dec, dsp_mod.signal_fs);
}

static void dsp_ConvertSignalToUint16(void) {
}
