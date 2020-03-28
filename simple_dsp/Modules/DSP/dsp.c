/*
 * dsp.c
 *
 *  Created on: Mar 27, 2020
 *      Author: javi
 */

/* Includes */
#include "dsp.h"
#include "filter_coeffs.h"
#include "../../HW/ADC/adc_driver.h"
#include "arm_math.h"

/* Private defines */
#define DSP_DECIMATION_FACTOR     (8)                         // decimation factor
#define DSP_BLOCK_FS_N_SAMPLES    (1024)                      // number of samples per block of processing at sampling frequency
#define DSP_BLOCK_DEC_N_SAMPLES   (DSP_BLOCK_FS_N_SAMPLES/DSP_DECIMATION_FACTOR)  // number of samples per block of processing at decimated frequency
#define DSP_ADC_BUFFER_N_SAMPLES  (2*DSP_BLOCK_FS_N_SAMPLES)  // 2 times DSP_BLOCK_N_SAMPLES in order to process one block while the other is being written by the adc
#define DSP_DAC_BUFFER_N_SAMPLES  (2*DSP_BLOCK_FS_N_SAMPLES)  // 2 times DSP_BLOCK_N_SAMPLES in order to write one block while the other is being used by the dac

#define DSP_FILTER_DEC_STATE_SIZE (FILTER_COEFFS_DEC_NTAPS+DSP_BLOCK_FS_N_SAMPLES-1)

/* Private variables */
static struct dsp_mod_tag {                     // dsp module structure
  // buffers
  uint16_t adcBuffer[DSP_ADC_BUFFER_N_SAMPLES]; //TODO check if this should be volatile, look at ST library // adc data is written to this buffer
  float signal_fs[DSP_BLOCK_FS_N_SAMPLES]; // signal at sampling frequency. used before decimating and after interpolating */
  float signal_dec[DSP_BLOCK_DEC_N_SAMPLES];    // signal at decimated frequency
  float signal_filt[DSP_BLOCK_DEC_N_SAMPLES]; // signal filtered at decimated frequency
  uint16_t dacBuffer[DSP_ADC_BUFFER_N_SAMPLES]; // dac data is written to this buffer
  // control
  adc_buffer_number_t nextBufferToProcess;   // number of next buffer to process
  dsp_mode_t mode;                              // type of filter to apply
} dsp_mod;

static struct dsp_filter_dec_tag {                // decimation filter structure
  arm_fir_decimate_instance_f32 filter;
  float filter_dec_state[DSP_FILTER_DEC_STATE_SIZE]; // decimation filter state variable
} dsp_filter_dec;

/* Private function prototypes */
static void dsp_InitFilters(void);
static void dsp_InitFilterDecimation(void);
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
  dsp_InitFilterDecimation();
}

/**
 * Initializes decimation filter
 */
static void dsp_InitFilterDecimation(void) {
  arm_status res;
  res = arm_fir_decimate_init_f32(&dsp_filter_dec.filter,
  FILTER_COEFFS_DEC_NTAPS, DSP_DECIMATION_FACTOR, filter_coeffs_dec,
      dsp_filter_dec.filter_dec_state, DSP_BLOCK_FS_N_SAMPLES);
  //TODO assert (res == ARM_MATH_SUCCESS)
}

static void dsp_BypassSignal(void) {
}

static void dsp_RemoveDcFromSignal(void) {
}

static void dsp_ConvertSignalToFloat(void) {
}

//void TestDecimator(void) {
//#define FILTER_DEC_NTAPS            (51)
//#define FILTER_DEC_M                (8)
//#define FILTER_DEC_SRC_BLOCK_SIZE   (1024)
//#define FILTER_DEC_DEST_BLOCK_SIZE  (FILTER_DEC_SRC_BLOCK_SIZE/FILTER_DEC_M)
//#define FILTER_DEC_STATE_SIZE       (FILTER_DEC_NTAPS+FILTER_DEC_SRC_BLOCK_SIZE-1)
//
//  arm_status res;
//
//  arm_fir_decimate_instance_f32 filter_dec;
//  const float filter_dec_coeffs[FILTER_DEC_NTAPS * 10] ;
//  float filter_dec_state[FILTER_DEC_STATE_SIZE];
//  float src[FILTER_DEC_SRC_BLOCK_SIZE];
//  float dst[FILTER_DEC_DEST_BLOCK_SIZE];
//
//  res = arm_fir_decimate_init_f32(&filter_dec, FILTER_DEC_NTAPS, FILTER_DEC_M,
//      filter_dec_coeffs, filter_dec_state, FILTER_DEC_SRC_BLOCK_SIZE);
//  if (res != ARM_MATH_SUCCESS) {
//    while (1)
//      ; // Failed
//  }
//
//  while (1) {
//    LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin);
//    arm_fir_decimate_f32(&filter_dec, src, dst, FILTER_DEC_SRC_BLOCK_SIZE);
//    LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);
//    HAL_Delay(100);
//  }
//}

/**
 * Decimates the signal from a sampling frequency of 48KHz to 6KHz
 */
static void dsp_DecimateSignal(void) {
  int i;
  const float test_src[] = { 0.000000e+00f, 7.802302e-01f, 1.529767e+00f,
      2.219469e+00f, 2.823216e+00f, 3.319212e+00f, 3.691046e+00f, 3.928463e+00f,
      4.027801e+00f, 3.992066e+00f, 3.830649e+00f, 3.558705e+00f, 3.196227e+00f,
      2.766871e+00f, 2.296607e+00f, 1.812255e+00f, 1.340017e+00f, 9.040613e-01f,
      5.252563e-01f, 2.201129e-01f, 1.110223e-16f, -1.293307e-01f,
      -1.678864e-01f, -1.212370e-01f, -5.551115e-17f, 1.809618e-01f,
      4.035766e-01f, 6.477718e-01f, 8.927785e-01f, 1.118457e+00f, 1.306563e+00f,
      1.441880e+00f, 1.513153e+00f, 1.513773e+00f, 1.442166e+00f, 1.301879e+00f,
      1.101345e+00f, 8.533571e-01f, 5.742810e-01f, 2.830482e-01f, 1.720846e-15f,
      -2.543527e-01f, -4.605805e-01f, -6.015225e-01f, -6.633407e-01f,
      -6.363793e-01f, -5.157826e-01f, -3.018340e-01f, -5.551115e-16f,
      3.793228e-01f, 8.213398e-01f, 1.307639e+00f, 1.817170e+00f, 2.327372e+00f,
      2.815372e+00f, 3.259198e+00f, 3.638931e+00f, 3.937735e+00f, 4.142703e+00f,
      4.245482e+00f, 4.242641e+00f, 4.135765e+00f, 3.931278e+00f, 3.640005e+00f,
      3.276508e+00f, 2.858240e+00f, 2.404555e+00f, 1.935655e+00f, 1.471515e+00f,
      1.030859e+00f, 6.302362e-01f, 2.832538e-01f, 3.219647e-15f,
      -2.133152e-01f, -3.544876e-01f, -4.252150e-01f, -4.307785e-01f,
      -3.795325e-01f, -2.822441e-01f, -1.513239e-01f, -2.109424e-15f,
      1.585146e-01f, 3.118092e-01f, 4.489729e-01f, 5.611631e-01f, 6.420157e-01f,
      6.878779e-01f, 6.978587e-01f, 6.736990e-01f, 6.194804e-01f, 5.411961e-01f,
      4.462191e-01f, 3.427056e-01f, 2.389756e-01f, 1.429140e-01f, 6.142824e-02f,
      1.110223e-16f, -3.764508e-02f, -4.973020e-02f, -3.647504e-02f,
      3.330669e-15f, 5.590155e-02f, 1.261029e-01f, 2.045682e-01f, 2.848294e-01f,
      3.604799e-01f, 4.256509e-01f, 4.754355e-01f, 5.062326e-01f, 5.159876e-01f,
      5.043145e-01f, 4.724921e-01f, 4.233389e-01f, 3.609750e-01f, 2.904916e-01f,
      2.175527e-01f, 1.479585e-01f, 8.720316e-02f, 4.005835e-02f, 1.021377e-02f,
      0.000000e+00f, 1.021377e-02f, 4.005835e-02f, 8.720316e-02f, 1.479585e-01f,
      2.175527e-01f, 2.904916e-01f, 3.609750e-01f, 4.233389e-01f, 4.724921e-01f,
      5.043145e-01f, 5.159876e-01f, 5.062326e-01f, 4.754355e-01f, 4.256509e-01f,
      3.604799e-01f, 2.848294e-01f, 2.045682e-01f, 1.261029e-01f, 5.590155e-02f,
      -2.331468e-15f, -3.647504e-02f, -4.973020e-02f, -3.764508e-02f,
      -4.329870e-15f, 6.142824e-02f, 1.429140e-01f, 2.389756e-01f,
      3.427056e-01f, 4.462191e-01f, 5.411961e-01f, 6.194804e-01f, 6.736990e-01f,
      6.978587e-01f, 6.878779e-01f, 6.420157e-01f, 5.611631e-01f, 4.489729e-01f,
      3.118092e-01f, 1.585146e-01f, 4.218847e-15f, -1.513239e-01f,
      -2.822441e-01f, -3.795325e-01f, -4.307785e-01f, -4.252150e-01f,
      -3.544876e-01f, -2.133152e-01f, -2.220446e-15f, 2.832538e-01f,
      6.302362e-01f, 1.030859e+00f, 1.471515e+00f, 1.935655e+00f, 2.404555e+00f,
      2.858240e+00f, 3.276508e+00f, 3.640005e+00f, 3.931278e+00f, 4.135765e+00f,
      4.242641e+00f, 4.245482e+00f, 4.142703e+00f, 3.937735e+00f, 3.638931e+00f,
      3.259198e+00f, 2.815372e+00f, 2.327372e+00f, 1.817170e+00f, 1.307639e+00f,
      8.213398e-01f, 3.793228e-01f, 2.664535e-15f, -3.018340e-01f,
      -5.157826e-01f, -6.363793e-01f, -6.633407e-01f, -6.015225e-01f,
      -4.605805e-01f, -2.543527e-01f, -1.720846e-15f, 2.830482e-01f,
      5.742810e-01f, 8.533571e-01f, 1.101345e+00f, 1.301879e+00f, 1.442166e+00f,
      1.513773e+00f, 1.513153e+00f, 1.441880e+00f, 1.306563e+00f, 1.118457e+00f,
      8.927785e-01f, 6.477718e-01f, 4.035766e-01f, 1.809618e-01f, 2.609024e-15f,
      -1.212370e-01f, -1.678864e-01f, -1.293307e-01f, -3.330669e-16f,
      2.201129e-01f, 5.252563e-01f, 9.040613e-01f, 1.340017e+00f, 1.812255e+00f,
      2.296607e+00f, 2.766871e+00f, 3.196227e+00f, 3.558705e+00f, 3.830649e+00f,
      3.992066e+00f, 4.027801e+00f, 3.928463e+00f, 3.691046e+00f, 3.319212e+00f,
      2.823216e+00f, 2.219469e+00f, 1.529767e+00f, 7.802302e-01f, 3.795167e-15f,
      -7.802302e-01f, -1.529767e+00f, -2.219469e+00f, -2.823216e+00f,
      -3.319212e+00f, -3.691046e+00f, -3.928463e+00f, -4.027801e+00f,
      -3.992066e+00f, -3.830649e+00f, -3.558705e+00f, -3.196227e+00f,
      -2.766871e+00f, -2.296607e+00f, -1.812255e+00f, -1.340017e+00f,
      -9.040613e-01f, -5.252563e-01f, -2.201129e-01f, 1.443290e-15f,
      1.293307e-01f, 1.678864e-01f, 1.212370e-01f, 5.051515e-15f,
      -1.809618e-01f, -4.035766e-01f, -6.477718e-01f, -8.927785e-01f,
      -1.118457e+00f, -1.306563e+00f, -1.441880e+00f, -1.513153e+00f,
      -1.513773e+00f, -1.442166e+00f, -1.301879e+00f, -1.101345e+00f,
      -8.533571e-01f, -5.742810e-01f, -2.830482e-01f, -9.492407e-15f,
      2.543527e-01f, 4.605805e-01f, 6.015225e-01f, 6.633407e-01f, 6.363793e-01f,
      5.157826e-01f, 3.018340e-01f, -4.329870e-15f, -3.793228e-01f,
      -8.213398e-01f, -1.307639e+00f, -1.817170e+00f, -2.327372e+00f,
      -2.815372e+00f, -3.259198e+00f, -3.638931e+00f, -3.937735e+00f,
      -4.142703e+00f, -4.245482e+00f, -4.242641e+00f, -4.135765e+00f,
      -3.931278e+00f, -3.640005e+00f, -3.276508e+00f, -2.858240e+00f,
      -2.404555e+00f, -1.935655e+00f, -1.471515e+00f, -1.030859e+00f,
      -6.302362e-01f, -2.832538e-01f, -1.154632e-14f, 2.133152e-01f,
      3.544876e-01f, 4.252150e-01f, 4.307785e-01f, 3.795325e-01f, 2.822441e-01f,
      1.513239e-01f, 8.659740e-15f, -1.585146e-01f, -3.118092e-01f,
      -4.489729e-01f, -5.611631e-01f, -6.420157e-01f, -6.878779e-01f,
      -6.978587e-01f, -6.736990e-01f, -6.194804e-01f, -5.411961e-01f,
      -4.462191e-01f, -3.427056e-01f, -2.389756e-01f, -1.429140e-01f,
      -6.142824e-02f, -6.772360e-15f, 3.764508e-02f, 4.973020e-02f,
      3.647504e-02f, -6.938894e-15f, -5.590155e-02f, -1.261029e-01f,
      -2.045682e-01f, -2.848294e-01f, -3.604799e-01f, -4.256509e-01f,
      -4.754355e-01f, -5.062326e-01f, -5.159876e-01f, -5.043145e-01f,
      -4.724921e-01f, -4.233389e-01f, -3.609750e-01f, -2.904916e-01f,
      -2.175527e-01f, -1.479585e-01f, -8.720316e-02f, -4.005835e-02f,
      -1.021377e-02f, 0.000000e+00f, -1.021377e-02f, -4.005835e-02f,
      -8.720316e-02f, -1.479585e-01f, -2.175527e-01f, -2.904916e-01f,
      -3.609750e-01f, -4.233389e-01f, -4.724921e-01f, -5.043145e-01f,
      -5.159876e-01f, -5.062326e-01f, -4.754355e-01f, -4.256509e-01f,
      -3.604799e-01f, -2.848294e-01f, -2.045682e-01f, -1.261029e-01f,
      -5.590155e-02f, -5.551115e-15f, 3.647504e-02f, 4.973020e-02f,
      3.764508e-02f, 6.661338e-16f, -6.142824e-02f, -1.429140e-01f,
      -2.389756e-01f, -3.427056e-01f, -4.462191e-01f, -5.411961e-01f,
      -6.194804e-01f, -6.736990e-01f, -6.978587e-01f, -6.878779e-01f,
      -6.420157e-01f, -5.611631e-01f, -4.489729e-01f, -3.118092e-01f,
      -1.585146e-01f, 6.661338e-16f, 1.513239e-01f, 2.822441e-01f,
      3.795325e-01f, 4.307785e-01f, 4.252150e-01f, 3.544876e-01f, 2.133152e-01f,
      1.998401e-14f, -2.832538e-01f, -6.302362e-01f, -1.030859e+00f,
      -1.471515e+00f, -1.935655e+00f, -2.404555e+00f, -2.858240e+00f,
      -3.276508e+00f, -3.640005e+00f, -3.931278e+00f, -4.135765e+00f,
      -4.242641e+00f, -4.245482e+00f, -4.142703e+00f, -3.937735e+00f,
      -3.638931e+00f, -3.259198e+00f, -2.815372e+00f, -2.327372e+00f,
      -1.817170e+00f, -1.307639e+00f, -8.213398e-01f, -3.793228e-01f,
      -1.110223e-14f, 3.018340e-01f, 5.157826e-01f, 6.363793e-01f,
      6.633407e-01f, 6.015225e-01f, 4.605805e-01f, 2.543527e-01f, 1.720846e-15f,
      -2.830482e-01f, -5.742810e-01f, -8.533571e-01f, -1.101345e+00f,
      -1.301879e+00f, -1.442166e+00f, -1.513773e+00f, -1.513153e+00f,
      -1.441880e+00f, -1.306563e+00f, -1.118457e+00f, -8.927785e-01f,
      -6.477718e-01f, -4.035766e-01f, -1.809618e-01f, -7.216450e-15f,
      1.212370e-01f, 1.678864e-01f, 1.293307e-01f, 1.143530e-14f,
      -2.201129e-01f, -5.252563e-01f, -9.040613e-01f, -1.340017e+00f,
      -1.812255e+00f, -2.296607e+00f, -2.766871e+00f, -3.196227e+00f,
      -3.558705e+00f, -3.830649e+00f, -3.992066e+00f, -4.027801e+00f,
      -3.928463e+00f, -3.691046e+00f, -3.319212e+00f, -2.823216e+00f,
      -2.219469e+00f, -1.529767e+00f, -7.802302e-01f, -7.590334e-15f,
      7.802302e-01f, 1.529767e+00f, 2.219469e+00f, 2.823216e+00f, 3.319212e+00f,
      3.691046e+00f, 3.928463e+00f, 4.027801e+00f, 3.992066e+00f, 3.830649e+00f,
      3.558705e+00f, 3.196227e+00f, 2.766871e+00f, 2.296607e+00f, 1.812255e+00f,
      1.340017e+00f, 9.040613e-01f, 5.252563e-01f, 2.201129e-01f, 4.329870e-15f,
      -1.293307e-01f, -1.678864e-01f, -1.212370e-01f, -1.054712e-14f,
      1.809618e-01f, 4.035766e-01f, 6.477718e-01f, 8.927785e-01f, 1.118457e+00f,
      1.306563e+00f, 1.441880e+00f, 1.513153e+00f, 1.513773e+00f, 1.442166e+00f,
      1.301879e+00f, 1.101345e+00f, 8.533571e-01f, 5.742810e-01f, 2.830482e-01f,
      9.325873e-15f, -2.543527e-01f, -4.605805e-01f, -6.015225e-01f,
      -6.633407e-01f, -6.363793e-01f, -5.157826e-01f, -3.018340e-01f,
      1.332268e-15f, 3.793228e-01f, 8.213398e-01f, 1.307639e+00f, 1.817170e+00f,
      2.327372e+00f, 2.815372e+00f, 3.259198e+00f, 3.638931e+00f, 3.937735e+00f,
      4.142703e+00f, 4.245482e+00f, 4.242641e+00f, 4.135765e+00f, 3.931278e+00f,
      3.640005e+00f, 3.276508e+00f, 2.858240e+00f, 2.404555e+00f, 1.935655e+00f,
      1.471515e+00f, 1.030859e+00f, 6.302362e-01f, 2.832538e-01f, 2.331468e-14f,
      -2.133152e-01f, -3.544876e-01f, -4.252150e-01f, -4.307785e-01f,
      -3.795325e-01f, -2.822441e-01f, -1.513239e-01f, -9.992007e-15f,
      1.585146e-01f, 3.118092e-01f, 4.489729e-01f, 5.611631e-01f, 6.420157e-01f,
      6.878779e-01f, 6.978587e-01f, 6.736990e-01f, 6.194804e-01f, 5.411961e-01f,
      4.462191e-01f, 3.427056e-01f, 2.389756e-01f, 1.429140e-01f, 6.142824e-02f,
      1.720846e-14f, -3.764508e-02f, -4.973020e-02f, -3.647504e-02f,
      1.043610e-14f, 5.590155e-02f, 1.261029e-01f, 2.045682e-01f, 2.848294e-01f,
      3.604799e-01f, 4.256509e-01f, 4.754355e-01f, 5.062326e-01f, 5.159876e-01f,
      5.043145e-01f, 4.724921e-01f, 4.233389e-01f, 3.609750e-01f, 2.904916e-01f,
      2.175527e-01f, 1.479585e-01f, 8.720316e-02f, 4.005835e-02f, 1.021377e-02f,
      0.000000e+00f, 1.021377e-02f, 4.005835e-02f, 8.720316e-02f, 1.479585e-01f,
      2.175527e-01f, 2.904916e-01f, 3.609750e-01f, 4.233389e-01f, 4.724921e-01f,
      5.043145e-01f, 5.159876e-01f, 5.062326e-01f, 4.754355e-01f, 4.256509e-01f,
      3.604799e-01f, 2.848294e-01f, 2.045682e-01f, 1.261029e-01f, 5.590155e-02f,
      -1.504352e-14f, -3.647504e-02f, -4.973020e-02f, -3.764508e-02f,
      -4.440892e-15f, 6.142824e-02f, 1.429140e-01f, 2.389756e-01f,
      3.427056e-01f, 4.462191e-01f, 5.411961e-01f, 6.194804e-01f, 6.736990e-01f,
      6.978587e-01f, 6.878779e-01f, 6.420157e-01f, 5.611631e-01f, 4.489729e-01f,
      3.118092e-01f, 1.585146e-01f, 1.731948e-14f, -1.513239e-01f,
      -2.822441e-01f, -3.795325e-01f, -4.307785e-01f, -4.252150e-01f,
      -3.544876e-01f, -2.133152e-01f, -4.385381e-14f, 2.832538e-01f,
      6.302362e-01f, 1.030859e+00f, 1.471515e+00f, 1.935655e+00f, 2.404555e+00f,
      2.858240e+00f, 3.276508e+00f, 3.640005e+00f, 3.931278e+00f, 4.135765e+00f,
      4.242641e+00f, 4.245482e+00f, 4.142703e+00f, 3.937735e+00f, 3.638931e+00f,
      3.259198e+00f, 2.815372e+00f, 2.327372e+00f, 1.817170e+00f, 1.307639e+00f,
      8.213398e-01f, 3.793228e-01f, -1.032507e-14f, -3.018340e-01f,
      -5.157826e-01f, -6.363793e-01f, -6.633407e-01f, -6.015225e-01f,
      -4.605805e-01f, -2.543527e-01f, -1.337819e-14f, 2.830482e-01f,
      5.742810e-01f, 8.533571e-01f, 1.101345e+00f, 1.301879e+00f, 1.442166e+00f,
      1.513773e+00f, 1.513153e+00f, 1.441880e+00f, 1.306563e+00f, 1.118457e+00f,
      8.927785e-01f, 6.477718e-01f, 4.035766e-01f, 1.809618e-01f, 1.171285e-14f,
      -1.212370e-01f, -1.678864e-01f, -1.293307e-01f, 5.884182e-15f,
      2.201129e-01f, 5.252563e-01f, 9.040613e-01f, 1.340017e+00f, 1.812255e+00f,
      2.296607e+00f, 2.766871e+00f, 3.196227e+00f, 3.558705e+00f, 3.830649e+00f,
      3.992066e+00f, 4.027801e+00f, 3.928463e+00f, 3.691046e+00f, 3.319212e+00f,
      2.823216e+00f, 2.219469e+00f, 1.529767e+00f, 7.802302e-01f, 3.625450e-14f,
      -7.802302e-01f, -1.529767e+00f, -2.219469e+00f, -2.823216e+00f,
      -3.319212e+00f, -3.691046e+00f, -3.928463e+00f, -4.027801e+00f,
      -3.992066e+00f, -3.830649e+00f, -3.558705e+00f, -3.196227e+00f,
      -2.766871e+00f, -2.296607e+00f, -1.812255e+00f, -1.340017e+00f,
      -9.040613e-01f, -5.252563e-01f, -2.201129e-01f, -1.509903e-14f,
      1.293307e-01f, 1.678864e-01f, 1.212370e-01f, 1.326717e-14f,
      -1.809618e-01f, -4.035766e-01f, -6.477718e-01f, -8.927785e-01f,
      -1.118457e+00f, -1.306563e+00f, -1.441880e+00f, -1.513153e+00f,
      -1.513773e+00f, -1.442166e+00f, -1.301879e+00f, -1.101345e+00f,
      -8.533571e-01f, -5.742810e-01f, -2.830482e-01f, -4.496403e-14f,
      2.543527e-01f, 4.605805e-01f, 6.015225e-01f, 6.633407e-01f, 6.363793e-01f,
      5.157826e-01f, 3.018340e-01f, 1.032507e-14f, -3.793228e-01f,
      -8.213398e-01f, -1.307639e+00f, -1.817170e+00f, -2.327372e+00f,
      -2.815372e+00f, -3.259198e+00f, -3.638931e+00f, -3.937735e+00f,
      -4.142703e+00f, -4.245482e+00f, -4.242641e+00f, -4.135765e+00f,
      -3.931278e+00f, -3.640005e+00f, -3.276508e+00f, -2.858240e+00f,
      -2.404555e+00f, -1.935655e+00f, -1.471515e+00f, -1.030859e+00f,
      -6.302362e-01f, -2.832538e-01f, -2.631229e-14f, 2.133152e-01f,
      3.544876e-01f, 4.252150e-01f, 4.307785e-01f, 3.795325e-01f, 2.822441e-01f,
      1.513239e-01f, -1.332268e-15f, -1.585146e-01f, -3.118092e-01f,
      -4.489729e-01f, -5.611631e-01f, -6.420157e-01f, -6.878779e-01f,
      -6.978587e-01f, -6.736990e-01f, -6.194804e-01f, -5.411961e-01f,
      -4.462191e-01f, -3.427056e-01f, -2.389756e-01f, -1.429140e-01f,
      -6.142824e-02f, -9.658940e-15f, 3.764508e-02f, 4.973020e-02f,
      3.647504e-02f, -2.287059e-14f, -5.590155e-02f, -1.261029e-01f,
      -2.045682e-01f, -2.848294e-01f, -3.604799e-01f, -4.256509e-01f,
      -4.754355e-01f, -5.062326e-01f, -5.159876e-01f, -5.043145e-01f,
      -4.724921e-01f, -4.233389e-01f, -3.609750e-01f, -2.904916e-01f,
      -2.175527e-01f, -1.479585e-01f, -8.720316e-02f, -4.005835e-02f,
      -1.021377e-02f, 0.000000e+00f, -1.021377e-02f, -4.005835e-02f,
      -8.720316e-02f, -1.479585e-01f, -2.175527e-01f, -2.904916e-01f,
      -3.609750e-01f, -4.233389e-01f, -4.724921e-01f, -5.043145e-01f,
      -5.159876e-01f, -5.062326e-01f, -4.754355e-01f, -4.256509e-01f,
      -3.604799e-01f, -2.848294e-01f, -2.045682e-01f, -1.261029e-01f,
      -5.590155e-02f, 2.176037e-14f, 3.647504e-02f, 4.973020e-02f,
      3.764508e-02f, 1.232348e-14f, -6.142824e-02f, -1.429140e-01f,
      -2.389756e-01f, -3.427056e-01f, -4.462191e-01f, -5.411961e-01f,
      -6.194804e-01f, -6.736990e-01f, -6.978587e-01f, -6.878779e-01f,
      -6.420157e-01f, -5.611631e-01f, -4.489729e-01f, -3.118092e-01f,
      -1.585146e-01f, 5.218048e-15f, 1.513239e-01f, 2.822441e-01f,
      3.795325e-01f, 4.307785e-01f, 4.252150e-01f, 3.544876e-01f, 2.133152e-01f,
      3.719247e-14f, -2.832538e-01f, -6.302362e-01f, -1.030859e+00f,
      -1.471515e+00f, -1.935655e+00f, -2.404555e+00f, -2.858240e+00f,
      -3.276508e+00f, -3.640005e+00f, -3.931278e+00f, -4.135765e+00f,
      -4.242641e+00f, -4.245482e+00f, -4.142703e+00f, -3.937735e+00f,
      -3.638931e+00f, -3.259198e+00f, -2.815372e+00f, -2.327372e+00f,
      -1.817170e+00f, -1.307639e+00f, -8.213398e-01f, -3.793228e-01f,
      -2.886580e-15f, 3.018340e-01f, 5.157826e-01f, 6.363793e-01f,
      6.633407e-01f, 6.015225e-01f, 4.605805e-01f, 2.543527e-01f, 2.370326e-14f,
      -2.830482e-01f, -5.742810e-01f, -8.533571e-01f, -1.101345e+00f,
      -1.301879e+00f, -1.442166e+00f, -1.513773e+00f, -1.513153e+00f,
      -1.441880e+00f, -1.306563e+00f, -1.118457e+00f, -8.927785e-01f,
      -6.477718e-01f, -4.035766e-01f, -1.809618e-01f, -1.759703e-14f,
      1.212370e-01f, 1.678864e-01f, 1.293307e-01f, 4.884981e-14f,
      -2.201129e-01f, -5.252563e-01f, -9.040613e-01f, -1.340017e+00f,
      -1.812255e+00f, -2.296607e+00f, -2.766871e+00f, -3.196227e+00f,
      -3.558705e+00f, -3.830649e+00f, -3.992066e+00f, -4.027801e+00f,
      -3.928463e+00f, -3.691046e+00f, -3.319212e+00f, -2.823216e+00f,
      -2.219469e+00f, -1.529767e+00f, -7.802302e-01f, -1.518067e-14f,
      7.802302e-01f, 1.529767e+00f, 2.219469e+00f, 2.823216e+00f, 3.319212e+00f,
      3.691046e+00f, 3.928463e+00f, 4.027801e+00f, 3.992066e+00f, 3.830649e+00f,
      3.558705e+00f, 3.196227e+00f, 2.766871e+00f, 2.296607e+00f, 1.812255e+00f,
      1.340017e+00f, 9.040613e-01f, 5.252563e-01f, 2.201129e-01f, 1.953993e-14f,
      -1.293307e-01f, -1.678864e-01f, -1.212370e-01f, -1.953993e-14f,
      1.809618e-01f, 4.035766e-01f, 6.477718e-01f, 8.927785e-01f, 1.118457e+00f,
      1.306563e+00f, 1.441880e+00f, 1.513153e+00f, 1.513773e+00f, 1.442166e+00f,
      1.301879e+00f, 1.101345e+00f, 8.533571e-01f, 5.742810e-01f, 2.830482e-01f,
      4.451994e-14f, -2.543527e-01f, -4.605805e-01f, -6.015225e-01f,
      -6.633407e-01f, -6.363793e-01f, -5.157826e-01f, -3.018340e-01f,
      -1.454392e-14f, 3.793228e-01f, 8.213398e-01f, 1.307639e+00f,
      1.817170e+00f, 2.327372e+00f, 2.815372e+00f, 3.259198e+00f, 3.638931e+00f,
      3.937735e+00f, 4.142703e+00f, 4.245482e+00f, 4.242641e+00f, 4.135765e+00f,
      3.931278e+00f, 3.640005e+00f };
  for(i=0;i<(sizeof(test_src)/sizeof(test_src[0]));i++){
      dsp_mod.signal_fs[i] = test_src[i];
  }

  arm_fir_decimate_f32(&dsp_filter_dec.filter, &dsp_mod.signal_fs[0],
      &dsp_mod.signal_dec[0], DSP_BLOCK_FS_N_SAMPLES);
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
