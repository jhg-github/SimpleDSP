/*
 * filter_decimation.c
 *
 *  Created on: Mar 30, 2020
 *      Author: javi
 */


/* Includes */
#include "filter_decimation.h"
#include "../Memory/static_allocator.h"
#include "arm_math.h"

// arm_fir_decimate_init_f32 (    &filter_dec, FILTER_DEC_NTAPS, FILTER_DEC_M, filter_dec_coeffs, filter_dec_state, FILTER_DEC_SRC_BLOCK_SIZE );
// arm_fir_init_f32 (             &filter_low, FILTER_LOW_NTAPS,               filter_low_coeffs, filter_low_state, FILTER_LOW_BLOCK_SIZE );
// arm_fir_interpolate_init_f32 ( &filter_int, FILTER_INT_NTAPS, FILTER_DEC_M, filter_int_coeffs, filter_int_state, FILTER_INT_BLOCK_SIZE );

/* Private variables */
struct filter_dec_t{        // decimation filter structure
  uint16_t numTaps;         // number of coefficients in the filter
  const float *pCoeffs;     // points to the filter coefficients
  uint32_t blockSize;       // number of input samples to process per call
  float *pState;            // points to the state buffer
  uint8_t decFactor;        // decimation factor
  arm_fir_decimate_instance_f32 *pArmFirDecimateInstanceF32;  // points to a cmsis-dsp decimator filter instance
};


/* Public functions */

/**
 * Decimation filter constructor
 * @param numTaps, number of coefficients in the filter
 * @param pCoeffs, points to the filter coefficients
 * @param blockSize, number of input samples to process per call
 * @param decFactor, decimation factor
 * @return pointer to decimation filter instance
 */
filter_dec_t *filter_dec_Ctor(uint16_t numTaps, const float *pCoeffs, uint32_t blockSize, uint8_t decFactor){
  //TODO assert(numTaps)
  //TODO assert(pCoeffs)
  //TODO assert(blockSize)
  //TODO assert(decFactor)
  arm_status res;
  filter_dec_t *filt = (filter_dec_t *)stalloc_AllocAlignWord(sizeof(filter_dec_t));
  //TODO assert(filt)
  filt->numTaps = numTaps;
  filt->pCoeffs = pCoeffs;
  filt->blockSize = blockSize;
  filt->pState = (float *)stalloc_AllocAlignWord((numTaps + blockSize - 1)*sizeof(float));
  //TODO assert(filt->pState)
  filt->decFactor = decFactor;
  filt->pArmFirDecimateInstanceF32 = (arm_fir_decimate_instance_f32 *)stalloc_AllocAlignWord(sizeof(arm_fir_decimate_instance_f32));
  //TODO assert(filt->pArmFirDecimateInstanceF32)
  res = arm_fir_decimate_init_f32 (filt->pArmFirDecimateInstanceF32, numTaps, decFactor, pCoeffs, filt->pState, blockSize );
  //TODO assert(res)
  return filt;
}

/**
 * Decimates signal using decimation filter
 * @param this, decimation filter instance
 * @param pSrc, points to the block of input data
 * @param pDst, points to the block of output data
 */
void filter_dec_Decimate(filter_dec_t *this, const float *pSrc, float *pDst){
  //TODO assert(this)
  //TODO assert(pSrc)
  //TODO assert(pDst)
  arm_fir_decimate_f32(this->pArmFirDecimateInstanceF32, pSrc, pDst, this->blockSize);
}
