/*
 * filter_interpolation.c
 *
 *  Created on: Mar 31, 2020
 *      Author: javi
 */


/* Includes */
#include "filter_interpolation.h"
#include "../Memory/static_allocator.h"
#include "arm_math.h"


// arm_fir_decimate_init_f32 (    &filter_dec, FILTER_DEC_NTAPS, FILTER_DEC_M, filter_dec_coeffs, filter_dec_state, FILTER_DEC_SRC_BLOCK_SIZE );
// arm_fir_init_f32 (             &filter_low, FILTER_LOW_NTAPS,               filter_low_coeffs, filter_low_state, FILTER_LOW_BLOCK_SIZE );
// arm_fir_interpolate_init_f32 ( &filter_int, FILTER_INT_NTAPS, FILTER_DEC_M, filter_int_coeffs, filter_int_state, FILTER_INT_BLOCK_SIZE );

/* Private variables */
struct filter_int_t{        // interpolation filter structure
  const float *pCoeffs;     // points to the filter coefficients
  uint32_t blockSize;       // number of input samples to process per call
  float *pState;            // points to the state buffer
  uint16_t numTaps;         // number of coefficients in the filter
  uint8_t intFactor;        // interpolation factor
  arm_fir_interpolate_instance_f32 *pArmFirInterpolateInstanceF32;  // points to a cmsis-dsp interpolator filter instance
};


/* Public functions */

/**
 * Interpolation filter constructor
 * @param numTaps, number of coefficients in the filter
 * @param pCoeffs, points to the filter coefficients
 * @param blockSize, number of input samples to process per call
 * @param intFactor, interpolation factor
 * @return pointer to interpolation filter instance
 */
filter_int_t *filter_int_Ctor(uint16_t numTaps, const float *pCoeffs, uint32_t blockSize, uint8_t intFactor){
  //TODO assert(numTaps)
  //TODO assert(pCoeffs)
  //TODO assert(blockSize)
  //TODO assert(intFactor)
  arm_status res;
  filter_int_t *filt = (filter_int_t *)stalloc_AllocAlignWord(sizeof(filter_int_t));
  //TODO assert(filt)
  filt->numTaps = numTaps;
  filt->pCoeffs = pCoeffs;
  filt->blockSize = blockSize;
  filt->intFactor = intFactor;
  filt->pState = (float *)stalloc_AllocAlignWord((filt->blockSize+(filt->numTaps/filt->intFactor)-1)*sizeof(float));
  //TODO assert(filt->pState)
  filt->pArmFirInterpolateInstanceF32 = (arm_fir_interpolate_instance_f32 *)stalloc_AllocAlignWord(sizeof(arm_fir_interpolate_instance_f32));
  //TODO assert(filt->pArmFirDecimateInstanceF32)
  res = arm_fir_interpolate_init_f32 (filt->pArmFirInterpolateInstanceF32, filt->intFactor, filt->numTaps, filt->pCoeffs, filt->pState, filt->blockSize );
  //TODO assert(res)
  return filt;
}

/**
 * Interpolates signal using interpolation filter
 * @param this, interpolation filter instance
 * @param pSrc, points to the block of input data
 * @param pDst, points to the block of output data
 */
void filter_int_Interpolate(filter_int_t *this, const float *pSrc, float *pDst){
  //TODO assert(this)
  //TODO assert(pSrc)
  //TODO assert(pDst)
  arm_fir_interpolate_f32(this->pArmFirInterpolateInstanceF32, pSrc, pDst, this->blockSize);
}
