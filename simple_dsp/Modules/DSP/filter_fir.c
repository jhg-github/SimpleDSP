/*
 * filter_fir.c
 *
 *  Created on: Mar 31, 2020
 *      Author: javi
 */


/* Includes */
#include "filter_fir.h"
#include "../Memory/static_allocator.h"
#include "arm_math.h"


// arm_fir_decimate_init_f32 (    &filter_dec, FILTER_DEC_NTAPS, FILTER_DEC_M, filter_dec_coeffs, filter_dec_state, FILTER_DEC_SRC_BLOCK_SIZE );
// arm_fir_init_f32 (             &filter_low, FILTER_LOW_NTAPS,               filter_low_coeffs, filter_low_state, FILTER_LOW_BLOCK_SIZE );
// arm_fir_interpolate_init_f32 ( &filter_int, FILTER_INT_NTAPS, FILTER_DEC_M, filter_int_coeffs, filter_int_state, FILTER_INT_BLOCK_SIZE );

/* Private variables */
struct filter_fir_t{        // fir filter structure
  const float *pCoeffs;     // points to the filter coefficients
  uint32_t blockSize;       // number of input samples to process per call
  float *pState;            // points to the state buffer
  uint16_t numTaps;         // number of coefficients in the filter
  arm_fir_instance_f32 *pArmFirInstanceF32;  // points to a cmsis-dsp fir filter instance
};


/* Public functions */

/**
 * FIR filter constructor
 * @param numTaps, number of coefficients in the filter
 * @param pCoeffs, points to the filter coefficients
 * @param blockSize, number of input samples to process per call
 * @return pointer to fir filter instance
 */
filter_fir_t *filter_fir_Ctor(uint16_t numTaps, const float *pCoeffs, uint32_t blockSize){
  //TODO assert(numTaps)
  //TODO assert(pCoeffs)
  //TODO assert(blockSize)
  filter_fir_t *filt = (filter_fir_t *)stalloc_AllocAlignWord(sizeof(filter_fir_t));
  //TODO assert(filt)
  filt->numTaps = numTaps;
  filt->pCoeffs = pCoeffs;
  filt->blockSize = blockSize;
  filt->pState = (float *)stalloc_AllocAlignWord((filt->numTaps + filt->blockSize - 1)*sizeof(float));
  //TODO assert(filt->pState)
  filt->pArmFirInstanceF32 = (arm_fir_instance_f32 *)stalloc_AllocAlignWord(sizeof(arm_fir_instance_f32));
  //TODO assert(filt->pArmFirInstanceF32)
  arm_fir_init_f32 (filt->pArmFirInstanceF32, filt->numTaps, filt->pCoeffs, filt->pState, filt->blockSize );
  return filt;
}

/**
 * Filters signal using fir filter
 * @param this, fir filter instance
 * @param pSrc, points to the block of input data
 * @param pDst, points to the block of output data
 */
void filter_fir_Filter(filter_fir_t *this, const float *pSrc, float *pDst){
  //TODO assert(this)
  //TODO assert(pSrc)
  //TODO assert(pDst)
  arm_fir_f32(this->pArmFirInstanceF32, pSrc, pDst, this->blockSize);
}
