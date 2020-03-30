/*
 * filter.c
 *
 *  Created on: Mar 30, 2020
 *      Author: javi
 */


/* Includes */
#include "filter.h"
#include "../Memory/static_allocator.h"


// arm_fir_decimate_init_f32 (    &filter_dec, FILTER_DEC_NTAPS, FILTER_DEC_M, filter_dec_coeffs, filter_dec_state, FILTER_DEC_SRC_BLOCK_SIZE );
// arm_fir_init_f32 (             &filter_low, FILTER_LOW_NTAPS,               filter_low_coeffs, filter_low_state, FILTER_LOW_BLOCK_SIZE );
// arm_fir_interpolate_init_f32 ( &filter_int, FILTER_INT_NTAPS, FILTER_DEC_M, filter_int_coeffs, filter_int_state, FILTER_INT_BLOCK_SIZE );

/* Private variables */
struct filter_t{            // base filter structure
  uint16_t numTaps;         // number of coefficients in the filter
  const float *pCoeffs;     // points to the filter coefficients
  float *pState;            // points to the state buffer
  uint32_t blockSize;       // number of input samples to process per call
};


/* Public functions */

/**
 * Filter constructor
 * @param numTaps,    number of coefficients in the filter
 * @param pCoeffs,    points to the filter coefficients
 * @param blockSize,  number of input samples to process per call
 * @return pointer to filter instance
 */
filter_t *filter_Ctor(uint16_t numTaps, const float *pCoeffs, uint32_t blockSize){
  //TODO assert(numTaps)
  //TODO assert(pCoeffs)
  //TODO assert(blockSize)
  filter_t *filt = (filter_t *)stalloc_AllocAlignWord(sizeof(filter_t));
  //TODO assert(filt)
  filt->numTaps = numTaps;
  filt->pCoeffs = pCoeffs;
  filt->blockSize = blockSize;
  filt->pState = (float *)stalloc_AllocAlignWord(numTaps + blockSize - 1);
  //TODO assert(filt->pState)
  return filt;
}
