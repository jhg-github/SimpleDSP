/*
 * filter_fir.h
 *
 *  Created on: Mar 31, 2020
 *      Author: javi
 */

#ifndef DSP_FILTER_FIR_H_
#define DSP_FILTER_FIR_H_


/* Includes */
#include "main.h"


/* Public typedef */
typedef struct filter_fir_t filter_fir_t;   // opaque declaration of fir filter


/* Public functions */

/**
 * FIR filter constructor
 * @param numTaps, number of coefficients in the filter
 * @param pCoeffs, points to the filter coefficients
 * @param blockSize, number of input samples to process per call
 * @return pointer to fir filter instance
 */
filter_fir_t *filter_fir_Ctor(uint16_t numTaps, const float *pCoeffs, uint32_t blockSize);

/**
 * Filters signal using fir filter
 * @param this, fir filter instance
 * @param pSrc, points to the block of input data
 * @param pDst, points to the block of output data
 */
void filter_fir_Filter(filter_fir_t *this, const float *pSrc, float *pDst);


#endif /* DSP_FILTER_FIR_H_ */
