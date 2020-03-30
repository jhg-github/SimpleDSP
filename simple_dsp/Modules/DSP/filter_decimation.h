/*
 * filter_decimation.h
 *
 *  Created on: Mar 30, 2020
 *      Author: javi
 */

#ifndef DSP_FILTER_DECIMATION_H_
#define DSP_FILTER_DECIMATION_H_


/* Includes */
#include "main.h"


/* Public typedef */
typedef struct filter_dec_t filter_dec_t;   // opaque declaration of decimation filter


/* Public functions */

/**
 * Decimation filter constructor
 * @param numTaps, number of coefficients in the filter
 * @param pCoeffs, points to the filter coefficients
 * @param blockSize, number of input samples to process per call
 * @param decFactor, decimation factor
 * @return pointer to decimation filter instance
 */
filter_dec_t *filter_dec_Ctor(uint16_t numTaps, const float *pCoeffs, uint32_t blockSize, uint8_t decFactor);

/**
 * Decimates signal using decimation filter
 * @param this, decimation filter instance
 * @param pSrc, points to the block of input data
 * @param pDst, points to the block of output data
 */
void filter_dec_Decimate(filter_dec_t *this, const float *pSrc, float *pDst);


#endif /* DSP_FILTER_DECIMATION_H_ */
