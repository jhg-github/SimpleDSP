/*
 * filter_interpolation.h
 *
 *  Created on: Mar 31, 2020
 *      Author: javi
 */

#ifndef DSP_FILTER_INTERPOLATION_H_
#define DSP_FILTER_INTERPOLATION_H_


/* Includes */
#include "main.h"


/* Public typedef */
typedef struct filter_int_t filter_int_t;   // opaque declaration of interpolation filter


/* Public functions */

/**
 * Interpolation filter constructor
 * @param numTaps, number of coefficients in the filter
 * @param pCoeffs, points to the filter coefficients
 * @param blockSize, number of input samples to process per call
 * @param intFactor, interpolation factor
 * @return pointer to interpolation filter instance
 */
filter_int_t *filter_int_Ctor(uint16_t numTaps, const float *pCoeffs, uint32_t blockSize, uint8_t intFactor);

/**
 * Interpolates signal using interpolation filter
 * @param this, interpolation filter instance
 * @param pSrc, points to the block of input data
 * @param pDst, points to the block of output data
 */
void filter_int_Interpolate(filter_int_t *this, const float *pSrc, float *pDst);


#endif /* DSP_FILTER_INTERPOLATION_H_ */
