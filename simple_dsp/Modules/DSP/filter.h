/*
 * filter.h
 *
 *  Created on: Mar 30, 2020
 *      Author: javi
 */

#ifndef DSP_FILTER_H_
#define DSP_FILTER_H_


/* Includes */
#include "main.h"


/* Public typedef */
typedef struct filter_t filter_t;   // opaque declaration of filter


/* Public functions */

/**
 * Filter constructor
 * @param numTaps,    number of coefficients in the filter
 * @param pCoeffs,    points to the filter coefficients
 * @param blockSize,  number of input samples to process per call
 * @return pointer to filter instance
 */
filter_t *filter_Ctor(uint16_t numTaps, const float *pCoeffs, uint32_t blockSize);


#endif /* DSP_FILTER_H_ */
