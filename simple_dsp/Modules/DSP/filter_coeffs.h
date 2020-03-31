/*
 * filter_coeffs.h
 *
 *  Created on: Mar 28, 2020
 *      Author: javi
 */

#ifndef DSP_FILTER_COEFFS_H_
#define DSP_FILTER_COEFFS_H_


/* Public defines */

#define FILTER_COEFFS_DEC_NTAPS (51)
#define FILTER_COEFFS_INT_NTAPS (56)


/* Public variables */

extern const float filter_coeffs_dec[FILTER_COEFFS_DEC_NTAPS];  // decimation filter
extern const float filter_coeffs_int[FILTER_COEFFS_INT_NTAPS];  // interpolation filter


#endif /* DSP_FILTER_COEFFS_H_ */
