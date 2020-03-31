/*
 * filter_coeffs.h
 *
 *  Created on: Mar 28, 2020
 *      Author: javi
 */

#ifndef DSP_FILTER_COEFFS_H_
#define DSP_FILTER_COEFFS_H_


/* Public defines */

#define FILTER_COEFFS_DEC_NTAPS   (51)
#define FILTER_COEFFS_LOW_NTAPS   (51)
#define FILTER_COEFFS_BAND_NTAPS  (51)
#define FILTER_COEFFS_HIGH_NTAPS  (51)
#define FILTER_COEFFS_INT_NTAPS   (56)


/* Public variables */

extern const float filter_coeffs_dec[FILTER_COEFFS_DEC_NTAPS];    // decimation filter coefficients
extern const float filter_coeffs_low[FILTER_COEFFS_LOW_NTAPS];    // lowpass filter coefficients
extern const float filter_coeffs_band[FILTER_COEFFS_BAND_NTAPS];  // bandpass filter coefficients
extern const float filter_coeffs_high[FILTER_COEFFS_HIGH_NTAPS];  // highpass filter coefficients
extern const float filter_coeffs_int[FILTER_COEFFS_INT_NTAPS];    // interpolation filter coefficients


#endif /* DSP_FILTER_COEFFS_H_ */
