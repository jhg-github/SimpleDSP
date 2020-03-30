/*
 * filter_decimation.c
 *
 *  Created on: Mar 30, 2020
 *      Author: javi
 */


/* Includes */
#include "filter.h"
#include "../Memory/static_allocator.h"
#include "arm_math.h"

// arm_fir_decimate_init_f32 (    &filter_dec, FILTER_DEC_NTAPS, FILTER_DEC_M, filter_dec_coeffs, filter_dec_state, FILTER_DEC_SRC_BLOCK_SIZE );
// arm_fir_init_f32 (             &filter_low, FILTER_LOW_NTAPS,               filter_low_coeffs, filter_low_state, FILTER_LOW_BLOCK_SIZE );
// arm_fir_interpolate_init_f32 ( &filter_int, FILTER_INT_NTAPS, FILTER_DEC_M, filter_int_coeffs, filter_int_state, FILTER_INT_BLOCK_SIZE );

/* Private variables */
struct filter_dec_t{        // decimation filter structure
  filter_t *pBase;          // points to base filter
  arm_fir_decimate_instance_f32 *pArmFirDecimateInstanceF32;  // points to cmsis-dsp decimator filter instance
  uint8_t decFactor;        // decimation factor
};
