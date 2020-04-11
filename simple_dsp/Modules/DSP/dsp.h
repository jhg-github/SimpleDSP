/*
 * dsp.h
 *
 *  Created on: Mar 27, 2020
 *      Author: javi
 */

#ifndef MODULES_DSP_DSP_H_
#define MODULES_DSP_DSP_H_


/* Includes */
#include "main.h"


/* Public enums */
typedef enum {
  DSP_MODE_BYPASS, DSP_MODE_LOWPASS, DSP_MODE_BANDPASS, DSP_MODE_HIGHPASS,
  // keep last
  DSP_MODE_SIZE,
} dsp_mode_t;


/* Public functions */

/**
 * Initializes dsp module
 */
void dsp_Init(void);

/**
 * Dsp module process
 */
void dsp_Process(void);

/**
 * Changes dsp mode
 * - DSP_MODE_BYPASS
 * - DSP_MODE_LOWPASS
 * - DSP_MODE_BANDPASS
 * - DSP_MODE_HIGHPASS
 * @param newMode dsp mode
 */
void dsp_ChangeDSPMode(dsp_mode_t newMode);


#endif /* MODULES_DSP_DSP_H_ */
