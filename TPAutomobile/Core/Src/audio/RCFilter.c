/*
 * RCFilter.c
 *
 *  Created on: Dec 5, 2025
 *      Author: mbeng
 */

#include "audio/RCFilter.h"

void RC_filter_init(h_RC_filter_t * h_RC_filter,
uint16_t cutoff_frequency){
	h_RC_filter->coeff_A=1;
	h_RC_filter->coeff_B=SAMPLING_FREQUENCY/(2*PI*cutoff_frequency);
	h_RC_filter->coeff_D=1 + (SAMPLING_FREQUENCY/(2*PI*cutoff_frequency));
	h_RC_filter->out_prev = 0;
}

uint16_t RC_filter_update(h_RC_filter_t * h_RC_filter, int16_t input){
	uint32_t B = h_RC_filter->coeff_B;
	uint32_t D=h_RC_filter->coeff_D;
	uint16_t prev=h_RC_filter->out_prev;

	uint32_t output;
	output= (input + B*prev)/D;

	h_RC_filter->out_prev=output;
	return (uint16_t)output;
}

void RC_buffer_stereo(int16_t *buffer, uint32_t stereo_samples){
	for (uint32_t i = 0; i < stereo_samples * 2; i += 2){
		buffer[i]   = (int16_t)RC_filter_update(&h_RC_filter,buffer[i]);  // Left
		buffer[i+1] = (int16_t)RC_filter_update(&h_RC_filter,buffer[i+1]);  // Right (same for mono-like triangle)b
	}
}
