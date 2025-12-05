/*
 * RCFilter.h
 *
 *  Created on: Dec 5, 2025
 *      Author: mbeng
 */

#ifndef SRC_RCFILTER_RCFILTER_H_
#define SRC_RCFILTER_RCFILTER_H_

#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
#include "stdint.h"
#define PI 3.14159
#define SAMPLING_FREQUENCY 48000U

typedef struct {
uint32_t coeff_A;
uint32_t coeff_B;
uint32_t coeff_D;
uint16_t out_prev;
} h_RC_filter_t;

extern h_RC_filter_t  h_RC_filter;

// Calcule les coefficients A, B et D
// Et les stocke dans la structure
void RC_filter_init(h_RC_filter_t * h_RC_filter, uint16_t cutoff_frequency);
// Implémente l'équation de récurrence
// Faites attention au type des différentes variables
uint16_t RC_filter_update(h_RC_filter_t * h_RC_filter, int16_t input);

void RC_buffer_stereo(int16_t *buffer, uint32_t stereo_samples);
#endif /* SRC_RCFILTER_RCFILTER_H_ */
