/*
 * audio.h
 *
 *  Created on: Dec 5, 2025
 *      Author: mantoumben
 */

#ifndef INC_AUDIO_AUDIO_H_
#define INC_AUDIO_AUDIO_H_

#include "stdint.h"
#include "sgtl5000.h"
#include "sai.h"

#define SGTL5000_DEVADDRESS 0x14
#define SGTL5000_ID_REG 0x00

extern h_sgtl5000_t h_sgtl5000;

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai);
void audio_task(void *argument);
void audio_process_task(void *argument);
void fill_triangle_wave_stereo(int16_t *buffer, uint32_t stereo_samples, float freq_hz, int16_t amplitude);

#endif /* INC_AUDIO_AUDIO_H_ */
