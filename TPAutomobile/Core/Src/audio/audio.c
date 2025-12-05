/*
 * audio.c
 *
 *  Created on: Dec 5, 2025
 *      Author: mantoumben
 */
#include "audio/audio.h"
#include "stdbool.h"
#include "cmsis_os.h"

/* SGTL5000 AUDIO Codec */
#define AUDIO_BUFFER_SAMPLES 2048
#define AUDIO_BUFFER_BYTES (AUDIO_BUFFER_SAMPLES * 2)
#define AUDIO_SAMPLE_RATE 48000U


// --- SGTL5000 -----
static int16_t sai_tx_dma_buffer[AUDIO_BUFFER_BYTES] ;
volatile bool sai_tx_half_complete = false;
volatile bool sai_tx_full_complete = false;
//static int16_t sai_rx_dma_buffer[AUDIO_BUFFER_BYTES] ;
//volatile bool rx_half_ready = false;
//volatile bool rx_full_ready = false;

static uint32_t triangle_phase = 0;  // Persistent phase for perfect continuity

// ===  SAI DMA CallBacks

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_A)
    {
        sai_tx_half_complete = true;
    }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_A)
    {
        sai_tx_full_complete = true;
    }
}

//void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
//{
//	sai_rx_half_complete = true;
//}
//
//void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
//{
//	sai_rx_full_complete = true;
//}


// ====== Tache FreeRTOS

void audio_task(void *argument)
{
    const uint32_t half_buffer_stereo = AUDIO_BUFFER_SAMPLES / 2;

    // Start circular DMA transmission
    HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*)sai_tx_dma_buffer, AUDIO_BUFFER_BYTES);

    // Optional: fill first half immediately so we don't start with silence
    fill_triangle_wave_stereo(sai_tx_dma_buffer, half_buffer_stereo, 440.0f, 12000);

    for (;;)
    {
        if (sai_tx_half_complete || sai_tx_full_complete)
        {
            int16_t *buf_ptr;
            uint32_t samples_to_fill;

            if (sai_tx_half_complete)
            {
                sai_tx_half_complete = false;
                buf_ptr = sai_tx_dma_buffer;
                samples_to_fill = half_buffer_stereo;
            }
            else
            {
                sai_tx_full_complete = false;
                buf_ptr = sai_tx_dma_buffer + AUDIO_BUFFER_SAMPLES;  // Second half
                samples_to_fill = half_buffer_stereo;
            }

            // Generate 440 Hz triangle wave (A4)
            fill_triangle_wave_stereo(buf_ptr, samples_to_fill, 440.0f, 12000);  // ~36% amplitude → safe
        }

        // Small delay to avoid hogging CPU
        vTaskDelay(1);
    }
}

/**
 * @brief Génère une onde triangulaire stéréo dans le buffer TX
 * @param frequency_hz : fréquence du triangle (ex: 440 pour La)
 * @param sample_rate  : 48000 typiquement
 * @param amplitude    : 0 à 32767 (32767 = pleine échelle sans clip)
 */
void fill_triangle_wave_stereo(int16_t *buffer, uint32_t stereo_samples, float freq_hz, int16_t amplitude)
{
    uint32_t samples_per_period = (uint32_t)((float)AUDIO_SAMPLE_RATE / freq_hz);
    if (samples_per_period < 8) samples_per_period = 8;

    for (uint32_t i = 0; i < stereo_samples * 2; i += 2)
    {
        uint32_t pos = triangle_phase % samples_per_period;
        int32_t sample;

        if (pos < (samples_per_period >> 1))
        {
            // Rising edge: -amp → +amp
            sample = -amplitude + (int32_t)((4LL * amplitude * pos) / samples_per_period);
        }
        else
        {
            // Falling edge: +amp → -amp
            sample = amplitude - (int32_t)((4LL * amplitude * (pos - (samples_per_period >> 1))) / samples_per_period);
        }

        buffer[i]   = (int16_t)sample;  // Left
        buffer[i+1] = (int16_t)sample;  // Right (same for mono-like triangle)

        triangle_phase++;
    }
}


