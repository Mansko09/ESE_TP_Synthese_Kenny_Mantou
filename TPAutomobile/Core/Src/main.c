/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
#include "shell.h"
#include "sgtl5000.h"
#include "../MCP23S17_/mcp23s17.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* MCP23S17 GPIO EXPANDER */
#define OPCODE 0x40
#define IODIRA_Reg 0x00
#define IODIRB_Reg 0x01
#define MODE_OUTPUT_MCP 0x00
#define GPIOA_VALUE 0x12
#define GPIOB_VALUE 0x13

/* SGTL5000 AUDIO Codec */
#define AUDIO_BUFFER_SIZE 4096
#define AUDIO_BUFFER_BYTES (AUDIO_BUFFER_SIZE * 2 * 2)
#define SGTL5000_DEVADDRESS 0x14
#define SGTL5000_ID_REG 0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

static int16_t sai_rx_dma_buffer[AUDIO_BUFFER_BYTES] ;
static int16_t sai_tx_dma_buffer[AUDIO_BUFFER_BYTES] ;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_shell_t h_shell;
mcp23s17_handle_t mcp;
h_sgtl5000_t h_sgtl5000;

// FreeRTOS
SemaphoreHandle_t mutex;


volatile bool tx_half_ready = false;
volatile bool tx_full_ready = false;
volatile bool rx_half_ready = false;
volatile bool rx_full_ready = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
	return chr;
}

uint8_t drv_uart_receive(char * pData, uint16_t size)
{
	HAL_UART_Receive(&huart2, (uint8_t*)pData, size, HAL_MAX_DELAY);
	return 0;
}

uint8_t drv_uart_transmit(char * pData, uint16_t size)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)pData, size, HAL_MAX_DELAY);
	return 0;
}

h_shell_t h_shell =
{
		.drv_shell = {
				.drv_shell_transmit = drv_uart_transmit,
				.drv_shell_receive = drv_uart_receive
		}
};

// Callbacks MCP23S17
static void cs_low(void *d)   { HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, RESET); }
static void cs_high(void *d)  { HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, SET); }
static void spi_send(const uint8_t *tx, uint8_t *rx, uint16_t len, void *d)
{
	HAL_SPI_Transmit(&hspi3, tx, len, 100);
}
static void spi_recv(const uint8_t *tx, uint8_t *rx, uint16_t len, void *d)
{
	HAL_SPI_TransmitReceive(&hspi3, tx, rx, len, 100);

}
static void delay_ms(uint32_t ms, void *d) { vTaskDelay(pdMS_TO_TICKS(ms)); }


void task_xpander(void * unused)
{
	mcp23s17_write_reg(&mcp, MCP23S17_IODIRA, 0x00);  // Port A tout sortie
	mcp23s17_write_reg(&mcp, MCP23S17_IODIRB, 0x00);  // Port B tout sortie
	vTaskDelay(1000);
	mcp23s17_write_reg(&mcp, MCP23S17_GPIOA,  0xFF);  // Port A allumé
	mcp23s17_write_reg(&mcp, MCP23S17_GPIOB,  0xFF);  // Port B allumé
	vTaskDelay(1000);
	for (;;){
		mcp23s17_digital_write(&mcp,  0, 1);  // GPA0 = 1
		mcp23s17_digital_write(&mcp,  8, 1);  // GPB0 = 1
		vTaskDelay(500);
		mcp23s17_digital_write(&mcp,  0, 0);  // GPA0 = 1
		mcp23s17_digital_write(&mcp,  8, 0);  // GPB0 = 1
		vTaskDelay(500);
	}
}

int fonction(h_shell_t * h_shell, int argc, char ** argv)
{
	printf("Je suis une fonction bidon\r\n");

	printf("argc = %d\r\n", argc);

	for (int i = 0 ; i < argc ; i++)
	{
		printf("argv[%d] = %s\r\n", i, argv[i]);
	}

	return 0;
}

int addition(h_shell_t * h_shell, int argc, char ** argv)
{
	if (argc != 3)
	{
		printf("Error: expected two arguments\r\n");
		return -1;
	}

	int a = atoi(argv[1]);
	int b = atoi(argv[2]);
	int c = a + b;

	printf("%d + %d = %d\r\n", a, b, c);

	return 0;
}

int chenillard(h_shell_t * h_shell, int argc, char ** argv)
{
	if (argc != 2)
	{
		printf("Error: expected one arguments\r\n");
		return -1;
	}
	int a = atoi(argv[1]);
	if (a <= 15){
		mcp23s17_SetAllOFF(&mcp);
		mcp23s17_SetLed(&mcp, a);
		printf("Led %d \r\n", a);
	}

	return 0;
}

void task_shell(void * unused)
{
	shell_init(&h_shell);
	shell_add(&h_shell, 'f', fonction, "Une fonction inutile");
	shell_add(&h_shell, 'a', addition, "Ma super addition");
	shell_add(&h_shell, 'c', chenillard, "Mes LEDS sont folles !");
	shell_run(&h_shell);

	// Une tâche ne doit *JAMAIS* retourner
	// Ici elle ne retourne pas parce qu'il y a une boucle infinie dans shell_run();
}

void audio_task(void *argument)
{
	// Démarre le SAI Tx (et Rx si tu veux)
	HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*)sai_tx_dma_buffer, AUDIO_BUFFER_BYTES);

	for(;;)
	{
		// Attend que la moitié ou la totalité du buffer soit libre
		if (tx_half_ready || tx_full_ready)
		{
			int16_t *buf_to_fill;
			uint32_t samples_to_fill;

			if (tx_half_ready)
			{
				tx_half_ready = false;
				buf_to_fill = sai_tx_dma_buffer;
				samples_to_fill = AUDIO_BUFFER_SAMPLES / 2;  // moitié du buffer
			}
			else
			{
				tx_full_ready = false;
				buf_to_fill = sai_tx_dma_buffer + AUDIO_BUFFER_SAMPLES;
				samples_to_fill = AUDIO_BUFFER_SAMPLES / 2;
			}

			// Génère un beau triangle à 440 Hz (La)
			fill_triangle_wave(buf_to_fill, samples_to_fill, 440.0f, 48000, 12000);
		}

		vTaskDelay(1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)	// LPUART1
	{
		// Caractère reçu : Donner le sémaphore pour débloquer task_shell
		shell_uart_rx_callback(&h_shell);
	}
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	tx_half_ready = true;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	tx_full_ready = true;
}
void task_led(void * unused)
{
	for (;;)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		vTaskDelay(250);
	}
}

/**
 * @brief Génère une onde triangulaire stéréo dans le buffer TX
 * @param frequency_hz : fréquence du triangle (ex: 440 pour La)
 * @param sample_rate  : 48000 typiquement
 * @param amplitude    : 0 à 32767 (32767 = pleine échelle sans clip)
 */
void fill_triangle_wave(int16_t *buffer, uint32_t buffer_samples_stereo,
		float frequency_hz, uint32_t sample_rate, int16_t amplitude)
{
	static uint32_t phase = 0;          // garde la phase entre les appels (continuité parfaite)
	uint32_t samples_per_period = (uint32_t)((float)sample_rate / frequency_hz);

	if (samples_per_period < 4) samples_per_period = 4;  // sécurité

	for (uint32_t i = 0; i < buffer_samples_stereo; i += 2)
	{
		// Position dans le cycle : 0 → samples_per_period-1
		uint32_t pos = phase % samples_per_period;

		// Triangle : monte de -amp à +amp puis redescend
		int32_t sample;
		if (pos < samples_per_period / 2)
		{
			// Phase montante : -amp → +amp
			sample = -amplitude + (int32_t)((4 * amplitude * pos) / samples_per_period);
		}
		else
		{
			// Phase descendante : +amp → -amp
			sample = amplitude - (int32_t)((4 * amplitude * (pos - samples_per_period/2)) / samples_per_period);
		}

		// Stéréo identique (tu peux faire gauche/droite différent si tu veux)
		buffer[i]   = (int16_t)sample;   // Gauche
		buffer[i+1] = (int16_t)sample;   // Droite

		phase++;
	}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	Error_Handler();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */


	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C2_Init();
	MX_SAI2_Init();
	MX_SPI3_Init();
	MX_USART2_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	/* USER CODE BEGIN 2 */
	__HAL_SAI_ENABLE(&hsai_BlockA2);
	/*handlers */
	mcp = (mcp23s17_handle_t){
		.addr_pins    = 0b000,
				.user_data    = NULL,
				.cs_low       = cs_low,
				.cs_high      = cs_high,
				.spi_receive = spi_recv,
				.spi_transmit = spi_send,
				.delay_ms     = delay_ms,
				.mutex        = mutex
	};
	mcp23s17_init(&mcp);


	h_sgtl5000.hi2c = &hi2c2;
	h_sgtl5000.dev_address = SGTL5000_DEVADDRESS;
	sgtl5000_init(&h_sgtl5000);

	//  HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t *) sai_tx_dma_buffer, AUDIO_BUFFER_BYTES);
	//	HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t *) sai_rx_dma_buffer, AUDIO_BUFFER_BYTES);

	//  if (xTaskCreate(task_xpander, "xpander", 256, NULL, 3, NULL) != pdPASS)
	//    	{
	//    		printf("Error creating task xpander\r\n");
	//    		Error_Handler();
	//    	}
	if (xTaskCreate(task_shell, "Shell", 512, NULL, 1, NULL) != pdPASS)
	{
		printf("Error creating task Shell\r\n");
		Error_Handler();
	}

	if (xTaskCreate(task_led, "LED", 128, NULL, 2, NULL) != pdPASS)
	{
		printf("Error creating task LED\r\n");
		Error_Handler();
	}

	if (xTaskCreate(audio_task, "AUDIO", 512, NULL, 3, NULL) != pdPASS)
	{
		printf("Error creating task LED\r\n");
		Error_Handler();
	}

	vTaskStartScheduler();
	/* USER CODE END 2 */

	/* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */



	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		//HAL_Delay(500);
		//printf("toogle %d \n\r",x++);


	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
	PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI2;
	PeriphClkInit.PLLSAI2.PLLSAI2Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI2.PLLSAI2M = 1;
	PeriphClkInit.PLLSAI2.PLLSAI2N = 26;
	PeriphClkInit.PLLSAI2.PLLSAI2P = RCC_PLLP_DIV17;
	PeriphClkInit.PLLSAI2.PLLSAI2R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
