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
#include "../shell/user_function.h"
#include "sgtl5000.h"
#include "../MCP23S17_/mcp23s17.h"
#include "audio/audio.h"
#include "RCFilter.h"

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_shell_t h_shell;
mcp23s17_handle_t mcp;
h_sgtl5000_t h_sgtl5000;

// FreeRTOS
SemaphoreHandle_t mutex;

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

/* ===== API =====*/

// BSP MCP23S17
void cs_low(void *d)   { HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, RESET); }
void cs_high(void *d)  { HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, SET); }
void delay_ms(uint32_t ms, void *d) { vTaskDelay(pdMS_TO_TICKS(ms)); }

void spi_send(const uint8_t *tx, uint8_t *rx, uint16_t len, void *d)
{
	HAL_SPI_Transmit(&hspi3, tx, len, 100);
}
void spi_recv(const uint8_t *tx, uint8_t *rx, uint16_t len, void *d)
{
	HAL_SPI_TransmitReceive(&hspi3, tx, rx, len, 100);
}

// BSP shell

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



void task_led(void * unused)
{
	for (;;)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		vTaskDelay(250);
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

	// --------- MCP23s17
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

	// --------- SGTL5000
	h_sgtl5000.hi2c = &hi2c2;
	h_sgtl5000.dev_address = SGTL5000_DEVADDRESS;
	sgtl5000_init(&h_sgtl5000);

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

	if (xTaskCreate(audio_task, "AUDIO", 1024, NULL, 3, NULL) != pdPASS)
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
