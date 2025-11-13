/*
 * Driver_LED.h
 *
 *  Created on: Nov 13, 2025
 *      Author: mantoumben
 */

#ifndef SRC_DRIVER_LED_H_
#define SRC_DRIVER_LED_H_

#include "stm32l4xx_hal.h"

//Définition des registres MCP23S17 dans mcp23s17.

#define MCP23S17_WRITE_OPCODE 0x40

typedef struct{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef * cs_port;
	uint16_t cs_pin;

}Driver_LED_HandleTypeDef;

void Driver_LED_Init(Driver_LED_HandleTypeDef *dev);
void Driver_LED_SetLEDA(Driver_LED_HandleTypeDef* dev,uint8_t value);
void Driver_LED_SetLEDB(Driver_LED_HandleTypeDef* dev,uint8_t value);
uint8_t Driver_LED_ReadLEDA(Driver_LED_HandleTypeDef* dev);
uint8_t Driver_LED_ReadLEDB(Driver_LED_HandleTypeDef* dev);
#endif /* SRC_DRIVER_LED_H_ */
