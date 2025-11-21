/*
 * shell.c
 *
 *  Created on: 7 juin 2019
 *      Author: Laurent Fiack
 */

#include "shell.h"

#include <stdio.h>

#include "usart.h"
#include "gpio.h"

static char uart_read(h_shell_t * h_shell) {
	char c;
//
//	//HAL_UART_Receive(&UART_DEVICE, (uint8_t*)(&c), 1, HAL_MAX_DELAY);
//	HAL_UART_Receive_IT(h_shell->huart, (uint8_t*)(&c), 1);
//
//	// bloquer la tache jusqu'à reception de caractère
//	xSemaphoreTake(h_shell->sem_uart_rx, portMAX_DELAY);
//
//	return c;
	h_shell->drv_shell.drv_shell_receive(&c, 1);

	return c;
}

static int uart_write(h_shell_t * h_shell, char * s, uint16_t size) {
//	HAL_UART_Transmit(h_shell->huart, (uint8_t*)s, size, HAL_MAX_DELAY);
//
//	return size;

	h_shell->drv_shell.drv_shell_transmit(s, size);
	return size;
}

static int sh_help(h_shell_t * h_shell, int argc, char ** argv) {
	int i;

	for(i = 0 ; i < h_shell->shell_func_list_size ; i++) {
		int size;
		size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "%c: %s\r\n", h_shell->shell_func_list[i].c, h_shell->shell_func_list[i].description);
		uart_write(h_shell, h_shell->print_buffer, size);
	}

	return 0;
}

void shell_init(h_shell_t * h_shell) {
	int size = 0;

	// Création du sémaphore
	h_shell->sem_uart_rx = xSemaphoreCreateBinary();

	h_shell->shell_func_list_size = 0;

	size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "\r\n\r\n===== Monsieur Shell v0.2 =====\r\n");
	uart_write(h_shell, h_shell->print_buffer, size);

	shell_add(h_shell, 'h', sh_help, "Help");
}

int shell_add(h_shell_t * h_shell, char c, int (* pfunc)(h_shell_t * h_shell, int argc, char ** argv), char * description) {
	if (h_shell->shell_func_list_size < SHELL_FUNC_LIST_MAX_SIZE) {
		h_shell->shell_func_list[h_shell->shell_func_list_size].c = c;
		h_shell->shell_func_list[h_shell->shell_func_list_size].func = pfunc;
		h_shell->shell_func_list[h_shell->shell_func_list_size].description = description;
		h_shell->shell_func_list_size++;
		return 0;
	}

	return -1;
}

static int shell_exec(h_shell_t * h_shell, char * buf) {
	int i;

	char c = buf[0];

	int argc;
	char * argv[ARGC_MAX];
	char *p;

	for(i = 0 ; i < h_shell->shell_func_list_size ; i++) {
		if (h_shell->shell_func_list[i].c == c) {
			argc = 1;
			argv[0] = buf;

			for(p = buf ; *p != '\0' && argc < ARGC_MAX ; p++){
				if(*p == ' ') {
					*p = '\0';
					argv[argc++] = p+1;
				}
			}

			return h_shell->shell_func_list[i].func(h_shell, argc, argv);
		}
	}

	int size;
	size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "%c: no such command\r\n", c);
	uart_write(h_shell, h_shell->print_buffer, size);
	return -1;
}

int shell_run(h_shell_t * h_shell) {
	int reading = 0;
	int pos = 0;

	while (1) {
		uart_write(h_shell, "> ", 2);
		reading = 1;

		while(reading) {
			char c = uart_read(h_shell);
			int size;

			switch (c) {
			//process RETURN key
			case '\r':
				//case '\n':
				size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "\r\n");
				uart_write(h_shell, h_shell->print_buffer, size);
				h_shell->cmd_buffer[pos++] = 0;     //add \0 char at end of string
				size = snprintf (h_shell->print_buffer, BUFFER_SIZE, ":%s\r\n", h_shell->cmd_buffer);
				uart_write(h_shell, h_shell->print_buffer, size);
				reading = 0;        //exit read loop
				pos = 0;            //reset buffer
				break;
				//backspace
			case '\b':
				if (pos > 0) {      //is there a char to delete?
					pos--;          //remove it in buffer

					uart_write(h_shell, "\b \b", 3);	// delete the char on the terminal
				}
				break;
				//other characters
			default:
				//only store characters if buffer has space
				if (pos < BUFFER_SIZE) {
					uart_write(h_shell, &c, 1);
					h_shell->cmd_buffer[pos++] = c; //store
				}
			}
		}
		shell_exec(h_shell, h_shell->cmd_buffer);
	}
	return 0;
}

void shell_uart_rx_callback(h_shell_t * h_shell)
{
	BaseType_t hptw;
	xSemaphoreGiveFromISR(h_shell->sem_uart_rx, &hptw);
	portYIELD_FROM_ISR(hptw);
}
