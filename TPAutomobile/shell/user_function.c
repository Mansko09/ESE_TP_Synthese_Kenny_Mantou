/*
 * user_function.c
 *
 *  Created on: Dec 5, 2025
 *      Author: mantoumben
 */
#include "user_function.h"
#include <stdio.h>
#include "stdlib.h"

extern mcp23s17_handle_t mcp;


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
