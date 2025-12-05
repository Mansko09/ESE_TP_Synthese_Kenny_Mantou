/*
 * user_function.h
 *
 *  Created on: Dec 5, 2025
 *      Author: mantoumben
 */

#ifndef USER_FUNCTION_H_
#define USER_FUNCTION_H_

#include "shell.h"
#include "../MCP23S17_/mcp23s17.h"
#include "audio/RCFilter.h"


int fonction(h_shell_t * h_shell, int argc, char ** argv);
int addition(h_shell_t * h_shell, int argc, char ** argv);
int chenillard(h_shell_t * h_shell, int argc, char ** argv);
int shellModifierFreqCoupure(h_shell_t * h_shell, int argc, char ** argv);

#endif /* USER_FUNCTION_H_ */
