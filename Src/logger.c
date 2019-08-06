/*
 * logger.c
 *
 *  Created on: 27 αιεμι 2019
 *      Author: Amit
*/

#include <string.h>
#include "logger.h"

//comment out this in order to disable the logs
//can be used for debugging the logs module, and for release versions
//#define ENABLE_LOGS

#define bool uint8_t
#define true 1
#define false 0

#define LOG_UART_TIMEOUT	5000

UART_HandleTypeDef * p_huart = NULL;
bool is_uart_busy = false;

void Logger_Init(UART_HandleTypeDef * p_app_huart_for_logger)
{
	//TODO consider if the logger should  init and manage the UART it is using
	//instead of receiving it from the main
	p_huart = p_app_huart_for_logger;
}

void Logger_Send_Log(char * str, uint16_t str_len)
{
#ifdef ENABLE_LOGS
	if (p_huart == NULL)
	{
		//TODO - return num of bytes sent,
		//let the client know what failed
		return;
	}

	if (str_len > LOG_MAX_BUFFER_LENGTH)
	{
		//TODO - return num of bytes sent,
		//let the client know what failed
		return;
	}

	if (is_uart_busy)
	{
		//let the client know what failed
		return;
	}

	is_uart_busy = true;
	HAL_UART_Transmit(p_huart, (uint8_t*)str, str_len, LOG_UART_TIMEOUT);
	is_uart_busy = false; //Note! uart is in blocking mode at the moment
#else
	//do nothing
#endif
}
