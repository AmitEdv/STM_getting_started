/*
 * logger.c
 *
 *  Created on: 27 αιεμι 2019
 *      Author: Amit
*/

#include "logger.h"

#define LOG_UART_TIMEOUT	5000

UART_HandleTypeDef * huart = NULL;

void Logger_Init(UART_HandleTypeDef * app_huart_for_logger)
{
	//TODO consider if the logger should  init and manage the UART it is using
	//instead of receiving it from the main
	huart = app_huart_for_logger;
}

void Logger_Send_Log(char * str, uint16_t str_len)
{
	if (huart == NULL)
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

	HAL_UART_Transmit(huart, (uint8_t*)str, str_len, LOG_UART_TIMEOUT);
}
