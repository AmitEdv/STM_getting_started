/*
 * logger.h
 *
 *  Created on: 27 αιεμι 2019
 *      Author: Amit
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "stm32l1xx_hal.h"

#define LOG_MAX_BUFFER_LENGTH	255

void Logger_Init(UART_HandleTypeDef * app_huart_for_logger);
void Logger_Send_Log(char * str, uint16_t str_len);

#endif /* LOGGER_H_ */
