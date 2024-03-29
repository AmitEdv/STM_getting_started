/*
 * accelerometer.c
 *
 *  Created on: 27 ����� 2019
 *      Author: Amit
 */
#include "accelerometer.h"
#include <iks01a3_motion_sensors.h>
#include <iks01a3_motion_sensors_ex.h>
#include <stdio.h>
#include <string.h>
#include "logger.h"

uint32_t AccInstance = IKS01A3_LSM6DSO_0;

void Accelerometer_Init(void)
{
  char log[LOG_MAX_BUFFER_LENGTH] = "Accelerometer_Init\n\r";
  Logger_Send_Log(log, strlen(log));

#warning - Acc code crashs. use below implementation once it does
//  int32_t bsp_status = IKS01A3_MOTION_SENSOR_Init(AccInstance, MOTION_ACCELERO);
//  if (bsp_status != BSP_ERROR_NONE)
//  {
//    char log_tx_buffer[LOG_MAX_BUFFER_LENGTH];
//    snprintf(log_tx_buffer, strlen(log_tx_buffer), "Error- acc init failed, err code %d\n\r", (int)bsp_status);
//    Logger_Send_Log(log_tx_buffer, strlen(log_tx_buffer));
//    //TODO - return status to client, handle failure in the client level.
//    return;
//  }
}

/**
 * @brief  Enable
 * @retval None
 */
void Accelerometer_Enable(void)
{
  char log[LOG_MAX_BUFFER_LENGTH] = "Accelerometer_Enable\n\r";
  Logger_Send_Log(log, strlen(log));

#warning - Acc code crashs. use below implementation once it does
//  int32_t bsp_status = IKS01A3_MOTION_SENSOR_Enable(AccInstance, MOTION_ACCELERO);
//  if (bsp_status != BSP_ERROR_NONE)
//  {
//    char log_tx_buffer[LOG_MAX_BUFFER_LENGTH];
//    snprintf(log_tx_buffer, strlen(log_tx_buffer), "Error- acc enable failed, err code %d\n\r", (int)bsp_status);
//    Logger_Send_Log(log_tx_buffer, strlen(log_tx_buffer));
//    //TODO - return status to client, handle failure in the client level.
//    return;
//  }
}

/**
 * @brief  Disable
 * @retval None
 */
void Accelerometer_Disable(void)
{
#warning - Acc code crashs. use below implementation once it does
	//  (void)IKS01A3_MOTION_SENSOR_Disable(AccInstance, MOTION_ACCELERO);
}

/**
 * @brief  Handles the ACCELERO axes data getting
 * @param  o_axis_data output param to contain the received data.
 *         Note!! must be in size of (sizeof(int32_t) * NUM_OF_AXIS),
 *         meaning must be in size of (4 * 3) bytes.
 * @param  Instance the device instance
 * @retval None
 */
void Accelerometer_Sensor_Read_Axis(Accelerometer_Axes_t* const o_axis_data)
{
  char log[LOG_MAX_BUFFER_LENGTH] = "Accelerometer_Sensor_Read_Axis\n\r";
  Logger_Send_Log(log, strlen(log));

  o_axis_data->axis_x_val = (int32_t)2;
  o_axis_data->axis_y_val = (int32_t)3;
  o_axis_data->axis_z_val = (int32_t)4;
  return;

#warning - Acc code crashs. use below implementation once it does
//  IKS01A3_MOTION_SENSOR_Axes_t acceleration;
//  uint8_t status = 0;
//
//  if (o_axis_data == NULL)
//  {
//    char log[LOG_MAX_BUFFER_LENGTH] = "Error- pointer for data is null\n\r";
//	  Logger_Send_Log(log, strlen(log));
//	  //TODO - return status to client, handle failure in the client level.
//	  return;
//  }
//
//  if ((IKS01A3_MOTION_SENSOR_Get_DRDY_Status(AccInstance, (uint32_t)MOTION_ACCELERO, &status) != BSP_ERROR_NONE)
//	  || (status != ACC_STATUS_OK))
//  {
//    char log[LOG_MAX_BUFFER_LENGTH] = "Error- could not receive acc data\n\r";
//	  Logger_Send_Log(log, strlen(log));
//	  //TODO - return status to client, handle failure in the client level.
//	  return;
//  }
//
// (void)IKS01A3_MOTION_SENSOR_GetAxes(AccInstance, MOTION_ACCELERO, &acceleration);
//  o_axis_data->axis_x_val = acceleration.x;
//  o_axis_data->axis_y_val = acceleration.y;
//  o_axis_data->axis_z_val = acceleration.z;
//
//  //TODO - return status ok to client
}
