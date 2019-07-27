/*
 * accelerometer.c
 *
 *  Created on: 27 αιεμι 2019
 *      Author: Amit
 */
#include "accelerometer.h"
#include <iks01a2_motion_sensors.h>
#include <iks01a2_motion_sensors_ex.h>
#include <stdio.h>
#include "logger.h"

uint32_t AccInstance = IKS01A2_LSM6DSL_0;

void Accelerometer_Init(void)
{
  Logger_Send_Log("Accelerometer_Init\n\r", 21);
  int32_t bsp_status = IKS01A2_MOTION_SENSOR_Init(AccInstance, MOTION_ACCELERO);
  if (bsp_status != BSP_ERROR_NONE)
  {
    char log_tx_buffer[LOG_MAX_BUFFER_LENGTH];
    snprintf(log_tx_buffer, strlen(log_tx_buffer), "Error- acc init failed, err code %d\n\r", (int)bsp_status);
    Logger_Send_Log(log_tx_buffer, strlen(log_tx_buffer));
    //TODO - return status to client, handle failure in the client level.
    return;
  }
}

/**
 * @brief  Enable
 * @retval None
 */
void Accelerometer_Enable(void)
{
  Logger_Send_Log("Accelerometer_Enable\n\r", 23);
  int32_t bsp_status = -1;
  //int32_t bsp_status = IKS01A2_MOTION_SENSOR_Enable(AccInstance, MOTION_ACCELERO);
  if (bsp_status != BSP_ERROR_NONE)
  {
    char log_tx_buffer[LOG_MAX_BUFFER_LENGTH];
    snprintf(log_tx_buffer, strlen(log_tx_buffer), "Error- acc enable failed, err code %d\n\r", (int)bsp_status);
    Logger_Send_Log(log_tx_buffer, strlen(log_tx_buffer));
    //TODO - return status to client, handle failure in the client level.
    return;
  }
}

/**
 * @brief  Disable
 * @retval None
 */
void Accelerometer_Disable(void)
{
  (void)IKS01A2_MOTION_SENSOR_Disable(AccInstance, MOTION_ACCELERO);
}

/**
 * @brief  Handles the ACCELERO axes data getting
 * @param  o_axis_data output param to contain the received data.
 *         Note!! must be in size of (sizeof(int32_t) * NUM_OF_AXIS),
 *         meaning must be in size of (4 * 3) bytes.
 * @param  Instance the device instance
 * @retval None
 */
void Accelerometer_Sensor_Read_Axis(int32_t* const o_axis_data)
{
  Logger_Send_Log("Accelerometer_Sensor_Read_Axis\n\r", 33);
  IKS01A2_MOTION_SENSOR_Axes_t acceleration;
  uint8_t status = 0;

  if (o_axis_data == NULL)
  {
	  Logger_Send_Log("Error- pointer for data is null\n\r", 35);
	  //TODO - return status to client, handle failure in the client level.
	  return;
  }

  if ((IKS01A2_MOTION_SENSOR_Get_DRDY_Status(AccInstance, (uint32_t)MOTION_ACCELERO, &status) != BSP_ERROR_NONE)
	  || (status != ACC_STATUS_OK))
  {
	  Logger_Send_Log("Error- could not receive acc data\n\r", 36);
	  //TODO - return status to client, handle failure in the client level.
	  return;
  }

 (void)IKS01A2_MOTION_SENSOR_GetAxes(AccInstance, MOTION_ACCELERO, &acceleration);
  o_axis_data[ACC_DATA_INDEX_AXIS_X] = acceleration.x;
  o_axis_data[ACC_DATA_INDEX_AXIS_Y] = acceleration.y;
  o_axis_data[ACC_DATA_INDEX_AXIS_Z] = acceleration.z;

  //TODO - return status ok to client
}
