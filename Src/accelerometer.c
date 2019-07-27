/*
 * accelerometer.c
 *
 *  Created on: 27 αιεμι 2019
 *      Author: Amit
 */
#include "accelerometer.h"
#include <iks01a2_motion_sensors.h>
#include <iks01a2_motion_sensors_ex.h>

uint32_t AccInstance = IKS01A2_LSM6DSL_0;

void Accelerometer_Init(void)
{
  (void)IKS01A2_MOTION_SENSOR_Init(AccInstance, MOTION_ACCELERO);
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
  IKS01A2_MOTION_SENSOR_Axes_t acceleration;
  uint8_t status = 0;

  if (o_axis_data == NULL)
  {
	  return;
  }

  if ((IKS01A2_MOTION_SENSOR_Get_DRDY_Status(AccInstance, (uint32_t)MOTION_ACCELERO, &status) != BSP_ERROR_NONE)
	  || (status != ACC_STATUS_OK))
  {
	  //Error! could not receive acc data
	  //TODO - print log, return status to client, handle failure in the client level.
	  return;
  }

 (void)IKS01A2_MOTION_SENSOR_GetAxes(AccInstance, MOTION_ACCELERO, &acceleration);
  o_axis_data[ACC_DATA_INDEX_AXIS_X] = acceleration.x;
  o_axis_data[ACC_DATA_INDEX_AXIS_Y] = acceleration.y;
  o_axis_data[ACC_DATA_INDEX_AXIS_Z] = acceleration.z;

  //TODO - return status ok to client
}
