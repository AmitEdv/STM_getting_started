/*
 * accelerometer.c
 *
 *  Created on: 27 αιεμι 2019
 *      Author: Amit
 */
#include <iks01a2_motion_sensors.h>

uint32_t AccInstance = IKS01A2_LSM6DSL_0;

void Accelerometer_Init(void)
{
  (void)IKS01A2_MOTION_SENSOR_Init(AccInstance, MOTION_ACCELERO);
}

