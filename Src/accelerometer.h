/*
 * accelerometer.h
 *
 *  Created on: 27 αιεμι 2019
 *      Author: Amit
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include <stdio.h>


#define ACC_STATUS_OK	1U

typedef struct Accelerometer_Axes_t
{
	int32_t axis_x_val;
	int32_t axis_y_val;
	int32_t axis_z_val;
}Accelerometer_Axes_t;

void Accelerometer_Init(void);
void Accelerometer_Enable(void);
void Accelerometer_Disable(void);
void Accelerometer_Sensor_Read_Axis(Accelerometer_Axes_t* const o_axis_data);

#endif /* ACCELEROMETER_H_ */
