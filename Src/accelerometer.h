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

typedef enum acc_data_index_axis_e
{
	ACC_DATA_INDEX_AXIS_X = 0,
	ACC_DATA_INDEX_AXIS_Y,
	ACC_DATA_INDEX_AXIS_Z,

	ACC_DATA_NUM_OF_AXIS
}acc_data_index_axis_e;

void Accelerometer_Init(void);
void Accelerometer_Enable(void);
void Accelerometer_Disable(void);
void Accelerometer_Sensor_Read_Axis(int32_t* const o_axis_data);

#endif /* ACCELEROMETER_H_ */
