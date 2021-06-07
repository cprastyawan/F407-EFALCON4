/*
 * IMU.h
 *
 *  Created on: Jun 2, 2021
 *      Author: LENOVO
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "quaternionFilters.h"

#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

typedef struct {
	float yaw;
	float pitch;
	float roll;
}EulerAngles_t;

void getEulerAngles(EulerAngles_t *euler);
#endif /* INC_IMU_H_ */
