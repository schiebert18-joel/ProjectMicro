/*
 * MPU6050.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Schiebert Joel
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stdint.h"
#include "stdlib.h"
// MPU6050 address
#define MPU6050_ADDR 0xD0 // 0x68 << 1

// MPU6050 registers
#define WHO_AM_I 				0x75
#define DLPF_CONFIG_REG			0x1A
#define POWER_MANAGEMENT_REG 	0x6B
#define GYRO_CONFIG_REG 		0x1B
#define GYRO_XOUT_REG 			0x43
#define ACCEL_CONFIG_REG 		0x1C
#define ACCEL_XOUT_REG 			0x3B

// Default Values
#define WHO_AM_I_DEFAULT_VALUE 	0x68

#define timeout_duration 		10
#define MemAddSize				1

typedef struct{
	uint16_t accel;
	uint16_t gyro;
	uint16_t offsetAccel;
	uint16_t offsetGyro;

	int32_t rawYaw;
	int16_t yaw; 		/*!< Rotacion (en °) respecto al eje Z */
}_sMPUData;

typedef struct{
	_sMPUData x;
	_sMPUData y;
	_sMPUData z;
}_sMPUxyz;



#endif /* INC_MPU6050_H_ */
