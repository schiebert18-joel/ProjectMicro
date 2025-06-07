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
#define WHO_AM_I 				0x75 	/*!<	Identificación del sensor							*/
#define DLPF_CONFIG_REG			0x1A	/*!<	Configura el divisor de la frecuencia de muestreo	*/
#define POWER_MANAGEMENT_REG 	0x6B	/*!< 	Control de energía									*/
#define GYRO_CONFIG_REG 		0x1B	/*!< 	Configura el rango del giroscopio					*/
#define ACCEL_CONFIG_REG 		0x1C	/*!< 	Configura el rango del acelerómetro					*/
#define ACCEL_XOUT_REG 			0x3B
#define GYRO_XOUT_REG 			0x43


// Default Values
#define WHO_AM_I_DEFAULT_VALUE 	0x68

#define ACCEL_SCALE_FACTOR 		16384
#define GYRO_SCALE_FACTOR 		131
#define timeout_duration 		10
#define MemAddSize				1

typedef struct{
	int16_t accel;
	int16_t gyro;
	int16_t offsetAccel;
	int16_t offsetGyro;

	int32_t rawYaw;
	int16_t yaw; 		/*!< Rotacion (en °) respecto al eje Z */
}_sMPUData;

typedef struct{
	_sMPUData x;
	_sMPUData y;
	_sMPUData z;

	uint8_t bufData[14];
}_sMPUxyz;



#endif /* INC_MPU6050_H_ */
