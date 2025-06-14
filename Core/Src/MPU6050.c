/*
 * MPU6050.c
 *
 *  Created on: Jun 5, 2025
 *      Author: Schiebert Joel
 */

#include "MPU6050.h"

//static uint8_t (*I2C_Transmit)(uint16_t DevAddress,uint16_t reg,uint8_t *pData, uint16_t Size);
//static uint8_t (*I2C_Recive)(uint16_t DevAddress,uint16_t reg,uint8_t *pData, uint16_t Size);
//
//void MPU6050_NonBlocking_DMA(uint8_t (*Master_Transmit)(uint16_t DevAddress,uint16_t reg,uint8_t *pData, uint16_t Size),
//		uint8_t (*Master_Recive)(uint16_t DevAddress,uint16_t reg,uint8_t *pData, uint16_t Size)){
//
//	I2C_Transmit = Master_Transmit;
//	I2C_Recive = Master_Recive;
//
//}
