/*
 * ADC.h
 *
 *  Created on: May 23, 2025
 *      Author: Schiebert Joel
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_


#define NUM_CHANNELS_ADC 9

typedef struct{
	uint16_t bufferADCvalue[NUM_CHANNELS_ADC];	/*! donde guardo los valores dados por el DMA*/
	uint16_t currentValue[NUM_CHANNELS_ADC];	/*! Guardo los valores en currentValue para transmitir a QT */
}_sIrSensor;


#endif /* INC_ADC_H_ */
