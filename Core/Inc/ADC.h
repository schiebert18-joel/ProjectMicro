/*
 * ADC.h
 *
 *  Created on: May 23, 2025
 *      Author: Schiebert Joel
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_


#define NUM_CHANNELS_ADC 8

/**
 * @brief estructura de datos para los sensores IR
 * @param currentValue: donde guardo los valores presentes
 * @param blackValue:   valor que corresponde a linea negra
 * @param whiteValue:   Valor que corresponde a valores white
 */
typedef struct{
	volatile uint32_t bufferADCvalue[NUM_CHANNELS_ADC]; /*!< Es volatile porque lo llena el DMA */
	uint16_t currentValue[NUM_CHANNELS_ADC];
}_sIrSensor;


#endif /* INC_ADC_H_ */
