/*
 * ADC.h
 *
 *  Created on: May 23, 2025
 *      Author: Schiebert Joel
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

/**
 * @brief estructura de datos para los sensores IR
 * @param currentValue: donde guardo los valores presentes
 * @param blackValue:   valor que corresponde a linea negra
 * @param whiteValue:   Valor que corresponde a valores white
 */
typedef struct{
	uint16_t currentValue;
	uint16_t blackValue;
	uint16_t whiteValue;
}_sIrSensor;


#endif /* INC_ADC_H_ */
