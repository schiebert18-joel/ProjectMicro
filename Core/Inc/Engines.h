/*
 * Engines.h
 *
 *  Created on: May 26, 2025
 *      Author: Schiebert Joel
 */

#ifndef INC_ENGINES_H_
#define INC_ENGINES_H_

#include "stdint.h"

typedef enum{
	FRONT,		/*!< Motor en movimiento hacia adelante */
	BACK,		/*!< Motor en movimiento hacia atras */
	FREE,		/*!< Motor libre, sin movimiento */
	BRAKE		/*!< Motor frenado */
}_eEngState;

typedef struct{
	_eEngState estado;			/*!< Estado de direccion actual del motor */
	int32_t    speed;			/*!< Velocidad actual del motor */
	uint32_t   maxSpeed;		/*!< Valor maximo de velocidad permitida */
	void (*setPins)(_eEngState state); 	/*!< Puntero a función para establecer la dirección del motor */
	void (*setPWM)(uint16_t dCycle);	/*!< Puntero a función para establecer el ciclo de trabajo del PWM */
}_sEng;

/**
 * @brief Inicializa los parámetros del motor, incluyendo las funciones de control de pines y PWM.
 *
 * Esta función debe ser llamada para configurar un motor con su respectiva
 * función de control de pines y PWM, y el valor máximo de velocidad permitido.
 *
 * @param *engines: Puntero a la estructura del motor a inicializar.
 * @param PWM_set: Puntero a la función para establecer el ciclo de trabajo del PWM.
 * @param PIN_set: Puntero a la función para establecer la dirección del motor.
 * @param max_value: Valor máximo de velocidad permitida para el motor.
 */
void en_InitENG(_sEng *engines,void (*PWM_set)(uint16_t dCycle), void(*PIN_set)(_eEngState state), uint16_t max_Speed);

/**
 * @brief Ajusta la velocidad del motor utilizando un valor de 16 bits.
 *
 * La función establece la velocidad del motor en un rango de valores
 * entre maxValue y -maxValue, donde los valores negativos indican un estado de direccion de retroceso.
 *
 * @param *engines: Puntero a la estructura del motor.
 * @param newspeed: Velocidad deseada para el motor, en el rango de -maxValue a maxValue.
 * @param freno: Indico si es que el motor va a estar frenado o libre
 *
*/
void en_HandlerENG(_sEng *engines,int32_t newspeed,uint8_t freno);


#endif /* INC_ENGINES_H_ */
