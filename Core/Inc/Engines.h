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
	FRONT,
	BACK,
	FREE,
	BRAKE
}_eEngState;

typedef struct{
	_eEngState estado;
	int32_t    speed;
	uint32_t   maxSpeed;
	void (*setPins)(_eEngState state);
	void (*setPWM)(uint16_t dCycle);
}_sEng;

void en_InitENG(_sEng *engines,void (*PWM_set)(uint16_t dCycle), void(*PIN_set)(_eEngState state), uint16_t max_Speed);
void en_HandlerENG(_sEng *engines,int32_t newspeed,uint8_t freno);


#endif /* INC_ENGINES_H_ */
