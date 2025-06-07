/*
 * Engines.c
 *
 *  Created on: May 26, 2025
 *      Author: Schiebert Joel
 */

#include "Engines.h"
#include "stdlib.h"


void en_InitENG(_sEng *engines,void (*PWM_set)(uint16_t dCycle), void(*PIN_set)(_eEngState state), uint16_t max_Speed){

	engines->estado  = FREE;
	engines->setPins = PIN_set; /*!< seteo en setPins (de la estructura) en el motor correspondiente, la direccion de la funcion del main.c (MotorL_SetPIN) */
	engines->setPWM  = PWM_set;
	engines->speed	 = 0;
	engines->maxSpeed= max_Speed;

}


void en_HandlerENG(_sEng *engines, int32_t newspeed, uint8_t freno){

	if(engines->setPins == NULL || engines->setPWM == NULL)
		return;

	if(newspeed == engines->speed)
		return;

	if(freno == 1){
		engines->estado = BRAKE;
		engines->setPins(BACK);
		return;
	}

	if(newspeed > engines->maxSpeed)
		newspeed = engines->maxSpeed;

	if(newspeed < (engines->maxSpeed*-1))
		newspeed = engines->maxSpeed*-1;

	engines->speed = newspeed;

	if(newspeed < 0){
		engines->estado = BACK;
		engines->setPins(BACK);
		engines->setPWM((uint16_t)(engines->speed*-1));

	}else if(newspeed > 0){
		engines->estado = FRONT;
		engines->setPins(FRONT);
		engines->setPWM((uint16_t)engines->speed);
	}else if(newspeed==0){
		engines->estado = FREE;
		engines->setPins(FREE);
		engines->setPWM(0);
	}

}



