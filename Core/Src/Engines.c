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

/**
 * ARREGLAR
 */
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
		newspeed = engines->maxSpeed*-1; /*!< ARREGLAR (incongruencia linea 50)*/

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
		engines->estado = FRONT;
		engines->setPins(FRONT);
		engines->setPWM(0);
	}

}

//void en_HandlerENG(_sEng *engine1,_sEng *engine2, int32_t newspeed1,int32_t newspeed2,uint8_t freno){
//
//	if((engine1->setPins == NULL || engine1->setPWM == NULL) && (engine2->setPins == NULL || engine2->setPWM == NULL))
//		return;
//
//	if((newspeed1 == engine1->speed) && (newspeed2 == engine2->speed))
//		return;
//
////	if(freno == 1){
////		engines->estado = BRAKE;
////		engines->setPins(BACK);
////		return;
////	}
//
//	if(newspeed1 > engine1->maxSpeed)
//		newspeed1 = engine1->maxSpeed;
//	if(newspeed2 > engine2->maxSpeed)
//		newspeed2 = engine2->maxSpeed;
//
//	if(newspeed1 < engine1->maxSpeed*-1)
//		newspeed1 = engine1->maxSpeed*-1;
//	if(newspeed2 < engine2->maxSpeed*-1)
//			newspeed2 = engine2->maxSpeed*-1;
//
//	engine1->speed = newspeed1;
//	engine2->speed = newspeed2;
//
//	if(newspeed1 < 0){
//		engine1->estado = BACK;
//		engine1->setPins(BACK);
//		engine1->setPWM((uint16_t)(engine1->speed*-1));
//	}
//	if(newspeed2 < 0){
//		engine2->estado = BACK;
//		engine2->setPins(BACK);
//		engine2->setPWM((uint16_t)(engine2->speed*-1));
//	}
//
//	if(newspeed1 > 0){
//		engine1->estado = FRONT;
//		engine1->setPins(FRONT);
//		engine1->setPWM((uint16_t)engine1->speed);
//	}
//	if(newspeed2 > 0){
//		engine2->estado = FRONT;
//		engine2->setPins(FRONT);
//		engine2->setPWM((uint16_t)engine2->speed);
//	}
//
//	if(newspeed1==0){
//		engine1->estado = FREE;
//		engine1->setPins(FREE);
//		engine1->setPWM(0);
//	}
//
//	if(newspeed2==0){
//		engine2->estado = FREE;
//		engine2->setPins(FREE);
//		engine2->setPWM(0);
//	}
//
//
//}

