/*
 * Engines.c
 *
 *  Created on: May 26, 2025
 *      Author: Schiebert Joel
 */

#include "Engines.h"
#include "stdlib.h"

_sEng engine;


void en_InitENG(_sEng *engines,uint16_t maxSpeed){

	engines->estado=FREE;
	engines->speed=0;
	engines->maxSpeed=maxSpeed;

}

void en_HandlerENG(_sEng *engines,int32_t newspeed,uint8_t freno){

	if(newspeed==engines->speed)
		return;

	if(freno==1){
		engines->estado=BRAKE;
		return;
	}
	if(newspeed>engines->maxSpeed)
		newspeed=engines->maxSpeed;

	//agregar el caso anterior para negativo
	if(newspeed<0){
		engines->estado=BACK;
		engines->speed= -1 * (newspeed);
	}else if(newspeed>0){
		engines->estado=FRONT;
		engines->speed=newspeed;
	}else if(newspeed==0){
		engines->estado=FREE;
		engines->speed=0;
	}

}

