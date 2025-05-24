/*
 * UNERprotocol.c
 *
 *  Created on: May 20, 2025
 *      Author: Schiebert Joel
 */

#include "usbd_cdc_if.h"
#include "UNERprotocol.h"
#include "Utilities.h"
#include "ADC.h"

_work w;
_sIrSensor irSensor[8];
/**
 * recibo la informacion enviada por puerto USB (lo enviado por QT), y guardo los bytes recibidos en el buffer circular Rx.buffercomm[] de la estructura datosComSerie
 * UNER = 55 4E 45 52 // Nbytes= 02 // ':' = 3A // Alive= F0 // 0xC4 = checksum
 */

void datafromUSB(uint8_t *buf, uint16_t length){

  uint16_t i;

  for (i = 0; i < length; i++) {
	datosComSerie.Rx.buffercomm[datosComSerie.Rx.indexWrite] = buf[i];
	datosComSerie.Rx.indexWrite++;
  }

}

void comunicationsTask(_sDato *datosCom){

	//si llegó informacion entonces llamo a decodeheader para el analisis del protocolo "UNER"
	if(datosCom->Rx.indexRead!=datosCom->Rx.indexWrite ){ //si Recepcion write =! Recepcion read => buffer lleno
		DecodeHeader(datosCom);
		datosComSerie.Rx.indexRead=datosComSerie.Rx.indexWrite;
	}

//	if(datosCom->Tx.indexWrite > datosCom->Tx.indexRead){
//		datosComSerie.bytesTosend = datosCom->Tx.indexWrite - datosCom->Tx.indexRead;
//	}else{
//		datosComSerie.bytesTosend = 256 - datosCom->Tx.indexRead;
//	}
//
//	if(CDC_Transmit_FS(&datosComSerie.Tx.buffercomm[datosComSerie.Tx.indexRead], datosComSerie.bytesTosend) == USBD_OK){
//		datosComSerie.Tx.indexRead += datosComSerie.bytesTosend;
//	}
}

/**
 * Máquina de estados que busca: 'U', 'N', 'E', 'R', nBytes, ':', Payload, Checksum
 * Si todo es válido, llama a: decodeData(datosCom);
 */
void DecodeHeader(_sDato *datosCom){ //Recibo un puntero a la estructura de comunicación que contiene los buffers y los índices

    static uint8_t nBytes=0;		//Variable estática para recordar cuántos bytes de payload quedan por procesar

    uint8_t indexWriteRxCopy = datosCom->Rx.indexWrite; //Guardo una copia del índice de escritura para no interferir con interrupciones

    while (datosCom->Rx.indexRead!=indexWriteRxCopy)
    {
        switch (estadoProtocolo) {					//Mientras haya nuevos datos no leídos en el buffer de recepción...
            case START:
                if (datosCom->Rx.buffercomm[datosCom->Rx.indexRead++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosCom->cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosCom->Rx.buffercomm[datosCom->Rx.indexRead++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosCom->Rx.indexRead--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosCom->Rx.buffercomm[datosCom->Rx.indexRead++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosCom->Rx.indexRead--;
                   estadoProtocolo=START;
                }
                break;
			case HEADER_3:
				if (datosCom->Rx.buffercomm[datosCom->Rx.indexRead++]=='R')
					estadoProtocolo=NBYTES;
				else{
					datosCom->Rx.indexRead--;
				    estadoProtocolo=START;
				}
            break;
            case NBYTES: //Leer byte de cantidad de datos (nBytes) y avanzar
                datosCom->indexStart=datosCom->Rx.indexRead;
                nBytes=datosCom->Rx.buffercomm[datosCom->Rx.indexRead++];
                estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosCom->Rx.buffercomm[datosCom->Rx.indexRead++]==':'){

                   estadoProtocolo=PAYLOAD;
                    datosCom->cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                }
                else{
                    datosCom->Rx.indexRead--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:

                if (nBytes>1){
                    datosCom->cheksumRx ^= datosCom->Rx.buffercomm[datosCom->Rx.indexRead++];
                }
                nBytes--;
                if(nBytes<=0){ //Cuando ya se leyeron todos los datos, compara el checksum. Si es correcto, llama a decodeData() para procesar el mensaje.
                    estadoProtocolo=START;
                    if(datosCom->cheksumRx == datosCom->Rx.buffercomm[datosCom->Rx.indexRead]){
                        decodeData(datosCom);
                    }
                }

                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
}

//si el protocolo fue valido => preparo respuestas
void decodeData(_sDato *datosCom){ //responde segun el ID recibido. Busca el ID del comando en la tercera posición del payload (después del token y del byte de longitud).


    uint8_t bufAux[20], indiceAux=0,bytes=0;

    switch (datosCom->Rx.buffercomm[datosCom->indexStart+2])//CMD EN LA POSICION 2, /ID EN LA POSICION 2, porque es donde se adjunta el byte que te dice "ALIVE, FIRMWARE, ETC"
    {

	case ALIVE:
		bufAux[indiceAux++] = ALIVE;   	// ID de respuesta
		bufAux[indiceAux++] = 0x0D;    	// Respuesta: ACK
		bytes = 0x03;        			// NBYTES = 3 (ID + Dato + Checksum)
	break;
    case FIRMWARE:

		bufAux[indiceAux++]=FIRMWARE;
		bytes=0x02;

    break;

    break;

    case IR:
		bufAux[indiceAux++] = IR;
			for (int i = 0; i < 3; i++) {
				w.u16[0] = irSensor[i].currentValue;
				bufAux[indiceAux++] = w.u8[0];
				bufAux[indiceAux++] = w.u8[1];
			}
		bytes = 1 + (3 * 2);  // ID + 3 valores de 2 bytes
		break;
    default:
        bufAux[indiceAux++]=0xFF;
        bytes=0x02;
    break;
    }

    SendInfo(bufAux,bytes);
}

//calculo y envio el checksum
void SendInfo(uint8_t bufferAux[], uint8_t bytes){

    uint8_t bufAux[20], indiceAux=0,cks=0,i=0;

    bufAux[indiceAux++]='U';
    bufAux[indiceAux++]='N';
    bufAux[indiceAux++]='E';
    bufAux[indiceAux++]='R';

    bufAux[indiceAux++]=bytes;
    bufAux[indiceAux++]=':';

    for(i=0; i<bytes-1; i++){
        bufAux[indiceAux++] = bufferAux[i];
    }

    cks	= 0;

    //Cargar en Tx.buffercomm con checksum:
    for(i=0 ;i<indiceAux;i++){
        cks^= bufAux[i];
        datosComSerie.Tx.buffercomm[datosComSerie.Tx.indexWrite++]=bufAux[i];
    }
    // Agregar el checksum al final
    datosComSerie.Tx.buffercomm[datosComSerie.Tx.indexWrite++]=cks;
    // Cantidad total de bytes a transmitir (incluyendo checksum)
    datosComSerie.bytesTosend = datosComSerie.Tx.indexWrite;

    // Paquete enviado hacia QT: 55 4E 45 52   01 	 3A     F0         0D          C8
    //							 'U''N''E''R''Nbytes'':''ID:Alive''Payload: ACK''Cheksum'
    CDC_Transmit_FS(&datosComSerie.Tx.buffercomm[datosComSerie.Tx.indexRead], datosComSerie.bytesTosend); //transmision por USB hacia QT
    datosComSerie.Tx.indexWrite = 0;

}
