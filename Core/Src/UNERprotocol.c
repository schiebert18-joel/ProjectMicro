/*
 * UNERprotocol.c
 *
 *  Created on: May 20, 2025
 *      Author: Schiebert Joel
 */

#include "usbd_cdc_if.h"
#include "UNERprotocol.h"

/**
 * recibo la informacion enviada por puerto USB (lo enviado por QT), y guardo los bytes recibidos en el buffer circular bufferRx[] de la estructura datosComSerie
 * UNER = 55 4E 45 52 // Nbytes= 02 // ':' = 3A // Alive= F0 // 0xC4 = checksum
 */

void datafromUSB(uint8_t *buf, uint16_t length){

  uint16_t i;

  for (i = 0; i < length; i++) {
	datosComSerie.bufferRx[datosComSerie.indexWriteRx] = buf[i];
	datosComSerie.indexWriteRx++;
  }

}

void comunicationsTask(_sDato *datosCom){

	//si llegó informacion entonces llamo a decodeheader para el analisis del protocolo "UNER"
	if(datosCom->indexReadRx!=datosCom->indexWriteRx ){ //si Recepcion write =! Recepcion read => buffer lleno
		DecodeHeader(datosCom);
		datosComSerie.indexReadRx=datosComSerie.indexWriteRx;
	}

//	if(datosCom->indexWriteTx > datosCom->indexReadTx){
//		datosComSerie.bytesTosend = datosCom->indexWriteTx - datosCom->indexReadTx;
//	}else{
//		datosComSerie.bytesTosend = 256 - datosCom->indexReadTx;
//	}
//
//	if(CDC_Transmit_FS(&datosComSerie.bufferTx[datosComSerie.indexReadTx], datosComSerie.bytesTosend) == USBD_OK){
//		datosComSerie.indexReadTx += datosComSerie.bytesTosend;
//	}
}

/**
 * Máquina de estados que busca: 'U', 'N', 'E', 'R', nBytes, ':', Payload, Checksum
 * Si todo es válido, llama a: decodeData(datosCom);
 */
void DecodeHeader(_sDato *datosCom){ //Recibo un puntero a la estructura de comunicación que contiene los buffers y los índices

    static uint8_t nBytes=0;		//Variable estática para recordar cuántos bytes de payload quedan por procesar

    uint8_t indexWriteRxCopy=datosCom->indexWriteRx; //Guardo una copia del índice de escritura para no interferir con interrupciones

    while (datosCom->indexReadRx!=indexWriteRxCopy)
    {
        switch (estadoProtocolo) {					//Mientras haya nuevos datos no leídos en el buffer de recepción...
            case START:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosCom->cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosCom->indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
			case HEADER_3:
				if (datosCom->bufferRx[datosCom->indexReadRx++]=='R')
					estadoProtocolo=NBYTES;
				else{
					datosCom->indexReadRx--;
				    estadoProtocolo=START;
				}
            break;
            case NBYTES: //Leer byte de cantidad de datos (nBytes) y avanzar
                datosCom->indexStart=datosCom->indexReadRx;
                nBytes=datosCom->bufferRx[datosCom->indexReadRx++];
                estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosCom->bufferRx[datosCom->indexReadRx++]==':'){

                   estadoProtocolo=PAYLOAD;
                    datosCom->cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                }
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:

                if (nBytes>1){
                    datosCom->cheksumRx ^= datosCom->bufferRx[datosCom->indexReadRx++];
                }
                nBytes--;
                if(nBytes<=0){ //Cuando ya se leyeron todos los datos, compara el checksum. Si es correcto, llama a decodeData() para procesar el mensaje.
                    estadoProtocolo=START;
                    if(datosCom->cheksumRx == datosCom->bufferRx[datosCom->indexReadRx]){
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

    switch (datosCom->bufferRx[datosCom->indexStart+2])//CMD EN LA POSICION 2, /ID EN LA POSICION 2, porque es donde se adjunta el byte que te dice "ALIVE, FIRMWARE, ETC"
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

    case TEXT:

		bufAux[indiceAux++]=TEXT;
//		bytes=;

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

    //Cargar en bufferTx con checksum:
    for(i=0 ;i<indiceAux;i++){
        cks^= bufAux[i];
        datosComSerie.bufferTx[datosComSerie.indexWriteTx++]=bufAux[i];
    }
    // Agregar el checksum al final
    datosComSerie.bufferTx[datosComSerie.indexWriteTx++]=cks;
    // Cantidad total de bytes a transmitir (incluyendo checksum)
    datosComSerie.bytesTosend = datosComSerie.indexWriteTx;

    // Paquete enviado hacia QT: 55 4E 45 52   01 	 3A     F0         0D          C8
    //							 'U''N''E''R''Nbytes'':''ID:Alive''Payload: ACK''Cheksum'
    CDC_Transmit_FS(&datosComSerie.bufferTx[datosComSerie.indexReadTx], datosComSerie.bytesTosend); //transmision por USB hacia QT
    datosComSerie.indexWriteTx = 0;

}
