/*
 * UNERprotocol.c
 *
 *  Created on: May 20, 2025
 *      Author: Schiebert Joel
 */

#include "usbd_cdc_if.h"
#include "UNERprotocol.h"
#include "Utilities.h"
#include "stdlib.h"

_work w;
_sDato *datosComLib;
_eProtocolo estadoProtocolo;

void CommInitProtocol(_sDato *datosCom,uint8_t ringbuff){
		datosCom->Rx.buffercomm = malloc(ringbuff);
	    datosCom->Tx.buffercomm = malloc(ringbuff);
	    datosComLib=datosCom;
}

/**
 * @brief: 	recibo la informacion enviada por puerto USB (lo enviado por QT), y guardo los bytes recibidos
 *  		en el buffer circular Rx.buffercomm[] de la estructura datosComSerie. Es decir:
 * 			UNER = 55 4E 45 52 // Nbytes= 02 // ':' = 3A // Alive= F0 // 0xC4 = checksum
 *
 * @param: *buf, el buffer que posee la informacion para recibirla en el buffer de recepcion
 * @param: length, largo del buffer
 */

void CommDatafromUSB(uint8_t *buf, uint16_t length){

  uint16_t i;

  for (i = 0; i < length; i++) {
	  datosComLib->Rx.buffercomm[datosComLib->Rx.indexWrite] = buf[i];
	  datosComLib->Rx.indexWrite++;
  }

}

/**
 * @brief: 	Máquina de estados que busca: 'U', 'N', 'E', 'R', nBytes, ':', Payload, Checksum. Si todo es válido,
 * 		  	llama a: decodeData(datosCom);
 *
 *  @param: _sDato *datosComLib, estructura donde tengo toda la informacion
 */
void CommDecodeHeader(_sDato *datosComLib){

    static uint8_t nBytes=0;		/*!< Variable estática para recordar cuántos bytes de payload quedan por procesar */

    uint8_t indexWriteRxCopy = datosComLib->Rx.indexWrite; /*!< Guardo una copia del índice de escritura para no interferir con interrupciones */

    while (datosComLib->Rx.indexRead!=indexWriteRxCopy)
    {
        switch (estadoProtocolo) {	/*!< Mientras haya nuevos datos no leídos en el buffer de recepción... */
            case START:
                if (datosComLib->Rx.buffercomm[datosComLib->Rx.indexRead++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosComLib->cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosComLib->Rx.buffercomm[datosComLib->Rx.indexRead++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                	datosComLib->Rx.indexRead--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosComLib->Rx.buffercomm[datosComLib->Rx.indexRead++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                	datosComLib->Rx.indexRead--;
                   estadoProtocolo=START;
                }
                break;
			case HEADER_3:
				if (datosComLib->Rx.buffercomm[datosComLib->Rx.indexRead++]=='R')
					estadoProtocolo=NBYTES;
				else{
					datosComLib->Rx.indexRead--;
				    estadoProtocolo=START;
				}
            break;
            case NBYTES: /*!< Leer byte de cantidad de datos (nBytes) y avanzar */
            	datosComLib->indexStart=datosComLib->Rx.indexRead;
                nBytes=datosComLib->Rx.buffercomm[datosComLib->Rx.indexRead++];
                estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosComLib->Rx.buffercomm[datosComLib->Rx.indexRead++]==':'){

                   estadoProtocolo=PAYLOAD;
                   datosComLib->cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                }
                else{
                	datosComLib->Rx.indexRead--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:

                if (nBytes>1){
                	datosComLib->cheksumRx ^= datosComLib->Rx.buffercomm[datosComLib->Rx.indexRead++];
                }
                nBytes--;
                if(nBytes<=0){ /*!< Cuando ya se leyeron todos los datos, compara el checksum. Si es correcto, llama a decodeData() para procesar el mensaje. */
                    estadoProtocolo=START;
                    if(datosComLib->cheksumRx == datosComLib->Rx.buffercomm[datosComLib->Rx.indexRead]){
                        CommDecodeData(datosComLib);
                    }
                }

                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
}

/**
 * @brief responde segun el ID recibido.
 *
 */
void CommDecodeData(_sDato *datosComLib){
    uint8_t bufAux[20], indiceAux=0,bytes=0;

    switch (datosComLib->Rx.buffercomm[datosComLib->indexStart+2])/*!< CMD EN LA POSICION 2, /ID EN LA POSICION 2, porque es donde se adjunta el byte que te dice "ALIVE, FIRMWARE, ETC" */
    {

	case ALIVE:
		bufAux[indiceAux++] = ALIVE;   	/*!< ID de respuesta */
		bufAux[indiceAux++] = 0x0D;    	/*!< Respuesta: ACK */
		bytes = 0x03;        			/*!< NBYTES = 3 (ID + Dato + Checksum) */
	break;

    case FIRMWARE:
		bufAux[indiceAux++]=FIRMWARE;
		bytes=0x02;
    break;

    break;

    case IR:
		bufAux[indiceAux++] = IR;
		//w.u16[0] = IRsensor.bufferADCvalue[0];
		bufAux[indiceAux++] = w.u8[0];
		bufAux[indiceAux++] = w.u8[1];
		bytes = 3;
	break;

    default:
        bufAux[indiceAux++]=0xFF;
        bytes=0x02;
    break;
    }

    CommSendInfo(bufAux,bytes);
}

//calculo y envio el checksum
void CommSendInfo(uint8_t bufferAux[], uint8_t bytes){

    uint8_t bufAux[20], indiceAux=0,cks=0,i=0;

    bufAux[indiceAux++]='U';
    bufAux[indiceAux++]='N';
    bufAux[indiceAux++]='E';
    bufAux[indiceAux++]='R';

    bufAux[indiceAux++]=bytes;
    bufAux[indiceAux++]=':';

    for(i=0; i<bytes-1; i++)
        bufAux[indiceAux++] = bufferAux[i];


    cks	= 0;

    /*!< Cargar en Tx.buffercomm con checksum: */
    for(i=0 ;i<indiceAux;i++){
        cks^= bufAux[i];
        datosComLib->Tx.buffercomm[datosComLib->Tx.indexWrite++]=bufAux[i];
    }

     datosComLib->Tx.buffercomm[datosComLib->Tx.indexWrite++]=cks; /*!< Agregar el checksum al final */
     datosComLib->bytesTosend=indiceAux; /*!< Cantidad total de bytes a transmitir (incluyendo checksum)*/



//    CDC_Transmit_FS(&datosComSerie.Tx.buffercomm[datosComSerie.Tx.indexRead], datosComSerie.bytesTosend); //transmision por USB hacia QT
//    datosComSerie.Tx.indexWrite = 0;

}
