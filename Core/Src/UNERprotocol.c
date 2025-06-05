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

_eProtocolo estadoProtocolo;

/**
 * @brief:
 */
void CommInitProtocol(_sDato *datosCom, void (*Comm_DecodeData)(struct ComStruct *Comm_DataStruct), uint8_t *ringbuffRx, uint8_t *ringbuffTx){
		datosCom->Rx.buffercomm = ringbuffRx;
	    datosCom->Tx.buffercomm = ringbuffTx;
	    datosCom->CommDecodeData =  Comm_DecodeData;
}


/**
 * @brief: 	Máquina de estados que decodifica la informacion y
 * 			busca: 'U', 'N', 'E', 'R', nBytes, ':', Payload, Checksum.
 *
 *  @param: _sDato *datosCom, estructura donde tengo toda la informacion
 */
void CommDecodeHeader(_sDato *datosCom){

    static uint8_t nBytes=0;	/*!< Variable estática para recordar cuántos bytes de payload quedan por procesar */

    uint8_t indexWriteRxCopy = datosCom->Rx.indexWrite; /*!< Guardo una copia del índice de escritura para no interferir con interrupciones */

    while (datosCom->Rx.indexRead!=indexWriteRxCopy)
    {
        switch (estadoProtocolo) {	/*!< Mientras haya nuevos datos no leídos en el buffer de recepción... */
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
            case NBYTES: /*!< Leer byte de cantidad de datos (nBytes) y avanzar */
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
                if(nBytes<=0){ /*!< Cuando ya se leyeron todos los datos, compara el checksum. Si es correcto, llama a decodeData() para procesar el mensaje. */
                    estadoProtocolo=START;
                    if(datosCom->cheksumRx == datosCom->Rx.buffercomm[datosCom->Rx.indexRead]){
                    	if(datosCom->CommDecodeData != NULL)
                    		datosCom->CommDecodeData(datosCom);
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
 * @Brief: Calculo y envio el checksum
 */
void CommSendInfo(_sDato *datosCom ,uint8_t bufferAux[], uint8_t bytes){

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

    for(i=0 ;i<indiceAux;i++){    /*!< Cargar en Tx.buffercomm con checksum: */
        cks^= bufAux[i];
        datosCom->Tx.buffercomm[datosCom->Tx.indexWrite++]=bufAux[i];
    }

     datosCom->Tx.buffercomm[datosCom->Tx.indexWrite++]=cks;  /*!< Agregar el checksum al final */
     datosCom->bytesTosend=indiceAux; /*!< Cantidad total de bytes a transmitir (incluyendo checksum)*/


}
