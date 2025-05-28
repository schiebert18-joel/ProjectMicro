/*
 * UNERprotocol.h
 *
 *  Created on: May 20, 2025
 *      Author: Schiebert Joel
 */

#ifndef INC_UNERPROTOCOL_H_
#define INC_UNERPROTOCOL_H_

typedef struct{
	uint8_t indexWrite;			//!< Indice de escritura del buffer circular
	uint8_t indexRead;		 	//!< Indice de lectura del buffer circular
	uint8_t *buffercomm; 	//!< Buffer circular
}_sBus;

typedef struct ComStruct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t indexStart;      //!< Indice para saber en que parte del buffer circular arranca el ID
    uint8_t cheksumRx;       //!< Cheksumm RX
    _sBus	Tx;
    _sBus	Rx;
    uint8_t bytesTosend;	 //!< Cuantos bytes voy a trasnmitir
}_sDato;

typedef enum ProtocolState{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

typedef enum Comands{
    ALIVE=0xF0,
    FIRMWARE=0xF1,
    IR=0xA0,
//    LEDS=0x10,
//    PULSADORES=0x12,
//    SERVO=0xA2,
//    SCANNER=0xA8,
//    MOTOR=0xA1,
//    SPEED=0xA4,
//    SERVOCONFIG=0xA5,
//    BLACKIR=0xA6,
//    WHITEIR=0xA7,
//    ConstantError = 0xB0,
    ACKNOWLEDGE=0x0D,
    UNKNOWNCOMANND=0xFF
}_eEstadoMEFcmd;


void CommInitProtocol(_sDato *datosCom,uint8_t ringbuff);/*!<  */
void CommDatafromUSB(uint8_t *buf, uint16_t length);	 /*!< recibo la informacion enviada por puerto USB (lo enviado por QT), y guardo los bytes recibidos en el buffer circular bufferRx[] de la estructura datosComSerie */
void CommComunicationsTask(_sDato *datosCom);			 /*!< Verifico si llegó informacion */
void CommDecodeHeader(_sDato *datosComLib); 			 /*!< Recibo un puntero a la estructura de comunicación que contiene los buffers y los índices */
void CommDecodeData(_sDato *datosComLib);				 /*!< responde segun el ID recibido. Busca el ID del comando en la tercera posición del payload (después del token y del byte de longitud) */
void CommSendInfo(uint8_t bufferAux[], uint8_t bytes); 	 /*!< calculo y envio el checksum */



#endif /* INC_UNERPROTOCOL_H_ */
