/*
 * ssd1306_oled.h
 *
 *  Created on: Jun 14, 2025
 *      Author: anton
 */

#ifndef INC_SSD1306_OLED_H_
#define INC_SSD1306_OLED_H_

#include "fonts.h"

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR	0x78 /*! Direccion del OLED por I2C (para el ssd1306*/
#endif

#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH		128 /*! Ancho de la pantalla 						*/
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT   	64	/*! Alto de la pantalla 						*/
#endif

#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define SSD1306_DEACTIVATE_SCROLL                    0x2E
#define SSD1306_ACTIVATE_SCROLL                      0x2F
#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3

#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7

#ifndef ssd1306_I2C_TIMEOUT
#define ssd1306_I2C_TIMEOUT		20000
#endif

typedef enum { 		/*! Defino los colores que puedo tener en cada pixel (blanco o negro) 	*/
	BLACK = 0x00,
	WHITE = 0x01
}_eDisplay_COLOR;

typedef enum{		/*! Estado para saber si se está enviando comandos o datos 	*/
	CMD,
	Data
}_eDMA_Status;

typedef struct {		/*! Estadp actual del display	*/
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
	uint8_t Page;
	_eDMA_Status DMA;
	uint8_t Commands[8];
	uint8_t Needtorefresh;
	uint8_t DMAREADY;
} _sSSD1306;

void Display_Set_I2C_Master_Transmit(
		uint8_t (*Master_Transmit)(uint16_t DevAddress,uint16_t reg,uint8_t *pData, uint16_t Size),
		uint8_t (*Master_Transmit_Blocking)(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout));

/**
 * @brief Inicializa el display OLED SSD1306 por I2C
 * 		  Configura todos los registros internos y limpia la pantalla
 */
void SSD1306_Init();

/**
 * @brief Llena toda la pantalla con el color especificado (blanco o negro)
 * @param Color: Black o White
 */
void SSD1306_Fill(_eDisplay_COLOR Color);

/**
 * @brief Envia el contenido del buffer interno al display
 * 		  Debe llamarse despues de modificar el buffer (ej. con Fill, puts, Draw)
 */
void SSD1306_UpdateScreen(void);

/**
 * @brief Establece la posicion del cursor donde se dibujará el proximo carácter.
 * @param x: Coordenada horizontal en píxeles
 * @param y: Coordenada vertical en píxeles
 */
void SSD1306_GotoXY(uint16_t x, uint16_t y);

/**
 * @brief Escribe un texto en el display con la fuente y color especificados
 * @param str: puntero al string a escribir
 * @param Font: puntero a la fuente
 * @param color: color del texto (white o black)
 */
char SSD1306_Puts(char* str, FontDef_t* Font, _eDisplay_COLOR color);

/**
 * @brief Dibuja un solo caracter en la posicion actual
 * @param ch: Caracter ASCII a mostrar
 * @param Font: puntero a la fuente
 * @param color: color del carácter
 * @return El caracter mostrado, o 0 si no hay espacio suficiente
 */
char SSD1306_Putc(char ch, FontDef_t* Font, _eDisplay_COLOR color);

/**
 * @brief Dibuja un solo pixel en la posicion (x,y) del buffer
 * @param x: coordenada horizontal
 * @param y: coordenada vertical
 * @param color: Black (borrar) o White (encender)
 */
void SSD1306_DrawPixel(uint16_t x, uint16_t y, _eDisplay_COLOR color);

#endif /* INC_SSD1306_OLED_H_ */
