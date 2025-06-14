/*
 * ssd1306_oled.c
 *
 *  Created on: Jun 14, 2025
 *      Author: Schiebert Joel
 */

#include "ssd1306_oled.h"

#include <stdlib.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

///////////////////////////////////////////////////////////////////////////////////////////7
#define SSD1306_WRITECOMMAND(cmd)  do { \
    uint8_t b[2] = {0x00, (cmd)}; \
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, b, 2, HAL_MAX_DELAY); \
} while(0)

#define SSD1306_WRITEDATA(data) do { \
    uint8_t b[2] = {0x40, (data)}; \
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, b, 2, HAL_MAX_DELAY); \
} while(0)
///////////////////////////////////////////////////////////////////////////////////////////7

//#define SSD1306_WRITECOMMAND(command)	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, (command))
//#define SSD1306_WRITEDATA(data)      	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x40, (data))
#define ABS(x)   ((x) > 0 ? (x) : -(x))

static uint8_t (*I2C_DMA_Master_Transmit)(uint16_t DevAddress,uint16_t reg, uint8_t *pData, uint16_t Size);
//static uint8_t (*I2C_Master_Transmit_Blocking)(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static _sSSD1306 SSD1306;

void SSD1306_Init()
{
	SSD1306_WRITECOMMAND(0xAE);
	SSD1306_WRITECOMMAND(0x20);
	SSD1306_WRITECOMMAND(0x10);
	SSD1306_WRITECOMMAND(0xB0);
	SSD1306_WRITECOMMAND(0xC8);
	SSD1306_WRITECOMMAND(0x00);
	SSD1306_WRITECOMMAND(0x10);
	SSD1306_WRITECOMMAND(0x40);
	SSD1306_WRITECOMMAND(0x81);
	SSD1306_WRITECOMMAND(0xFF);
	SSD1306_WRITECOMMAND(0xA1);
	SSD1306_WRITECOMMAND(0xA6);
	SSD1306_WRITECOMMAND(0xA8);
	SSD1306_WRITECOMMAND(0x3F);
	SSD1306_WRITECOMMAND(0xA4);
	SSD1306_WRITECOMMAND(0xD3);
	SSD1306_WRITECOMMAND(0x00);
	SSD1306_WRITECOMMAND(0xD5);
	SSD1306_WRITECOMMAND(0xF0);
	SSD1306_WRITECOMMAND(0xD9);
	SSD1306_WRITECOMMAND(0x22);
	SSD1306_WRITECOMMAND(0xDA);
	SSD1306_WRITECOMMAND(0x12);
	SSD1306_WRITECOMMAND(0xDB);
	SSD1306_WRITECOMMAND(0x20);
	SSD1306_WRITECOMMAND(0x8D);
	SSD1306_WRITECOMMAND(0x14);
	SSD1306_WRITECOMMAND(0xAF);
	SSD1306_WRITECOMMAND(SSD1306_DEACTIVATE_SCROLL);

	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;
	SSD1306.Initialized = 1;
	SSD1306.Page=0;
	SSD1306.DMA = CMD;
	SSD1306.Needtorefresh=1;
	SSD1306.DMAREADY=0;
	SSD1306_Fill(BLACK);	/*! Limpio la pantalla		*/
	SSD1306_UpdateScreen(); /*! Actualizo la pantalla	*/

}

void SSD1306_Fill(_eDisplay_COLOR color)
{																					/*! Utilizo memset para poner todo el buffer en 0x00 (negro) o 0xFF (blanco)*/
	memset(SSD1306_Buffer, (color == BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer)); /*! No actualizo la pantalla, solo el buffer, por lo que necesito llamar a "UpdateScreen*/
}

void SSD1306_UpdateScreen(void)
{
	if(SSD1306.DMAREADY && SSD1306.Needtorefresh){
		switch(SSD1306.DMA){ /*! Para identificar si es envio de datos o comandos	*/
			case Data:
				if(I2C_DMA_Master_Transmit(SSD1306_I2C_ADDR,0x40, &SSD1306_Buffer[SSD1306_WIDTH*SSD1306.Page], SSD1306_WIDTH)==1){
					SSD1306.Page++;
					SSD1306.DMA=CMD;
					SSD1306.DMAREADY=0;
				}
				break;
			case CMD:

				SSD1306.Commands[0]=0xB0 + SSD1306.Page;
				SSD1306.Commands[1]=0x00;
				SSD1306.Commands[2]=0x10;

				if(I2C_DMA_Master_Transmit(SSD1306_I2C_ADDR,0x00, &SSD1306.Commands[0],3)==1){
					SSD1306.DMA=Data;
					SSD1306.DMAREADY=0;
				}
				break;

			default:
				SSD1306.Page=8;
				break;
			}
		if(SSD1306.Page>7){
			SSD1306.Page=0;
			SSD1306.Needtorefresh=0;
		}
	}
}

void SSD1306_GotoXY(uint16_t x, uint16_t y)
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

char SSD1306_Puts(char* str, FontDef_t* Font, _eDisplay_COLOR color)
{
	while(*str)
	{
		if(SSD1306_Putc(*str, Font, color) != *str){	/*! Llamo a putc para cada letra	*/
			return *str;
		}
		str++;
	}
	return *str;
}

char SSD1306_Putc(char ch, FontDef_t* Font, _eDisplay_COLOR color)
{
	uint32_t i, b, j;

	//Check si hay espeacio suficiente en pantalla para dibujar la letra
	if(SSD1306_WIDTH <= (SSD1306.CurrentX + Font->FontWidth) || SSD1306_HEIGHT <= (SSD1306.CurrentY + Font->FontHeight)){
		return 0;
	}

	for(i=0; i<Font->FontHeight; i++){
		b = Font->data[(ch - 32) * Font->FontHeight + i]; /*! con Font->data leo los bits del bitmap. Se utiliza ch -32 porque los caracteres en la fuente empiezan desde el ASCII 32*/
		for(j=0; j<Font->FontWidth; j++){
			if((b << j) & 0x8000){
				SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (_eDisplay_COLOR) color);
			}else{
				SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (_eDisplay_COLOR)!color);
			}
		}
	}
	SSD1306.CurrentX += Font->FontWidth; /*! Avanzo el cursor horizontalmente */
	return ch;
}

void SSD1306_DrawPixel(uint16_t x, uint16_t y, _eDisplay_COLOR color)
{
	if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT){ /*! Evito que se escriba fuera de la pantalla */
		return;
	}

	if(SSD1306.Inverted){					/*! Si el display está invertido, invierto los colores */
		color = (_eDisplay_COLOR)!color;
	}

	//el display organiza la pantalla en páginas de 8 pixeles verticales. Cada byte representa una columna vertical de 8 bits/pixeles
	if(color == WHITE){
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8); /*!	Modifico el ssd1306_buffer con 1 o 0	*/
	}else{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}
