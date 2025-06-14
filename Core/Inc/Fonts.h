/*
 * Fonts.h
 *
 *  Created on: Jun 14, 2025
 *      Author: Schiebert Joel
 */

#ifndef INC_FONTS_H_
#define INC_FONTS_H_

#include "main.h"

typedef struct { /*! Ancho y alto en pixeles de cada letra */
	uint8_t FontWidth;
	uint8_t FontHeight;
	const uint16_t *data; /*!Puntero al arreglo que contiene los datos binarios (bitmap) */
} FontDef_t;

typedef struct { /*!Estructura para guardar el tamaño total (en pixeles) de un string completo */
	uint16_t Length;
	uint16_t Height;
} FONTS_SIZE_t;

extern FontDef_t Font_7x10;
extern FontDef_t Font_11x18;
extern FontDef_t Font_16x26;

/**
 * @brief Calculo cuanto espacio ocupará un string en pantalla
 */
char* FONTS_GetStringSize(char* str, FONTS_SIZE_t* SizeStruct, FontDef_t* Font);

#endif /* INC_FONTS_H_ */
