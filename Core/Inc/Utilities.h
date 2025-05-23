/*
 * Utilities.h
 *
 *  Created on: May 23, 2025
 *      Author: Schiebert Joel
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

/**
 * @brief estructura de banderas
 */
typedef union {
    struct{
        uint8_t bit7:1;
        uint8_t bit6:1;
        uint8_t bit5:1;
        uint8_t bit4:1;
        uint8_t bit3:1;
        uint8_t bit2:1;
        uint8_t bit1:1;
        uint8_t bit0:1;
    }individualFlags;
    uint8_t allFlags;
}_bFlags;

enum{
	FALSE,
	TRUE
};

typedef union{
    uint8_t     u8[4];
    int8_t      i8[4];
    uint16_t    u16[2];
    int16_t     i16[2];
    uint32_t    u32;
    int32_t     i32;
    float       f;
}_work;


#endif /* INC_UTILITIES_H_ */

