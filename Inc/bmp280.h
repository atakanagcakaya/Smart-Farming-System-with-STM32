/*
 * bmp280.h
 *
 *  Created on: 5 Eyl 2023
 *      Author: Atakan Ağcakaya
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "main.h"
#include "math.h"

#define ADRR_BMP280 (0x76<<1)
#define RESET_BMP280 0xE0

#define STATUS 0xF3
#define MEAS_S 0x08
#define UPDATE_S 0x01
#define MEAS_C 0xF4

//oversampling
#define OS_16BIT 0x01
#define OS_17BIT 0x02
#define OS_18BIT 0x03
#define OS_19BIT 0x04
#define OS_20BIT 0x06
// modes
#define NORMAL 0x03
#define FORCED 0x02
#define SLEEP  0x00
// control measurement offsets
#define MEAS_T_OFF 0x05
#define MEAS_P_OFF 0x02
#define MEAS_M_OFF 0x00
//Temperature oversampling
#define OS_T_16BIT (OS_16BIT << MEAS_T_OFF)
#define OS_T_17BIT (OS_17BIT << MEAS_T_OFF)
#define OS_T_18BIT (OS_18BIT << MEAS_T_OFF)
#define OS_T_19BIT (OS_19BIT << MEAS_T_OFF)
#define OS_T_20BIT (OS_20BIT << MEAS_T_OFF)
//Pressure oversampling
#define OS_P_16BIT (OS_16BIT << MEAS_P_OFF)
#define OS_P_17BIT (OS_17BIT << MEAS_P_OFF)
#define OS_P_18BIT (OS_18BIT << MEAS_P_OFF)
#define OS_P_19BIT (OS_19BIT << MEAS_P_OFF)
#define OS_P_20BIT (OS_20BIT << MEAS_P_OFF)
//Power Mode Selection
#define M_SLEEP  (SLEEP  << MEAS_M_OFF)
#define M_NORMAL (NORMAL << MEAS_M_OFF)

#define CONFIG 0xF5
#define SB_OFF 0x05
#define FILTER_OFF 0x02
// standby setting
#define SB_125  (0x02 << SB_OFF)
#define SB_250  (0x03 << SB_OFF)
#define SB_500  (0x04 << SB_OFF)
#define SB_1000 (0x05 << SB_OFF)
//filter
#define FILTER_2  (0x02 << FILTER_OFF)
#define FILTER_5  (0x04 << FILTER_OFF)
#define FILTER_11 (0x08 << FILTER_OFF)
#define FILTER_22 (0x10 << FILTER_OFF)
//calibration
#define ADRESS_CAL 0x88 // calibration address
#define SIZE_CAL 24 // cal. size
//raw data
#define ADRESS_RAWD 0xF7
#define LENGTH_RAWD 6
//functions
uint8_t readSensor(uint8_t adress);
void writeSensor(uint8_t adress,	uint8_t data);
void initBMP280();
void calBMP280();
uint64_t getBMP(int i);

#endif /* INC_BMP280_H_ */
