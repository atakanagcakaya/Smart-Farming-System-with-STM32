/*
 * aht10.c
 *
 *  Created on: Sep 6, 2023
 *      Author: Atakan Ağcakaya
 *      This file was created for AHT10 temp&hum sensor. The I2C protocol was used.
 *      It includes necessary functions.
 */

#include "aht10.h"
#include "main.h"

I2C_HandleTypeDef  * _hi2c1;
uint8_t          address = (ADRESS_AHT10<<1);
uint8_t          bufferRawD[6] = {ERROR_AHT10, 0, 0, 0, 0, 0};
HAL_StatusTypeDef ret;

void initAHT10(I2C_HandleTypeDef * hi2c1) {
	_hi2c1 = hi2c1 ;
}

bool beginAHT10() {
	HAL_Delay(PWR_DELAY_AHT10);
	modeNormal();
	return FactoryCalCoeff_EN();
}

uint8_t readRawD() {
	uint8_t bufTX[3];
	bufTX[0] = START_MEAS_AHT10;
	bufTX[1] = DATA_MEAS_AHT10;
	bufTX[2] = AHT10_DATA_NOP;

	ret = HAL_I2C_Master_Transmit(_hi2c1, address, bufTX, 3,TIMEOUT_I2C);
	if (ret != HAL_OK)
		return ERROR_AHT10;

	if (getCalBit(FORCE_READ_AHT10) != 0x01)
		return ERROR_AHT10;
	if (getBusyBit(USE_READ_AHT10) != 0x00)
		HAL_Delay(MEAS_DELAY_AHT10);

	ret = HAL_I2C_Master_Receive(_hi2c1, address, bufferRawD, 6,TIMEOUT_I2C);

	if (ret != HAL_OK) {
		bufferRawD[0] = ERROR_AHT10;
		return ERROR_AHT10;
	}
	return true;
}

float readTemp(bool readI2C) {
	if (readI2C == FORCE_READ_AHT10) {
		if (readRawD() == ERROR_AHT10)
			return ERROR_AHT10;
	}

	if (bufferRawD[0] == ERROR_AHT10)
		return ERROR_AHT10;
	//temp data 20bit
	uint32_t temperature = ((uint32_t) (bufferRawD[3] & 0x0F) << 16)| ((uint16_t) bufferRawD[4] << 8) | bufferRawD[5];
	return (float) temperature * 0.000191 - 50;
}

float readHum(bool readI2C) {
	if (readI2C == FORCE_READ_AHT10) {
		if (readRawD() == ERROR_AHT10)
			return ERROR_AHT10;
	}
	if (bufferRawD[0] == ERROR_AHT10)
		return ERROR_AHT10;
	//hum data 20bit
	uint32_t rawData = (((uint32_t) bufferRawD[1] << 16)| ((uint16_t) bufferRawD[2] << 8) | (bufferRawD[3])) >> 4;
	float humidity = (float) rawData * 0.000095;

	if (humidity < 0)
		return 0;
	if (humidity > 100)
		return 100;
	return humidity;
}

bool modeNormal(void) {
	uint8_t bufTX[3];
	bufTX[0] = NORMAL_CYCLE_AHT10;
	bufTX[1] = AHT10_DATA_NOP;
	bufTX[2] = AHT10_DATA_NOP;
	ret = HAL_I2C_Master_Transmit(_hi2c1, address, bufTX, 3, TIMEOUT_I2C );
	if (ret != HAL_OK)
		return false;

	HAL_Delay(AHT10_CMD_DELAY);

	return true;
}

uint8_t readStatus() {

	ret = HAL_I2C_Master_Receive(_hi2c1, address, bufferRawD, 1,TIMEOUT_I2C);

	if (ret != HAL_OK) {
		bufferRawD[0] = ERROR_AHT10;
		return ERROR_AHT10;
	}
	return bufferRawD[0];
}


uint8_t getCalBit(bool readI2C) {
	uint8_t valueBit;
	if (readI2C == FORCE_READ_AHT10)
		bufferRawD[0] = readStatus(); //force to read status byte

	if (bufferRawD[0] != ERROR_AHT10)
	{
		valueBit = (bufferRawD[0] & 0x08);
		return (valueBit>>3); //get 3-rd bit 0001000
	}else{
		return ERROR_AHT10;
	}
}


bool FactoryCalCoeff_EN() {

	uint8_t bufTX[3];
	bufTX[0] = INIT_AHT10;
	bufTX[1] = INIT_CAL_EN_AHT10;
	bufTX[2] = AHT10_DATA_NOP;

	ret = HAL_I2C_Master_Transmit(_hi2c1, address, bufTX, 3, TIMEOUT_I2C);

	if (ret != HAL_OK)
		return false;

	HAL_Delay(AHT10_CMD_DELAY);

	if (getCalBit(FORCE_READ_AHT10) == 0x01)
		return true;
	else
		return false;

}

uint8_t getBusyBit(bool readI2C) {
	uint8_t valueBit;
	if (readI2C == FORCE_READ_AHT10)
		bufferRawD[0] = readStatus(); // Read status byte

	if (bufferRawD[0] != ERROR_AHT10)
	{
		valueBit = (bufferRawD[0] & 0x80);
		return (valueBit>>7); //get 7-th bit 1000 0000  0x80
	}
	else{
		return ERROR_AHT10;
	}
}

float AHT10_temperature, AHT10_humidity;
uint8_t getTemp(){
	AHT10_temperature = readTemp(true);
	if (AHT10_temperature != ERROR_AHT10)
		return (float) AHT10_temperature;
}

uint8_t getHum(){
	AHT10_humidity= readHum(true);
	if (AHT10_humidity != ERROR_AHT10)
		return (float) AHT10_humidity;
}
