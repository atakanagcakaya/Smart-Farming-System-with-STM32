/*
 * bmp280.c
 *
 *  Created on: 5 Eyl 2023
 *      Author: Atakan
 */

#include "bmp280.h"
extern I2C_HandleTypeDef hi2c1;
signed long temperature_raw;
signed long pressure_raw;
float temperature;
float pressure;
float altitude;

unsigned short D_T1;
unsigned short D_P1;
signed short D_T2;
signed short D_T3;
signed short D_P2;
signed short D_P3;
signed short D_P4;
signed short D_P5;
signed short D_P6;
signed short D_P7;
signed short D_P8;
signed short D_P9;


uint8_t readSensor(uint8_t adress){
	uint8_t buffer_RX;
	HAL_I2C_Mem_Read(&hi2c1, ADRR_BMP280, adress, 1, &buffer_RX, 1, 1000);
	return buffer_RX;
}

void writeSensor(uint8_t adress, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c1, ADRR_BMP280, adress, 1, &data, 1, 1000);
}

void calBMP280(){
	uint8_t data_cal[24] = {0};
	HAL_I2C_Mem_Read( &hi2c1, ADRR_BMP280, ADRESS_CAL, 1, data_cal, SIZE_CAL, 1000);
	D_T1 = ((data_cal[1]  << 8) | (data_cal[0]));
	D_T2 = ((data_cal[3]  << 8) | (data_cal[2]));
	D_T3 = ((data_cal[5]  << 8) | (data_cal[4]));
	D_P1 = ((data_cal[7]  << 8) | (data_cal[6]));
	D_P2 = ((data_cal[9]  << 8) | (data_cal[8]));
	D_P3 = ((data_cal[11] << 8) | (data_cal[10]));
	D_P4 = ((data_cal[13] << 8) | (data_cal[12]));
	D_P5 = ((data_cal[15] << 8) | (data_cal[14]));
	D_P6 = ((data_cal[17] << 8) | (data_cal[16]));
	D_P7 = ((data_cal[19] << 8) | (data_cal[18]));
	D_P8 = ((data_cal[21] << 8) | (data_cal[20]));
	D_P9 = ((data_cal[23] << 8) | (data_cal[22]));
}

void initBMP280(){
	writeSensor(MEAS_C, (OS_P_20BIT|M_NORMAL));
	writeSensor(CONFIG, (SB_1000| FILTER_22));
	calBMP280();
}

uint64_t getBMP(int i){
	uint8_t status;
	uint8_t rawData[6];
	do{
		status=readSensor(STATUS);
	} while(((status & 0x08) == MEAS_S)|| ((status & 0x01) == UPDATE_S));

	HAL_I2C_Mem_Read(&hi2c1, ADRR_BMP280, ADRESS_RAWD, 1, rawData, LENGTH_RAWD, 1000);
	temperature_raw =  ((rawData[3] << 12) | (rawData[4] << 4) | (rawData[5] >> 4));
	pressure_raw    =  ((rawData[0] << 12) | (rawData[1] << 4) | (rawData[2] >> 4));
	double var1, var2;
	var1=(((double)temperature_raw)/16384.0-((double)D_T1)/1024.0)*((double)D_T2);
	var2=((((double)temperature_raw)/131072.0-((double)D_T1)/8192.0)*(((double)temperature_raw)/131072.0-((double)D_T1)/8192.0))*((double)D_T3);
	double t_fine = (int32_t)(var1+var2);
	volatile double T = (var1+var2)/5120.0;
	var1=((double)t_fine/2.0)-64000.0;
	var2=var1*var1*((double)D_P6)/32768.0;
	var2=var2+var1*((double)D_P5)*2.0;
	var2=(var2/4.0)+(((double)D_P4)*65536.0);
	var1=(((double)D_P3)*var1*var1/524288.0+((double)D_P2)*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((double)D_P1);
	volatile double p = 1048576.0-(double)pressure_raw;
	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((double)D_P9)*p*p/2147483648.0;
	var2=p*((double)D_P8)/32768.0;
	p=p+(var1+var2+((double)D_P7))/16.0;
	pressure = p;
	altitude=44330.0f*(1-powf(pressure/101325.0f,1.0f/5.255f));

	switch(i){
	case 1:
		return pressure;
		break;
	case 2:
		return altitude;
		break;
	}
}
