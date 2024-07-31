/*
 * system_lighting.c
 *
 *  Created on: Aug 31, 2023
 *      Author: Atakan Ağcakaya
 *      colors: 1 -> Green , 2 -> Red , 3 -> Blue 0 -> RESET state
 */
#include <systemLighting.h>
#include "main.h"
#include "math.h"

#define NUM_OF_LED 120
#define LOW_STATE 0

uint8_t data_lighting[NUM_OF_LED][3];
uint8_t data_bright[NUM_OF_LED][3];

int flag_datasent=0;
int Lednum;
extern TIM_HandleTypeDef htim2;
void LightSystemSet(int color,int brightness){
	if (brightness>=100){
			brightness=100;
		}
			else if (brightness <= 0){
			brightness=0;
		}

	int hStateBright = (255 * brightness)/100;
	round(hStateBright);

	switch(color){
	case 1:
	for(int i = 0; i<NUM_OF_LED;i++){
		data_lighting[i][0] = hStateBright;
		data_lighting[i][1] = LOW_STATE;
		data_lighting[i][2] = LOW_STATE;}
	break;
	case 2:
	for(int i = 0; i<NUM_OF_LED;i++){
		data_lighting[i][0] = LOW_STATE;
		data_lighting[i][1] = hStateBright;
		data_lighting[i][2] = LOW_STATE;}
	break;
	case 3:
	for(int i = 0; i<NUM_OF_LED;i++){
		data_lighting[i][0] = LOW_STATE;
		data_lighting[i][1] = LOW_STATE;
		data_lighting[i][2] = hStateBright;}
	break;
	case 0:
	for(int i = 0; i<NUM_OF_LED;i++){
		data_lighting[i][0] = LOW_STATE;
		data_lighting[i][1] = LOW_STATE;
		data_lighting[i][2] = LOW_STATE;}
	break;
}
}
uint16_t pwmData[(24*NUM_OF_LED)+50]; // 24 bit +  50 us bekleme için

void LightSystemSent(void){
	uint32_t sendingcolor;
	uint32_t index = 0;

	for(int i = 0 ; i<NUM_OF_LED; i++){
		sendingcolor = ((data_lighting[i][0]<<16) | (data_lighting[i][1]<<8) | (data_lighting[i][2]));

		for(int j = 23 ; j>=0; j--){
			if (sendingcolor&(1<<j)){
				pwmData[index] = 27;
			}
			else
				pwmData[index] = 13;
			index++;
		}
	}

	for(int i = 0;i<50;i++){
		pwmData[index] = 0;
		index++;
	}
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint16_t *)pwmData,index);
	while(!flag_datasent){};
	flag_datasent = 0;
}
