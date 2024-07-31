/*
 * system_lighting.h
 *
 *  Created on: Aug 30, 2023
 *      Author: Atakan Ağcakaya
 *      System : Lighting System With WS2812B Led using DMA
 */

#ifndef INC_SYSTEMLIGHTING_H_
#define INC_SYSTEMLIGHTING_H_
#include "main.h"
void LightSystemSet(int color,int brightness);
void LightSystemSent(void);
#endif /* INC_SYSTEMLIGHTING_H_ */
