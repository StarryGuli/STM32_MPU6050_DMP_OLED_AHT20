/*
 * aht20.h
 *
 *  Created on: Sep 23, 2025
 *      Author: Hallo
 */

#ifndef INC_AHT20_H_
#define INC_AHT20_H_

#include "i2c.h"

void AHT20_Init();

void AHT20_Measure();
void AHT20_Get();
void AHT20_Analysis(float *temperature,float *humidity);

void AHT20_Read(float *T,float *H);

#endif /* INC_AHT20_H_ */
