/*
 * Encoder.h
 *
 *  Created on: 27 сент. 2022 г.
 *      Author: serad
 */


#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim3;

static uint32_t cnt1 = 0;
static uint32_t cnt2 = 0;
static int16_t value = 0;

void Encoder_Init(void);
int16_t Enc_val(void);


#endif /* __ENCODER_H_ */


