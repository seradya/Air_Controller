/*
 * Encoder.c
 *
 *  Created on: 27 сент. 2022 г.
 *      Author: serad
 */

#include <encoder.h>

void Encoder_Init(void) {
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	__HAL_TIM_SET_COUNTER(&htim3, 32768);
	cnt1 = __HAL_TIM_GET_COUNTER(&htim3)>>1;
	cnt2 = __HAL_TIM_GET_COUNTER(&htim3)>>1;
};

int16_t Enc_val(void) {
	cnt2 = (__HAL_TIM_GET_COUNTER(&htim3)>>1);
	value = cnt2 - cnt1;
	cnt1 = cnt2;
	return value;
}


