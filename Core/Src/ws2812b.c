/*
 * ws2812b.c
 *
 *  Created on: 1 Jul 2022
 *      Author: ray
 */
#include "ws2812b.h"
#include "stm32f4xx.h"

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;

uint8_t ledData[MAX_LED][4];
uint16_t pwmData[(24 * MAX_LED) + 50];

void WS2812_setLed(uint8_t ledNum, uint8_t red, uint8_t green, uint8_t blue) {
	ledData[ledNum][0] = ledNum;
	ledData[ledNum][1] = green;
	ledData[ledNum][2] = red;
	ledData[ledNum][3] = blue;
}

void WS2812_update(void) {
	uint32_t indx = 0;
	uint32_t color;
	for (int i = 0; i < MAX_LED; i++) {
		color = ((ledData[i][1] << 16) | (ledData[i][2] << 8) | (ledData[i][3]));

		for (int i = 23; i >= 0; i--) {
			if (color & (1 << i)) {
				pwmData[indx] = 140;  // 2/3 of 90
			}

			else
				pwmData[indx] = 210 - 140;  // 1/3 of 90

			indx++;
		}

	}

	for (int i = 0; i < 50; i++) {
		pwmData[indx] = 0;
		indx++;
	}
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) pwmData, indx);
}

void WS2812_resetLed(void) {
	for (int i = 0; i < MAX_LED; i++) {
		ledData[i][0] = i;
		ledData[i][1] = 0;
		ledData[i][2] = 0;
		ledData[i][3] = 0;
	}
}
