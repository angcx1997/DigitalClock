/*
 * ws2812b.h
 *
 *  Created on: 1 Jul 2022
 *      Author: ray
 */

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

#include <stdint.h>

#define MAX_LED 24


void WS2812_setLed(uint8_t ledNum, uint8_t red, uint8_t green, uint8_t blue);

void WS2812_setWhole(uint8_t red, uint8_t green, uint8_t blue);

void WS2812_update(void);

void WS2812_resetLed(void);

#endif /* INC_WS2812B_H_ */
