/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <queue.h>
#include <limits.h>
#include <string.h>
#include "rtc.h"
#include "i2c_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	State_Normal = 0x00,
//	State_Menu = 0x02,
	State_Configure_Date_DD,
	State_Configure_Date_MM,
	State_Configure_Date_YY,
	State_Configure_Time_HH,
	State_Configure_Time_MM,
	State_Configure_Time_SS,
	State_End,
} State_e;

typedef struct {
	RTC_DateTypeDef *date;
	RTC_TimeTypeDef *time;
} RTC_DateTime_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;

extern TaskHandle_t task_rtc;
extern TaskHandle_t task_lcd;
extern TaskHandle_t task_stateControl;
extern TaskHandle_t task_interface;

extern QueueHandle_t queue_lcd;

State_e systemState = State_Normal;
State_e tmpState = State_Normal;
uint8_t keyInput;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Task_StateController(void *param) {
	BaseType_t notifiedValue;

	while (1) {
		if (xTaskNotifyWait(0x00, UINT_MAX, &notifiedValue,
		portMAX_DELAY) == pdTRUE) {
			//Delay added for debouncing
			vTaskDelay(pdMS_TO_TICKS(200));

			if (notifiedValue == 'U') {
				systemState--;
				if (systemState > UINT8_MAX - 1) //Check if overflow happens
					systemState = State_Configure_Time_SS;
			} else if (notifiedValue == 'D') {
				systemState++;
			}

			if (systemState == State_End) {
				systemState = State_Normal;
			}
			xTaskNotify(task_rtc, systemState, eSetValueWithOverwrite);

		}
	}

}

void Task_Rtc(void *param) {
	RTC_DateTime_t *dateTimeSender;
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;
	memset(&rtc_date, 0, sizeof(rtc_date));
	memset(&rtc_time, 0, sizeof(rtc_time));

	uint32_t ulNotifiedValue;
	while (1) {
		//Get RTC current date and time
		HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

		dateTimeSender = pvPortMalloc(sizeof(RTC_DateTime_t));
		dateTimeSender->date = &rtc_date;
		dateTimeSender->time = &rtc_time;
		xQueueSend(queue_lcd, &dateTimeSender, portMAX_DELAY);
		vTaskDelay(50);

#if 0
		switch (systemState) {
		case State_Normal:
			date = rtc_get_date();
			time = rtc_get_time();
			rtcSender = pvPortMalloc(sizeof(LcdDisplayData_t));
			rtcSender->line[0] = &date;
			rtcSender->line[1] = &time;
			xQueueSend(queue_lcd, &rtcSender, portMAX_DELAY);
			vTaskDelay(50);
			break;
//		case State_Configure_Date_DD:
//			rtcSender->line[0] = &date;
//
		default:
			break;
		}
#endif
	}
}

void Task_Lcd(void *param) {
//	LcdDisplayData_t *rtcReceiver;
	RTC_DateTime_t *rtcReceiver;
	uint8_t blink = 0;

	if (lcd16x2_i2c_init(&hi2c1) == false) {
		Error_Handler();
	}

	lcd16x2_i2c_setCursor(0, 0);
	lcd16x2_i2c_printf("Digital Clock");
	HAL_Delay(1);
	lcd16x2_i2c_setCursor(1, 0);
	lcd16x2_i2c_printf("Hello!");
	HAL_Delay(1000);
	lcd16x2_i2c_clear();
	while (1) {
		xQueueReceive(queue_lcd, &rtcReceiver, portMAX_DELAY);
		char *format;
		format = (rtcReceiver->time->TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";
		lcd16x2_i2c_setCursor(0, 0);
		lcd16x2_i2c_printf("%02d-%02d-%2d", rtcReceiver->date->Month, rtcReceiver->date->Date, 2000 + rtcReceiver->date->Year);
		vTaskDelay(1);
		lcd16x2_i2c_setCursor(1, 0);
		lcd16x2_i2c_printf("%02d:%02d:%02d [%s]", rtcReceiver->time->Hours, rtcReceiver->time->Minutes, rtcReceiver->time->Seconds, format);
		vPortFree(rtcReceiver);
		switch (systemState) {
		case State_Configure_Time_HH:
			if (blink ^= 1) {
				lcd16x2_i2c_setCursor(0, 0);
				for (int i = 0; i < 2; i++) {
					lcd16x2_i2c_printf(" ");
				}
			}
			vTaskDelay(1000);
			break;
		default:
			vTaskDelay(50);
			break;
		}

	}
}

void Task_Interface(void *param) {
	while (1) {
		char tmp = 0;
		/* Make ROW 1 LOW and all other ROWs HIGH */
		HAL_GPIO_WritePin(KEY_R1_GPIO_Port, KEY_R1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
		HAL_GPIO_WritePin(KEY_R2_GPIO_Port, KEY_R2_Pin, GPIO_PIN_SET);  // Pull the R2 High

		if ((HAL_GPIO_ReadPin(KEY_L1_GPIO_Port, KEY_L1_Pin)))   // if the Col 1 is low
		{
			tmp = 'D';
		}

		if ((HAL_GPIO_ReadPin(KEY_L2_GPIO_Port, KEY_L2_Pin)))   // if the Col 2 is low
		{
			tmp = 'O';
		}

		/* Make ROW 2 LOW and all other ROWs HIGH */
		HAL_GPIO_WritePin(KEY_R1_GPIO_Port, KEY_R1_Pin, GPIO_PIN_SET);  //Pull the R1 high
		HAL_GPIO_WritePin(KEY_R2_GPIO_Port, KEY_R2_Pin, GPIO_PIN_RESET);  // Pull the R2 low

		if ((HAL_GPIO_ReadPin(KEY_L1_GPIO_Port, KEY_L1_Pin)))   // if the Col 1 is low
		{
			tmp = 'U';
		}

		if ((HAL_GPIO_ReadPin(KEY_L2_GPIO_Port, KEY_L2_Pin)))   // if the Col 2 is low
		{
			tmp = 'I';
		}

		keyInput = tmp; //D: down, U: up, I: in, O: out
		xTaskNotify(task_stateControl, keyInput, eSetValueWithOverwrite);

	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
