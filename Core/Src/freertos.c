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
	State_Configure_Time_FORMAT,
	State_End,
} State_e;

typedef struct {
	RTC_DateTypeDef *date;
	RTC_TimeTypeDef *time;
} RTC_DateTime_t;

typedef struct {
	uint8_t row;
	uint8_t col;
} CursorPos_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CURSOR_ROW_DATE 0
#define CURSOR_ROW_TIME 1
#define CURSOR_DATE_DD 8
#define CURSOR_DATE_MM 5
#define CURSOR_DATE_YY 0
#define CURSOR_TIME_HH 0
#define CURSOR_TIME_MM 3
#define CURSOR_TIME_SS 6
#define CURSOR_TIME_FORMAT 10
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
extern QueueHandle_t queue_input;

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
	char userInput = 0;

	while (1) {
		if (xQueuePeek(queue_input, &(userInput), pdMS_TO_TICKS(10))) {
			if (userInput == 'U' || userInput == 'D') {
				xQueueReceive(queue_input, &(userInput), pdMS_TO_TICKS(0));
				if (userInput == 'U') {
					systemState--;
					if (systemState > UINT8_MAX - 1) //Check if overflow happens
						systemState = State_Configure_Time_FORMAT;
				} else if (userInput == 'D') {
					systemState++;
				}
				if (systemState == State_End) {
					systemState = State_Normal;
				}
			} else {
				xTaskNotify(task_rtc, 0, eNoAction);
			}
		}
	}

}

void Task_Rtc(void *param) {
	RTC_DateTime_t *dateTimeSender;
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;
	memset(&rtc_date, 0, sizeof(rtc_date));
	memset(&rtc_time, 0, sizeof(rtc_time));

	char userInput = '\0';
	int8_t action = 0;
	bool inConfigMode = false;
	while (1) {
		//Get RTC current date and time
		if (systemState != State_Normal) {
			if (xTaskNotifyWait(0x00, ULONG_MAX, NULL, pdMS_TO_TICKS(0)) == pdTRUE) {
				if (xQueueReceive(queue_input, &(userInput), pdMS_TO_TICKS(0)) == pdTRUE) {
					if (userInput == 'I')
						action = 1;
					else if (userInput == 'O')
						action = -1;
				}
			}
		}

		if(systemState != State_Normal){
			inConfigMode = true;
		}
		switch (systemState) {
		case State_Normal:
			if(inConfigMode){
				rtc_configure_time(&rtc_time);
				rtc_configure_date(&rtc_date);
				inConfigMode = false;
			}
			HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
			break;

		case State_Configure_Date_DD:
			rtc_date.Date += action;
			if (0 == rtc_date.Date){
				rtc_date.Date = 31;
			}else if (rtc_date.Date > 31){
				rtc_date.Date = 1;
			}
			break;
		case State_Configure_Date_MM:
			rtc_date.Month += action;
			if (0 == rtc_date.Month){
				rtc_date.Month = 12;
			}else if (rtc_date.Month > 31){
				rtc_date.Month = 1;
			}
			break;
		case State_Configure_Date_YY:
			rtc_date.Year += action;
			if (rtc_date.Year > UINT8_MAX - 1){
				rtc_date.Year = 99;
			}else if (rtc_date.Year > 99){
				rtc_date.Year = 0;
			}
			break;
		case State_Configure_Time_HH:
			rtc_time.Hours += action;
			if (0 == rtc_time.Hours){
				rtc_time.Hours = 12;
			}else if (rtc_time.Hours > 12) {
				rtc_time.Hours = 1;
			}
			break;
		case State_Configure_Time_MM:
			rtc_time.Minutes += action;
			if (rtc_time.Minutes > UINT8_MAX - 1) {
				rtc_time.Minutes = 59;
			}
			else if (rtc_time.Minutes >= 60) {
				rtc_time.Minutes = 0;
			}
			break;
		case State_Configure_Time_SS:
			rtc_time.Seconds += action;
			if (rtc_time.Seconds > UINT8_MAX - 1) {
				rtc_time.Seconds = 59;
			}
			 else if (rtc_time.Seconds >= 60) {
				rtc_time.Seconds = 0;
			}
			break;
		case State_Configure_Time_FORMAT:
			if(0 != action){
				rtc_time.TimeFormat ^= 0x40;
			}
			break;
		default:
			break;
		}

		dateTimeSender = pvPortMalloc(sizeof(RTC_DateTime_t));
		dateTimeSender->date = &rtc_date;
		dateTimeSender->time = &rtc_time;

		xQueueSend(queue_lcd, &dateTimeSender, portMAX_DELAY);
		vTaskDelay(50);
		action = 0;
	}
}

void Task_Lcd(void *param) {
//	LcdDisplayData_t *rtcReceiver;
	RTC_DateTime_t *rtcReceiver;
	uint8_t blink = 0;

	CursorPos_t cursor;
	uint8_t data_len = 0;
	memset(&cursor, 0, sizeof(CursorPos_t));

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
		xQueueReceive(queue_lcd, &rtcReceiver, 1);
		char *format;

		//Default space occupied is 2
		data_len = 2;

		//Display rtc data
		format = (rtcReceiver->time->TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";
		lcd16x2_i2c_setCursor(0, 0);
		lcd16x2_i2c_printf("%02d-%02d-%02d", 2000 + rtcReceiver->date->Year, rtcReceiver->date->Month, rtcReceiver->date->Date);
		vTaskDelay(1);
		lcd16x2_i2c_setCursor(1, 0);
		lcd16x2_i2c_printf("%02d:%02d:%02d [%s]", rtcReceiver->time->Hours, rtcReceiver->time->Minutes, rtcReceiver->time->Seconds, format);

		//Set cursor position accordingly
		switch (systemState) {
		case State_Normal:
			vTaskDelay(100);
			break;
		case State_Configure_Time_HH:
			cursor.row = CURSOR_ROW_TIME;
			cursor.col = CURSOR_TIME_HH;
			break;
		case State_Configure_Time_MM:
			cursor.row = CURSOR_ROW_TIME;
			cursor.col = CURSOR_TIME_MM;
			break;
		case State_Configure_Time_SS:
			cursor.row = CURSOR_ROW_TIME;
			cursor.col = CURSOR_TIME_SS;
			break;
		case State_Configure_Time_FORMAT:
			cursor.row = CURSOR_ROW_TIME;
			cursor.col = CURSOR_TIME_FORMAT;
			break;
		case State_Configure_Date_DD:
			cursor.row = CURSOR_ROW_DATE;
			cursor.col = CURSOR_DATE_DD;
			break;
		case State_Configure_Date_MM:
			cursor.row = CURSOR_ROW_DATE;
			cursor.col = CURSOR_DATE_MM;
			break;
		case State_Configure_Date_YY:
			cursor.row = CURSOR_ROW_DATE;
			cursor.col = CURSOR_DATE_YY;
			data_len = 4;
			break;
		default:
			break;
		}
		if (systemState != State_Normal) {
			//Add blinking feature
			if (blink ^= 1) {
				lcd16x2_i2c_setCursor(cursor.row, cursor.col);
				for (int i = 0; i < data_len; i++) {
					lcd16x2_i2c_printf(" ");
				}
				vTaskDelay(200);
			} else {
				vTaskDelay(500);
			}
		}
		vPortFree(rtcReceiver);
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
			while (HAL_GPIO_ReadPin(KEY_L1_GPIO_Port, KEY_L1_Pin));
			tmp = 'D';
		}

		if ((HAL_GPIO_ReadPin(KEY_L2_GPIO_Port, KEY_L2_Pin)))   // if the Col 2 is low
		{
			while (HAL_GPIO_ReadPin(KEY_L2_GPIO_Port, KEY_L2_Pin));
			tmp = 'O';
		}

		/* Make ROW 2 LOW and all other ROWs HIGH */
		HAL_GPIO_WritePin(KEY_R1_GPIO_Port, KEY_R1_Pin, GPIO_PIN_SET);  //Pull the R1 high
		HAL_GPIO_WritePin(KEY_R2_GPIO_Port, KEY_R2_Pin, GPIO_PIN_RESET);  // Pull the R2 low

		if ((HAL_GPIO_ReadPin(KEY_L1_GPIO_Port, KEY_L1_Pin)))   // if the Col 1 is low
		{
			while (HAL_GPIO_ReadPin(KEY_L1_GPIO_Port, KEY_L1_Pin));
			tmp = 'U';
		}

		if ((HAL_GPIO_ReadPin(KEY_L2_GPIO_Port, KEY_L2_Pin)))   // if the Col 2 is low
		{
			while (HAL_GPIO_ReadPin(KEY_L2_GPIO_Port, KEY_L2_Pin));
			tmp = 'I';
		}

		keyInput = tmp; //D: down, U: up, I: in, O: out
		if (keyInput != 0) {
			xQueueSend(queue_input, (void* ) &keyInput, pdMS_TO_TICKS(5));
		}
		vTaskDelay(30);
	}
}

void Task_LedRing(void* param){
	while(1){

	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
