/*
 * rtc.c
 *
 *  Created on: Feb 6, 2022
 *      Author: angcx
 */

#include "rtc.h"
#include "stdio.h"
#include "string.h"
extern RTC_HandleTypeDef hrtc;
//typedef QueueHandle_t queue_print;
static char showtime[40] = {NULL};
static char showdate[40] = {NULL};

void rtc_show_time_date_itm(void) {
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;

	memset(&rtc_date, 0, sizeof(rtc_date));
	memset(&rtc_time, 0, sizeof(rtc_time));

	//Get RTC current date and time
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	char *format;
//	char str[2];
	format = (rtc_time.TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";

	printf("%02d:%02d:%02d [%s]", rtc_time.Hours, rtc_time.Minutes,
			rtc_time.Seconds, format);
	printf("\t%02d-%02d-%2d\n", rtc_date.Month, rtc_date.Date,
			2000 + rtc_date.Year);
}

void rtc_show_time_date_serial(void) {
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;

	memset(&rtc_date, 0, sizeof(rtc_date));
	memset(&rtc_time, 0, sizeof(rtc_time));

	//Get RTC date and time
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	char *format;
	format = (rtc_time.TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";

	/* Display time Format : hh:mm:ss [AM/PM] */
	sprintf((char*) showtime, "%s:\t%02d:%02d:%02d [%s]", "\nCurrent Time&Date",
			rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, format);
//	xQueueSend(queue_print,&time,portMAX_DELAY);

	/* Display date Format : date-month-year */
	sprintf((char*) showdate, "\t%02d-%02d-%2d", rtc_date.Month,
			rtc_date.Date, 2000 + rtc_date.Year);
//	xQueueSend(queue_print,&date,portMAX_DELAY);
}

char* rtc_get_date(void) {
	RTC_DateTypeDef rtc_date;

	memset(&rtc_date, 0, sizeof(rtc_date));

	//Get RTC date and time
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	/* Display date Format : date-month-year */
	sprintf((char*) showdate, "%02d-%02d-%2d", rtc_date.Month,
			rtc_date.Date, 2000 + rtc_date.Year);

	return showdate;
}

char* rtc_get_time(void) {
	RTC_TimeTypeDef rtc_time;
	memset(&rtc_time, 0, sizeof(rtc_time));

	//Get RTC date and time
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);

	char *format;
	format = (rtc_time.TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";

	/* Display time Format : hh:mm:ss [AM/PM] */
	sprintf((char*) showtime, "%02d:%02d:%02d [%s]", rtc_time.Hours,
			rtc_time.Minutes, rtc_time.Seconds, format);

	return showtime;
}

void rtc_configure_time(RTC_TimeTypeDef *time) {
//	time->TimeFormat = RTC_HOURFORMAT12_AM;
	time->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	time->StoreOperation = RTC_STOREOPERATION_RESET;

	HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN);
}

void rtc_configure_date(RTC_DateTypeDef *date) {
	HAL_RTC_SetDate(&hrtc, date, RTC_FORMAT_BIN);
}

bool rtc_validate(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	if (time) {
		if ((time->Hours > 12) || (time->Minutes > 59) || (time->Seconds > 59))
			return false;
	}

	if (date) {
		if ((date->Date > 31) || (date->WeekDay > 7) || (date->Year > 99)
				|| (date->Month > 12))
			return false;
	}

	return true;
}

