/*
 * rtc.h
 *
 *  Created on: Feb 6, 2022
 *      Author: angcx
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "main.h"
#include <stdbool.h>

/**
 * @brief Show date and time in ITM
 * @param None
 * @retval None
 */
void rtc_show_time_date_itm(void);

/**
 * @brief Show date and time in Serial Terminal
 * @param None
 * @retval None
 */
void rtc_show_time_date_serial(void);

/**
 * @brief Configure time
 * @param pointer to RTC_TimeTypeDef
 * @retval None
 */
void rtc_configure_time(RTC_TimeTypeDef *time);

/**
 * @brief Get date
 * @retval pointer to date string(allocate array of size 40)
 */
char* rtc_get_date(void);

/**
 * @brief Get time
 * @retval pointer to time string(allocate array of size 40)
 */
char* rtc_get_time(void);

/**
 * @brief Configure date
 * @param pointer to RTC_DateTypedef
 * @retval None
 */
void rtc_configure_date(RTC_DateTypeDef *data);

/**
 * @brief Validate input information is correct
 * @param pointer to RTC_TimeTypeDef, RTC_DateTypedef
 * @retval 0 if false, 1 if correct
 */
bool rtc_validate(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

#endif /* INC_RTC_H_ */
