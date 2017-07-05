#ifndef HAL_RTC_UNIX_H
#define HAL_RTC_UNIX_H

#include <time.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"

HAL_StatusTypeDef HAL_RTC_GetUNIXTime(RTC_HandleTypeDef *hrtc, time_t *unixtime);
HAL_StatusTypeDef HAL_RTC_SetUNIXTime(RTC_HandleTypeDef *hrtc, time_t unixtime);


#endif//HAL_RTC_UNIX_H
