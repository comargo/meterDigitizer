#ifndef METER_DEVICE_H
#define METER_DEVICE_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"

HAL_StatusTypeDef MeterDevicesLoadValues();
HAL_StatusTypeDef MeterDevicesProcess();
HAL_StatusTypeDef MeterDeviceSetValue(uint dev, uint32_t val);
HAL_StatusTypeDef MeterDeviceShowValue(uint dev);
void MeterDeviceInterrupt(uint16_t GPIO_Pin);


#endif//METER_DEVICE_H
