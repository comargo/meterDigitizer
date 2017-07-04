/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct MeterDeviceState
{
    uint32_t debounceTimeout;
    uint32_t counter; // counter in liters (0.001 m^3)
};

struct MeterDevicePin
{
    GPIO_TypeDef *port;
    uint16_t pin;
    char *name;
};

struct MeterDeviceHighCounter
{
    uint16_t counter[NUM_METER_DEVICE];
};

volatile struct MeterDeviceState meterDevicesState[NUM_METER_DEVICE] = {};
const struct MeterDevicePin meterDevicesPin[NUM_METER_DEVICE] = {
{.port = HOT_WATER_1_GPIO_Port, .pin = HOT_WATER_1_Pin, .name=u8"ГВС 27"},
{.port = COLD_WATER_1_GPIO_Port, .pin = COLD_WATER_1_Pin, .name=u8"ХВС 27"},
{.port = HEAT_1_GPIO_Port, .pin = HEAT_1_Pin, .name=u8"Тепло 27"},
{.port = HOT_WATER_2_GPIO_Port, .pin = HOT_WATER_2_Pin, .name=u8"ГВС 28"},
{.port = COLD_WATER_2_GPIO_Port, .pin = COLD_WATER_2_Pin, .name=u8"ХВС 28"},
{.port = HEAT_2_GPIO_Port, .pin = HEAT_2_Pin, .name=u8"Тепло 28"}
};

volatile const struct MeterDeviceHighCounter * const meterDevicesHighCounter = (struct MeterDeviceHighCounter *)(FLASH_BASE+127*FLASH_PAGE_SIZE);
volatile const struct MeterDeviceHighCounter * const meterDevicesHighCounterBackup = (struct MeterDeviceHighCounter *)(FLASH_BASE+126*FLASH_PAGE_SIZE);

typedef enum {
    FLASH_Counter_Backup = 0,
    FLASH_Counter_Main = 1
} FLASH_Counter_Source;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void CheckHighCounterState();
static void LoadMeterDevicesState(FLASH_Counter_Source source);
static void ProcessMeterDevices();
static int SelectNearestDevice();
static void SendCounter(int dev);
static void SaveCounter(int dev);
static void WriteHighCounterToFlash(FLASH_Counter_Source source);
static HAL_StatusTypeDef BackupHighCounter();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
  CheckHighCounterState();
  LoadMeterDevicesState(FLASH_Counter_Main);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      ProcessMeterDevices();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      HAL_SuspendTick();
      HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI);
      HAL_ResumeTick();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HOT_WATER_1_Pin COLD_WATER_1_Pin HOT_WATER_2_Pin COLD_WATER_2_Pin 
                           HEAT_1_Pin HEAT_2_Pin */
  GPIO_InitStruct.Pin = HOT_WATER_1_Pin|COLD_WATER_1_Pin|HOT_WATER_2_Pin|COLD_WATER_2_Pin 
                          |HEAT_1_Pin|HEAT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA8 PA9 
                           PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB4 PB5 PB6 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void
CheckHighCounterState()
{
    // Check backup - it should be initialized
    for(int i=0; i<NUM_METER_DEVICE; ++i) {
        if(meterDevicesHighCounterBackup->counter[i] == 0xFFFF) {
            // If any value is not initialized, save backup
            BackupHighCounter();
            break;
        }
    }

    // Check main - it should be initialized
    for(int i=0; i<NUM_METER_DEVICE; ++i) {
        if(meterDevicesHighCounter->counter[i] == 0xFFFF) {
            // If any value is not initialized, load from backup and save
            LoadMeterDevicesState(FLASH_Counter_Backup);
            WriteHighCounterToFlash(FLASH_Counter_Backup);
        }
    }

}

void LoadMeterDevicesState(FLASH_Counter_Source source)
{
    for(int i=0; i<NUM_METER_DEVICE; ++i) {
        meterDevicesState[i].debounceTimeout = 0;
        switch(source) {
        case FLASH_Counter_Backup:
            meterDevicesState[i].counter = ((uint32_t)(meterDevicesHighCounterBackup->counter[i]) << 16);
            break;
        case FLASH_Counter_Main:
            meterDevicesState[i].counter = ((uint32_t)(meterDevicesHighCounter->counter[i]) << 16);
            break;
        }

        if(meterDevicesState[i].counter == 0xFFFF0000) {
            // if flash has not been initialized read 0
            meterDevicesState[i].counter = 0;
        }

        meterDevicesState[i].counter |= HAL_RTCEx_BKUPRead(&hrtc, i+1);
    }
}

void ProcessMeterDevices()
{
    int dev;
    while((dev = SelectNearestDevice()) >= 0 ) {
        while(HAL_GetTick() < meterDevicesState[dev].debounceTimeout) __NOP();
        meterDevicesState[dev].debounceTimeout = 0;
        if(HAL_GPIO_ReadPin(meterDevicesPin[dev].port, meterDevicesPin[dev].pin) == GPIO_PIN_RESET) {
            meterDevicesState[dev].counter++;
            if(meterDevicesState[dev].counter > 100000000) { // Counter overflow
                meterDevicesState[dev].counter = 0;
            }
            SendCounter(dev);
            SaveCounter(dev);
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        }
    }
}

int SelectNearestDevice()
{
    int selectedDevice = -1;
    for(int i=0; i<NUM_METER_DEVICE; ++i) {
        if(meterDevicesState[i].debounceTimeout == 0)
            continue; // Skip not-interrupted device

        if(selectedDevice < 0) {
            selectedDevice = i;
            continue;
        }


        if(meterDevicesState[i].debounceTimeout > UINT32_MAX/2 && meterDevicesState[selectedDevice].debounceTimeout < UINT32_MAX/2 ) {
            // The counter is likely to overflow, use current as minimum value
            selectedDevice = i;
            continue;
        }

        if(meterDevicesState[i].debounceTimeout < meterDevicesState[selectedDevice].debounceTimeout) {
            // The current device trigger earlier then selected device
            selectedDevice = i;
            continue;
        }
    }
    return selectedDevice;
}

void SendCounter(int dev)
{
    char counterMsg[256];
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    uint32_t intCnt = meterDevicesState[dev].counter/1000;
    uint16_t fracCnt =  meterDevicesState[dev].counter%1000;
    sprintf(counterMsg,"20%02u-%02u-%02uT%02u:%02u:%02u\t%d\t%s\t%05u.%03u\r\n",
            (uint)date.Year, (uint)date.Month, (uint)date.Date,
            (uint)time.Hours, (uint)time.Minutes, (uint)time.Seconds,
            dev, meterDevicesPin[dev].name,
            (uint)intCnt, (uint)fracCnt);
    CDC_Transmit_FS((uint8_t*)counterMsg, strlen(counterMsg));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for(int i=0; i<NUM_METER_DEVICE; ++i) {
        if(meterDevicesPin[i].pin == GPIO_Pin) {
            if(meterDevicesState[i].debounceTimeout == 0) {
                meterDevicesState[i].debounceTimeout = HAL_GetTick()+DEBOUNCER_TIMEOUT;
            }
            return;
        }
    }
}

void SaveCounter(int dev)
{
    uint16_t cntHigh = (meterDevicesState[dev].counter & 0xFFFF0000)>>16;
    uint16_t cntLow = (meterDevicesState[dev].counter & 0xFFFF);
    if(meterDevicesHighCounter->counter[dev] != cntHigh) {
        WriteHighCounterToFlash(FLASH_Counter_Main);
    }
    HAL_RTCEx_BKUPWrite(&hrtc, dev+1, cntLow);
}

void WriteHighCounterToFlash(FLASH_Counter_Source source)
{
    // Step 1:
    if(source != FLASH_Counter_Backup) {
        if(BackupHighCounter() != HAL_OK)
            return;
    }

    if(HAL_FLASH_Unlock() != HAL_OK)
        return;
    // Step 2:
    // Erase counter page (#127)
    FLASH_EraseInitTypeDef flashErase;
    flashErase.TypeErase = FLASH_TYPEERASE_PAGES;
    flashErase.Banks = FLASH_BANK_1;
    flashErase.PageAddress = FLASH_BASE+127*FLASH_PAGE_SIZE;
    flashErase.NbPages = 1;
    uint32_t pageError;
    if(HAL_FLASHEx_Erase(&flashErase, &pageError) != HAL_OK) {
        // Probably busy, try again later
        // TODO: Notify error condition!
        goto exit;
    }

    // Step 4:
    // Write to counter page(#127)
    for(int i=0; i<NUM_METER_DEVICE; ++i) {
        uint16_t val = (meterDevicesState[i].counter & 0xFFFF0000)>>16;
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&meterDevicesHighCounter->counter[i]), val) != HAL_OK) {
            // TODO: Notify error condition!
            goto exit;
        }
    }

    exit:
    HAL_FLASH_Lock();
}

HAL_StatusTypeDef BackupHighCounter()
{
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if(status != HAL_OK)
        return status;
    // Step 1:
    // Erase backup page (#126)
    FLASH_EraseInitTypeDef flashErase;
    flashErase.TypeErase = FLASH_TYPEERASE_PAGES;
    flashErase.Banks = FLASH_BANK_1;
    flashErase.PageAddress = FLASH_BASE+126*FLASH_PAGE_SIZE;
    flashErase.NbPages = 1;

    uint32_t pageError;
    status = HAL_FLASHEx_Erase(&flashErase, &pageError);
    if(status != HAL_OK) {
        // Probably busy, try again later
        goto exit;
    }

    // Step 2:
    // Copy counter page(#127) to backup page(#126)
    for(int i=0; i<NUM_METER_DEVICE; ++i) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&meterDevicesHighCounterBackup->counter[i]), meterDevicesHighCounter->counter[i]);
        if(status != HAL_OK) {
            // TODO: Notify error condition!
            goto exit;
        }
    }
    exit:
    HAL_FLASH_Lock();
    return status;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
#define FAULT_DELAY(x) for(int i=0; i<x*500000; ++i) __NOP()
  /* User can add his own implementation to report the HAL error return state */
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    while(1)
    {

        FAULT_DELAY(6);
        for(int i=0; i<3; ++i) {
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
            FAULT_DELAY(1);
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
            FAULT_DELAY(1);
        }
        FAULT_DELAY(2);
        for(int i=0; i<3; ++i) {
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
            FAULT_DELAY(3);
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
            FAULT_DELAY(1);
        }
        FAULT_DELAY(2);
        for(int i=0; i<3; ++i) {
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
            FAULT_DELAY(1);
            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
            FAULT_DELAY(1);
        }
    }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
