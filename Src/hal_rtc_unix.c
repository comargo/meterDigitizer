#include <hal_rtc_unix.h>
/* Part of the code is copyrighted by STMicroelectronics */
 /******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

static HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef* hrtc);
static HAL_StatusTypeDef  RTC_ExitInitMode(RTC_HandleTypeDef* hrtc);
static uint32_t           RTC_ReadTimeCounter(RTC_HandleTypeDef* hrtc);
static HAL_StatusTypeDef  RTC_WriteTimeCounter(RTC_HandleTypeDef* hrtc, uint32_t TimeCounter);
/**
  * @brief  Read the time counter available in RTC_CNT registers.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval Time counter
  */
static uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef* hrtc)
{
  uint16_t high1 = 0, high2 = 0, low = 0;
  uint32_t timecounter = 0;

  high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);

  if (high1 != high2)
  { /* In this case the counter roll over during reading of CNTL and CNTH registers,
       read again CNTL register then return the counter value */
    timecounter = (((uint32_t) high2 << 16 ) | READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  { /* No counter roll over during reading of CNTL and CNTH registers, counter
       value is equal to first value of CNTL and CNTH */
    timecounter = (((uint32_t) high1 << 16 ) | low);
  }

  return timecounter;
}

/**
  * @brief  Write the time counter in RTC_CNT registers.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  TimeCounter: Counter to write in RTC_CNT registers
  * @retval HAL status
  */
static HAL_StatusTypeDef RTC_WriteTimeCounter(RTC_HandleTypeDef* hrtc, uint32_t TimeCounter)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Set Initialization mode */
  if(RTC_EnterInitMode(hrtc) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  else
  {
    /* Set RTC COUNTER MSB word */
    WRITE_REG(hrtc->Instance->CNTH, (TimeCounter >> 16));
    /* Set RTC COUNTER LSB word */
    WRITE_REG(hrtc->Instance->CNTL, (TimeCounter & RTC_CNTL_RTC_CNT));

    /* Wait for synchro */
    if(RTC_ExitInitMode(hrtc) != HAL_OK)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}
/**
  * @brief  Enters the RTC Initialization mode.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
static HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef* hrtc)
{
  uint32_t tickstart = 0;

  tickstart = HAL_GetTick();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);


  return HAL_OK;
}

/**
  * @brief  Exit the RTC Initialization mode.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
static HAL_StatusTypeDef RTC_ExitInitMode(RTC_HandleTypeDef* hrtc)
{
  uint32_t tickstart = 0;

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  tickstart = HAL_GetTick();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef HAL_RTC_GetUNIXTime(RTC_HandleTypeDef *hrtc, time_t *unixtime)
{
    /* Check input parameters */
    if((hrtc == NULL) || (unixtime == NULL))
    {
       return HAL_ERROR;
    }

    /* Check if counter overflow occurred */
    if (__HAL_RTC_OVERFLOW_GET_FLAG(hrtc, RTC_FLAG_OW))
    {
        return HAL_ERROR;
    }

    *unixtime = RTC_ReadTimeCounter(hrtc);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_RTC_SetUNIXTime(RTC_HandleTypeDef *hrtc, time_t unixtime)
{
    /* Check input parameters */
    if(hrtc == NULL)
    {
       return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hrtc);

    hrtc->State = HAL_RTC_STATE_BUSY;

    /* Write time counter in RTC registers */
    if (RTC_WriteTimeCounter(hrtc, unixtime) != HAL_OK)
    {
      /* Set RTC state */
      hrtc->State = HAL_RTC_STATE_ERROR;

      /* Process Unlocked */
      __HAL_UNLOCK(hrtc);

      return HAL_ERROR;
    }
    else {
        hrtc->State = HAL_RTC_STATE_READY;

       __HAL_UNLOCK(hrtc);

       return HAL_OK;
    }
}

