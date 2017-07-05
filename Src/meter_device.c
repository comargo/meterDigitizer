#include "meter_device.h"
#include "hal_rtc_unix.h"
#include "usbd_cdc_if.h"
#include "rtc.h"

#define DEV0_BKUP_REGISTER RTC_BKP_DR2
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

static void CheckHighCounterState();
static void LoadMeterDevicesState(FLASH_Counter_Source source);
static int SelectNearestDevice();
static void SaveCounter(int dev);
static void WriteHighCounterToFlash(FLASH_Counter_Source source);
static HAL_StatusTypeDef BackupHighCounter();

HAL_StatusTypeDef MeterDevicesLoadValues()
{
    CheckHighCounterState();
    LoadMeterDevicesState(FLASH_Counter_Main);
    return HAL_OK;
}

HAL_StatusTypeDef MeterDevicesProcess()
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
            MeterDeviceShowValue(dev);
            SaveCounter(dev);
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef MeterDeviceSetValue(uint dev, uint32_t val)
{
    meterDevicesState[dev].counter = val;
    MeterDeviceShowValue(dev);
    SaveCounter(dev);
    return HAL_OK;
}

HAL_StatusTypeDef MeterDeviceShowValue(uint dev)
{
    char counterMsg[256];
    time_t unixtime = 0;
    HAL_RTC_GetUNIXTime(&hrtc, &unixtime);
    struct tm calendarTime;
    gmtime_r(&unixtime, &calendarTime);

    uint32_t intCnt = meterDevicesState[dev].counter/1000;
    uint16_t fracCnt =  meterDevicesState[dev].counter%1000;
    sprintf(counterMsg,"%04d-%02d-%02dT%02d:%02d:%02d\t%d\t%s\t%05u.%03u\r\n",
            calendarTime.tm_year+1900, calendarTime.tm_mon+1, calendarTime.tm_mday,
            calendarTime.tm_hour, calendarTime.tm_min, calendarTime.tm_sec,
            dev, meterDevicesPin[dev].name,
            (uint)intCnt, (uint)fracCnt);
    CDC_Transmit_FS((uint8_t*)counterMsg, strlen(counterMsg));
    CDC_WaitForTransmit();
    return HAL_OK;
}

void MeterDeviceInterrupt(uint16_t GPIO_Pin)
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

void CheckHighCounterState()
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

        meterDevicesState[i].counter |= HAL_RTCEx_BKUPRead(&hrtc, DEV0_BKUP_REGISTER+i);
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

void SaveCounter(int dev)
{
    uint16_t cntHigh = (meterDevicesState[dev].counter & 0xFFFF0000)>>16;
    uint16_t cntLow = (meterDevicesState[dev].counter & 0xFFFF);
    if(meterDevicesHighCounter->counter[dev] != cntHigh) {
        WriteHighCounterToFlash(FLASH_Counter_Main);
    }
    HAL_RTCEx_BKUPWrite(&hrtc, DEV0_BKUP_REGISTER+dev, cntLow);
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

