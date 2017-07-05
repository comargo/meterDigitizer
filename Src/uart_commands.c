#include "uart_commands.h"
#include "usbd_cdc_if.h"
#include "meter_device.h"
#include "rtc.h"
#include "hal_rtc_unix.h"

typedef HAL_StatusTypeDef (*CmdProccessor)(const char *cmd);
struct UARTCommand {
    const char *cmd;
    CmdProccessor processor;
};

static CmdProccessor FindCommand(const char* cmd);
static HAL_StatusTypeDef CmdSetTime(const char *cmd);
static HAL_StatusTypeDef CmdListMeters(const char *cmd);
static HAL_StatusTypeDef CmdSetMeter(const char *cmd);
static HAL_StatusTypeDef CmdSetEcho(const char *cmd);

const struct UARTCommand uartCommands[] = {
{ .cmd = "SET TIME ", .processor = &CmdSetTime },
{ .cmd = "LIST", .processor = &CmdListMeters },
{ .cmd = "SET METER ", .processor = &CmdSetMeter },
{ .cmd = "ECHO ", .processor = &CmdSetEcho },
{ .cmd = NULL, .processor = NULL }
};

const char ATError[] = "Error\r\n";
const char ATOK[] = "OK\r\n";


HAL_StatusTypeDef UARTCommandsProcess()
{
    const int size = 2*APP_RX_DATA_SIZE;
    int head = USBRxCircBuf.head;
    int tail = USBRxCircBuf.tail;
    if(CIRC_CNT(head, tail, size)) {
        for(int i=tail; i!=head; i=(i+1)&(size-1)) {
            if(USBRxCircBuf.buf[i] == '\r') {
                int cmdSize = CIRC_CNT(i, tail, size)+1;
                int copyLeft = cmdSize-1;
                char *cmd = malloc(cmdSize);
                char *tmp = cmd;
                while(copyLeft) {
                    int copySize = CIRC_CNT_TO_END(i, tail, size);
                    memcpy(tmp, USBRxCircBuf.buf+tail, copySize);
                    tail = (tail+copySize)&(size-1);
                    copyLeft -= copySize;
                    tmp += copySize;
                }
                cmd[cmdSize-1] = 0;
                CmdProccessor processor = FindCommand(cmd);
                HAL_StatusTypeDef status = HAL_ERROR;
                if(processor) {
                    status = processor(cmd);
                }
                if(status == HAL_OK)  {
                    CDC_Transmit_FS((uint8_t*)ATOK, sizeof(ATOK)-1);
                }
                else  {
                    CDC_Transmit_FS((uint8_t*)ATError, sizeof(ATError)-1);
                }
                free(cmd);
                tail = (tail+1)&(size-1);
            }
        }
        USBRxCircBuf.tail = tail;
    }
    return HAL_OK;
}

CmdProccessor FindCommand(const char *cmd)
{
    const struct UARTCommand *uartCmd;
    for(uartCmd = uartCommands; uartCmd->cmd; ++uartCmd) {
        if(strncmp(uartCmd->cmd, cmd, strlen(uartCmd->cmd)) == 0) {
            break;
        }
    }
    return uartCmd->processor;
}

HAL_StatusTypeDef CmdSetEcho(const char *cmd)
{
    if(strcmp(cmd, "ECHO ON") == 0) {
        USBEchoState = ECHO_ON;
        return HAL_OK;
    }
    else if(strcmp(cmd, "ECHO OFF") == 0) {
        USBEchoState = ECHO_OFF;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef CmdListMeters(const char *cmd)
{
    for(int dev = 0; dev < NUM_METER_DEVICE; ++dev) {
        MeterDeviceShowValue(dev);
    }
    return HAL_OK;
}

HAL_StatusTypeDef CmdSetMeter(const char *cmd)
{
    uint dev, intVal;
    char fracValStr[4] = {0,0,0,0};
    int fields = sscanf(cmd, "SET METER %u %u.%3s", &dev, &intVal, fracValStr);
    if(fields != 3)
        return HAL_ERROR;
    fracValStr[3] = 0;
    for(int i=0; i<3; ++i) {
        if(fracValStr[2-i] == 0) fracValStr[2-i] = '0';
    }
    int fracVal = atoi(fracValStr);
    MeterDeviceSetValue(dev, intVal*1000+fracVal);
    return HAL_OK;
}

HAL_StatusTypeDef CmdSetTime(const char *cmd)
{
    struct tm calendarTime;
    int fields = sscanf(cmd, "SET TIME %u-%u-%uT%u:%u:%u"
                        , &calendarTime.tm_year, &calendarTime.tm_mon, &calendarTime.tm_mday
                        , &calendarTime.tm_hour, &calendarTime.tm_min, &calendarTime.tm_sec);
    if(fields != 6)
        return HAL_ERROR;
    calendarTime.tm_mon--;
    calendarTime.tm_year -= 1900;
    calendarTime.tm_wday = -1;
    calendarTime.tm_yday = -1;
    calendarTime.tm_isdst = 0;
    HAL_RTC_SetUNIXTime(&hrtc, mktime(&calendarTime));
    return HAL_OK;
}

