#include "sbus.h"
#include "usart.h"
#include <string.h>

#define SBUS_OFFLINE_MS 100

static void rc_map(void);

volatile RC_ctrl_t rc_data;
volatile uint32_t sbus_tick;

int16_t CH[10];//调零后原始数据

static uint8_t buf[SBUS_LEN];
static volatile uint8_t rx_flag;

void sbus_start(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buf,SBUS_LEN);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance==USART3)
    {
        if(Size==SBUS_LEN&&buf[0]==0x0f)
        {
            sbus_parse(buf);
            sbus_tick =HAL_GetTick();
        }
        rx_flag=1;
        sbus_start();
    }
}

uint8_t sbus_online(void)
{
    return (HAL_GetTick()-sbus_tick)<SBUS_OFFLINE_MS;
}

void sbus_parse(uint8_t *p)
{
    uint32_t acc=0;
    uint8_t bits=0;
    uint8_t i=0;

    for(uint8_t b=1;b<=22&&i<10;b++)
    {
        acc |=(uint32_t)p[b]<<bits;
        bits+=8;

        while(bits>=11&&i<10)
        {
            uint16_t raw=(uint16_t)(acc&0b11111111111);
            CH[i]=(int16_t)raw-1024;
            i++;
            acc>>=11;
            bits-=11;
        }
    }

    rc_map();

    rc_data.byte_lose    = (p[23] & 0x04) ? 1 : 0;// byte24 状态字节 bit2 = 丢帧
    rc_data.error_protect = (p[23] & 0x08) ? 1 : 0;// bit3 = 失控保护（failsafe）
}

static void rc_map(void)
{
    rc_data.rocker_r_0 = CH[0];
    rc_data.rocker_r_1 = CH[1];
    rc_data.rocker_l_1 = CH[2];
    rc_data.rocker_l_0 = CH[3];
    rc_data.SWA        = CH[4];
    rc_data.SWB        = CH[5];
    rc_data.SWC        = CH[6];
    rc_data.SWD        = CH[7];
    rc_data.VRA        = CH[8];
    rc_data.VRB        = CH[9];
}