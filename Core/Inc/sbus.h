#ifndef SBUS_H
#define SBUS_H

#include<stdint.h>

#define SBUS_LEN 25

typedef struct{
    int16_t rocker_r_0;// 右摇杆
    int16_t rocker_r_1;// 右摇杆
    int16_t rocker_l_1;// 左摇杆
    int16_t rocker_l_0;// 左摇杆
    int16_t SWA;// 开关A
    int16_t SWB;// 开关B
    int16_t SWC;// 开关C
    int16_t SWD;// 开关D
    int16_t VRA;// 旋钮A
    int16_t VRB;// 旋钮B
    uint8_t error_protect;// 故障保护激活位：遥控器关了/失联时置1
    uint8_t byte_lose;// 丢帧标志

}RC_ctrl_t;

extern volatile RC_ctrl_t rc_data;

extern volatile uint32_t sbus_tick;

void sbus_start(void);
uint8_t sbus_online(void);
void sbus_parse(uint8_t *p);

#endif