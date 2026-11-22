#ifndef MOTOR_H
#define MOTOR_H

#include "can.h"
#include <stdint.h>

#define MOTOR_ID 1
#define MOTOR_RX_ID (0x200+MOTOR_ID)
#define CURRENT_FILTER 0.1f

typedef struct
{
    CAN_HandleTypeDef *can_handle;     // CAN句柄
    uint32_t rx_id;                    // 反馈报文 ID
    CAN_TxHeaderTypeDef txconf;        // 控制报文头（0x200、标准帧、数据帧、DLC 8）
    uint8_t  tx_buff[8];               // 控制报文的 8 字节
    uint8_t  rx_buff[8];               // 反馈报文的 8 字节
    uint8_t  rx_len;                   // 反馈报文 DLC
    uint32_t tx_mailbox;
} motor_t;

typedef struct
{
    uint16_t ecd;//原始值
    int16_t  speed_rpm;//转速原始值
    int16_t  real_current;//转矩电流原始值
    uint8_t  temperature;//温度
    float    real_current_handle;//滤波后电流
    float    speed_aps;//滤波后转速
    uint16_t last_ecd;//上一帧编码器值
    float    angle_single_round;//单圈角度
    int32_t  total_round;//累计圈数
    float    total_angle;//累计角度
} motor_measure_t;

extern motor_t motor1;
extern motor_measure_t motor1_measure;

void motor_init(void);
void motor_send(int16_t cmd);
void motor_data_read(void);

#endif // !MOTOR_H

