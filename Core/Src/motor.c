#include"motor.h"
#include<string.h>

motor_t motor1;
motor_measure_t motor1_measure;

void motor_init(void)
{
    motor1.can_handle=&hcan1;
    motor1.rx_id =MOTOR_RX_ID;

    motor1.txconf.StdId=0x200;
    motor1.txconf.IDE=CAN_ID_STD;
    motor1.txconf.RTR=CAN_RTR_DATA;
    motor1.txconf.DLC=8;
    
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.SlaveStartFilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    filter.FilterActivation = CAN_FILTER_ENABLE;

    filter.FilterIdHigh     = motor1.rx_id << 5;
    filter.FilterIdLow      = motor1.rx_id << 5;
    filter.FilterMaskIdHigh = motor1.rx_id << 5;
    filter.FilterMaskIdLow  = motor1.rx_id << 5;

    HAL_CAN_ConfigFilter(&hcan1, &filter);

    HAL_CAN_ConfigFilter(&hcan1,&filter);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

}

void motor_data_read(void)
{
    motor1_measure.ecd         = (uint16_t)((motor1.rx_buff[0] << 8) | motor1.rx_buff[1]);
    motor1_measure.speed_rpm   = (int16_t)((motor1.rx_buff[2] << 8) | motor1.rx_buff[3]);
    motor1_measure.real_current = (int16_t)((motor1.rx_buff[4] << 8) | motor1.rx_buff[5]);
    motor1_measure.temperature = motor1.rx_buff[6];
}

void motor_send(int16_t cmd)
{
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0)
    {
        return;
    }

    uint16_t raw=(uint16_t)cmd;

    motor1.tx_buff[0]=(uint8_t)(raw>>8);
    motor1.tx_buff[1] = (uint8_t)(raw & 0xFFU);
    for(uint8_t i=2;i<8;i++)
    {
        motor1.tx_buff[i]=0;
    }
    HAL_CAN_AddTxMessage(&hcan1,&motor1.txconf,motor1.tx_buff,&motor1.tx_mailbox);
}

static void can_fifo_callback(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
    CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan,fifox))
    {
        HAL_CAN_GetRxMessage(hcan, fifox, &rxconf, can_rx_buff);
        
        if(hcan==motor1.can_handle && rxconf.StdId==motor1.rx_id)
        {
            memcpy(motor1.rx_buff,can_rx_buff,rxconf.DLC);
            motor1_measure.last_ecd=motor1_measure.ecd;

            motor_data_read();

            motor1_measure.angle_single_round = 360.0f / 8192.0f * (float)motor1_measure.ecd;
            motor1_measure.speed_aps = (1.0f - 0.85f) * motor1_measure.speed_aps + 0.85f * (360.0f / 60.0f) * (float)motor1_measure.speed_rpm;
            motor1_measure.real_current_handle = (1.0f - CURRENT_FILTER) * motor1_measure.real_current_handle + CURRENT_FILTER * (float)motor1_measure.real_current;
            if (motor1_measure.ecd - motor1_measure.last_ecd > 4096)
            {
                motor1_measure.total_round--;
            }
            else if (motor1_measure.ecd - motor1_measure.last_ecd < -4096)
            {
                motor1_measure.total_round++;
            }
            motor1_measure.total_angle = (float)motor1_measure.total_round * 360.0f + motor1_measure.angle_single_round;
            return;
        }   
    }
    
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_fifo_callback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_fifo_callback(hcan, CAN_RX_FIFO1);
}