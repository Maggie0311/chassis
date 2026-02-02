#include "drv_UartSendCtrl.h"

#include "usart.h"

#include "stm32f3xx_hal.h"

#include "drv_Queue.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

VOFA_Send_t Send_Struct;

static void uart_sendbuff( uint8_t *buff, uint32_t len)
{
    // 这里必须先关闭对应的通道，不然在单通道模式下将不能再次发送
    HAL_UART_Abort_IT(&huart1);
    HAL_UART_Transmit_DMA(&huart1, buff, len);
}

/**
@brief 将数据添加到下次发送

@param Data 数据
@return int 0————添加成功，-1————添加失败
 */
int VOFA_Add_SendBuff(float Data)
{
    if(Send_Struct.Write_P_f < BUFF_NUM_MAX - 1)
    {
        Send_Struct.Send_Buff.f[Send_Struct.Write_P_f] = Data;
        Send_Struct.Write_P_f ++;
        Send_Struct.Write_P_u8 += 4;
    }
    else
        return -1;
    return 0;
}

/**
@brief 以 VOFA+ 的 JustFloat 格式发送数据的函数

@param buff 存储数据的数组
@param num 发送的浮点数的个数
@param div 发送时的分频系数
 */
void VOFA_JustFloat_Send(uint8_t div,uint32_t current_time)
{
    if( !(current_time % div) )
    {
        Send_Struct.Send_Buff.u8[Send_Struct.Write_P_u8]     = 0x00;
        Send_Struct.Send_Buff.u8[Send_Struct.Write_P_u8 + 1] = 0x00;
        Send_Struct.Send_Buff.u8[Send_Struct.Write_P_u8 + 2] = 0x80;
        Send_Struct.Send_Buff.u8[Send_Struct.Write_P_u8 + 3] = 0x7f;

        uart_sendbuff(Send_Struct.Send_Buff.u8 ,Send_Struct.Write_P_u8 + 4 );

        
    }
    Send_Struct.Write_P_f = 0;
    Send_Struct.Write_P_u8 = 0;
}
