/*
 * jy901.c
 *
 *  Created on: 2024年7月29日
 *      Author: 25016
 */

#include "ti_msp_dl_config.h"

#define JY901_PACKET_LENGTH 11

union short_div {
    short val;
    uint8_t div[2];
};

static union short_div angle_yaw;

volatile float g_jy901_yaw;

void jy901_int_enable()
{
    NVIC_ClearPendingIRQ(UART_JY901_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_JY901_INST_INT_IRQN);
}

static void copeJY901_data(uint8_t data)
{
    static uint8_t rxBuffer[JY901_PACKET_LENGTH] = {0}; // 数据包
    static uint8_t rxCount = 0;        // 接收计数

    rxBuffer[rxCount++] = data; // 将收到的数据存入缓冲区中

    if (rxBuffer[0] != 0x55)
    {
        // 数据头不对，则重新开始寻找0x55数据头
        rxCount = 0; // 清空缓存区
        return;
    }
    if (rxCount < JY901_PACKET_LENGTH)
        return; // 数据不满11个，则返回

    /*********** 只有接收满11个字节数据 才会进入以下程序 ************/
    uint8_t rxCheck = 0;
    for (int i = 0; i < JY901_PACKET_LENGTH - 1; i++)
        rxCheck += rxBuffer[i];

    if (0x53 == rxBuffer[1] && rxCheck == rxBuffer[JY901_PACKET_LENGTH - 1]) // 判断数据包校验是否正确
    {
        angle_yaw.div[0] = rxBuffer[6];
        angle_yaw.div[1] = rxBuffer[7];
        g_jy901_yaw = angle_yaw.val  / 32768.0f * 180.0f;
    }
    rxCount = 0;
}

void UART_JY901_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_JY901_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            copeJY901_data(DL_UART_Main_receiveData(UART_JY901_INST));
            break;
        default:
            break;
    }
}
