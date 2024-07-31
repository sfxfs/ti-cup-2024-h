/*
 * jy901.c
 *
 *  Created on: 2024��7��29��
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
    static uint8_t rxBuffer[JY901_PACKET_LENGTH] = {0}; // ���ݰ�
    static uint8_t rxCount = 0;        // ���ռ���

    rxBuffer[rxCount++] = data; // ���յ������ݴ��뻺������

    if (rxBuffer[0] != 0x55)
    {
        // ����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
        rxCount = 0; // ��ջ�����
        return;
    }
    if (rxCount < JY901_PACKET_LENGTH)
        return; // ���ݲ���11�����򷵻�

    /*********** ֻ�н�����11���ֽ����� �Ż�������³��� ************/
    uint8_t rxCheck = 0;
    for (int i = 0; i < JY901_PACKET_LENGTH - 1; i++)
        rxCheck += rxBuffer[i];

    if (0x53 == rxBuffer[1] && rxCheck == rxBuffer[JY901_PACKET_LENGTH - 1]) // �ж����ݰ�У���Ƿ���ȷ
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
