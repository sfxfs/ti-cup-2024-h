/*
 * encoder.c
 *
 *  Created on: 2024Äê7ÔÂ29ÈÕ
 *      Author: 25016
 */

#include "ti_msp_dl_config.h"

volatile long g_encoder_count;

void encoder_int_enable()
{
    NVIC_EnableIRQ(GPIO_GRP_ENCODER_INT_IRQN);
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
    case GPIO_GRP_ENCODER_INT_IIDX:
        switch (DL_GPIO_getPendingInterrupt(GPIO_GRP_ENCODER_PORT))
        {
        case GPIO_GRP_ENCODER_PIN_A_IIDX:
            if (DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_B_PIN))
                g_encoder_count--;
            else
                g_encoder_count++;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}
