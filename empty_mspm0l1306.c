/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include "encoder.h"
#include "sensor.h"
#include "jy901.h"
#include "motor.h"
#include "utils.h"
#include "task.h"

int main(void)
{
    SYSCFG_DL_init();

    jy901_int_enable();
    encoder_int_enable();
    sensor_init();

    delay_ms(1100); // wait for jy901 boot up

    int task_index = 0;

    DL_GPIO_setPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_LED_PIN);   // gets ready
    while (1) {
        if (DL_GPIO_readPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_BUTTON_PIN))
        {
            delay_ms(400);
            if (DL_GPIO_readPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_BUTTON_PIN))
            {
                DL_GPIO_clearPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_LED_PIN); // led off
                switch (task_index)
                {
                case 0:
                    task_test();
                    break;
                case 1:
                    task1();
                    break;
                case 2:
                    task2();
                    break;
                case 3:
                    task3();
                    break;
                case 4:
                    task4();
                    break;
                default:
                    break;
                }
            }
            else
            {
                task_index++;
                for (int i = 0; i < task_index; i++)
                {
                    DL_GPIO_setPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_LED_PIN); // led on
                    delay_ms(400);
                    DL_GPIO_clearPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_LED_PIN); // led off
                    delay_ms(400);
                }
            }
        }
    }
}