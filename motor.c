/*
 * motor.c
 *
 *  Created on: 2024Äê7ÔÂ29ÈÕ
 *      Author: 25016
 */

#include "motor.h"
#include "utils.h"

void motor_A_C0_L_set_speed(float percent) // 0.0 ~ 1.0 AIN
{
    percent = constrain(percent, -1.0f, 1.0f);
    if (0.0f == percent)
    {
        DL_GPIO_clearPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_AIN2_PIN);
        return;
    }
    if (percent < 0.0f) // forward
    {
        DL_GPIO_clearPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_AIN1_PIN);
        DL_GPIO_setPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_AIN2_PIN);
    }
    else // backward
    {
        DL_GPIO_setPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_AIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)(512.0f * percent), GPIO_PWM_MOTOR_C0_IDX); // 512 is Period Count
}

void motor_B_C1_R_set_speed(float percent) // -1.0 ~ 1.0 BIN
{
    percent = constrain(percent, -1.0f, 1.0f);
    if (0.0f == percent)
    {
        DL_GPIO_clearPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_BIN2_PIN);
        return;
    }
    if (percent < 0.0f) // forward
    {
        DL_GPIO_clearPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_BIN1_PIN);
        DL_GPIO_setPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_BIN2_PIN);
    }
    else // backward
    {
        DL_GPIO_setPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_GRP_MOTOR_PORT, GPIO_GRP_MOTOR_PIN_BIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)(512.0f * percent), GPIO_PWM_MOTOR_C1_IDX); // 512 is Period Count
}
