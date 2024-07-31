/*
 * task.c
 *
 *  Created on: 2024锟斤拷7锟斤拷29锟斤拷
 *      Author: 25016
 */

#include "ti_msp_dl_config.h"
#include "math.h"

#include "encoder.h"
#include "sensor.h"
#include "jy901.h"
#include "motor.h"
#include "utils.h"
#include "task.h"
#include "pid.h"

#define TURN_1 20
#define TURN_2 22
#define TURN_3 24
#define TURN_4 26
#define TURN_5 28
#define TURN_6 35
#define TURN_7 42

static PIDController yaw_pid = {
                         .Kp = 1.5f,
                         .Ki = 0.0f,
                         .Kd = 0.0f,

                         .tau = 0.003f,

                         .limMax = 100.0f,
                         .limMin = -100.0f,

                         .limMaxInt = 50.0f,
                         .limMinInt = -50.0f,

                         .T = 0.02f,    // 20ms = 0.02s

                         .en = 1,
};

static PIDController line_pid = {
                         .Kp = 0.6f,
                         .Ki = 0.0f,
                         .Kd = 0.0f,

                         .tau = 0.003f,

                         .limMax = 100.0f,
                         .limMin = -100.0f,

                         .limMaxInt = 50.0f,
                         .limMinInt = -50.0f,

                         .T = 0.02f,    // 20ms = 0.02s

                         .en = 1,
};

static void light_up_sound_on(int ms, float speed)
{
    motor_A_C0_L_set_speed(speed);
    motor_B_C1_R_set_speed(speed);
    DL_GPIO_setPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_LED_PIN);
    DL_GPIO_setPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_BUZZER_PIN);
    delay_ms(ms);
    DL_GPIO_clearPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_LED_PIN);
    DL_GPIO_clearPins(GPIO_GRP_BORAD_PORT, GPIO_GRP_BORAD_PIN_BUZZER_PIN);
}

void go_slash_no_line(float base_speed, float expect, bool turn_right)
{
    float pid_output = 0;
    yaw_pid.limMin = -base_speed * 100.0f;

    PIDController_Init(&yaw_pid);

    sensor_update();
    while (g_sensor_data == 0xff)
    {
        pid_output = PIDController_Update_Yaw(&yaw_pid, expect, g_jy901_yaw) / 100.0f;

        motor_A_C0_L_set_speed(base_speed - pid_output);
        motor_B_C1_R_set_speed(base_speed + pid_output);

        delay_ms(20);
        sensor_update();
    }
    if (turn_right == true)
    {
        motor_A_C0_L_set_speed(base_speed);
        motor_B_C1_R_set_speed(-base_speed);
        delay_ms(250);
        motor_A_C0_L_set_speed(0);
        motor_B_C1_R_set_speed(0);
    }
    else
    {
        motor_A_C0_L_set_speed(-base_speed);
        motor_B_C1_R_set_speed(base_speed);
        delay_ms(250);
        motor_A_C0_L_set_speed(0);
        motor_B_C1_R_set_speed(0);
    }
}

void go_straight_no_line(float base_speed, float expect)
{
    float pid_output = 0;
    yaw_pid.limMin = -base_speed * 100.0f;

    PIDController_Init(&yaw_pid);

    sensor_update();
    while (g_sensor_data == 0xff)
    {
        pid_output = PIDController_Update_Yaw(&yaw_pid, expect, g_jy901_yaw) / 100.0f;

        motor_A_C0_L_set_speed(base_speed - pid_output);
        motor_B_C1_R_set_speed(base_speed + pid_output);

        delay_ms(20);
        sensor_update();
    }
}

void track_line(float base_speed, bool only_turn_right, float enter_yaw)
{
    float expect_turn_yaw = enter_yaw + 180;
    if (expect_turn_yaw >= 180)
        expect_turn_yaw = expect_turn_yaw - 360;
    float pid_output = 0;
    uint8_t temp_sensor_data = 0;
    float feedback = 0;

    PIDController_Init(&line_pid);

    sensor_update();
    while (g_sensor_data != 0xff || fabs(expect_turn_yaw - g_jy901_yaw) >= 35)
    {
        temp_sensor_data = ~g_sensor_data;
        switch (temp_sensor_data)
        {
            /*锟斤拷锟斤拷*/
            case 0x18:
                feedback = 0;
                break;
            /*锟揭侧（5~8锟斤拷锟斤拷锟竭ｏ拷锟斤拷锟斤拷*/
            case 0x10:
                feedback = TURN_1;
                break;
            case 0x30:
                feedback = TURN_2;
                break;
            case 0x20:
                feedback = TURN_3;
                break;
            case 0x60:
                feedback = TURN_4;
                break;
            case 0x40:
                feedback = TURN_5;
                break;
            case 0xc0:
                feedback = TURN_6;
                break;
            case 0x80:
                feedback = TURN_7;
                break;
            /*锟斤拷啵�1~4锟斤拷锟斤拷锟竭ｏ拷锟斤拷锟斤拷*/
            case 0x01:
                feedback = -TURN_7;
                break;
            case 0x03:
                feedback = -TURN_6;
                break;
            case 0x02:
                feedback = -TURN_5;
                break;
            case 0x06:
                feedback = -TURN_4;
                break;
            case 0x04:
                feedback = -TURN_3;
                break;
            case 0x0c:
                feedback = -TURN_2;
                break;
            case 0x08:
                feedback = -TURN_1;
                break;
            default:    //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷止pid锟斤拷锟斤拷锟戒化锟届常
                break;
        }
        if (only_turn_right == true && feedback < 0)
            feedback = feedback * 0.5f;
        if (only_turn_right == false && feedback > 0)
            feedback = feedback * 0.5f;
        pid_output = PIDController_Update(&line_pid, 0, feedback) / 100.0f;
        motor_A_C0_L_set_speed(base_speed - pid_output);
        motor_B_C1_R_set_speed(base_speed + pid_output);
        delay_ms(20);
        sensor_update();
    }
}

void task_test()
{
    for (int i = 0; i <= 100; i++)
    {
        motor_A_C0_L_set_speed(0.01*i);
        delay_ms(20);
    }
    motor_A_C0_L_set_speed(0);
    for (int i = 0; i <= 100; i++)
    {
        motor_B_C1_R_set_speed(0.01*i);
        delay_ms(20);
    }
    motor_B_C1_R_set_speed(0);
}

#define TASK1_BASE_SPEED 0.6f

void task1()
{
    // start
    light_up_sound_on(100, 0);

    // routine
    go_straight_no_line(TASK1_BASE_SPEED, g_jy901_yaw);

    // end
    light_up_sound_on(800, 0);

    for (;;);
}

#undef TASK1_BASE_SPEED

#define TASK2_LINE_BASE_SPEED 0.45f
#define TASK2_NO_LINE_BASE_SPEED 0.6f
#define TASK2_SLOW_BASE_SPEED 0.4f

void task2()
{
    // start
    light_up_sound_on(100, 0);

    // go to B
    go_straight_no_line(TASK2_NO_LINE_BASE_SPEED, g_jy901_yaw);

    // get B
    light_up_sound_on(100, TASK2_SLOW_BASE_SPEED);

    // go to C
    track_line(TASK2_LINE_BASE_SPEED, true, g_jy901_yaw);

    // get C
    light_up_sound_on(100, TASK2_SLOW_BASE_SPEED);

    // go to D
    go_straight_no_line(TASK2_NO_LINE_BASE_SPEED, g_jy901_yaw - 2.5); // offset

    // get D
    light_up_sound_on(100, TASK2_SLOW_BASE_SPEED);

    // go to A
    track_line(TASK2_LINE_BASE_SPEED, true, g_jy901_yaw);

    // get A
    light_up_sound_on(800, 0);

    for (;;);
}

#undef TASK2_LINE_BASE_SPEED
#undef TASK2_NO_LINE_BASE_SPEED
#undef TASK2_SLOW_BASE_SPEED

void task3()
{
    // go to C slash
    go_slash_no_line(0.7f, g_jy901_yaw, false);

    // get C
    light_up_sound_on(100, 0.4);

    // go to B
    track_line(0.4, false, g_jy901_yaw);

    //get B
    light_up_sound_on(100, 0.4);

    // go to D slash
    go_slash_no_line(0.7f, g_jy901_yaw + 41.0f, true); // turn left a bit

    // get D
    light_up_sound_on(100, 0.4);

    // go to A
    track_line(0.45, true, g_jy901_yaw);

    // get A
    light_up_sound_on(800, 0);

    for (;;);
}

#define TASK4_LINE_BASE_SPEED 0.4f
#define TASK4_NO_LINE_BASE_SPEED 0.7f
#define TASK4_SLOW_BASE_SPEED 0.35f

void task4()
{
    // go to C slash
    go_slash_no_line(TASK4_NO_LINE_BASE_SPEED, g_jy901_yaw, false);

    // get C
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to B
    track_line(TASK4_LINE_BASE_SPEED, false, g_jy901_yaw);

    //get B
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to D slash
    go_slash_no_line(TASK4_NO_LINE_BASE_SPEED, g_jy901_yaw + 40.5f, true); // turn left a bit

    // get D
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to A
    track_line(TASK4_LINE_BASE_SPEED, true, g_jy901_yaw);

    // get A
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // round 2---------------------------------------------------------------

    // go to C slash
    go_slash_no_line(TASK4_NO_LINE_BASE_SPEED, g_jy901_yaw - 41.0f, false); // turn right a bit

    // get C
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to B
    track_line(TASK4_LINE_BASE_SPEED, false, g_jy901_yaw);

    //get B
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to D slash
    go_slash_no_line(TASK4_NO_LINE_BASE_SPEED, g_jy901_yaw + 41.0f, true); // turn left a bit

    // get D
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to A
    track_line(TASK4_LINE_BASE_SPEED, true, g_jy901_yaw);

    // get A
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // round 3---------------------------------------------------------------

    // go to C slash
    go_slash_no_line(TASK4_NO_LINE_BASE_SPEED, g_jy901_yaw - 41.0f, false); // turn right a bit

    // get C
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to B
    track_line(TASK4_LINE_BASE_SPEED, false, g_jy901_yaw);

    //get B
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to D slash
    go_slash_no_line(TASK4_NO_LINE_BASE_SPEED, g_jy901_yaw + 40.5f, true); // turn left a bit

    // get D
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to A
    track_line(TASK4_LINE_BASE_SPEED, true, g_jy901_yaw);

    // get A
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // round 4---------------------------------------------------------------

    // go to C slash
    go_slash_no_line(TASK4_NO_LINE_BASE_SPEED, g_jy901_yaw - 41.0f, false); // turn right a bit

    // get C
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to B
    track_line(TASK4_LINE_BASE_SPEED, false, g_jy901_yaw);

    //get B
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to D slash
    go_slash_no_line(TASK4_NO_LINE_BASE_SPEED, g_jy901_yaw + 41.0f, true); // turn left a bit

    // get D
    light_up_sound_on(100, TASK4_SLOW_BASE_SPEED);

    // go to A
    track_line(TASK4_LINE_BASE_SPEED, true, g_jy901_yaw);

    // get A
    light_up_sound_on(800, 0);

    for (;;);
}

#undef TASK4_LINE_BASE_SPEED
#undef TASK4_NO_LINE_BASE_SPEED
#undef TASK4_SLOW_BASE_SPEED
