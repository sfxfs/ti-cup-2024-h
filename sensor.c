/*
 * sensor.c
 *
 *  Created on: 2024Äê7ÔÂ29ÈÕ
 *      Author: 25016
 */

#include "ti_msp_dl_config.h"

#include "i2c_app.h"
#include "sensor.h"

volatile uint8_t g_sensor_data;

void sensor_init()
{
    uint8_t data = GW_GRAY_DIGITAL_MODE;
    i2c_app_write(I2C_SENSOR_INST, GW_GRAY_ADDR_DEF, &data, 1);
}

void sensor_update()
{
    i2c_app_read(I2C_SENSOR_INST, GW_GRAY_ADDR_DEF, (uint8_t *)&g_sensor_data, 1);
}
