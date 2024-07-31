/*
 * i2c_app.c
 *
 *  Created on: 2024��7��29��
 *      Author: 25016
 */

#include "i2c_app.h"

#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

static __IO uint32_t  I2CTimeout = I2CT_LONG_TIMEOUT;

uint8_t i2c_app_write (I2C_Regs *i2c, uint8_t addr, uint8_t *dat, uint8_t len)
{
    DL_I2C_fillControllerTXFIFO(i2c, dat, len);

    I2CTimeout = I2CT_FLAG_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE)) if ((I2CTimeout--) == 0) return 1;

    DL_I2C_startControllerTransfer(i2c, addr, DL_I2C_CONTROLLER_DIRECTION_TX, len);

    I2CTimeout = I2CT_FLAG_TIMEOUT;
    while (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) if ((I2CTimeout--) == 0) return 2;

    if (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) return 1;

    I2CTimeout = I2CT_FLAG_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE)) if ((I2CTimeout--) == 0) return 3;

    return 0;
}

uint8_t i2c_app_read (I2C_Regs *i2c, uint8_t addr, uint8_t *dat, uint8_t len)
{
    DL_I2C_startControllerTransfer(i2c, addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (uint8_t i = 0; i < len; i++) {
        I2CTimeout = I2CT_FLAG_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(i2c)) if ((I2CTimeout--) == 0) return 1;
        dat[i] = DL_I2C_receiveControllerData(i2c);
    }

    return 0;
}
