/*
 * utils.h
 *
 *  Created on: 2024��7��29��
 *      Author: 25016
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "ti_msp_dl_config.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define delay_us(us) delay_cycles(CPUCLK_FREQ / 1000000 * (us))
#define delay_ms(ms) delay_cycles(CPUCLK_FREQ / 1000 * (ms))

#endif /* UTILS_H_ */
