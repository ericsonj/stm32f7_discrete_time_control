/*
 * pidUtil.h
 *
 *  Created on: Oct 2, 2019
 *      Author: ericson
 */

#ifndef PIDUTIL_H_
#define PIDUTIL_H_

#include "stm32f7xx_hal.h"

uint32_t PIDUTIL_getValue(ADC_HandleTypeDef *adc);

void PIDUTIL_ftoa(float n, char *res, int afterpoint);

#endif /* PIDUTIL_H_ */
