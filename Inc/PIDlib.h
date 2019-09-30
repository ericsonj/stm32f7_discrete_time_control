/*
 * PIDlib.h
 *
 *  Created on: Sep 30, 2019
 *      Author: ericson
 */

#ifndef PIDLIB_H_
#define PIDLIB_H_


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


void PID_initTask10Hz();

void PID_deInitTask10Hz();

#endif /* PIDLIB_H_ */
