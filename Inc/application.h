/*
 * PIDlib.h
 *
 *  Created on: Sep 30, 2019
 *      Author: ericson
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"



#define PID					0
#define PPLACE		 		0
#define IDENTFICATION		1


#define DEPRECATE			0


void APP_init();

void APP_deInit();

#endif /* APPLICATION_H_ */
