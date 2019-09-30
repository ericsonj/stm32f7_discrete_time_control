/*
 * PIDlib.c
 *
 *  Created on: Sep 30, 2019
 *      Author: Ericson Joseph
 */
#include "main.h"
#include "cmsis_os.h"

static osThreadId_t defaultTaskHandle;
static void task10HzSignal(void *pvParam);

void PID_initTask10Hz(){
	 const osThreadAttr_t defaultTask_attributes = {
	    .name = "Task 10Hz",
	    .priority = (osPriority_t) osPriorityLow,
	    .stack_size = 128
	  };
	  defaultTaskHandle = osThreadNew(task10HzSignal, NULL, &defaultTask_attributes);
}

void task10HzSignal(void *pvParam){
	for(;;){
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

void PID_deInitTask10Hz(){

}
