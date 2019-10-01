/*
 * PIDlib.c
 *
 *  Created on: Sep 30, 2019
 *      Author: Ericson Joseph
 */
#include "main.h"
#include "cmsis_os.h"

extern DAC_HandleTypeDef hdac;

static osThreadId_t defaultTaskHandle;
static void task10HzSignal(void *pvParam);

void PID_initTask10Hz() {
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	const osThreadAttr_t defaultTask_attributes = { .name = "Task 10Hz",
			.priority = (osPriority_t) osPriorityLow, .stack_size = 128 };
	defaultTaskHandle = osThreadNew(task10HzSignal, NULL,
			&defaultTask_attributes);
}

void task10HzSignal(void *pvParam) {
	for (;;) {
		vTaskDelay(100 / portTICK_RATE_MS);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
		vTaskDelay(100 / portTICK_RATE_MS);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	}
}

void PID_deInitTask10Hz() {

}
