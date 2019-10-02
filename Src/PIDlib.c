/*
 * PIDlib.c
 *
 *  Created on: Sep 30, 2019
 *      Author: Ericson Joseph
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "stm32f7xx_nucleo_144.h"
#include "PIDlib.h"
#include <string.h>
#include <stdio.h>

#define DATA_BUFFER_LEN 1000

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;

static uint32_t dataBuffer[DATA_BUFFER_LEN];
static uint32_t signalBuffer[DATA_BUFFER_LEN];
static char format[] = "%d , %d;\n";
static char buffer[32];

uint32_t count = 0;
uint32_t ADCValue;
TickType_t h_ms = 1 / portTICK_RATE_MS;

static osThreadId_t signalTaskHandle;
static osThreadId_t pidTaskHandle;
static SemaphoreHandle_t pidSemaph;

static void task10HzSignal(void *pvParam);
static void taskPID(void *pvParam);

void PID_initTask10Hz() {
	BSP_LED_Init(LED_BLUE);
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

	const osThreadAttr_t signalTaskAttr = { .name = "Task 10Hz", .priority =
			(osPriority_t) osPriorityNormal, .stack_size = 128 };
	signalTaskHandle = osThreadNew(task10HzSignal, NULL, &signalTaskAttr);

	const osThreadAttr_t pidTaskAttr = { .name = "Task PID", .priority =
			(osPriority_t) osPriorityNormal, .stack_size = 512 };
	pidTaskHandle = osThreadNew(taskPID, NULL, &pidTaskAttr);

	pidSemaph = xSemaphoreCreateBinary();

}

void task10HzSignal(void *pvParam) {
	xSemaphoreGive(pidSemaph);
	for (;;) {
		vTaskDelay(100 / portTICK_RATE_MS);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
		vTaskDelay(100 / portTICK_RATE_MS);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	}
}

void taskPID(void *pvParam) {

	TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	xSemaphoreTake(pidSemaph, portMAX_DELAY);

	for (;;) {

		vTaskDelayUntil(&xLastWakeTime, h_ms);

		HAL_ADC_Start(&hadc1);

		if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK) {
			ADCValue = HAL_ADC_GetValue(&hadc1);
		}

		if (count < DATA_BUFFER_LEN) {
			signalBuffer[count] = HAL_DAC_GetValue(&hdac, DAC1_CHANNEL_1);
			dataBuffer[count] = ADCValue;
			count++;
		} else {
//			BSP_LED_Toggle(LED_BLUE);
			for (int i = 0; i < DATA_BUFFER_LEN; ++i) {
				snprintf(buffer, 32, format, dataBuffer[i], signalBuffer[i]);
				HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer),
						100);
			}
			BSP_LED_Toggle(LED_BLUE);
			vTaskDelay(portMAX_DELAY);
			count = 0;
		}

	}

}

void PID_deInitTask10Hz() {

}
