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
#include "pidUtil.h"
#include <string.h>
#include <stdio.h>
#include "pplace.h"
#include "pid_controller.h"

#define DATA_BUFFER_LEN 600
#define _1V2  				(int)((float)1.2/(float)3.3)*4095))

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;

static uint32_t dataBuffer[DATA_BUFFER_LEN];
static uint32_t signalBuffer[DATA_BUFFER_LEN];
static uint32_t errorBuffer[DATA_BUFFER_LEN];

static float dataBufferFloat[DATA_BUFFER_LEN];
static float signalBufferFloat[DATA_BUFFER_LEN];
static float errorBufferFloat[DATA_BUFFER_LEN];

static char format[] = "%d , %d, %d;\n";
static char formatFloat[] = "%s , %s , %s ;\n";
static char buffer[64];

uint32_t count = 0;
uint32_t ADCValue;

uint32_t r_value;
uint32_t y_value;
int32_t e_value;
uint32_t u_value;

float r_fValue;
float y_fValue;
float e_fValue;
float u_fValue;

PIDController_t PsPIDController;

pplace_config_t pplace_config;


TickType_t h_ms = 1 / portTICK_RATE_MS;

static osThreadId_t signalTaskHandle;
static osThreadId_t pidTaskHandle;
static SemaphoreHandle_t pidSemaph;

static void task10HzSignal(void *pvParam);
static void taskPID(void *pvParam);

static void saveAndPrintData(uint32_t y, uint32_t r, uint32_t e);
static void saveAndPrintDataFloat(float y, float r, float e);

static uint32_t getRValue();
static uint32_t getYValue();
static void setUValue(uint32_t value);

static float getR_fValue();
static float getY_fValue();
static void setU_fValue(float value);

void PID_initTask10Hz() {

	BSP_LED_Init(LED_BLUE);
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

//  Punto 5 PID
//	pidInit(&PsPIDController, 5.0,                   // Kp
//			1.0,                                      // Ki
//			0.0,                                      // Kd
//			((float) h_ms) / 1000.0f,                 // h en [s]
//			20.0f,                 // N
//			1.2f,                 // b
//			0.05f,                 // u_min
//			3.3f                   // u_max
//			);

	pidInit(&PsPIDController, 15.0,                   // Kp
			1.0,                                      // Ki
			1.0,                                      // Kd
			((float) h_ms) / 1000.0f,                 // h en [s]
			10.0f,                 // N
			1.0f,                 // b
			0.05f,                 // u_min
			3.3f                   // u_max
			);

//        Punto 7
//        0.09516
// y1:  ----------
//       z - 0.9048
	pplace_config.Ad = 0.90484;
	pplace_config.Bd = 0.095163;
	pplace_config.Cd = 1.0;

	pplace_init(&pplace_config, 0.4);

	const osThreadAttr_t signalTaskAttr = { .name = "Task 10Hz", .priority =
			(osPriority_t) osPriorityNormal, .stack_size = 128 };
	signalTaskHandle = osThreadNew(task10HzSignal, NULL, &signalTaskAttr);

	const osThreadAttr_t pidTaskAttr = { .name = "Task PID", .priority =
			(osPriority_t) osPriorityNormal, .stack_size = 1024 };
	pidTaskHandle = osThreadNew(taskPID, NULL, &pidTaskAttr);

	pidSemaph = xSemaphoreCreateBinary();

}

void task10HzSignal(void *pvParam) {
	xSemaphoreGive(pidSemaph);
	for (;;) {
		vTaskDelay(100 / portTICK_RATE_MS);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 2000);
		vTaskDelay(100 / portTICK_RATE_MS);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	}
}

void taskPID(void *pvParam) {

	TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	xSemaphoreTake(pidSemaph, portMAX_DELAY);

	float array_y[2];
	array_y[0] = 0.0;
	array_y[1] = 0.0;

	for (;;) {

		vTaskDelayUntil(&xLastWakeTime, h_ms);

		r_fValue = getR_fValue();
		y_fValue = getY_fValue();

		e_fValue = r_fValue - y_fValue;

//		u_fValue = pidCalculateControllerOutput(&PsPIDController, y_fValue,
//				r_fValue);
//
		array_y[0] = array_y[1];
		array_y[1] = y_fValue;

		u_fValue = pplace_control(&pplace_config, array_y[0], r_fValue);

		setU_fValue(u_fValue);

//		u_fValue = 0.0;
//		setU_fValue(r_fValue);

		saveAndPrintDataFloat(y_fValue, r_fValue, u_fValue);

	}

}

static void saveAndPrintDataFloat(float y, float r, float e) {
	if (count < DATA_BUFFER_LEN) {
		signalBufferFloat[count] = r;
		dataBufferFloat[count] = y;
		errorBufferFloat[count] = e;
		count++;
	} else {

		for (int i = 0; i < DATA_BUFFER_LEN; ++i) {

			char strValue1[6];
			char strValue2[6];
			char strValue3[6];

			PIDUTIL_ftoa(dataBufferFloat[i], strValue1, 3);
			PIDUTIL_ftoa(signalBufferFloat[i], strValue2, 3);
			PIDUTIL_ftoa(errorBufferFloat[i], strValue3, 3);

			snprintf(buffer, 64, formatFloat, strValue1, strValue2, strValue3);
			HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer), 100);

		}
		BSP_LED_Toggle(LED_BLUE);
		vTaskDelay(portMAX_DELAY);
		count = 0;
	}
}

static void saveAndPrintData(uint32_t y, uint32_t r, uint32_t e) {
	if (count < DATA_BUFFER_LEN) {
		signalBuffer[count] = r;
		dataBuffer[count] = y;
		errorBuffer[count] = e;
		count++;
	} else {

		for (int i = 0; i < DATA_BUFFER_LEN; ++i) {
			snprintf(buffer, 32, format, dataBuffer[i], signalBuffer[i],
					errorBuffer[i]);
			HAL_UART_Transmit(&huart3, (uint8_t*) buffer, strlen(buffer), 100);
		}
		BSP_LED_Toggle(LED_BLUE);
		vTaskDelay(portMAX_DELAY);
		count = 0;
	}
}

static uint32_t getRValue() {
	return PIDUTIL_getValue(&hadc2);
}

static uint32_t getYValue() {
	return PIDUTIL_getValue(&hadc1);
}

static void setUValue(uint32_t value) {
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, value);
}

static float getR_fValue() {
	uint32_t value = getRValue();
	float value_f = (((float) value / 4096) * (float) 3.3);
	return value_f;
}

static float getY_fValue() {
	uint32_t value = getYValue();
	float value_f = (((float) value / 4096) * (float) 3.3);
	return value_f;
}

static void setU_fValue(float value) {
	uint32_t result = ((value / (float) 3.3) * 4096);
	result = (result >= 4096 ? 4095 : result);
	setUValue(result);
}

void PID_deInitTask10Hz() {

}
