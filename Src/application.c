/*
 * PIDlib.c
 *
 *  Created on: Sep 30, 2019
 *      Author: Ericson Joseph
 */
#include <application.h>
#include <PIDUtil.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "stm32f7xx_nucleo_144.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pplace.h"
#include "pid_controller.h"
#include "identification_ls.h"
#include "identification_rls.h"
#include "identification_tasks.h"

#define DATA_BUFFER_LEN 600
#define _1V2  				(int)((float)1.2/(float)3.3)*4095))

#define DAC_REFERENCE_VALUE_HIGH   2668  // 4095 = 3.3V, 666 = 2.15V
#define DAC_REFERENCE_VALUE_LOW    2000  // 4095 = 3.3V,

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;

#if DEPRECATE
static uint32_t dataBuffer[DATA_BUFFER_LEN];
static uint32_t signalBuffer[DATA_BUFFER_LEN];
static uint32_t errorBuffer[DATA_BUFFER_LEN];
#endif

static float dataBufferFloat[DATA_BUFFER_LEN];
static float signalBufferFloat[DATA_BUFFER_LEN];
static float errorBufferFloat[DATA_BUFFER_LEN];

#if DEPRECATE
static char format[] = "%d , %d, %d;\n";
#endif

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

t_IRLSdata* tIRLS1;
t_ILSdata* tILS1;

TickType_t h_ms = 1 / portTICK_RATE_MS;

#if PID || PPLACE
static osThreadId_t signalTaskHandle;
static osThreadId_t controlTaskHandle;
#endif

static osThreadId_t idenTaskHandle;

static SemaphoreHandle_t pidSemaph;

#if PID || PPLACE
static void task10HzSignal(void *pvParam);
static void taskPID(void *pvParam);
#endif

#if DEPRECATE
static void saveAndPrintData(uint32_t y, uint32_t r, uint32_t e);
#endif

static void saveAndPrintDataFloat(float y, float r, float e);

static uint32_t getRValue();
static uint32_t getYValue();
static void setUValue(uint32_t value);

static float getR_fValue();
static float getY_fValue();
static void setU_fValue(float value);

static void receiveData(float* buffer);

void APP_init() {

	BSP_LED_Init(LED_BLUE);
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

#if PID
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
#endif

#if PPLACE
//        Punto 7
//        0.09516
// y1:  ----------
//       z - 0.9048
	pplace_config.Ad = 0.90484;
	pplace_config.Bd = 0.095163;
	pplace_config.Cd = 1.0;

	pplace_init(&pplace_config, 0.4);
#endif


#if PID || PPLACE

	const osThreadAttr_t signalTaskAttr = { .name = "Task 10Hz", .priority =
			(osPriority_t) osPriorityNormal, .stack_size = 128 };
	signalTaskHandle = osThreadNew(task10HzSignal, NULL, &signalTaskAttr);

	const osThreadAttr_t pidTaskAttr = { .name = "Task PID", .priority =
			(osPriority_t) osPriorityNormal, .stack_size = 1024 };
	controlTaskHandle = osThreadNew(taskPID, NULL, &pidTaskAttr);

	pidSemaph = xSemaphoreCreateBinary();

#endif

#if IDENTFICATION
//  Punto 8
//    tIRLS1 = (t_IRLSdata*) pvPortMalloc (sizeof(t_IRLSdata));
    tILS1 = (t_ILSdata*) pvPortMalloc (sizeof(t_ILSdata));

//	IRLS_Init(tIRLS1, 1, receiveData);
	ILS_Init(tILS1, 50, 1, receiveData);

	const osThreadAttr_t idenTaskAttr = { .name = "Identification Task", .priority =
				(osPriority_t) osPriorityNormal, .stack_size = 1024 };
	idenTaskHandle = osThreadNew(ILS_Task, (void*)tILS1, &idenTaskAttr);
#endif

}

#if PID || PPLACE
void task10HzSignal(void *pvParam) {
	xSemaphoreGive(pidSemaph);
	for (;;) {
		vTaskDelay(100 / portTICK_RATE_MS);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 2000);
		vTaskDelay(100 / portTICK_RATE_MS);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	}
}
#endif

#if PID || PPLACE
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

#if PID
		u_fValue = pidCalculateControllerOutput(&PsPIDController, y_fValue,
				r_fValue);
#endif

#if PPLACE
		array_y[0] = array_y[1];
		array_y[1] = y_fValue;
		u_fValue = pplace_control(&pplace_config, array_y[0], r_fValue);
#endif

		setU_fValue(u_fValue);

//		u_fValue = 0.0;
//		setU_fValue(r_fValue);

		saveAndPrintDataFloat(y_fValue, r_fValue, u_fValue);

	}

}
#endif

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

#if DEPRECATE
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
#endif

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

void receiveData(float* buffer) {
	float Y, U;

	uint32_t dacValue = 0;

	// random = limite_inferior + rand() % (limite_superior +1 - limite_inferior);
	dacValue = DAC_REFERENCE_VALUE_LOW
			+ rand() % (DAC_REFERENCE_VALUE_HIGH + 1 - DAC_REFERENCE_VALUE_LOW);

	setUValue(dacValue);

	// dacSample = (1023.0 / 3.3) * sampleInVolts
	// 1023.0 / 3.3 = 310.0
	U = (float) dacValue * 3.3 / 4095.0;
	Y = getY_fValue();

	buffer[0] = U;
	buffer[1] = Y;
}

void APP_deInit() {

}
