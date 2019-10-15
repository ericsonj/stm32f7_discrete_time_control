/*=====[identification_task]===================================================
 * Copyright 2019 Diego Fernández <dfernandez202@gmail.com>
 * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.0.0
 * Creation Date: 2019/09/23
 */

/*=====[Inclusions of private function dependencies]=========================*/

#include "identification_tasks.h"

#include "arm_math.h"
#include <math.h>
#include <PIDUtil.h>
#include <string.h>

#include "identification_ls.h"
#include "identification_rls.h"


/*=====[Definition macros of private constants]==============================*/

/*=====[Private function-like macros]========================================*/

/*=====[Definitions of private data types]===================================*/

/*=====[Definitions of external public global variables]=====================*/
extern UART_HandleTypeDef huart3;
extern DAC_HandleTypeDef hdac;
/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/
static char printBuff[64];

/*=====[Prototypes (declarations) of private functions]======================*/

void console_print(float* buffer);

/*=====[Implementations of public functions]=================================*/

void ILS_Task(void* taskParmPtr) {

	t_ILSdata* tILS;
	portTickType xLastWakeTime;

	tILS = (t_ILSdata*) taskParmPtr;

	// Para implementar un delay relativo
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		// Ejecuto el Identificador
		ILS_Run(tILS);

		if (tILS->i == 2) {
			// Imprimo los parámetros calculados
			console_print(tILS->buffer_T);
			vTaskDelay(portMAX_DELAY);
		}

		vTaskDelayUntil(&xLastWakeTime, (tILS->ts_Ms / portTICK_RATE_MS));
	}
}

void IRLS_Task(void* taskParmPtr) {
	t_IRLSdata* tIRLS;
	portTickType xLastWakeTime;

	tIRLS = (t_IRLSdata*) taskParmPtr;

	// Para implementar un delay relativo
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		// Ejecuto el Identificador
		IRLS_Run(tIRLS);

		// Imprimo los parámetros calculados
		console_print(tIRLS->buffer_T);

		vTaskDelayUntil(&xLastWakeTime, (tIRLS->ts_Ms / portTICK_RATE_MS));
	}
}

/*=====[Implementations of interrupt functions]==============================*/

/*=====[Implementations of private functions]================================*/

// Función para imprimir los parámetros la planta (2do orden)
void console_print(float* buffer) {


	buffer[0] = buffer[0] - 0.08;
	buffer[2] = buffer[2] + 0.08;
	// Imprimo los parámetros calculados
	snprintf(printBuff, 64, "%s", "Parametros = [ ");
	HAL_UART_Transmit(&huart3, (uint8_t*) printBuff, strlen(printBuff), 100);

	char strValue[6];
	uint32_t i;
	for (i = 0; i < M_SIZE; i++) {
		PIDUTIL_ftoa(buffer[i], strValue, 3);
		snprintf(printBuff, 64, " %s ", strValue);
		HAL_UART_Transmit(&huart3, (uint8_t*) printBuff, strlen(printBuff),
				100);
	}
	snprintf(printBuff, 64, "%s", "]\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*) printBuff, strlen(printBuff), 100);

	float R = (-0.001)/(logf(buffer[0])*0.000001);
	snprintf(printBuff, 64, "R = %ld Ohms\r\n", (uint32_t)R);
	HAL_UART_Transmit(&huart3, (uint8_t*) printBuff, strlen(printBuff), 100);

}
