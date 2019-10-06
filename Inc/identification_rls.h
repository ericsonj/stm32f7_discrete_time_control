/*=====[identification_rls]====================================================
 * Copyright 2019 Diego Fernández <dfernandez202@gmail.com>
 * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.0.0
 * Creation Date: 2019/09/23
 */

/*=====[Avoid multiple inclusion - begin]====================================*/

#ifndef _IDENTIFICATION_RLS_H_
#define _IDENTIFICATION_RLS_H_

/*=====[Inclusions of public function dependencies]==========================*/

/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Definition macros of public constants]===============================*/

// Order of process to identify
#define Q_ORDER         1

// Order of matrix to calculate identify
#define M_SIZE          (( 2 * Q_ORDER ) + 1)

/*=====[Public function-like macros]=========================================*/

/*=====[Definitions of public data types]====================================*/

struct type_IRLSdata
{
	uint32_t ts_Ms;

	void (*p_receive) (float32_t*);	        // Recibir señal de la planta

	float32_t buffer_E[1];			        // Error 1x1

	float32_t buffer_P[M_SIZE * M_SIZE];	// Buffer de la matriz de covarianza P(k) (M_SIZE x M_SIZE)
	float32_t buffer_T[M_SIZE];			    // Buffer del vector de parametros estimados Theta(k) (M_SIZE x 1)
	float32_t buffer_F[M_SIZE];			    // Buffer del vector de salidas y entradas Phi(k) (M_SIZE x 1)
	float32_t buffer_K[M_SIZE * M_SIZE];	// Buffer de la matriz de ganancia K(k) (M_SIZE x M_SIZE)

	float32_t buffer_TT[M_SIZE];			// Buffer del vector traspuesto de parametros estimados (1 x M_SIZE)
	float32_t buffer_FT[M_SIZE];			// Buffer del vector traspuesto de salidas y entradas (1 x M_SIZE)

	float32_t buffer_aux0[M_SIZE];		    // Buffer de la matriz aux0 = P(k-1) Phi'(k) (M_SIZE x 1)
	float32_t buffer_aux1[1];		        // Buffer de la matriz aux1 = Phi(k) aux0 (1 x 1)
	float32_t buffer_aux2[M_SIZE];		    // Buffer de la matriz aux2 = Phi(k) P(k-1) (1 x M_SIZE)
	float32_t buffer_aux3[M_SIZE * M_SIZE];	// Buffer de la matriz aux3 = K(k) aux2 (M_SIZE x M_SIZE)
	float32_t buffer_aux4[1];	            // Buffer de la matriz aux4 = Phi(k) Theta(k-1) (1 x 1)
	float32_t buffer_aux5[1];	            // Buffer de la matriz aux5 = K(k) Error(k) (1 x 1)

	float32_t Yk;		// Valor actual de Y. Y(k)
	float32_t Yk_1;		// Valor de Y(k-1)
	float32_t Yk_2;		// Valor de Y(k-2)
	float32_t Uk;		// Valor actual de U. U(k)
	float32_t Uk_1;		// Valor de U(k-1)
	float32_t Uk_2;		// Valor de U(k-2)
};

typedef struct type_IRLSdata t_IRLSdata;

/*=====[Prototypes (declarations) of public functions]=======================*/

// Configura la estructura de datos del Identificador
void IRLS_Init (t_IRLSdata* iData, uint32_t ts_Ms, void (*pfR)(float32_t*));

// Ejecucion recurrente del Identificador
void IRLS_Run(t_IRLSdata* iData);

/*=====[Prototypes (declarations) of public interrupt functions]=============*/

/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* _IDENTIFICATION_RLS_H_ */
