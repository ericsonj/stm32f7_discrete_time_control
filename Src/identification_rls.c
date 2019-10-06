/*=====[identification_rls]=============================================
 * Copyright 2019 Diego Fernández <dfernandez202@gmail.com>
 * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.0.0
 * Creation Date: 2019/09/23
 */

/*=====[Inclusions of private function dependencies]=========================*/

#include <stdlib.h>
#include "arm_math.h"

#include "identification_rls.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Private function-like macros]========================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

arm_matrix_instance_f32 E;      // Error(k) = y(k) - aux4 -> 1 x 1
arm_matrix_instance_f32 P;      // P(k) = P(k-1) - M8 -> M_SIZE x M_SIZE
arm_matrix_instance_f32 T;      // Theta(k) = Theta(k-1) - aux5 -> M_SIZE x 1
arm_matrix_instance_f32 F;      // Phi(k) -> 1 x M_SIZE
arm_matrix_instance_f32 K;      // K(k) = aux0/aux4 -> M_SIZE x 1
arm_matrix_instance_f32 TT;     // Theta'(k) -> 1 x M_SIZE
arm_matrix_instance_f32 FT;     // Phi'(k) -> M_SIZE x 1

arm_matrix_instance_f32 aux0;   // aux0 = P(k-1) Phi'(k) -> M_SIZE x 1
arm_matrix_instance_f32 aux1;   // aux1 = Phi(k) aux0 -> 1 x 1
arm_matrix_instance_f32 aux2;   // aux2 = Phi(k) P(k-1) -> 1 x M_SIZE
arm_matrix_instance_f32 aux3;   // aux3 = K(k) aux2 -> M_SIZE x M_SIZE
arm_matrix_instance_f32 aux4;   // aux4 = Phi(k) Theta(k-1) -> 1 x 1
arm_matrix_instance_f32 aux5;   // aux5 = K(k) Error(k) -> 1 x 1

/*=====[Prototypes (declarations) of private functions]======================*/

/*=====[Implementations of public functions]=================================*/

// Configura la estructura de datos del Identificador
void IRLS_Init (t_IRLSdata* iData, uint32_t ts_Ms, void (*pfR)(float32_t*))
{
	uint32_t i;
	uint32_t j;

	iData->ts_Ms = ts_Ms;
	iData->p_receive = pfR;

	// Inicializazo P(0) como matriz identidad
    for (i=0; i<M_SIZE;i++)
    {
	    for (j=0; j<M_SIZE;j++)
        {
            if (i==j) iData->buffer_P[i*M_SIZE+j] = 1;
            else iData->buffer_P[i*M_SIZE+j] = 0;
        }
    }

	// Inicializazo Theta(0) en cero
	for (i=0; i<M_SIZE;i++)
	{
		iData->buffer_T[i] = 0;
	}

	// Inicialización de matrices
    /**
    * @brief  Floating-point matrix initialization.
    * @param[in,out] S         points to an instance of the floating-point 
    *                          matrix structure.
    * @param[in]     nRows     number of rows in the matrix.
    * @param[in]     nColumns  number of columns in the matrix.
    * @param[in]     pData     points to the matrix data array.
    */
	arm_mat_init_f32(&E, 1, 1, iData->buffer_E);
	arm_mat_init_f32(&P, M_SIZE, M_SIZE, iData->buffer_P);
	arm_mat_init_f32(&T, M_SIZE, 1, iData->buffer_T);
	arm_mat_init_f32(&F, M_SIZE, 1, iData->buffer_F);
	arm_mat_init_f32(&K, M_SIZE, M_SIZE, iData->buffer_K);
	arm_mat_init_f32(&TT, 1, M_SIZE, iData->buffer_TT);
	arm_mat_init_f32(&FT, 1, M_SIZE, iData->buffer_FT);
	arm_mat_init_f32(&aux1, 1, 1, iData->buffer_aux1);
	arm_mat_init_f32(&aux2, 1, M_SIZE, iData->buffer_aux2);
	arm_mat_init_f32(&aux0, M_SIZE, 1, iData->buffer_aux0);
	arm_mat_init_f32(&aux3, M_SIZE, M_SIZE, iData->buffer_aux3);

	// Valores iniciales
	iData->Yk_1 = 0;
	iData->Yk_2 = 0;
	iData->Uk_1 = 0;
	iData->Uk_2 = 0;
}

// Ejecución recurrente del Identificador
void IRLS_Run(t_IRLSdata* iData)
{
	float32_t buffer[2];
	uint32_t i;

	iData->p_receive(buffer);
	iData->Uk = buffer[0];
	iData->Yk = buffer[1];

    // Con q=1 ==> Phi(k) = [y(k-1) u(k) u(k-1)]
    // Con q=2 ==> Phi(k) = [y(k-1) y(k-2) u(k) u(k-1) u(k-2)]
	iData->buffer_F[0] = iData->Yk_1;
	iData->buffer_F[1] = iData->Yk_2;
	iData->buffer_F[2] = iData->Uk;
	iData->buffer_F[3] = iData->Uk_1;
	iData->buffer_F[4] = iData->Uk_2;

	// Cálculo de matrices traspuestas
    // Theta'(k)
	arm_mat_trans_f32(&T, &TT);
    // Phi'(k)
	arm_mat_trans_f32(&F, &FT);

	// Cálculo de K(k)
    //               P(k-1) Phi'(k)
    // K(k) = ______________________________
    //         1 + Phi(k) P(k-1) Phi'(k)
	arm_mat_mult_f32(&P, &FT, &aux0);
	arm_mat_mult_f32(&F, &aux0, &aux1);
    for( i=0; i<M_SIZE; i++ ) {
        iData->buffer_K[i] = iData->buffer_aux0[i] / (1 + iData->buffer_aux1[0]);
    }

	// Cálculo de P(k)
    //                  P(k-1) Phi'(k) Phi(k) P(k-1)
    // P(k) = P(k-1) - ______________________________
    //                   1 + Phi(k) P(k-1) Phi'(k)
	arm_mat_mult_f32(&F, &P, &aux2);
	arm_mat_mult_f32(&K, &aux2, &aux3);
	arm_mat_sub_f32(&P, &aux3, &P);

	// Cálculo de Error
	arm_mat_mult_f32(&F, &T, &aux4);
	iData->buffer_E[0] = iData->Yk - iData->buffer_aux4[0];

	// Cálculo de Theta(k)
    // Theta(k) = Theta(k-1) + K(k)[ y(k) - Phi(k) Theta(k-1) ]
	arm_mat_mult_f32(&K, &E, &aux5);
	arm_mat_add_f32(&T, &aux5, &T);

	iData->Uk_2 = iData->Uk_1;
	iData->Uk_1 = iData->Uk;
	iData->Yk_2 = iData->Yk_1;
	iData->Yk_1 = iData->Yk;
}

/*=====[Implementations of interrupt functions]==============================*/

/*=====[Implementations of private functions]================================*/