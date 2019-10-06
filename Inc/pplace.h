/*
 * pplace.h
 *
 *  Created on: Oct 5, 2019
 *      Author: ericson
 */

#ifndef PPLACE_H_
#define PPLACE_H_

typedef struct {
	float Ad;
	float Bd;
	float Cd;
	float K;
	float Ko;
} pplace_config_t;

void pplace_init(pplace_config_t *config, float desired_pole);

float pplace_control(pplace_config_t *config, float state, float reference);

#endif /* PPLACE_H_ */
