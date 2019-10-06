/*
 * pplace.c
 *
 *  Created on: Oct 5, 2019
 *      Author: Ericson Joseph
 */
#include "pplace.h"

void pplace_init(pplace_config_t *config, float desired_pole) {
	config->K = (config->Ad - desired_pole) / (config->Bd);
	config->Ko = 1 / (config->Cd / (1 - config->Ad + config->Bd * config->K) * config->Bd);
}

float pplace_control(pplace_config_t *config, float state, float reference) {
	return (config->Ko * reference) - (config->K * state);
}
