/*
 * agn_micros.c
 *
 *  Created on: Dec 15, 2017
 *      Author: Kris
 */

#include "agn_micros.h"

uint32_t agnMicros() {
	//return (DWT->CYCCNT) / (SystemCoreClock / 1000000);
	return (DWT->CYCCNT) >> 4;
}

static uint32_t prevMicros = 0;

uint32_t agnProfile() {
	uint32_t now = agnMicros();
	uint32_t retval = now - prevMicros;
	prevMicros = now;
	return retval;
}



