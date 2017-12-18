/*
 * agn_range_finder.h
 *
 *  Created on: Dec 15, 2017
 *      Author: Kris
 */

#ifndef AGN_RANGE_FINDER_H_
#define AGN_RANGE_FINDER_H_
#include "stm32f4xx_hal.h"

/* Initialization */
void AGN_RANGE_INITIALIZE();

/* Calibration */
void AGN_RANGE_SET_A(uint32_t a);
void AGN_RANGE_SET_B(uint32_t b);

/* On Pin Change Interrupt */
void AGN_RANGE_RISING(uint8_t channel);
void AGN_RANGE_FALLING(uint8_t channel);

/* Trigger Pin Output */
void AGN_RANGE_TRIGGER(uint8_t channel);

/* Get Range Value */
uint32_t AGN_RANGE_GET(uint8_t channel);

#endif /* AGN_RANGE_FINDER_H_ */
