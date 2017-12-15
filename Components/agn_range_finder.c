/*
 * agn_range_finder.c
 *
 *  Created on: Dec 15, 2017
 *      Author: Kris
 */

// Standard Library Includes
#include <agn_errno.h>
#include "string.h"
#include "stdio.h"

// STM Includes
#include "main.h" // error_handler, pins def

// User Library Includes
#include "agn_range_finder.h"
#include "agn_micros.h"
#include "agn_logger.h"

// Calibration Factors (Y = ax + b)
uint32_t agn_range_a = 100, agn_range_b = 100;

uint32_t _AGN_RANGE = 0;
uint32_t _LAST_EDGE_TIME = 0;

void AGN_RANGE_SET_A(uint32_t a) {
	agn_range_a = a;
}

void AGN_RANGE_SET_B(uint32_t b) {
	agn_range_b = b;
}

void AGN_RANGE_RISING() {
	_LAST_EDGE_TIME = agnMicros();
}

void AGN_RANGE_FALLING() {
	uint32_t newValue = agnMicros() - _LAST_EDGE_TIME;

	// Use this value if edge has risen and value not over limit.
	if (_LAST_EDGE_TIME != 0 && newValue < 700000) {

		// Low Pass Filter
		_AGN_RANGE = (newValue * 7 + _AGN_RANGE) / 8;
	}
	char str[100];
	sprintf(str, "_AGN_RANGE = %lu", _AGN_RANGE);
	AGN_LOG_DEBUG(str);
}

void AGN_RANGE_TRIGGER() {

	__disable_irq();

	// set trigger pin LOW
	HAL_GPIO_WritePin(RANGE_TRIG_GPIO_Port, RANGE_TRIG_Pin, GPIO_PIN_RESET);

	// Delay for 10 microseconds
	unsigned long duration = 10; //us
	unsigned long startPulse = agnMicros();
	while(agnMicros() - startPulse < duration);

	// set trigger pin HIGH
	HAL_GPIO_WritePin(RANGE_TRIG_GPIO_Port, RANGE_TRIG_Pin, GPIO_PIN_SET);

	// Delay for 50 microseconds
	duration = 50; //us
	startPulse = agnMicros();
	while(agnMicros() - startPulse < duration);

	// set trigger pin LOW
	HAL_GPIO_WritePin(RANGE_TRIG_GPIO_Port, RANGE_TRIG_Pin, GPIO_PIN_RESET);

	// Delay for 10 microseconds
	duration = 10; //us
	startPulse = agnMicros();
	while(agnMicros() - startPulse < duration);

	__enable_irq();
}

uint32_t AGN_RANGE_GET() {
	return _AGN_RANGE;
}
