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

#define AGN_MAX_RANGE_COMPONENTS 16

// Calibration Factors (Y = ax + b)
static int32_t agn_range_a[AGN_MAX_RANGE_COMPONENTS], agn_range_b[AGN_MAX_RANGE_COMPONENTS];

static uint32_t _AGN_RANGE[AGN_MAX_RANGE_COMPONENTS];
static uint32_t _LAST_EDGE_TIME[AGN_MAX_RANGE_COMPONENTS];
static uint8_t  _AGN_RANGE_AVAILABLE[AGN_MAX_RANGE_COMPONENTS];

void AGN_RANGE_INITIALIZE() {
	for (int i = 0; i < AGN_MAX_RANGE_COMPONENTS; i++) {
		_AGN_RANGE[i] = 0;
		_LAST_EDGE_TIME[i] = 0;
		_AGN_RANGE_AVAILABLE[i] = 0;
		agn_range_a[i] = 1;
		agn_range_b[i] = 0;
	}
}

void AGN_RANGE_SET_A(int32_t a, uint8_t channel) {
	agn_range_a[channel] = a;
}

void AGN_RANGE_SET_B(int32_t b, uint8_t channel) {
	agn_range_b[channel] = b;
}

void AGN_RANGE_RISING(uint8_t channel) {
	_LAST_EDGE_TIME[channel] = agnMicros();

}

void AGN_RANGE_FALLING(uint8_t channel) {
	uint32_t newValue = agnMicros() - _LAST_EDGE_TIME[channel];

	// Use this value if edge has risen and value not over limit.
	if (_LAST_EDGE_TIME[channel] != 0 && newValue < 70000000) {

		// Low Pass Filter
		_AGN_RANGE[channel] = (newValue * 7 + _AGN_RANGE[channel]) / 8;

		//_AGN_RANGE = newValue;
		_AGN_RANGE_AVAILABLE[channel] = 1;
	}
/*
	char str[100];
	sprintf(str, "LAST_EDGE_TIMES[%lu, %lu, %lu, %lu]", _LAST_EDGE_TIME[0], _LAST_EDGE_TIME[1], _LAST_EDGE_TIME[2], _LAST_EDGE_TIME[3]);
	AGN_LOG_INFO(str);
*/
}

void AGN_RANGE_TRIGGER(uint8_t channel) {

	//__disable_irq();

	GPIO_TypeDef* RANGE_TRIG_GPIO_Port;
	uint16_t RANGE_TRIG_Pin;
	if (channel == 1) {
		RANGE_TRIG_GPIO_Port = RANGE1_TRIG_GPIO_Port;
		RANGE_TRIG_Pin = RANGE1_TRIG_Pin;
	} else if (channel == 2) {
		RANGE_TRIG_GPIO_Port = RANGE2_TRIG_GPIO_Port;
		RANGE_TRIG_Pin = RANGE2_TRIG_Pin;
	} else {
		return;
	}

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

	//__enable_irq();
}

uint32_t AGN_RANGE_GET(uint8_t channel) {
	_AGN_RANGE_AVAILABLE[channel] = 0;
	return _AGN_RANGE[channel] * agn_range_a[channel] + agn_range_b[channel];
}

uint8_t AGN_RANGE_IS_CONNECTED(uint8_t channel) {
	// If responseless for >= 5 seconds, then it is disconnected
	if (agnMicros() - _LAST_EDGE_TIME[channel] >= 5000000) {
		return 0;
	} else {
		return 1;
	}
}
