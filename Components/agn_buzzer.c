/*
 * agn_buzzer.c
 *
 *  Created on: Dec 16, 2017
 *      Author: Kris
 */

#include "agn_buzzer.h"

#include "agn_errno.h"
#include "stm32f4xx_hal.h"

static DAC_HandleTypeDef *buzzer_hdac;

// Config Variables
static uint8_t agn_buzzer_waveform;
static uint8_t agn_buzzer_switch;
static uint32_t agn_buzzer_freq;

// Internal Variables
static uint8_t agn_buzzer_state;
static uint32_t agn_buzzer_period;
static uint32_t agn_buzzer_counter;
static uint32_t _agn_buzzer_vpos;

void AGN_BUZZER_INITIALIZE(DAC_HandleTypeDef *_buzzer_hdac) {
	buzzer_hdac = _buzzer_hdac;

	agn_buzzer_waveform = AGN_WAVEFORM_SQ;
	agn_buzzer_switch = AGN_BUZZER_ON;
	agn_buzzer_freq = 1000;

	agn_buzzer_state = 0;
	agn_buzzer_period = 100;
	agn_buzzer_counter = 0;
	_agn_buzzer_vpos = 0x0000;

	HAL_GPIO_WritePin(BUZZER_IO_GPIO_Port, BUZZER_IO_Pin, GPIO_PIN_RESET);
	HAL_DAC_SetValue(buzzer_hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0x0000);
}


// Called by the interrupt
void AGN_BUZZER_TICK() {
	// Do nothing if BUZZER is off
	if (agn_buzzer_switch == AGN_BUZZER_OFF) {
		return;
	}

	agn_buzzer_counter++;
	if (agn_buzzer_counter >= agn_buzzer_period) {
		agn_buzzer_counter = 0;
		switch (agn_buzzer_waveform) {
		case AGN_WAVEFORM_SQ:
			AGN_BUZZER_HANDLE_SQ();
			break;
		case AGN_WAVEFORM_TRI:
			AGN_BUZZER_HANDLE_TRI();
			break;
		default:
			setErrno(AGN_ERRNO_BUZZER_INVALID_WAVE);
			break;
		}
	}
}

void AGN_BUZZER_HANDLE_SQ() {
	if (agn_buzzer_state == 0) {
		HAL_DAC_SetValue(buzzer_hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, _agn_buzzer_vpos);
		HAL_GPIO_WritePin(BUZZER_IO_GPIO_Port, BUZZER_IO_Pin, GPIO_PIN_RESET);
	} else {
		HAL_DAC_SetValue(buzzer_hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, _agn_buzzer_vpos);
		HAL_GPIO_WritePin(BUZZER_IO_GPIO_Port, BUZZER_IO_Pin, GPIO_PIN_SET);
	}

	if (agn_buzzer_state == 0) {
		_agn_buzzer_vpos = 0x0FFF;
	} else if (agn_buzzer_state == 1) {
		_agn_buzzer_vpos = 0x0000;
	}

	// Next state logic
	if (agn_buzzer_state == 0) {
		agn_buzzer_state = 1;
	} else if (agn_buzzer_state == 1){
		agn_buzzer_state = 0;
	}
}

void AGN_BUZZER_HANDLE_TRI() {

	// Output logic
	if (agn_buzzer_state == 0) {
		HAL_DAC_SetValue(buzzer_hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, _agn_buzzer_vpos);
		HAL_GPIO_WritePin(BUZZER_IO_GPIO_Port, BUZZER_IO_Pin, GPIO_PIN_RESET);
	} else {
		HAL_DAC_SetValue(buzzer_hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, _agn_buzzer_vpos);
		HAL_GPIO_WritePin(BUZZER_IO_GPIO_Port, BUZZER_IO_Pin, GPIO_PIN_SET);
	}

	// Handle VPOS for TRI waveform
	if (agn_buzzer_state == 0) {
		_agn_buzzer_vpos += 0x0100;
	} else if (agn_buzzer_state == 1) {
		_agn_buzzer_vpos -= 0x0100;
	}

	// Next state logic
	if (agn_buzzer_state == 0 && _agn_buzzer_vpos >= 0x0F00) {
		agn_buzzer_state = 1;
	} else if (agn_buzzer_state == 1 && _agn_buzzer_vpos <= 0x0000){
		agn_buzzer_state = 0;
	}
}

void AGN_BUZZER_SET_SWTICH(uint8_t val){
	agn_buzzer_switch = val;
}

void AGN_BUZZER_TOGGLE_SWITCH() {
	if (agn_buzzer_switch == AGN_BUZZER_ON) {
		agn_buzzer_switch = AGN_BUZZER_OFF;
	} else {
		agn_buzzer_switch = AGN_BUZZER_ON;
	}
}

void AGN_BUZZER_SET_WAVEFORM(uint8_t waveform) {
	if (waveform == AGN_WAVEFORM_TRI) {
		agn_buzzer_waveform = AGN_WAVEFORM_TRI;
	} else if (waveform == AGN_WAVEFORM_SQ) {
		agn_buzzer_waveform = AGN_WAVEFORM_SQ;
	} else {
		setErrno(AGN_ERRNO_BUZZER_INVALID_WAVE);
	}
	AGN_BUZZER_SET_FREQ(agn_buzzer_freq);
}

void AGN_BUZZER_SET_FREQ(double freq) {
	agn_buzzer_freq = (uint32_t) freq;
	// if freq > 100000 or freq == 0, do nothing
	if (agn_buzzer_waveform == AGN_WAVEFORM_TRI) {
		freq *= 16;
	}
	if (freq < 100000 && freq > 0) {
		// set period (100000 = 100kHz)
		agn_buzzer_period = 100000 / freq;
		agn_buzzer_counter = 0;
		agn_buzzer_state = 0;
		_agn_buzzer_vpos = 0x0000;
	} else {
		setErrno(AGN_ERRNO_BUZZER_INVALID_FREQ);
	}
}



