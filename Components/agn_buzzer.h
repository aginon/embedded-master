/*
 * agn_buzzer.h
 *
 *  Created on: Dec 16, 2017
 *      Author: Kris
 */

#ifndef AGN_BUZZER_H_
#define AGN_BUZZER_H_

#include <stm32f4xx_hal.h>

#define AGN_BUZZER_ON 0
#define AGN_BUZZER_OFF 1

// Tone values are doubled
#define AGN_TONE_C4 	523.25
#define AGN_TONE_DB4 	554.37
#define AGN_TONE_D4 	587.33
#define AGN_TONE_EB4 	622.25
#define AGN_TONE_E4 	659.25
#define AGN_TONE_F4 	698.46
#define AGN_TONE_GB4 	739.99
#define AGN_TONE_G4 	783.99
#define AGN_TONE_AB4 	830.61
#define AGN_TONE_A4 	880.00
#define AGN_TONE_BB4 	932.33
#define AGN_TONE B4 	987.77

#define AGN_WAVEFORM_TRI 0
#define AGN_WAVEFORM_SQ  1

void AGN_BUZZER_INITIALIZE(DAC_HandleTypeDef *_buzzer_hdac);

// Called by the interrupt
void AGN_BUZZER_TICK();
void _AGN_BUZZER_HANDLE_SQ();
void _AGN_BUZZER_HANDLE_TRI();

void AGN_BUZZER_SET_WAVEFORM(uint8_t waveform);
void AGN_BUZZER_SET_FREQ(double freq);

void AGN_BUZZER_SET_SWTICH(uint8_t val);
void AGN_BUZZER_TOGGLE_SWTICH();

uint8_t AGN_BUZZER_GET_SWTICH();

#endif /* AGN_BUZZER_H_ */
