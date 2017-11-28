/*
 * types.h
 *
 *  Created on: Nov 21, 2017
 *      Author: Kris
 */

#ifndef TYPES_H_
#define TYPES_H_
#include "stm32f4xx_hal.h"


// Single Types //

typedef uint8_t cid;
typedef uint8_t command_t;


// Vectors //

typedef struct {
	uint8_t x, y, z;
} vec3u8;

typedef struct {
	int8_t x, y, z;
} vec3s8;

typedef struct {
	uint16_t x, y, z;
} vec3u16;

typedef struct {
	int16_t x, y, z;
} vec3s16;

typedef struct {
	float x, y, z;
} vec3fp;

typedef struct {
	double x, y, z;
} vec3dp;


// DTOs //

typedef struct {
	uint8_t isThief;
	uint8_t isSensorWorking[];
} push_t;

typedef struct {
	command_t rcmd;
	uint8_t sysEn;
	uint8_t isHome;
} pull_t;

typedef struct {
	vec3u16 acc, gyr;
} imur_t;

#endif /* TYPES_H_ */
