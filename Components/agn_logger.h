/*
 * AGN_Logger.h
 *
 *  Created on: Nov 21, 2017
 *      Author: Kris
 */

#ifndef AGN_LOGGER_H_
#define AGN_LOGGER_H_

#include "stm32f4xx_hal.h"

#define _AGN_LOG_PROTOTYPE(x, y) _AGN_LOG_FORMAT(x, y, __FILE__, __LINE__)
#define AGN_LOG_FATAL(x) _AGN_LOG_PROTOTYPE(x, "FATAL")
#define AGN_LOG_ERROR(x) _AGN_LOG_PROTOTYPE(x, "ERROR")
#define AGN_LOG_INFO(x) _AGN_LOG_PROTOTYPE(x, "INFO ")
#define AGN_LOG_DEBUG(x) _AGN_LOG_PROTOTYPE(x, "DEBUG")
#define AGN_LOG_TRACE(x) _AGN_LOG_PROTOTYPE(x, "TRACE")

// Constructor
void AGN_LOG_INITIALIZE(UART_HandleTypeDef * logger_huart);

// Base (Private)
void _AGN_LOG_FORMAT(const char * message, const char * level, const char * file, int line);
void AGN_LOG_ERRNO();

#endif /* AGN_LOGGER_H_ */
