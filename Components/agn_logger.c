/* AGN_Logger.c */

// Standard Library Includes
#include <agn_errno.h>
#include "string.h"
#include "stdio.h"
#include "time.h"
#include "stdarg.h"

// STM Includes
#include "main.h" // error_handler

// User Library Includes
#include "agn_logger.h"

#define AGN_LOG_TIMEOUT_MS 1000

static UART_HandleTypeDef * _logger_huart;

void AGN_LOG_INITIALIZE(UART_HandleTypeDef * logger_huart) {
	_logger_huart = logger_huart;

	// Display Welcome message
	char buff[1024];
	int len = sprintf(buff, "\r%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n",
			"================================================================================",
			"                      Logger for Aginon-IoT Master (STM32F4)                    ",
			"                                                                                ",
			" If you are seeing this, then you have successfully connected to the debug/log  ",
			" port of the Master!  Aginon-IoT is a project for 2110363 HW SYN LAB, Department",
			" of Computer Engineering, Chulalongkorn University. For more information, please",
			" checkout https://github.com/aginon .                                           ",
			"                                                            (C) Aginon-IoT 2017 ",
			"================================================================================");
	HAL_StatusTypeDef transmitStatus = HAL_UART_Transmit(_logger_huart, (uint8_t *) buff, len, AGN_LOG_TIMEOUT_MS);

	// Handle errors
	if (transmitStatus == HAL_ERROR) {
		setErrno(AGN_ERRNO_LOGGING_ERROR);
	} else if (transmitStatus == HAL_TIMEOUT) {
		setErrno(AGN_ERRNO_LOGGING_TIMEOUT);
	}
}

void _AGN_LOG_FORMAT(const char * message, const char * level, const char * file, int line) {
	char buff[1024];

	// Format message string and get length of string
	int len = sprintf(buff, "T+%ld (%s:%d) [%s] %s\r\n", HAL_GetTick(), file, line, level, message);

	// Transmit message
	HAL_StatusTypeDef transmitStatus = HAL_UART_Transmit(_logger_huart, (uint8_t *) buff, len, AGN_LOG_TIMEOUT_MS);

	// Handle errors
	if (transmitStatus == HAL_ERROR) {
		setErrno(AGN_ERRNO_LOGGING_ERROR);
	} else if (transmitStatus == HAL_TIMEOUT) {
		setErrno(AGN_ERRNO_LOGGING_TIMEOUT);
	}
}
