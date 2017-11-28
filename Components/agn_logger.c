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

UART_HandleTypeDef * _logger_huart;

void AGN_INITIALIZE(UART_HandleTypeDef * logger_huart) {
	_logger_huart = logger_huart;
}

void _AGN_LOG_FORMAT(const char * message, const char * level, const char * file, int line) {
	char buff[1024];
	// Time Str //
	int len = sprintf(buff, "T+%ld (%s:%d) [%s] %s\r\n", HAL_GetTick(), file, line, level, message);
	//int len = 4;
	if (HAL_UART_Transmit(_logger_huart, (uint8_t *) buff, len, AGN_LOG_TIMEOUT_MS) != HAL_OK) {
		setErrno(AGN_ERRNO_LOGGING_FAILED);
	}
}
