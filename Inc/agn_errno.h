/*
 * errno.h
 *
 *  Created on: Nov 21, 2017
 *      Author: Kris
 */

#ifndef AGN_ERRNO_H_
#define AGN_ERRNO_H_


#include "stm32f4xx_hal.h"

#define AGN_ERRNO_UNKNOWN 				0x01
#define AGN_ERRNO_RESET_ERRNO 			0x02

#define AGN_ERRNO_LOGGING_ERROR 		0x11
#define AGN_ERRNO_LOGGING_TIMEOUT		0x12

#define AGN_ERRNO_GATEWAY_SEND_ERROR 	0x21
#define AGN_ERRNO_GATEWAY_SEND_TIMEOUT 	0x22
#define AGN_ERRNO_GATEWAY_RECV_ERROR	0x23
#define AGN_ERRNO_GATEWAY_RECV_TIMEOUT	0x24

#define AGN_ERRNO_BUZZER_ERROR			0x31
#define AGN_ERRNO_BUZZER_INVALID_WAVE	0x32
#define AGN_ERRNO_BUZZER_INVALID_FREQ	0x33

void resetErrno();
void setErrno(int errCode);

int getErrno();


#endif /* AGN_ERRNO_H_ */
