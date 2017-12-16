/*
 * errno.h
 *
 *  Created on: Nov 21, 2017
 *      Author: Kris
 */

#ifndef AGN_ERRNO_H_
#define AGN_ERRNO_H_


#include "stm32f4xx_hal.h"

#define AGN_ERRNO_UNKNOWN 				1
#define AGN_ERRNO_RESET_ERRNO 			2
#define AGN_ERRNO_LOGGING_ERROR 		3
#define AGN_ERRNO_LOGGING_TIMEOUT		4
#define AGN_ERRNO_GATEWAY_SEND_ERROR 	5
#define AGN_ERRNO_GATEWAY_SEND_TIMEOUT 	6
#define AGN_ERRNO_GATEWAY_RECV_ERROR	7
#define AGN_ERRNO_GATEWAY_RECV_TIMEOUT	8

void resetErrno();
void setErrno(int errCode);

int getErrno();


#endif /* AGN_ERRNO_H_ */
