/*
 * errno.c
 *
 *  Created on: Nov 22, 2017
 *      Author: Kris
 */

#include <agn_errno.h>


// Error indicator
int AGN_ERRNO;

void resetErrno() {
	AGN_ERRNO = 0;
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
}

void setErrno(int errCode) {
	// Check if user trying to reset errno
	if (errCode == 0) {
		AGN_ERRNO = AGN_ERRNO_RESET_ERRNO;
	} else {
		AGN_ERRNO = errCode;
	}

	// Also Set Red LED Pin to HIGH (Indicating Error)
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
}

int getErrno() {
	return AGN_ERRNO;
}
