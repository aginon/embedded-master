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
}

void setErrno(int errCode) {
	// Check if user trying to reset errno
	if (errCode == 0) {
		AGN_ERRNO = AGN_ERRNO_RESET_ERRNO;
	} else {
		AGN_ERRNO = errCode;
	}

	// Also Set Red LED Pin to HIGH (Indicating Error)
	HAL_GPIO_WritePin(GPIOD, LD5_Pin, 1);
}

int getErrno() {
	return AGN_ERRNO;
}
