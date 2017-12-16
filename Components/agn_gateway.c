/*
 * agn_gateway.c
 *
 *  Created on: Nov 23, 2017
 *      Author: Kris
 */
#include "stm32f4xx_hal.h"

#include "agn_gateway.h"
#include "agn_errno.h"

static UART_HandleTypeDef* _gateway_huart;

#define AGN_GATEWAY_TIMEOUT_MS 1000

void AGN_GATEWAY_INITIALIZE(UART_HandleTypeDef * gw_huart) {
	_gateway_huart = gw_huart;
}


void AGN_GATEWAY_SEND_BYTES(uint8_t* bytes, uint16_t size) {
	HAL_StatusTypeDef transmitStatus = HAL_UART_Transmit(_gateway_huart, bytes, size, AGN_GATEWAY_TIMEOUT_MS);
	if (transmitStatus == HAL_ERROR) {
		setErrno(AGN_ERRNO_GATEWAY_SEND_ERROR);
	} else if (transmitStatus == HAL_TIMEOUT) {
		setErrno(AGN_ERRNO_GATEWAY_SEND_TIMEOUT);
	}
}

void AGN_GATEWAY_SEND_PACKET(struct AGN_PACKET* packet) {
	uint8_t bytes[AGN_PACKET_SIZE];
	AGN_PACKET_SERIALIZE(packet, bytes);
	AGN_GATEWAY_SEND_BYTES(bytes, AGN_PACKET_SIZE);
}

void AGN_GATEWAY_RECEIVE_BYTES(uint8_t* bytes, uint8_t size) {
	HAL_StatusTypeDef transmitStatus = HAL_UART_Receive(_gateway_huart, bytes, size, AGN_GATEWAY_TIMEOUT_MS);
	if (transmitStatus == HAL_ERROR) {
		setErrno(AGN_ERRNO_GATEWAY_RECV_ERROR);
	} else if (transmitStatus == HAL_TIMEOUT) {
		setErrno(AGN_ERRNO_GATEWAY_RECV_TIMEOUT);
	}
}

void AGN_GATEWAY_RECEIVE_PACKET(struct AGN_PACKET* packet) {
	uint8_t bytes[AGN_PACKET_SIZE];
	AGN_GATEWAY_RECEIVE_BYTES(bytes, AGN_PACKET_SIZE);
	AGN_PACKET_DESERIALIZE(packet, bytes);
}
