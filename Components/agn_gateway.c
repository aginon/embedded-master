/*
 * agn_gateway.c
 *
 *  Created on: Nov 23, 2017
 *      Author: Kris
 */
#include "stm32f4xx_hal.h"

#include "agn_gateway.h"
#include "agn_errno.h"
#include "agn_logger.h"

static UART_HandleTypeDef* _gateway_huart;

#define AGN_GATEWAY_TIMEOUT_MS 10000

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

// NOT Reassured with GATEWAY_RECONNECT
void AGN_GATEWAY_RECEIVE_BYTES(uint8_t* bytes, uint8_t size) {
	HAL_StatusTypeDef transmitStatus = HAL_UART_Receive(_gateway_huart, bytes, size, AGN_GATEWAY_TIMEOUT_MS);
	if (transmitStatus == HAL_ERROR) {
		setErrno(AGN_ERRNO_GATEWAY_RECV_ERROR);
	} else if (transmitStatus == HAL_TIMEOUT) {
		setErrno(AGN_ERRNO_GATEWAY_RECV_TIMEOUT);
	}
}

// Reassured with GATEWAY_RECONNECT
void AGN_GATEWAY_RECEIVE_PACKET(struct AGN_PACKET* packet) {
	uint8_t bytes[AGN_PACKET_SIZE];
	AGN_GATEWAY_RECEIVE_BYTES(bytes, AGN_PACKET_SIZE);
	AGN_PACKET_DESERIALIZE(packet, bytes);
	char str[200];
	sprintf(str, "Received Bytes: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6]);
	AGN_LOG_DEBUG(str);

	while(packet->magic != 0xA0A0) {

		// Move buffer to position after magic
		AGN_GATEWAY_RESYNC(bytes);

		// Receive the rest of the bitstream
		AGN_GATEWAY_RECEIVE_BYTES(bytes + 2, AGN_PACKET_SIZE - 2);
		AGN_PACKET_DESERIALIZE(packet, bytes);
	}
}

/* sets GATEWAY UART buffer to next position after magic
 * sets the first `bytes` to magic
 */
// TODO Support multiple magic
int AGN_GATEWAY_RESYNC(uint8_t* bytes) {
	uint8_t buffer;
	uint16_t lastByte = 0x0000;
	AGN_GATEWAY_RECEIVE_BYTES(&buffer, 1);
	uint8_t timer = 100;
	lastByte |= (buffer & 0xFF);
	while(lastByte != 0xA0A0 && timer != 0) {
		lastByte <<= 8;
		AGN_GATEWAY_RECEIVE_BYTES(&buffer, 1);
		lastByte |= (buffer & 0xFF);
		timer--;
	}

	// Handle timeout error
	if (timer == 0) {
		return 1;
	} else {
		bytes[0] = 0xA0;
		bytes[1] = 0xA0;
		return 0;
	}
}
