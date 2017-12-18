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
#include "string.h"

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
#ifdef AGN_DEBUG
	char str[200];
	sprintf(str, "Sent Bytes(%d): %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
			sizeof(bytes), bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9]);
	AGN_LOG_DEBUG(str);
#endif
}

// NOT Reassured with GATEWAY_RECONNECT
void AGN_GATEWAY_RECEIVE_BYTES(uint8_t* bytes, uint8_t size) {
	HAL_StatusTypeDef transmitStatus = HAL_UART_Receive(_gateway_huart, bytes, size, AGN_GATEWAY_TIMEOUT_MS);
	if (transmitStatus == HAL_ERROR) {
		setErrno(AGN_ERRNO_GATEWAY_RECV_ERROR);
	} else if (transmitStatus == HAL_TIMEOUT) {
		setErrno(AGN_ERRNO_GATEWAY_RECV_TIMEOUT);
	}

#ifdef AGN_DEBUG
	char str[200];
	char header[30] = "Received Internal:";
	sprintf(str, "%s", header);
	char * a = str + strlen(header);
	for (int i = 0; i < size; i++) {
		sprintf(a, " %02x", bytes[i]);
		a += 3;
	}
	AGN_LOG_DEBUG(str);
#endif
}

// Reassured with GATEWAY_RECONNECT
void AGN_GATEWAY_RECEIVE_PACKET(struct AGN_PACKET* packet) {
	uint8_t bytes[AGN_PACKET_SIZE];
	AGN_GATEWAY_RECEIVE_BYTES(bytes, AGN_PACKET_SIZE);
	AGN_PACKET_DESERIALIZE(packet, bytes);

	while(packet->magic != 0xA0A0) {

		// get synced bytes
		AGN_GATEWAY_RESYNC(bytes, AGN_PACKET_SIZE);

		AGN_PACKET_DESERIALIZE(packet, bytes);
	}
}

/* sets GATEWAY UART buffer to next position after magic
 * sets the first `bytes` to magic
 */
// TODO Support multiple magic
int AGN_GATEWAY_RESYNC(uint8_t* bytes, uint8_t size) {
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

	bytes[0] = 0xA0;
	bytes[1] = 0xA0;

	// Receive the rest of the bitstream
	// (** for some bizzare reason, the already read magic bytes are still read again!)
	AGN_GATEWAY_RECEIVE_BYTES(bytes, AGN_PACKET_SIZE);

	AGN_LOG_DEBUG("GATEWAY_RESYNC");

	// Handle timeout error
	if (timer == 0) {
		return 1;
	} else {
		return 0;
	}
}
