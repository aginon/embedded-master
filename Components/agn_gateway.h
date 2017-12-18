/*
 * agn_gateway.h
 *
 *  Created on: Nov 23, 2017
 *      Author: Kris
 */

#ifndef AGN_GATEWAY_H_
#define AGN_GATEWAY_H_

#include "stm32f4xx_hal.h"
#include "agn_packet.h"

void AGN_GATEWAY_INITIALIZE(UART_HandleTypeDef * gateway_huart);

void AGN_GATEWAY_SEND_BYTES(uint8_t* bytes, uint16_t size);
void AGN_GATEWAY_SEND_PACKET(struct AGN_PACKET* packet);

void AGN_GATEWAY_RECEIVE_BYTES(uint8_t* bytes, uint8_t size);
void AGN_GATEWAY_RECEIVE_PACKET(struct AGN_PACKET* packet);

int AGN_GATEWAY_RESYNC(uint8_t* bytes, uint8_t size);
#endif /* AGN_GATEWAY_H_ */
