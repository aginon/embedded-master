/*
 * agn_packet.c
 *
 *  Created on: Dec 16, 2017
 *      Author: Kris
 */

#include "agn_packet.h"
#include <string.h>

// SIZE IN BYTES
const int AGN_PACKET_SIZE = 10;

const struct AGN_PACKET_SIZES AGN_PACKET_SIZES_CONFIG =  {
		16, 	// magic is 16 bit
		32, 	// depth is 32 bit
		16,		// status is 16 bit
		8,		// mode is 8 bit
		4, 		// hex1 is 4 bit
		4		// hex2 is 4 bit
};

void AGN_PACKET_SERIALIZE(struct AGN_PACKET *packet, uint8_t* bytes) {
	uint16_t magic = 0xA0A0;
	memcpy(&bytes[0], &magic, 2);
	uint32_t depth = packet->depth;
	memcpy(&bytes[2], &depth, 4);
	uint16_t status = packet->status;
	memcpy(&bytes[6], &status, 2);
	uint8_t mode = packet->mode;
	memcpy(&bytes[8], &mode, 1);
	uint8_t hex = packet->hex1 << 4 | (packet->hex2 & 0x0F);
	memcpy(&bytes[9], &hex, 1);
}

void AGN_PACKET_DESERIALIZE(struct AGN_PACKET *packet, uint8_t* bytes) {
	uint16_t magic;
	memcpy(&magic, &bytes[0], 2);
	packet->magic = magic;

	uint32_t depth = 0;
	memcpy(&depth, &bytes[2], 4);
	packet->depth = depth;

	uint16_t status = 0;
	memcpy(&status, &bytes[6], 2);
	packet->status = status;

	uint8_t mode = 0;
	memcpy(&mode, &bytes[8], 1);
	packet->mode = mode;

	uint8_t hex = 0;
	memcpy(&hex, &bytes[9], 1);
	packet->hex1 = (hex >> 4) & 0xF;
	packet->hex2 = (hex) & 0xF;
}
