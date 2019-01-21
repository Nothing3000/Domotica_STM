/*
 * xbee.h
 *
 *  Created on: Jan 16, 2019
 *      Author: marlon
 */

#ifndef XBEE_H_
#define XBEE_H_

#include "xbee/device.h"

#define LIGHT_RED_ENDPOINT 0x02
#define LIGHT_GREEN_ENDPOINT 0x03
#define BUTTON_ENDPOINT 0x04

xbee_dev_t my_xbee;

typedef enum
{
	on = 1,
	off = 2,
	toggle = 3
}lightState_t;

typedef PACKED_STRUCT
{
	uint16_t network_addr;
	uint8_t dest_endpoint;
}buttonData_t;

int lightEndpoint(const wpan_envelope_t *, struct wpan_ep_state_t *);
int buttonEndpoint(const wpan_envelope_t *, struct wpan_ep_state_t *);

void setButtonEnvelope(uint16_t,uint8_t);

void xbeeConfigTask(void *);
void pollButonTask(void *);

#endif /* XBEE_H_ */
