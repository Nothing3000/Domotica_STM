/*
 * xbee.h
 *
 *  Created on: Jan 16, 2019
 *      Author: marlon
 */

#ifndef XBEE_H_
#define XBEE_H_

#include "xbee/device.h"

xbee_dev_t my_xbee;

void toggleGreen(void);
void toggleRed(void);

void setButtonEnvelope(uint16_t,uint8_t);

void xbeeConfigTask(void *);
void pollButonTask(void *);

#endif /* XBEE_H_ */
