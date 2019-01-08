/*
 * xbee_platform_stm32f0x8.c
 *
 *  Created on: Dec 13, 2018
 *      Author: marlon
 */

#include "xbee/platform.h"

uint32_t xbee_seconds_timer()
{
	return xTaskGetTickCount() / 1000;
}

uint32_t xbee_milliseconds_timer()
{
	return xTaskGetTickCount();
}
