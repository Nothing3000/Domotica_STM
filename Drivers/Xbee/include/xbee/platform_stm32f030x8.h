/*
 * platform_stm32f030x8.h
 *
 *  Created on: Dec 13, 2018
 *      Author: marlon
 */

#ifndef __XBEE_PLATFORM_STM32F030X8_H_
#define __XBEE_PLATFORM_STM32F030X8_H_

	#include "stm32f0xx.h"
	#include "cmsis_os.h"
	#include <sys/types.h>
	#include <stdint.h>

	#define _f_memcpy		memcpy
	#define _f_memset		memset

	#define FAR

	typedef uint8_t bool_t;

	struct xbee_cbuf_t;
	typedef struct xbee_serial_t {
		uint32_t					baudrate;
		USART_TypeDef			*port;
	} xbee_serial_t;

	#define ZCL_TIME_EPOCH_DELTA	0
	#define XBEE_MS_TIMER_RESOLUTION

#endif /* __XBEE_PLATFORM_STM32F030X8_H_ */
