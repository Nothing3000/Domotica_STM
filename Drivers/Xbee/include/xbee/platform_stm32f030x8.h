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
	#define PACKED_STRUCT struct __attribute__ ((__packed__))

	#define INTERRUPT_DISABLE __disable_irq()
	#define INTERRUPT_ENABLE __enable_irq()

	#define XBEE_WIFI_ENABLED 0
	#define XBEE_CELLULAR_ENABLED 0

	typedef uint8_t bool_t;

	typedef struct xbee_serial_t {
		uint32_t					baudrate;
		USART_TypeDef			*port;
	} xbee_serial_t;

	#define ZCL_TIME_EPOCH_DELTA	0
	#define XBEE_MS_TIMER_RESOLUTION

#endif /* __XBEE_PLATFORM_STM32F030X8_H_ */
