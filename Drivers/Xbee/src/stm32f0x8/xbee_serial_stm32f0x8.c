/*
 * xbee_serial_stm32f0x8.c
 *
 *  Created on: Dec 13, 2018
 *      Author: marlon
 */

#include <limits.h>
#include "stm32f0xx_hal.h"
#include "xbee/platform.h"
#include "xbee/serial.h"

#define XBEE_SER_CHECK(ptr)	\
	do { if (xbee_ser_invalid(ptr)) return -EINVAL; } while (0)

bool_t xbee_ser_invalid( xbee_serial_t *serial)
{
	if(serial && (HAL_UART_GetState(serial->port) != HAL_UART_STATE_ERROR))
	{
		return 0;
	}
	return 1;
}

const char *xbee_ser_portname( xbee_serial_t *serial)
{
	return "Not implemented";
}

int xbee_ser_write( xbee_serial_t *serial, const void FAR *buffer,
	int length)
{
	HAL_StatusTypeDef result;

	XBEE_SER_CHECK(serial);
	if(length < 0)
	{
		return -EINVAL;
	}

	result = HAL_UART_Transmit(serial->port, buffer, length, 1000);
	if(result == HAL_OK || result == HAL_TIMEOUT)
	{
		return serial->port->TxXferSize - serial->port->TxXferCount;
	}
	else
	{
		return -EIO;
	}
}

int xbee_ser_read( xbee_serial_t *serial, void FAR *buffer, int bufsize)
{
	HAL_StatusTypeDef result;

	XBEE_SER_CHECK(serial);
	if( !buffer || bufsize < 0)
	{
		return -EINVAL;

	}

	result = HAL_UART_Receive(serial->port, buffer, bufsize, 1000);
	if(result == HAL_OK || result == HAL_TIMEOUT)
	{
		return serial->port->RxXferSize - serial->port->RxXferCount;
	}
	else
	{
		return -EIO;
	}
}

int xbee_ser_putchar( xbee_serial_t *serial, uint8_t ch)
{
	int retval;

	retval = xbee_ser_write( serial, &ch, 1);
	if (retval == 1)
	{
		return 0;
	}
	else if (retval == 0)
	{
		return -ENOSPC;
	}
	else
	{
		return retval;
	}
}

int xbee_ser_getchar( xbee_serial_t *serial)
{
	uint8_t ch = 0;
	int retval;

	retval = xbee_ser_read( serial, &ch, 1);
	if (retval != 1)
	{
		return retval ? retval : -ENODATA;
	}

	return ch;
}

int xbee_ser_tx_free( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);
	return INT_MAX;
}

int xbee_ser_tx_used( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);
	return 0;
}

int xbee_ser_tx_flush( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);
	HAL_UART_AbortTransmit(serial->port);
	return 0;
}

int xbee_ser_rx_free( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);
	return INT_MAX;
}

int xbee_ser_rx_used( xbee_serial_t *serial)
{
	//HALP
	return 0;
}

int xbee_ser_rx_flush( xbee_serial_t *serial)
{
	XBEE_SER_CHECK(serial);
	HAL_UART_AbortReceive(serial->port);
	return 0;
}

int xbee_ser_open( xbee_serial_t *serial, uint32_t baudrate)
{

}

int xbee_ser_baudrate( xbee_serial_t *serial, uint32_t baudrate);
int xbee_ser_close( xbee_serial_t *serial);
int xbee_ser_break( xbee_serial_t *serial, bool_t enabled);
int xbee_ser_flowcontrol( xbee_serial_t *serial, bool_t enabled);
int xbee_ser_set_rts( xbee_serial_t *serial, bool_t asserted);
int xbee_ser_get_cts( xbee_serial_t *serial);
