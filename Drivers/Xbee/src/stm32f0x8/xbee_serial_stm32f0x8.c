/*
 * xbee_serial_stm32f0x8.c
 *
 *  Created on: Dec 13, 2018
 *      Author: marlon
 */

#include <limits.h>
#include "stm32f0xx.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_rcc.h"
#include "xbee/platform.h"
#include "xbee/serial.h"
#include "xbee/cbuf.h"

#define XBEE_SER_CHECK(ptr)	\
	do { if (xbee_ser_invalid(ptr)) return -EINVAL; } while (0)

static char RXBuffer_Space[XBEE_CBUF_OVERHEAD+127];
static xbee_cbuf_t *RXBuffer;

bool_t xbee_ser_invalid( xbee_serial_t *serial)
{
	if(serial)
	{
		return (serial->port != USART1);
	}
	else
	{
		return 1;
	}
}

const char *xbee_ser_portname( xbee_serial_t *serial)
{
	return "USART1";
}

int xbee_ser_write( xbee_serial_t *serial, const void FAR *buffer,
	int length)
{

	XBEE_SER_CHECK( serial);

	if (length < 0)
	{
		return -EINVAL;
	}


	for(int i = 0; i < length; i++)
	{
		while ( (!LL_USART_IsActiveFlag_TC(serial->port)) && (!LL_USART_IsActiveFlag_TXE(serial->port)) )
		{
			// do nothing
		}
		LL_USART_TransmitData8(serial->port,((const uint8_t FAR *)buffer)[i]);
	}

	return length;
}

int xbee_ser_read( xbee_serial_t *serial, void FAR *buffer, int bufsize)
{
	int retval;

	XBEE_SER_CHECK( serial);

	if (bufsize == 0)
	{
		return 0;
	}

	if (bufsize < 0)
	{
		return -EINVAL;
	}

	retval = xbee_cbuf_get( RXBuffer, buffer,
		(bufsize > 255) ? 255 : (uint8_t) bufsize);


	return retval;
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
	return 0;
}

int xbee_ser_rx_free( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);
	return xbee_cbuf_free(RXBuffer);
}

int xbee_ser_rx_used( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);
	return xbee_cbuf_used(RXBuffer);
}

int xbee_ser_rx_flush( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);
	return xbee_cbuf_flush(RXBuffer);
}

int xbee_ser_baudrate( xbee_serial_t *serial, uint32_t baudrate)
{
	XBEE_SER_CHECK( serial);

	LL_USART_DisableIT_RXNE(serial->port);
	LL_USART_Disable(serial->port);
	LL_USART_SetBaudRate(serial->port,
						 LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE),
						 LL_USART_GetOverSampling(serial->port),
						 serial->baudrate);
	LL_USART_Enable(serial->port);
	LL_USART_EnableIT_RXNE(serial->port);
	return 0;

}

int xbee_ser_open( xbee_serial_t *serial, uint32_t baudrate)
{
	RXBuffer = (xbee_cbuf_t *) RXBuffer_Space;
	xbee_cbuf_init( RXBuffer, 127);
	serial->port = USART1;
	serial->baudrate = 9600;


	return xbee_ser_baudrate(serial,serial->baudrate);
}

int xbee_ser_close( xbee_serial_t *serial)
{
	return 0;
}

int xbee_ser_break( xbee_serial_t *serial, bool_t enabled)
{
	return 0;
}

int xbee_ser_flowcontrol( xbee_serial_t *serial, bool_t enabled)
{
	return 0;
}

int xbee_ser_set_rts( xbee_serial_t *serial, bool_t asserted)
{
	return 0;
}

int xbee_ser_get_cts( xbee_serial_t *serial)
{
	return 1;
}

void USART1_RXInterrupt(void)
{
	xbee_cbuf_putch(RXBuffer, LL_USART_ReceiveData8(USART1));
}
