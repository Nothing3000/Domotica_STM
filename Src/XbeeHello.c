/*
 * XbeeHello.c
 *
 *  Created on: 8 jan. 2019
 *      Author: Marlon
 */

#include "cmsis_os.h"
#include "xbee/device.h"

int frame_handler(
	struct xbee_dev_t				*xbee,
	const void 				FAR	*frame,
	uint16_t 						length,
	void 						FAR	*context);

xbee_dev_t my_xbee;

const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
	{
		(xbee_dispatch_table_entry_t){0x10, 0, frame_handler, NULL},
		(xbee_dispatch_table_entry_t)XBEE_FRAME_TABLE_END
	};

int frame_handler(
	struct xbee_dev_t				*xbee,
	const void 				FAR	*frame,
	uint16_t 						length,
	void 						FAR	*context)
{
	return 0;
}

void xbeeHelloTask(void * pvParameters)
{
	volatile UBaseType_t uxHighWaterMark;
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	xbee_serial_t XBEE_SERPORT;
	xbee_dev_init( &my_xbee, &XBEE_SERPORT, NULL, NULL);
	xbee_dev_flowcontrol( &my_xbee, 0);

	for(;;)
	{
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		vTaskDelay(100);
		xbee_dev_tick(&my_xbee);
	}
}
