/*
 * XbeeHello.c
 *
 *  Created on: 8 jan. 2019
 *      Author: Marlon
 */

#include "cmsis_os.h"
#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee/wpan.h"
#include "wpan/aps.h"
#include "zigbee/zcl.h"

int frame_handler(
	struct xbee_dev_t				*xbee,
	const void 				FAR	*frame,
	uint16_t 						length,
	void 						FAR	*context);

xbee_dev_t my_xbee;
wpan_envelope_t my_envelope;

wpan_cluster_table_entry_t zcl_cluster_table[] =
{
	{ 0, &zcl_general_command, zcl_attributes_none, WPAN_CLUST_FLAG_INOUT },

	WPAN_CLUST_ENTRY_LIST_END
};

wpan_endpoint_table_entry_t endpoints[] =
{
	(wpan_endpoint_table_entry_t)
	{
		0x1,
		WPAN_PROFILE_ZDO,
		NULL,
		NULL,
		0x0000,
		0x00,
		zcl_cluster_table
	},
	WPAN_ENDPOINT_TABLE_END
};

const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
	{
		XBEE_FRAME_HANDLE_LOCAL_AT,
		XBEE_FRAME_HANDLE_RX_EXPLICIT,
		(xbee_dispatch_table_entry_t){XBEE_FRAME_RECEIVE, 0, frame_handler, NULL},
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
	xbee_serial_t XBEE_SERPORT;
	xbee_dev_init( &my_xbee, &XBEE_SERPORT, NULL, NULL);
	xbee_dev_flowcontrol( &my_xbee, 0);
	xbee_wpan_init(&my_xbee, endpoints);

	wpan_envelope_create(&my_envelope,&my_xbee.wpan_dev, WPAN_IEEE_ADDR_COORDINATOR,WPAN_NET_ADDR_COORDINATOR);
	//my_envelope.dest_endpoint = WPAN_ENDPOINT_DIGI_DATA;
	//my_envelope.source_endpoint = WPAN_ENDPOINT_DIGI_DATA;
	//my_envelope.cluster_id = ZCL_CLUST_BINARY_VALUE;
	my_envelope.profile_id = WPAN_PROFILE_DIGI;
	my_envelope.payload = "Test";
	my_envelope.length = 5;
	//wpan_envelope_send(&my_envelope);

	for(;;)
	{
		vTaskDelay(1);
		wpan_tick(&my_xbee.wpan_dev);
	}
}
