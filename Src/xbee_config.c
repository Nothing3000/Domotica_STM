/*
 * XbeeHello.c
 *
 *  Created on: 8 jan. 2019
 *      Author: Marlon
 */

#include "cmsis_os.h"
#include "xbee.h"
#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee/wpan.h"
#include "wpan/aps.h"
#include "zigbee/zcl.h"

//xbee_dev_t my_xbee;

wpan_cluster_table_entry_t zcl_cluster_table[] =
{
	WPAN_CLUST_ENTRY_LIST_END
};

wpan_endpoint_table_entry_t endpoints[] =
{
	{
		0x1,
		WPAN_PROFILE_ZDO,
		NULL,
		NULL,
		0x0000,
		0x00,
		zcl_cluster_table
	},
	{
		0x2,
		WPAN_PROFILE_DIGI,
		toggleGreen,
		NULL,
		0x0000,
		0x00,
		zcl_cluster_table
	},
	{
		0x3,
		WPAN_PROFILE_DIGI,
		toggleRed,
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
		XBEE_FRAME_TABLE_END
	};

void xbeeConfigTask(void * pvParameters)
{
	xbee_serial_t XBEE_SERPORT;
	xbee_dev_init( &my_xbee, &XBEE_SERPORT, NULL, NULL);
	xbee_dev_flowcontrol( &my_xbee, 0);
	xbee_wpan_init(&my_xbee, endpoints);

	setButtonEnvelope(0xAE5E,0x03);

	xTaskCreate( pollButonTask,
			    (char *) "pollButonTask",
				configMINIMAL_STACK_SIZE,
				NULL,
				3,
				NULL
			   );

	for(;;)
	{
		vTaskDelay(1);
		wpan_tick(&my_xbee.wpan_dev);
	}
}
