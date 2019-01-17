#include "xbee.h"

#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"


#include "xbee/wpan.h"
#include "wpan/aps.h"
#include "xbee/byteorder.h"

wpan_envelope_t buttonEnvelope;
static uint8_t payload = 0x03;


static void sendButtonMessage()
{
	wpan_envelope_send(&buttonEnvelope);
}

void setButtonEnvelope(uint16_t network_addr,uint8_t dest_endpoint)
{
	wpan_envelope_create(&buttonEnvelope,&my_xbee.wpan_dev,WPAN_IEEE_ADDR_UNDEFINED,network_addr);
	buttonEnvelope.dest_endpoint = dest_endpoint;
	buttonEnvelope.source_endpoint = 0xE8;
	buttonEnvelope.profile_id = WPAN_PROFILE_DIGI;
	buttonEnvelope.payload = (void *)&payload;
	buttonEnvelope.length = 1;
}

int buttonEndpoint(const wpan_envelope_t *envelope, struct wpan_ep_state_t *ep_state)
{
	const buttonData_t *buttonData;
	uint16_t length = envelope->length;

	if(length != sizeof(buttonData_t))
	{
		return -1;
	}

	buttonData = envelope->payload;

	setButtonEnvelope(be16toh(buttonData->network_addr),buttonData->dest_endpoint);
	return 0;
}

void pollButonTask(void * pvParameters)
{
	GPIO_PinState currentState = HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin);
	GPIO_PinState previousState = currentState;

	for(;;)
	{
		currentState = HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin);
		if(currentState != previousState && currentState == GPIO_PIN_RESET)
		{
			sendButtonMessage();
		}
		previousState = currentState;
		vTaskDelay(200);
	}
}
