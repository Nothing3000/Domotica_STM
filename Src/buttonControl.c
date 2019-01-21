#include "xbee.h"

#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"


#include "xbee/wpan.h"
#include "wpan/aps.h"
#include "xbee/byteorder.h"

static wpan_envelope_t buttonEnvelope;
static const uint8_t payload = 0x03;
__attribute__((__section__(".user_data"),aligned(4))) static const uint8_t userData[1024];
static buttonData_t *buttonData = (buttonData_t *)&userData[0];

static void sendButtonMessage()
{
	wpan_envelope_send(&buttonEnvelope);
}

static void setButtonEnvelope()
{

	wpan_envelope_create(&buttonEnvelope,&my_xbee.wpan_dev,WPAN_IEEE_ADDR_UNDEFINED,buttonData->network_addr);
	buttonEnvelope.dest_endpoint = buttonData->dest_endpoint;
	buttonEnvelope.source_endpoint = 0xE8;
	buttonEnvelope.profile_id = WPAN_PROFILE_DIGI;
	buttonEnvelope.payload = (void *)&payload;
	buttonEnvelope.length = 1;
}

static void saveButtonData(uint16_t network_addr, uint8_t dest_endpoint)
{
	uint8_t dataBytes[4];
	uint32_t *data = (uint32_t *)dataBytes;
	uint32_t address = (uint32_t)&userData[0];
	uint32_t pageError;

	FLASH_EraseInitTypeDef EraseUserData =
	{
		FLASH_TYPEERASE_PAGES,
		(uint32_t)userData,
		1
	};

	dataBytes[0] = ((uint8_t *)&network_addr)[0];
	dataBytes[1] = ((uint8_t *)&network_addr)[1];
	dataBytes[2] = dest_endpoint;
	dataBytes[3] = 0xFF;

	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

	HAL_FLASHEx_Erase(&EraseUserData, &pageError);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *data);

	HAL_FLASH_Lock();
}

/*
 * Endpoint handler for changing button settings.
 */
int buttonEndpoint(const wpan_envelope_t *envelope, struct wpan_ep_state_t *ep_state)
{
	const buttonData_t *receivedButtonData;
	uint16_t length = envelope->length;

	if(length != sizeof(buttonData_t))
	{
		return -1;
	}

	receivedButtonData = envelope->payload;

	saveButtonData(be16toh(receivedButtonData->network_addr),receivedButtonData->dest_endpoint);

	setButtonEnvelope();
	return 0;
}

/*
 * Task for checking the button.
 */
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
