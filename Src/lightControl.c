/*
 * blinkyDemo.c
 *
 *  Created on: 14 dec. 2018
 *      Author: maxso
 */

#include "xbee.h"
#include <string.h>
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

int lightEndpoint(const wpan_envelope_t *envelope, struct wpan_ep_state_t *ep_state)
{
	const lightState_t *state;
	const uint16_t length = envelope->length;
	uint8_t dest_endpoint = envelope->dest_endpoint;
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;



	if(length != 1)
	{
		return -1;
	}

	switch(dest_endpoint)
	{
	case LIGHT_RED_ENDPOINT:
		GPIOx = LD1_GPIO_Port;
		GPIO_Pin = LD1_Pin;
		break;
	case LIGHT_GREEN_ENDPOINT:
		GPIOx = LD2_GPIO_Port;
		GPIO_Pin = LD2_Pin;
		break;
	default:
		return -1;
	}

	state = (const lightState_t *)envelope->payload;

	switch(*state)
	{
	case on:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		break;
	case off:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
		break;
	case toggle:
		HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
		break;
	default:
		return -1;
	}


	return 0;
}
