/*
 * blinkyDemo.c
 *
 *  Created on: 14 dec. 2018
 *      Author: maxso
 */

#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

void blinkyDemoTask(void* parameters)
{
	GPIO_PinState status;

	for(;;)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		vTaskDelay(100);
	}
}

