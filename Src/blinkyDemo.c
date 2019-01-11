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
		status = HAL_GPIO_ReadPin(XB1_GPIO_Port, XB1_Pin);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, status);
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, !status);
		vTaskDelay(100);
	}
}

