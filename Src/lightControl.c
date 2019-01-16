/*
 * blinkyDemo.c
 *
 *  Created on: 14 dec. 2018
 *      Author: maxso
 */

#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

void toggleGreen()
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void toggleRed()
{
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}
