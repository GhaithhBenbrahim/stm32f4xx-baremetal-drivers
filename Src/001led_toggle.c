/*
 * 001led_toggle.c
 *
 *  Created on: Apr 20, 2025
 *      Author: benbr
 */
#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0; i<500000; i++);
}

int main(void)
{
	GPIO_Handle_t GPIO_Led;

	GPIO_Led.pGPIOx = GPIOD;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIO_Led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD ,GPIO_PIN_NO_12 );
		delay();
	}


	return 0;
}

