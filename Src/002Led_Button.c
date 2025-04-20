/*
 * 002Led_Button.c
 *
 *  Created on: Apr 20, 2025
 *      Author: benbr
 */

#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0; i<50000; i++);
}

int main(void)
{
	GPIO_Handle_t GPIO_Led , GPIO_Button;

	GPIO_Led.pGPIOx = GPIOD;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Button.pGPIOx = GPIOA;
	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	/*GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);*/

	GPIO_Init(&GPIO_Led);
	GPIO_Init(&GPIO_Button);

#if 0
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0))
		{
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, SET);
		}
		else GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, RESET);

	}
#endif

	static uint8_t led_state = 0;

	while(1)
	{
	    // Detect rising edge (button pressed)
	    if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
	    {
	        // Simple debounce delay
	        delay();

	        // Wait until button is released
	        while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

	        // Toggle LED state
	        led_state ^= 1;
	        GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, (led_state) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	        // Additional debounce delay
	        delay();
	    }
	}


	return 0;
}
