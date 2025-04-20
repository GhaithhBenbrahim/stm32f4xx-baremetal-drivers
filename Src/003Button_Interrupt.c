/*
 * 003Button_Interrupt.c
 *
 *  Created on: Apr 20, 2025
 *      Author: benbr
 */

#include <string.h>
#include "stm32f407xx.h"


void delay(void)
{
	for(uint32_t i = 0; i<250000; i++);
}

int main(void)
{
	GPIO_Handle_t GPIO_Led , GPIO_Button;
	memset(&GPIO_Led,0,sizeof(GPIO_Led));
	memset(&GPIO_Button,0,sizeof(GPIO_Button));


	GPIO_Led.pGPIOx = GPIOD;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIO_Led);

	GPIO_Button.pGPIOx = GPIOA;
	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_Init(&GPIO_Button);

	//IRQ configurations
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI6 );

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, 12);
}
