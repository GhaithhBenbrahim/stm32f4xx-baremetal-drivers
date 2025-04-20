/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 19, 2025
 *      Author: benbr
 */

#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn              - GPIO_Init
 *
 * @brief           - This function initializes the mode and configuration of the given GPIO port
 *                    based on the provided handle structure. It configures pin mode, speed,
 *                    pull-up/pull-down resistors, and output type as specified.
 *
 * @param[in]       - pGPIOHandle: Pointer to GPIO_Handle_t structure containing:
 *                    - GPIOx: GPIO port base address (GPIOA, GPIOB, etc.)
 *                    - GPIO_PinConfig: Configuration structure for the pin(s) including:
 *                      - PinNumber: GPIO pin number(s) to configure
 *                      - Mode: Input/Output/Analog/Alternate function
 *                      - Speed: Output speed (Low/Medium/High/Very High)
 *                      - PuPdControl: Pull-up/Pull-down configuration
 *                      - OpType: Output type (Push-pull/Open-drain)
 *                      - AltFunMode: Alternate function mode (if applicable)
 *
 * @return          - None
 *
 * @Note            - None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	 uint32_t temp = 0; //temporary register

	 //enable the peripheral clock

	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1 . configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else
	{
		//this part will code later . ( interrupt mode)

	}

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. configure the pull up pull down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	//4. configure the out put type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alternate function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}
}

/*********************************************************************
 * @fn              - GPIO_DeInit
 *
 * @brief           - Resets all registers of the specified GPIO port to their default reset values.
 *                    This effectively deinitializes the GPIO peripheral.
 *
 * @param[in]       - pGPIOx: Pointer to the GPIO port to be deinitialized (GPIOA, GPIOB, etc.)
 *                    Must be a valid GPIO_RegDef_t pointer.
 *
 * @return          - None
 *
 * @Note            - 1. This function uses peripheral reset registers (RCC) to perform the reset.
 *                    2. The function checks all possible GPIO ports (A-I) and performs the reset
 *                       if the input pointer matches a known GPIO base address.
 *                    3. No action is taken if an invalid GPIO port pointer is provided.
 *                    4. After reset, all GPIO registers (MODER, OTYPER, OSPEEDR, PUPDR, etc.)
 *                       will be restored to their default (reset) values.
 *                    5. Ensure the corresponding GPIO clock is enabled before calling this function.
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

/*********************************************************************
 * @fn              - GPIO_ReadFromInputPin
 *
 * @brief           - Reads the digital value (0 or 1) from a specific pin of a GPIO port
 *
 * @param[in]       - pGPIOx: Pointer to the GPIO port base address (GPIOA, GPIOB, etc.)
 * @param[in]       - PinNumber: GPIO pin number to read (0..15)
 *
 * @return          - uint8_t: The digital value read from the pin (0 or 1)
 *
 * @Note            - 1. The pin must be configured in input mode before calling this function
 *                    2. This function reads the IDR (Input Data Register) of the GPIO port
 *                    3. The function does not validate the PinNumber parameter for performance
 *                       reasons ( 0-15 for STM32F4 MCUs)
 *                    4. For analog pins, the digital read value may not be meaningful
 *                    5. The function masks and shifts the IDR value to return only the
 *                       requested pin's state
 *                    6. Ensure the GPIO clock is enabled before calling this function
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;
   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 );
   return value;
}

/*********************************************************************
 * @fn              - GPIO_ReadFromInputPort
 * @brief           - Reads all 16 pins of a GPIO port at once
 * @param[in]       - pGPIOx: GPIO port to read (GPIOA, GPIOB, etc.)
 * @return          - uint16_t bitmask of pin states (bit 0 = pin 0, ..., bit 15 = pin 15)
 * @Note            - Port must be in input mode. Reads IDR register directly.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return (uint16_t)pGPIOx->IDR;
}

/*********************************************************************
 * @fn              - GPIO_WriteToOutputPin
 * @brief           - Sets or clears a single GPIO output pin
 * @param[in]       - pGPIOx: GPIO port to modify
 * @param[in]       - PinNumber: Pin to write (0-15)
 * @param[in]       - Value: GPIO_PIN_SET or GPIO_PIN_RESET
 * @Note            - Pin must be in output mode. Modifies ODR register.
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET) {
        pGPIOx->ODR |= (1 << PinNumber);
    } else {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/*********************************************************************
 * @fn              - GPIO_WriteToOutputPort
 * @brief           - Writes a 16-bit value to all GPIO output pins
 * @param[in]       - pGPIOx: GPIO port to write
 * @param[in]       - Value: 16-bit pattern for all pins
 * @Note            - Port must be in output mode. Direct ODR write.
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn              - GPIO_ToggleOutputPin
 * @brief           - Toggles the state of a single GPIO output pin
 * @param[in]       - pGPIOx: GPIO port to modify
 * @param[in]       - PinNumber: Pin to toggle (0-15)
 * @Note            - Pin must be in output mode. Uses ODR XOR operation.
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}
