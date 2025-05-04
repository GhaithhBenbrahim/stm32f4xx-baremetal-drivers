/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: May 4, 2025
 *      Author: benbr
 */

#include "stm32f407xx_usart_driver.h"
#include <stddef.h>

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Configures USART baud rate
 *
 * @param[in]         - pUSARTx:   Pointer to USART peripheral register structure
 * @param[in]         - BaudRate:  Desired baud rate in bits per second
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              - Calculates BRR register value based on APB clock
 *                    - Handles both OVER8=0 (16x) and OVER8=1 (8x) modes
 *                    - Supports all USART peripherals (APB1 and APB2 buses)
 *                    - Uses integer math for optimal performance
 *
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    //Variable to hold the APB clock
    uint32_t PCLKx;
    uint32_t usartdiv;

    //variables to hold Mantissa and Fraction values
    uint32_t M_part, F_part;
    uint32_t tempreg = 0;

    //Get the value of APB bus clock in to the variable PCLKx
    if(pUSARTx == USART1 || pUSARTx == USART6)
    {
        //USART1 and USART6 are hanging on APB2 bus
        PCLKx = RCC_GetPCLK2Value();
    }
    else
    {
        PCLKx = RCC_GetPCLK1Value();
    }

    //Check for OVER8 configuration bit
    if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        //OVER8 = 1, over sampling by 8
        usartdiv = ((25 * PCLKx) / (2 * BaudRate));
    }
    else
    {
        //over sampling by 16
        usartdiv = ((25 * PCLKx) / (4 * BaudRate));
    }

    //Calculate the Mantissa part
    M_part = usartdiv / 100;

    //Place the Mantissa part in appropriate bit position (BRR[15:4])
    tempreg |= M_part << 4;

    //Extract the fraction part
    F_part = (usartdiv - (M_part * 100));

    //Calculate the final fractional part
    if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        //OVER8 = 1, over sampling by 8 (3-bit fraction)
        F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
    }
    else
    {
        //over sampling by 16 (4-bit fraction)
        F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
    }

    //Place the fractional part in appropriate bit position (BRR[3:0])
    tempreg |= F_part;

    //copy the value of tempreg in to BRR register
    pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Initializes USART peripheral with given configuration
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure containing
 *                      configuration parameters and register base address
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              - Configures CR1, CR2, CR3 and BRR registers based on handle
 *                      parameters. Enables peripheral clock before configuration.
 *
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
    //Temporary variable
    uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

    //Implement the code to enable the Clock for given USART peripheral
    USART_PeriClockControl(pUSARTHandle->pUSARTx,ENABLE);

    //Enable USART Tx and Rx engines according to the USART_Mode configuration item
    if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        //Implement the code to enable the Receiver bit field
        tempreg|= (1 << USART_CR1_RE);
    }else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        //Implement the code to enable the Transmitter bit field
        tempreg |= ( 1 << USART_CR1_TE );
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        //Implement the code to enable the both Transmitter and Receiver bit fields
        tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
    }

    //Implement the code to configure the Word length configuration item
    tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;

    //Configuration of parity control bit fields
    if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        //Implement the code to enable the parity control
        tempreg |= ( 1 << USART_CR1_PCE);
        //EVEN parity is default when PCE=1 and PS=0
    }
    else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
    {
        //Implement the code to enable the parity control
        tempreg |= ( 1 << USART_CR1_PCE);
        //Implement the code to enable ODD parity
        tempreg |= ( 1 << USART_CR1_PS);
    }

    //Program the CR1 register
    pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

    tempreg=0;

    //Implement the code to configure the number of stop bits inserted during USART frame transmission
    tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

    //Program the CR2 register
    pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

    tempreg=0;

    //Configuration of USART hardware flow control
    if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
    {
        //Implement the code to enable CTS flow control
        tempreg |= ( 1 << USART_CR3_CTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
    {
        //Implement the code to enable RTS flow control
        tempreg |= ( 1 << USART_CR3_RTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        //Implement the code to enable both CTS and RTS Flow control
        tempreg |= ( (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE) );
    }

    pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

    //Implement the code to configure the baud rate
    USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
}

/*********************************************************************
 * @fn      		  - USART_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
    if(pUSARTx == USART1)
    {
        USART1_REG_RESET();
    }else if (pUSARTx == USART2)
    {
        USART2_REG_RESET();
    }else if (pUSARTx == USART3)
    {
        USART3_REG_RESET();
    }else if (pUSARTx == UART4)
    {
        UART4_REG_RESET();
    }else if (pUSARTx == UART5)
    {
        UART5_REG_RESET();
    }else if (pUSARTx == USART6)
    {
        USART6_REG_RESET();
    }
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - Enables or disables the USART peripheral
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral register structure
 * @param[in]         - Cmd: ENABLE or DISABLE macro
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              - Controls the UE (USART Enable) bit in CR1 register
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd)
{
	if(Cmd == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - Controls clock gating for USART peripherals
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral register structure
 * @param[in]         - EnorDi: ENABLE or DISABLE macro
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              - Handles clock enable/disable for all USART/UART instances
 *
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}else if (pUSARTx == USART3)
		{
			UART5_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - Checks the status of a specific USART flag
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral register structure
 * @param[in]         - StatusFlagName: USART status flag to check (e.g., USART_SR_RXNE)
 * @param[in]         -
 *
 * @return            - SET (1) if flag is set, RESET (0) if flag is not set
 *
 * @Note              - Reads the Status Register (SR) and checks the specified bit
 *
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
        return SET;
    }

    return RESET;
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Transmits data via USART peripheral
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 * @param[in]         - pTxBuffer: Pointer to transmit data buffer
 * @param[in]         - Len: Number of bytes to transmit
 *
 * @return            - void
 *
 * @Note              - Blocks until all data is transmitted
 *                    - Handles both 8-bit and 9-bit data formats
 *                    - Waits for TC (Transmission Complete) flag at end
 *                    - Manages buffer pointer increment based on word length
 *
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint16_t *pdata;

    //Loop over until "Len" number of bytes are transferred
    for(uint32_t i = 0 ; i < Len; i++)
    {
        //Wait until TXE (Transmit Data Register Empty) flag is set
        while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

        //Check the USART_WordLength item for 9BIT or 8BIT in a frame
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            //If 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
            pdata = (uint16_t*) pTxBuffer;
            pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

            //Check for USART_ParityControl
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity - 9 bits of user data will be sent
                pTxBuffer++;
                pTxBuffer++; //Increment twice for 16-bit value
            }
            else
            {
                //Parity enabled - 8 bits user data + 1 parity bit
                pTxBuffer++; //Increment once (9th bit is parity)
            }
        }
        else
        {
            //8-bit data transfer
            pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
            pTxBuffer++; //Increment buffer pointer
        }
    }

    //Wait until TC (Transmission Complete) flag is set
    while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - This function receives data over USART peripheral.
 *
 * @param[in]         - pUSARTHandle : Pointer to the USART handle structure
 * @param[in]         - pRxBuffer    : Pointer to the receive buffer where data will be stored
 * @param[in]         - Len          : Number of bytes of data to receive
 *
 * @return            - None
 *
 * @Note              - Supports 8-bit and 9-bit data frame reception.
 *                      Parity control is also handled based on configuration.
 *                      This is a blocking call and waits until data is received.
 */


void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//Now , increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - Sends data over USART using interrupt-based transmission
 *
 * @param[in]         - pUSARTHandle : Pointer to the USART handle structure
 * @param[in]         - pTxBuffer    : Pointer to the buffer containing data to be sent
 * @param[in]         - Len          : Length of data to be sent (in bytes)
 *
 * @return            - Transmission state (USART_BUSY_IN_TX or USART_READY)
 *
 * @Note              - Enables TXEIE and TCIE interrupts to handle transmission in ISR
 *                      User must implement the ISR to handle the interrupt-driven process.
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable TXE interrupt
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);

		// Enable Transmission Complete interrupt
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);
	}

	return txstate;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - Receives data over USART using interrupt-based reception
 *
 * @param[in]         - pUSARTHandle : Pointer to the USART handle structure
 * @param[in]         - pRxBuffer    : Pointer to the buffer where received data will be stored
 * @param[in]         - Len          : Expected length of data to be received (in bytes)
 *
 * @return            - Reception state (USART_BUSY_IN_RX or USART_READY)
 *
 * @Note              - Enables RXNEIE interrupt to handle reception in ISR.
 *                      Dummy read of DR is done to clear RXNE flag if set before starting.
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		// Dummy read to clear RXNE if already set
		(void)pUSARTHandle->pUSARTx->DR;

		// Enable RXNE interrupt
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}


/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - Clears specific status flags in USART Status Register
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral register structure
 * @param[in]         - StatusFlagName: Flag(s) to clear (bitmask)
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              - Applicable to only USART_CTS_FLAG, USART_LBD_FLAG,
 *                      USART_TC_FLAG, USART_TXE_FLAG flags
 *                    - Uses bitwise NOT and AND operation to clear flags
 *
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    pUSARTx->SR &= ~(StatusFlagName);
}

/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - Enables/disables USART interrupt in NVIC
 *
 * @param[in]         - IRQNumber: Interrupt number (0-95)
 * @param[in]         - EnorDi: ENABLE or DISABLE macro
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              - Configures NVIC ISER/ICER registers
 *                    - Handles all three NVIC register banks
 *                    - No parameter validation performed
 *
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            //program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
        {
            //program ISER1 register
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            //program ISER2 register //64 to 95
            *NVIC_ISER3 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if(IRQNumber <= 31)
        {
            //program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            //program ICER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 6 && IRQNumber < 96)
        {
            //program ICER2 register
            *NVIC_ICER3 |= (1 << (IRQNumber % 64));
        }
    }
}

/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - Sets interrupt priority in NVIC
 *
 * @param[in]         - IRQNumber: Interrupt number (0-95)
 * @param[in]         - IRQPriority: Priority value (0-15 typically)
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              - Calculates correct IPR register and bit position
 *                    - Handles processors with limited priority bits
 *                    - NVIC_PR_BASE_ADDR must be defined
 *
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    //1. first lets find out the ipr register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - USART_IRQHandling
 *
 * @brief             - Handles all USART peripheral interrupt requests
 *
 * @param[in]         - pUSARTHandle : Pointer to USART handle structure
 *
 * @return            - None
 *
 * @Note              - This function should be called from the actual IRQ Handler
 *                      defined in the startup file (e.g., USART1_IRQHandler).
 *                      It handles TXE, TC, RXNE, CTS, IDLE, ORE, and other error flags.
 *********************************************************************/

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					 pUSARTHandle->RxLen-=1;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}


}

/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             - Application callback function to handle various USART events.
 *
 * @param[in]         - pUSARTHandle : Pointer to USART handle structure
 * @param[in]         - event        : Event code indicating the type of event occurred.
 *                                     Possible values:
 *                                     - USART_EVENT_TX_CMPLT  : Transmission complete
 *                                     - USART_EVENT_RX_CMPLT  : Reception complete
 *                                     - USART_EVENT_CTS       : CTS pin toggled (if CTS enabled)
 *                                     - USART_EVENT_IDLE      : IDLE line detected
 *                                     - USART_ERR_FE          : Framing error
 *                                     - USART_ERR_NE          : Noise error
 *                                     - USART_ERR_ORE         : Overrun error
 *
 * @return            - None
 *
 * @Note              - This is a `__weak` function. The user application can override
 *                      this function to handle events.
 *********************************************************************/
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event)
{

}

