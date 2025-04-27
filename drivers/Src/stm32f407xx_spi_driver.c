/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Apr 26, 2025
 *      Author: benbr
 */

#include <stddef.h>
#include "stm32f407xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/******************************************************************************
* @fn      		  - SPI_PeriClockControl
*
* @brief             - Enables or disables peripheral clock for the specified SPI
*
* @param[in]         - pSPIx: Base address of the SPI peripheral (SPI1, SPI2, or SPI3)
* @param[in]         - EnorDi: ENABLE or DISABLE macro to control the clock
*
* @return            - void
*
* @Note              - Currently only implements clock enable functionality
*                    - Clock disable functionality
******************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
    	 if(pSPIx == SPI1)
    	 {
    		 SPI1_PCLK_DI();
         }
         else if (pSPIx == SPI2)
  	     {
        	 SPI2_PCLK_DI();
         }
    	 else if (pSPIx == SPI3)
    	 {
             SPI3_PCLK_DI();
    	 }

    }
}

/******************************************************************************
* @fn                - SPI_Init
*
* @brief             - Initializes the SPI peripheral with specified configuration
*
* @param[in]         - pSPIHandle: Pointer to SPI handle structure containing:
*                     - pSPIx: SPI peripheral base address
*                     - SPIConfig: SPI configuration structure
*
* @return            - void
*
* @Note              - Enables peripheral clock before configuration
*                    - Configures all essential SPI parameters in CR1 register
******************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    // Enable peripheral clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    uint32_t tempreg = 0;

    // 1. Configure device mode (Master/Slave)
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. Configure bus configuration
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // Full-duplex mode: clear BIDIMODE
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // Half-duplex mode: set BIDIMODE
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // Simplex RX-only mode: clear BIDIMODE, set RXONLY
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Configure serial clock speed (baud rate)
    tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4. Configure data frame format (8-bit or 16-bit)
    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. Configure clock polarity (CPOL)
    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    // 6. Configure clock phase (CPHA)
    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    // 7. Configure software slave management (SSM)
    tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    // Write configuration to CR1 register
    pSPIHandle->pSPIx->CR1 = tempreg;
}

/******************************************************************************
* @fn                - SPI_DeInit
*
* @brief             - Deinitializes the SPI peripheral and resets all registers
*                      to their default values
*
* @param[in]         - pSPIx: Pointer to SPI peripheral register definition
*                      (SPI1, SPI2, or SPI3)
*
* @return            - void
*
* @Note              - Should disable peripheral clock when implemented
*                    - Should reset all configuration registers to defaults
*                    - Should handle any necessary peripheral cleanup
******************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if(pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/******************************************************************************
* @fn                - SPI_SendData
*
* @brief             - Transmits data over SPI in blocking mode
*
* @param[in]         - pSPIx: Pointer to SPI peripheral register definition
* @param[in]         - pTxBuffer: Pointer to transmit data buffer
* @param[in]         - Len: Length of data to transmit (in bytes)
*
* @return            - void
*
* @Note              - Blocking call - waits for each byte/word to transmit
*                    - Handles both 8-bit and 16-bit data frame formats
*                    - Automatically detects DFF setting (8/16-bit mode)
*                    - Decrements buffer pointer and length appropriately
******************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        // 1. Wait until TX buffer is empty (ready for new data)
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // 2. Check data frame format (8-bit or 16-bit)
        if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
        {
            // 16-bit data frame format
            pSPIx->DR = *((uint16_t*)pTxBuffer);  // Load 16-bit data
            Len -= 2;                             // Decrement length counter
            pTxBuffer += 2;                       // Increment buffer pointer
        }
        else
        {
            // 8-bit data frame format
            pSPIx->DR = *pTxBuffer;  // Load 8-bit data
            Len--;                   // Decrement length counter
            pTxBuffer++;             // Increment buffer pointer
        }
    }
}

/*********************************************************************
 * @fn              - SPI_ReceiveData
 *
 * @brief           - Receives data from the SPI peripheral and stores it in a buffer
 *
 * @param[in]       - pSPIx: Pointer to the SPI peripheral register definition structure.
 *                     This should be the base address of the SPI peripheral (e.g., SPI1, SPI2)
 * @param[in]       - pRxBuffer: Pointer to the buffer where received data will be stored
 * @param[in]       - Len: Number of data elements to receive (in bytes)
 *
 * @return          - None
 *
 * @Note            - 1. The function assumes the SPI peripheral is already properly configured
 *                    2. This is a blocking function - it will wait until all data is received
 *                    3. The caller must ensure the RxBuffer is large enough to hold Len bytes
 *                    4. SPI slave selection (SS) should be handled externally if needed
 *                    5. Data reception is done in 8-bit mode (1 byte at a time)
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

			//2. check the DFF bit in CR1
			if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data from DR to Rx buffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR ;
				Len -= 2;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIx->DR ;
				Len--;
				pRxBuffer++;
			}
		}

}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}


}


/*********************************************************************
 * @fn      		  - SPI_SSIConfig
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
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}


}

/******************************************************************************
* @fn                - SPI_SSOEConfig
*
* @brief             - Configures the Slave Select Output Enable (SSOE) setting
*                      for SPI master mode
*
* @param[in]         - pSPIx: Pointer to SPI peripheral register definition
* @param[in]         - EnOrDi: ENABLE or DISABLE macro to control SSOE bit
*
* @return            - void
*
* @Note              - SSOE bit controls NSS output in master mode:
*                     - When ENABLED: NSS signal is driven automatically
*                     - When DISABLED: NSS signal must be managed manually
*                    - Only relevant in SPI master mode
*                    - Affects hardware NSS management
******************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // Enable Slave Select output (automatic NSS management)
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        // Disable Slave Select output (manual NSS management)
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

/*********************************************************************
 * @fn              - SPI_IRQInterruptConfig
 *
 * @brief           - Configures the NVIC (Nested Vectored Interrupt Controller) for SPI interrupts
 *
 * @param[in]       - IRQNumber: The interrupt number to configure (0-95)
 * @param[in]       - EnorDi: Enable or disable the interrupt (ENABLE or DISABLE macros)
 *
 * @return          - None
 *
 * @Note            - 1. This function handles NVIC registers ISER0-ISER2 for enabling interrupts
 *                      and ICER0-ICER2 for disabling interrupts
 *                    2. The function automatically determines which NVIC register to modify
 *                       based on the IRQ number
 *                    3. For Cortex-M processors with different NVIC register layouts,
 *                       this function may need modification
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/*********************************************************************
 * @fn              - SPI_IRQPriorityConfig
 *
 * @brief           - Sets the priority for a specific SPI interrupt
 *
 * @param[in]       - IRQNumber: The interrupt number to configure (0-95)
 * @param[in]       - IRQPriority: The priority value to set (lower number = higher priority)
 *
 * @return          - None
 *
 * @Note            - 1. The priority is set in the NVIC Priority Register (IPR)
 *                    2. The function calculates the correct IPR register and bit position
 *                    3. NO_PR_BITS_IMPLEMENTED should be defined based on the MCU's
 *                       actual implemented priority bits
 *                    4. Priority value is shifted to occupy the implemented priority bits
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

/*********************************************************************
 * @fn              - SPI_SendDataIT
 *
 * @brief           - Initiates SPI data transmission in interrupt mode
 *
 * @param[in]       - pSPIHandle: Pointer to SPI handle structure containing configuration
 * @param[in]       - pTxBuffer: Pointer to transmit data buffer
 * @param[in]       - Len: Length of data to transmit (in bytes)
 *
 * @return          - Current SPI transmission state (SPI_READY or SPI_BUSY_IN_TX)
 *
 * @Note            - 1. This is non-blocking function (returns immediately)
 *                    2. If SPI is not busy, it sets up the transfer and enables TX interrupt
 *                    3. The actual transmission happens in the interrupt service routine
 *                    4. Caller should check return value to know if transmission was started
 *                    5. TXEIE (Transmit buffer empty interrupt enable) is enabled
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}


	return state;
}

/*********************************************************************
 * @fn              - SPI_ReceiveDataIT
 *
 * @brief           - Initiates SPI data reception in interrupt mode
 *
 * @param[in]       - pSPIHandle: Pointer to SPI handle structure containing configuration
 * @param[in]       - pRxBuffer: Pointer to receive data buffer
 * @param[in]       - Len: Length of data to receive (in bytes)
 *
 * @return          - Current SPI reception state (SPI_READY or SPI_BUSY_IN_RX)
 *
 * @Note            - 1. This is non-blocking function (returns immediately)
 *                    2. If SPI is not busy, it sets up the reception and enables RX interrupt
 *                    3. The actual reception happens in the interrupt service routine
 *                    4. Caller should check return value to know if reception was started
 *                    5. RXNEIE (Receive buffer not empty interrupt enable) is enabled
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}


	return state;
}

/*********************************************************************
 * @fn              - SPI_IRQHandling
 *
 * @brief           - Handles SPI interrupt requests by checking status flags and managing data transmission/reception
 *
 * @param[in]       - pHandle: Pointer to SPI handle structure containing:
 *                      - SPI peripheral configuration (pSPIx)
 *                      - Current state (RxState, TxState)
 *                      - Data buffers (pTxBuffer, pRxBuffer)
 *                      - Buffer lengths and counters (TxLen, RxLen)
 *
 * @return          - None
 *
 * @Note            - 1. This function should be called from the SPI interrupt service routine (ISR)
 *                    2. Handles both transmission and reception interrupts:
 *                       - TXE (Transmit buffer empty) interrupts
 *                       - RXNE (Receive buffer not empty) interrupts
 *                    3. Manages buffer pointers and counters automatically
 *                    4. Clears interrupt flags after handling
 *                    5. Calls application callback when transfer is complete:
 *                       - SPI_ApplicationEventCallback() with APP_TX_CMPLT/APP_RX_CMPLT events
 *                    6. Disables interrupts when transfer is complete
 *                    7. Resets state to READY when transfer completes
 *                    8. Handles both master and slave modes
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

//some helper function implementations

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}


static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL ;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}
