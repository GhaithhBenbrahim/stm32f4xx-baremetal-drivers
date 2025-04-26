/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Apr 26, 2025
 *      Author: benbr
 */

#include "stm32f407xx_spi_driver.h"
/*
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);*/

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
