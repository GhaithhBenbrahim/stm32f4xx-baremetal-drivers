/*
 * stm32f4xx.h
 *
 *  Created on: Apr 19, 2025
 *      Author: benbr
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR               0x08000000U
#define SRAM1_BASEADDR               0x20000000U
#define SRAM2_BASEADDR               0x2001C000U
#define ROM_BASEADDR                 0x1FFF0000U
#define SRAM                         SRAM1_BASE_ADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR                  0x40000000U
#define APB1PERIPH_BASEADDR              PERIPH_BASE_ADDR
#define APB2PERIPH_BASEADDR              0x40010000U
#define AHB1PERIPH_BASEADDR              0x40020000U
#define AHB2PERIPH_BASEADDR              0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)

/*************************************peripheral register definition structures *************************************/

typedef struct
{
    __vo uint32_t MODER;     /*!< Mode reg (input/output/alt/analog)        | 0x00 */
    __vo uint32_t OTYPER;    /*!< Output type (push-pull/open-drain)        | 0x04 */
    __vo uint32_t OSPEEDR;   /*!< Output speed (low/med/high/very high)     | 0x08 */
    __vo uint32_t PUPDR;     /*!< Pull-up/pull-down config                  | 0x0C */
    __vo uint32_t IDR;       /*!< Input data reg (read pin state)           | 0x10 */
    __vo uint32_t ODR;       /*!< Output data reg (set pin state)           | 0x14 */
    __vo uint32_t BSRR;      /*!< Bit set/reset (atomic write to ODR)       | 0x18 */
    __vo uint32_t LCKR;      /*!< Configuration lock reg                    | 0x1C */
    __vo uint32_t AFR[2];    /*!< Alternate func (AFRL:0-7, AFRH:8-15)      | 0x20-0x24 */
} GPIO_RegDef_t;

typedef struct
{
    __vo uint32_t CR;            /*!< Clock control (HSE/HSI/PLL on/off)     | 0x00 */
    __vo uint32_t PLLCFGR;       /*!< Main PLL config                        | 0x04 */
    __vo uint32_t CFGR;          /*!< System clock source & prescalers       | 0x08 */
    __vo uint32_t CIR;           /*!< Clock interrupt control/status         | 0x0C */
    __vo uint32_t AHB1RSTR;      /*!< AHB1 peripherals reset                 | 0x10 */
    __vo uint32_t AHB2RSTR;      /*!< AHB2 peripherals reset                 | 0x14 */
    __vo uint32_t AHB3RSTR;      /*!< AHB3 peripherals reset                 | 0x18 */
    uint32_t      RESERVED0;     /* Reserved                                 | 0x1C */
    __vo uint32_t APB1RSTR;      /*!< APB1 peripherals reset                 | 0x20 */
    __vo uint32_t APB2RSTR;      /*!< APB2 peripherals reset                 | 0x24 */
    uint32_t      RESERVED1[2];  /* Reserved                                 | 0x28-0x2C */
    __vo uint32_t AHB1ENR;       /*!< AHB1 peripherals clock enable          | 0x30 */
    __vo uint32_t AHB2ENR;       /*!< AHB2 peripherals clock enable          | 0x34 */
    __vo uint32_t AHB3ENR;       /*!< AHB3 peripherals clock enable          | 0x38 */
    uint32_t      RESERVED2;     /* Reserved                                 | 0x3C */
    __vo uint32_t APB1ENR;       /*!< APB1 peripherals clock enable          | 0x40 */
    __vo uint32_t APB2ENR;       /*!< APB2 peripherals clock enable          | 0x44 */
    uint32_t      RESERVED3[2];  /* Reserved                                 | 0x48-0x4C */
    __vo uint32_t AHB1LPENR;     /*!< AHB1 low-power clock enable            | 0x50 */
    __vo uint32_t AHB2LPENR;     /*!< AHB2 low-power clock enable            | 0x54 */
    __vo uint32_t AHB3LPENR;     /*!< AHB3 low-power clock enable            | 0x58 */
    uint32_t      RESERVED4;     /* Reserved                                 | 0x5C */
    __vo uint32_t APB1LPENR;     /*!< APB1 low-power clock enable            | 0x60 */
    __vo uint32_t APB2LPENR;     /*!< APB2 low-power clock enable            | 0x64 */
    uint32_t      RESERVED5[2];  /* Reserved                                 | 0x68-0x6C */
    __vo uint32_t BDCR;          /*!< Backup domain control (RTC/LSE)         | 0x70 */
    __vo uint32_t CSR;           /*!< Clock status & reset flags              | 0x74 */
    uint32_t      RESERVED6[2];  /* Reserved                                 | 0x78-0x7C */
    __vo uint32_t SSCGR;         /*!< Spread spectrum clock gen               | 0x80 */
    __vo uint32_t PLLI2SCFGR;    /*!< PLLI2S config                          | 0x84 */
    __vo uint32_t PLLSAICFGR;    /*!< PLLSAI config                          | 0x88 */
    __vo uint32_t DCKCFGR;       /*!< Dedicated clocks config                 | 0x8C */
    __vo uint32_t CKGATENR;      /*!< Peripheral clock gating enable          | 0x90 */
    __vo uint32_t DCKCFGR2;      /*!< Dedicated clocks config (additional)   | 0x94 */
} RCC_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()    	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB1ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()          do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
