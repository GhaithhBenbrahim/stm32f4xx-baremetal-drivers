# STM32F4xx Baremetal Drivers Development

![STM32](https://img.shields.io/badge/STM32F4-Baremetal-03234B?logo=stmicroelectronics)
![Status](https://img.shields.io/badge/Status-Active_Development-yellow)
![Hardware](https://img.shields.io/badge/Board-STM32F4xx-blueviolet)

## Introduction

This repository documents my learning journey in developing baremetal drivers for STM32F4 microcontrollers (specifically tested on STM32F407G-DISC1). The primary purpose is educational - to deeply understand microcontroller internals by working directly with hardware registers, rather than creating production-ready drivers.Currently implemented drivers include:

- **GPIO** (General Purpose Input/Output)
- **SPI** (Serial Peripheral Interface)
- **I2C** (Inter-Integrated Circuit)
- **USART** (Universal Synchronous/Asynchronous Receiver/Transmitter)

**Note:** For most real-world applications, ST's HAL library remains the better choice as it's thoroughly tested and maintained. This project serves as a learning exercise in embedded systems fundamentals.

## Why Baremetal? (Learning Objectives)
- 🎓 Understand register-level microcontroller programming
- 🔍 Learn how HAL libraries abstract hardware operations
- 🧩 Master peripheral initialization and configuration
- ⚡ Observe the direct performance impact of low-level coding
- 🛠️ Develop debugging skills for hardware-level issues

## Getting Started
For fellow learners:
```bash
git clone https://github.com/GhaithhBenbrahim/stm32f4xx-baremetal-drivers.git
````

## 🧠 API References

### 🔌 GPIO APIs

#### Clock Configuration
##### GPIO Peripheral Clock Control
`void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)`  
Enables/disables clock for specified GPIO port  
**Parameters:**  
- `pGPIOx`: GPIO port (GPIOA, GPIOB, etc.)  
- `EnorDi`: ENABLE or DISABLE  

#### Initialization
##### GPIO Initialization  
`void GPIO_Init(GPIO_Handle_t *pGPIOHandle)`  
Configures GPIO pin with specified settings  

##### GPIO Deinitialization  
`void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)`  
Resets all registers for specified GPIO port  

#### Input Operations
##### Read Input Pin  
`uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)`  
Returns digital value (0/1) from specified pin  

##### Read Input Port  
`uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)`  
Returns 16-bit value of entire port  

#### Output Operations
##### Write Output Pin  
`void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)`  
Sets output pin high (1) or low (0)  

##### Write Output Port  
`void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)`  
Writes 16-bit value to entire port  

##### Toggle Output Pin  
`void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)`  
Inverts current pin state  

#### Interrupt Handling
##### Configure Interrupt  
`void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)`  
Enables/disables interrupt for specified IRQ  

##### Set Interrupt Priority  
`void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)`  
Configures interrupt priority level (0-15)  

##### Handle Interrupt  
`void GPIO_IRQHandling(uint8_t PinNumber)`  
Clears pending interrupt flag  


