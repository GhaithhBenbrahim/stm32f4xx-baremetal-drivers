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

```c
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
```
Enables or disables the peripheral clock for the specified GPIO port.

```c
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
````
Initializes a GPIO pin with the provided configuration parameters (mode, speed, pull-up/down, etc.).

```c
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
````
Resets all GPIO registers of the given port to their default values.

```c
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
````
Reads and returns the logic level (0 or 1) from a specific input pin.

```c
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
````
Reads the logic levels of all 16 pins of the input port and returns the result as a 16-bit value.

```c
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
````
Writes a logic level (0 or 1) to a specific output pin.

```c
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
````
Writes a 16-bit value to the entire GPIO output port at once.

```c
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
````
Toggles the current state (0 → 1 or 1 → 0) of a given output pin.

```c
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
````
Enables or disables the interrupt for a specific IRQ number at the NVIC level.

```c
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
````
Configures the interrupt priority level for a given IRQ number.

```c
void GPIO_IRQHandling(uint8_t PinNumber);
````
Handles the interrupt for a specific GPIO pin (clears the pending bit).

