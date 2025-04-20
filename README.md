# STM32F4xx Baremetal Drivers Development

![STM32](https://img.shields.io/badge/STM32F4-Baremetal-03234B?logo=stmicroelectronics)
![Status](https://img.shields.io/badge/Status-Active_Development-yellow)
![License](https://img.shields.io/badge/License-MIT-green)
![Hardware](https://img.shields.io/badge/Board-STM32F407G--DISC1-blueviolet)

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

## Driver Structure

All drivers are located in the `drivers` directory. Each driver's API is documented directly in its header and source files.

## Example Usage

Below are quick examples showing how to use each driver. For complete API documentation, please refer to the comments in each driver's source files.

## Basic GPIO Examples
### 💡 Example: LED Toggle

```c
#include "stm32f407xx.h"

void delay(void) {
    for(uint32_t i = 0; i < 500000; i++);
}

int main(void) {
    GPIO_Handle_t GPIO_Led;
    
    GPIO_Led.pGPIOx = GPIOD;
    GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&GPIO_Led);

    while(1) {
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        delay();
    }
    return 0;
}
```

## 🤝 How to Contribute
- Report bugs via Issues
- Submit PRs with:
  - Hardware-tested code
  - Doxygen documentation
  - Example usage (if adding features)

## 📜 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
