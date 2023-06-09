# STM32_SPI_DeviceDriver
This repository contains a device driver for the Serial Peripheral Interface (SPI) protocol that is implemented for STM32 microcontrollers. The driver is written in C programming language and is intended to be used with the STM32F4 Discovery board. Note: This repository also contains the device driver for GPIO pins on the STM32 and is only used in the example programs, the SPI files are not dependent on them.

## Installation
To install the driver, simply clone this repository onto your local machine. The driver source code is located in the src directory within the drivers file. There is no need to keep the stand alone driver file if you intend to download for use. 

## Dependencies
The driver requires the following dependencies to be installed:

STM32CubeMX or 
Keil MDK-ARM

## Configuration
To configure the driver for your specific use case, you will need to modify the spi_driver_config.h file. This file contains configuration settings for the SPI driver, including the SPI clock frequency, the SPI data format, and the SPI slave select pin.

## Building
To build the driver, open the STM32_SPI_DeviceDriver.uvprojx project file in Keil MDK-ARM or STM32CubeMX and build the project.

## Usage
To use the driver in your own project, simply include the spi_driver.h header file and call the appropriate functions to initialize and communicate with the SPI peripheral. Example code is provided in the main.c file to demonstrate how to use the driver.

## Contributing
If you would like to contribute to this project, please open an issue or submit a pull request with your proposed changes.

## License
This project is licensed under the MIT License - see the LICENSE file for details.
