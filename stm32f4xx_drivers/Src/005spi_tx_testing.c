/*
 * 005spi_tx_testing.c
 *
 *  Created on: Mar 26, 2023
 *      Author: jhern
 */


/* AF MODE: 5
 * SPI2_NSS -> PB12
 * SPI2_SCK -> PB13
 * SPI2_MISO -> PB14
 * SPI2_MOSI -> PB15
 */

#include "stm32f407xx_SPI_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>

void SPI2_Inits(){

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //generates sclk of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //we dont need slave management

	SPI_Init(&SPI2handle);

}


void SPI2_GPIOInits(void){
	GPIO_Handle_t SPI2_pins;
	SPI2_pins.pGPIOx = GPIOB;
	SPI2_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI2_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	SPI2_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	SPI2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI2_pins);

	//MOSI
	SPI2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI2_pins);

	//MISO
	SPI2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI2_pins);

	SPI2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2_pins);
}

int main(void){

	char userdata[] = "Hello World";

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//this makes NSS signal internal and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//enables the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//to send data
	SPI_SendData(SPI2, (uint8_t*) userdata, strlen(userdata));

	//checks to see if SPI2 peripheral is busy sending data
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1);

	return 0;
}
