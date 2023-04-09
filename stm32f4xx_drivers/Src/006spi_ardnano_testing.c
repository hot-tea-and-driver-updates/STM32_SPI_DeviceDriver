/*
 * 006spi_ardnano_testing.c
 *
 *  Created on: Mar 27, 2023
 *      Author: jhern
 */


/* AF MODE: 5
 * SPI2_NSS -> PB12
 * SPI2_SCK -> PB13
 * SPI2_MISO -> PB14
 * SPI2_MOSI -> PB15
 */

/* AF MODE: 5
 * in case top ones don't work anymore.
 * SPI2_NSS -> PB9
 * SPI2_SCK -> PB10
 * SPI2_MISO -> PC2
 * SPI2_MOSI -> PC3
 */

#include "stm32f407xx_SPI_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>


void delay(void);
void SPI2_Inits(void);
void GPIO_ButtonInit(void);
void SPI2_GPIOInits(void);

void delay(void){
	for(uint32_t i = 0; i < 500000; i++);
}


void SPI2_Inits(void){

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generates sclk of 2MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //we dont need slave management

	SPI_Init(&SPI2handle);

}

void GPIO_ButtonInit(void){
	GPIO_Handle_t user_button;

	user_button.pGPIOx = GPIOA;
	user_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	user_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	user_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	user_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_Init(&user_button);

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
	//SPI2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPI2_pins);

	//NSS
	SPI2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2_pins);
}

int main(void){

	char userdata[] = "Hello World";

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * Making SSOE 1 enables NSS output
	 * The NSS pin is automatically managed by the hardware (SSM = 0)
	 * i.e. when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1){
		//while the value of the button is not 1, i.e has not been pressed stay in while loop
		while(!(GPIO_ReadInputPin(GPIOA, GPIO_PIN_NO_0)));

		//button de-bouncing delay
		delay();

		//enables the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//first send data length information
		uint8_t datalen = strlen(userdata);
		SPI_SendData(SPI2, &datalen, 1);

		//to send data
		SPI_SendData(SPI2, (uint8_t*) userdata, strlen(userdata));

		//lets confirm SPI is not busy before closing communication
		//SPI BSY flag is 1 when busy, 0 when not
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
		}
	return 0;
}
