/*
 * 007SPI_ArduinoRx.c
 *
 *  Created on: Mar 28, 2023
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


//command codes readable by Arduino via SPI
#define COMMAND_LED_CTRL	0x50
#define COMMAND_SENSOR_READ	0X51
#define COMMAND_LED_READ	0x52
#define COMMAND_PRINT		0X53
#define COMMAND_ID_READ		0X54

#define LED_ON	1
#define LED_OFF	0

//arduino analog pins
#define ANALOG_PIN0	0
#define ANALOG_PIN1 1
#define ANALOG_PIN2 2
#define ANALOG_PIN3	3
#define ANALOG_PIN4	4
#define ANALOG_PIN5	5

//arduino LED
#define LED_PIN	9


void delay(void);
void SPI2_Inits(void);
void GPIO_ButtonInit(void);
void SPI2_GPIOInits(void);

void delay(void){
	for(uint32_t i = 0; i < 500000; i++);
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

	//NSS
	SPI2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2_pins);
}

void SPI2_Inits(void){

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; //generates sclk of 2MHz
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
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


uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == (uint8_t) 0xF5){
		//ack
		return 1;
	}
	return 0;
}


int main(void){

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

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

		while(!GPIO_ReadInputPin(GPIOA, GPIO_PIN_NO_0));

		//button de-bouncing delay
		delay();

		//enables the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1 CMD_LED_CTRL <pin no(1)> <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandcode, 1);

		//dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read, 1);

		//send some dummy bits (1 byte) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		/*When this API is called, it returns response from the slave that would
		 * have arrived at the master, i.e reads the ack byte
		 */
		SPI_ReceiveData(SPI2,&ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte)){
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2,args, 2);
		}
		//end of COMMAND_LED_CTRL

		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
		}
	return 0;
}
