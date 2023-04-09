/*
 * 002_Led&UserButton.c
 *
 *  Created on: Mar 19, 2023
 *      Author: jhern
 */


#include "Stm32f407xx_gpio_driver.h"

#define HIGH ENABLE
#define BTN_PRESSED HIGH

void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void){

	GPIO_Handle_t user_button;
	GPIO_Handle_t LED1;

	user_button.pGPIOx = GPIOA;
	user_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	user_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	user_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	user_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	user_button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	LED1.pGPIOx = GPIOD;
	LED1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	LED1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	LED1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	LED1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&user_button);
	GPIO_Init(&LED1);

	while(1){
		if(GPIO_ReadInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}
	return 0;
}
