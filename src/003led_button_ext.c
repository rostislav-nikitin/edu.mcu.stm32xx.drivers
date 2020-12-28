/*
 * 002led_button.c
 *
 *  Created on: Aug 27, 2020
 *      Author: s0lid
 */


#include "stm32f407xx.h"

#define BUTTON_PRESSED		LOW

void delay(void)
{
	for(uint32_t i=0; i < 500000 / 2; i++);
}

int main(void)
{
	GPIO_PeriClockControl(GPIOD, ENABLE);


	GPIO_Handle_t gpioLed;
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpioLed);

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Handle_t gpioButton;
	gpioButton.pGPIOx = GPIOB;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Init(&gpioButton);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BUTTON_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
		//delay();
	}


	return 0;
}
