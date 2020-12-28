/*
 * 002led_button.c
 *
 *  Created on: Aug 27, 2020
 *      Author: s0lid
 */

#include <string.h>
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
	memset(&gpioLed, 0, sizeof(GPIO_Handle_t));

	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpioLed);

	//GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Handle_t gpioButton;
	memset(&gpioButton, 0, sizeof(GPIO_Handle_t));

	gpioButton.pGPIOx = GPIOD;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	//gpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Init(&gpioButton);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1)
	{
		;
	}

	/*while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BUTTON_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
		//delay();
	}*/


	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
