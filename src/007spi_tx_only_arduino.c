/*
 * 002led_button.c
 *
 *  Created on: Aug 27, 2020
 *      Author: s0lid
 */

#include <string.h>
#include "stm32f407xx.h"

//AF5

//PB12 SPI2_NSS
//PB13 SPI2_SCK
//PB14 SPI2_MISO
//PB15 SPI2_MOSI


#define BUTTON_PRESSED		HIGH

void delay(void)
{
	for(uint32_t i=0; i < 500000 / 2; i++);
}

void SPI2_GPIOInit()
{
	GPIO_Handle_t gpioSPI;
	//memset(&gpioSPI, 0, sizeof(GPIO_Handle_t));

	gpioSPI.pGPIOx = GPIOB;
	gpioSPI.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpioSPI.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	gpioSPI.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioSPI.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioSPI.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;


	//NSS
	gpioSPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&gpioSPI);

	//SCLK
	gpioSPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&gpioSPI);

	//MISO
	gpioSPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&gpioSPI);

	//MOSI
	gpioSPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&gpioSPI);



}

void SPI2_Init()
{
	SPI_Handle_t pSPI2;
	//memset(&pSPI2, 0, sizeof(SPI_Handle_t));
	pSPI2.pSPIx = SPI2;

	//pSPI2->pSPIx = SPI2;
	pSPI2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	pSPI2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
	pSPI2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	pSPI2.SPIConfig.SPI_SSM = SPI_SSM_EN;
	pSPI2.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	pSPI2.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	pSPI2.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&pSPI2);
}

void Button_Init()
{
	GPIO_Handle_t gpioButton;
	memset(&gpioButton, 0, sizeof(GPIO_Handle_t));

	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	//gpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	//gpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpioButton);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
}

int main(void)
{

	SPI2_GPIOInit();
	Button_Init();
	SPI2_Init();


	SPI_SSOConfig(SPI2, ENABLE);
	SPI_SSIConfig(SPI2, ENABLE);

	////SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	////SPI_PeripherelControl(SPI2, DISABLE);

	while(1);

	return 0;
}


void EXTI0_IRQHandler(void)
{
	char user_data[] = "Hello world";
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	SPI_PeripherelControl(SPI2, ENABLE);
	//GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	uint8_t sz_user_data_length = strlen(user_data);

	SPI_SendData(SPI2, &sz_user_data_length, sizeof(sz_user_data_length));
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) == FLAG_SET);

	SPI_PeripherelControl(SPI2, DISABLE);
}
