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


/*#define BUTTON_PRESSED		LOW

void delay(void)
{
	for(uint32_t i=0; i < 500000 / 2; i++);
}*/
void SPI2_GPIOInit()
{
	GPIO_Handle_t gpioSPI;
	//memset(&gpioSPI, 0, sizeof(GPIO_Handle_t));

	gpioSPI.pGPIOx = GPIOB;
	gpioSPI.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpioSPI.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	gpioSPI.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioSPI.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioSPI.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;


	gpioSPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&gpioSPI);

	gpioSPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&gpioSPI);

	gpioSPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&gpioSPI);

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
	pSPI2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	pSPI2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	pSPI2.SPIConfig.SPI_SSM = SPI_SSM_DI;
	pSPI2.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	pSPI2.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	pSPI2.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&pSPI2);
}

int main(void)
{
	SPI2_GPIOInit();

	SPI2_Init();

	char user_data[] = "Hello world";

	SPI_SSIConfig(SPI2, ENABLE);
	SPI_PeripherelControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)==FLAG_SET);
	SPI_PeripherelControl(SPI2, DISABLE);

	while(1);

	return 0;
}

/*void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}*/
