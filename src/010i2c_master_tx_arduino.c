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

#define	NACK				0xB5
#define ACK					0xF5

#define COMMAND_LED_CTRL	0x50
#define COMMAND_SENSOR_READ	0x51
#define COMMAND_LED_READ	0x52
#define COMMAND_PRINT		0x53
#define COMMAND_ID_READ		0x54

#define	LED_OFF				0x00
#define LED_ON				0x01

//Arduino analog pins
#define ANALOG_PIN0			0
#define ANALOG_PIN2			1
#define ANALOG_PIN3			2
#define ANALOG_PIN4			3
#define ANALOG_PIN5			4

#define LED_PIN				9

#define STATE_MACHINE_STATE_BEGIN		0
#define STATE_MACHINE_STATE_LED_CTRL	1
#define STATE_MACHINE_STATE_SENSOR_READ	2
#define STATE_MACHINE_STATE_LED_READ	3


#define BUTTON_PRESSED		HIGH

#define MY_ADDRESS			0x61


I2C_Handle_t pI2C1;
uint8_t test_data[] = "Hello world through I2C";

extern void initialise_monitor_handles();

/*void initialise_monitor_handles()
{
}*/

void delay(void)
{
	for(uint32_t i=0; i < 500000 / 2; i++);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, uint8_t event)
{

}

void I2C1_GPIOInit()
{
	GPIO_Handle_t gpioI2C;
	//memset(&gpioI2C, 0, sizeof(GPIO_Handle_t));

	gpioI2C.pGPIOx = GPIOB;

	gpioI2C.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpioI2C.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	gpioI2C.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioI2C.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioI2C.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	//SCL
	gpioI2C.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&gpioI2C);

	//SDA
	gpioI2C.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&gpioI2C);
}

void I2C1_Init()
{
	//memset(&pI2C1, 0, sizeof(I2C_Handle_t));
	pI2C1.pI2Cx = I2C1;

	pI2C1.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	pI2C1.I2C_Config.I2C_SCLSpeed = MY_ADDRESS;
	pI2C1.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	pI2C1.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&pI2C1);
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

#define SLAVE_ADDR			0x68

int main(void)
{
	initialise_monitor_handles();

	printf("Started\n");

	printf("I2C_GPIOInit\n");
	I2C1_GPIOInit();

	printf("Button_Init\n");
	Button_Init();

	printf("I2C_Init\n");
	I2C1_Init();

	//I2C_PeripherelControl(pI2C1.pI2Cx, ENABLE);


	//I2C_SSOConfig(I2C, ENABLE);
	//I2C_SSIConfig(I2C, ENABLE);


	while(1);

	printf("Finished");

	return 0;
}



uint8_t I2C_VerifyResponse(uint8_t command_acknowledement)
{
	uint8_t result;

	if(command_acknowledement == ACK)
	{
		result = ACK_OK;
	}
	else
	{
		result = ACK_ERROR;
	}

	return result;
}


uint8_t state_machine_state = STATE_MACHINE_STATE_BEGIN;

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	I2C_PeripherelControl(pI2C1.pI2Cx, ENABLE);
	I2C_MasterSendData(&pI2C1, test_data, strlen((char*)test_data), SLAVE_ADDR);
	I2C_PeripherelControl(pI2C1.pI2Cx, DISABLE);

}

