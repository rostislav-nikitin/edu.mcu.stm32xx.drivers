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

#define SLAVE_ADDR			0x68
#define MY_ADDRESS			SLAVE_ADDR


USART_Handle_t pUSART2Handle;
char *test_data = "Hello world through USART.";

extern void initialise_monitor_handles();

/*void initialise_monitor_handles()
{
}*/

void delay(void)
{
	for(uint32_t i=0; i < 500000 / 2; i++);
}

void USART2_GPIOInit()
{
	GPIO_Handle_t gpioUsart;
	memset(&gpioUsart, 0, sizeof(GPIO_Handle_t));

	gpioUsart.pGPIOx = GPIOA;

	gpioUsart.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpioUsart.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	gpioUsart.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioUsart.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioUsart.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//TX
	gpioUsart.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&gpioUsart);

	//TX
	gpioUsart.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&gpioUsart);
}

void USART2_Init()
{
	//memset(&pI2C1Handle, 0, sizeof(I2C_Handle_t));
	pUSART2Handle.pUSARTx = USART2;

	pUSART2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	pUSART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	pUSART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	pUSART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	pUSART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	pUSART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;


	USART_Init(&pUSART2Handle);
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

	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
}


int main(void)
{
	initialise_monitor_handles();

	printf("Started\n");

	printf("USART2_GPIOInit\n");
	USART2_GPIOInit();

	printf("Button_Init\n");
	Button_Init();

	printf("USART2_Init\n");
	USART2_Init();
	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);


	return 0;
}

static void OutputReadBuffer(uint8_t *rxBuffer, uint8_t length)
{
	rxBuffer[length] = '\0';
	printf("Length is %i\n", length);
	printf("%s\n", (char*)rxBuffer);
}


void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	printf("USART peripheral ENABLED.\n");
	USART_PeripheralControl(pUSART2Handle.pUSARTx, ENABLE);
	printf("Begin TXing.\n");

	USART_SendData(&pUSART2Handle, test_data, ((uint32_t)strlen(test_data)));

	printf("Done.\n");
}

