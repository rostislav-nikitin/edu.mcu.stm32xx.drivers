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

extern void initialise_monitor_handles();

/*void initialise_monitor_handles()
{
}*/

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
	initialise_monitor_handles();

	printf("Started\n");

	printf("SPI2_GPIOInit\n");
	SPI2_GPIOInit();

	printf("Button_Init\n");
	Button_Init();

	printf("SPI2_Init\n");
	SPI2_Init();


	SPI_SSOConfig(SPI2, ENABLE);
	SPI_SSIConfig(SPI2, ENABLE);


	while(1);

	printf("Finished");

	return 0;
}



uint8_t SPI_VerifyResponse(uint8_t command_acknowledement)
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
	uint8_t dummy_byte_to_write = 0xff;
	uint8_t dummy_byte_to_read = 0xff;
	uint8_t	command;
	uint8_t command_acknowledement;
	//char user_data[] = "Hello world";
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	SPI_PeripherelControl(SPI2, ENABLE);

	switch(state_machine_state)
	{
		case STATE_MACHINE_STATE_BEGIN:
			command = COMMAND_LED_CTRL;

			SPI_SendData(SPI2, &command, sizeof(command));
			// Dummy read to clear RXNE flag
			SPI_ReciveData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));

			// Send dummy byte to fetch ack/nack from the slave
			SPI_SendData(SPI2, &dummy_byte_to_write, sizeof(dummy_byte_to_write));
			SPI_ReciveData(SPI2, &command_acknowledement, sizeof(command_acknowledement));

			if(SPI_VerifyResponse(command_acknowledement) == ACK_OK)
			{
				// send led on
				uint8_t parameters[2] = { LED_PIN, LED_ON };
				SPI_SendData(SPI2, parameters, sizeof(parameters));
			}

			break;
		case STATE_MACHINE_STATE_LED_CTRL:
			command = COMMAND_SENSOR_READ;

			SPI_SendData(SPI2, &command, sizeof(command));
			// Dummy read to clear RXNE flag
			SPI_ReciveData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));

			// Send dummy byte to fetch ack/nack from the slave
			SPI_SendData(SPI2, &dummy_byte_to_write, sizeof(dummy_byte_to_write));
			SPI_ReciveData(SPI2, &command_acknowledement, sizeof(command_acknowledement));

			if(SPI_VerifyResponse(command_acknowledement) == ACK_OK)
			{
				// Send parameters
				uint8_t parameters[1] = { ANALOG_PIN0 };
				SPI_SendData(SPI2, parameters, sizeof(parameters));
				SPI_ReciveData(SPI2, &dummy_byte_to_read, sizeof(parameters));

				// delay for ADC
				delay();

				// Recive data from sensor
				uint8_t sendor_data;
				SPI_SendData(SPI2, &dummy_byte_to_write, sizeof(dummy_byte_to_write));
				SPI_ReciveData(SPI2, &sendor_data, sizeof(sendor_data));
			}

			break;
		case COMMAND_SENSOR_READ:
			command = COMMAND_LED_READ;

			SPI_SendData(SPI2, &command, sizeof(command));
			// Dummy read to clear RXNE flag
			SPI_ReciveData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));

			// Send dummy byte to fetch ack/nack from the slave
			SPI_SendData(SPI2, &dummy_byte_to_write, sizeof(dummy_byte_to_write));
			SPI_ReciveData(SPI2, &command_acknowledement, sizeof(command_acknowledement));

			if(SPI_VerifyResponse(command_acknowledement) == ACK_OK)
			{
				uint8_t led_state;
				SPI_SendData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));
				SPI_ReciveData(SPI2, &led_state, sizeof(led_state));
			}

			break;
		case COMMAND_LED_READ:
			command = COMMAND_PRINT;

			SPI_SendData(SPI2, &command, sizeof(command));
			// Dummy read to clear RXNE flag
			SPI_ReciveData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));

			// Send dummy byte to fetch ack/nack from the slave
			SPI_SendData(SPI2, &dummy_byte_to_write, sizeof(dummy_byte_to_write));
			SPI_ReciveData(SPI2, &command_acknowledement, sizeof(command_acknowledement));

			if(SPI_VerifyResponse(command_acknowledement) == ACK_OK)
			{
				char string_to_print[] = "Some string";
				uint8_t sz_string_to_print = strlen(string_to_print);

				// send string length
				SPI_SendData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));
				SPI_ReciveData(SPI2, &sz_string_to_print, sizeof(sz_string_to_print));
				// send string
				SPI_SendData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));
				SPI_ReciveData(SPI2, (uint8_t *)string_to_print, sz_string_to_print);
			}

			break;
		case COMMAND_PRINT:
			command = COMMAND_ID_READ;

			SPI_SendData(SPI2, &command, sizeof(command));
			// Dummy read to clear RXNE flag
			SPI_ReciveData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));

			// Send dummy byte to fetch ack/nack from the slave
			SPI_SendData(SPI2, &dummy_byte_to_write, sizeof(dummy_byte_to_write));
			SPI_ReciveData(SPI2, &command_acknowledement, sizeof(command_acknowledement));

			if(SPI_VerifyResponse(command_acknowledement) == ACK_OK)
			{
				uint8_t sz_id = 10;
				uint8_t id[sz_id];

				for(uint8_t i = 0; i < sz_id; i++)
				{
					SPI_SendData(SPI2, &dummy_byte_to_read, sizeof(dummy_byte_to_read));
					SPI_ReciveData(SPI2, &id[i], sizeof(id[i]));
				}
				id[10] = '\0';
				printf("COMMAND_ID: %s.", id);
			}

			break;

	}

	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) == FLAG_SET);
	SPI_PeripherelControl(SPI2, DISABLE);

	//SPI_PeripherelControl(SPI2, ENABLE);
	//GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	//uint8_t sz_user_data_length = strlen(user_data);

	//SPI_SendData(SPI2, &sz_user_data_length, sizeof(sz_user_data_length));
	//SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) == FLAG_SET);

	SPI_PeripherelControl(SPI2, DISABLE);
}
