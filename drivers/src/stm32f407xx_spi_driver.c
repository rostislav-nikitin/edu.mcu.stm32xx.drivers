/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 1, 2020
 *      Author: s0lid
 */

#include "stm32f407xx.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
		else if(pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
		else if(pSPIx == SPI6)
		{
			SPI6_PCLK_DI();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Configure SPI_CR1 register

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);


	uint32_t tempreg = 0;
	// 1.Configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_BIT;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode disable
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE_BIT);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode enable
		tempreg |= (1 << SPI_CR1_BIDI_MODE_BIT);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode disable
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE_BIT);
		// RXONLY mode enable
		tempreg |= ~(1 << SPI_CR1_RX_ONLY_BIT);
	}

	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR0_BIT);
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF_BIT);
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_BIT);
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_BIT);
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_BIT);

	pSPIHandle->pSPIx->CR[0] = tempreg;

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

void SPI_PeripherelControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		pSPIx->CR[0] |= (1 << SPI_CR1_SPE_BIT);
	}
	else
	{
		pSPIx->CR[0] &= ~(1 << SPI_CR1_SPE_BIT);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		pSPIx->CR[0] |= (1 << SPI_CR1_SSI_BIT);
	}
	else
	{
		pSPIx->CR[0] &= ~(1 << SPI_CR1_SSI_BIT);
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	uint8_t result;

	if(pSPIx->SR & flagName)
	{
		result = FLAG_SET;
	}
	else
	{
		result = FLAG_RESET;
	}

	return result;
}

void SPI_ClearOvrFlag(SPI_RegDef_t *pSPIx)
{
	pSPIx->DR;
	pSPIx->SR;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR[1] &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->txState = SPI_READY;
	pSPIHandle->txLen = 0;
	pSPIHandle->pTxBuffer = NULL;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR[1] &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->rxState = SPI_READY;
	pSPIHandle->rxLen = 0;
	pSPIHandle->pRxBuffer = NULL;
}

//while(!(pSPIx->SR & (1 << SPI_SR_TXE_BIT)));


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length)
{
	while(length > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		uint16_t data = 0;

		if((pSPIx->CR[0] & (1 << SPI_CR1_DFF_BIT)))
		{
			// 16BIT mode
			data = *((uint16_t*)pTxBuffer++);
			length--;
			length--;
			//(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8BIT mode
			data = *pTxBuffer++;
			length--;
		}


		pSPIx->DR = data;
	}
}
void SPI_ReciveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{
	while(length > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		if(pSPIx->CR[0] & (1 << SPI_CR1_DFF_BIT))
		{
			//16BIT
			*((uint16_t *)pRxBuffer++) = pSPIx->DR;
			length--;
			length--;
		}
		else
		{
			//8BIT
			(*pRxBuffer++) = (uint8_t)pSPIx->DR;
			length--;
		}
	}
}

uint8_t SPI_SendDataAsync(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length)
{
	if(pSPIHandle->txState != SPI_BUSY_IN_TX)
	{
		// 1.Save TX buffer address & length
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = length;
		// 2.Set busy state, to no code take over same SPI peripheral until transmission is over.
		//pSPIHandle->pSPIx->SR = (1 << SPI_SR_BSY_BIT);

		pSPIHandle->txState = SPI_BUSY_IN_TX;
		// 3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR.
		pSPIHandle->pSPIx->CR[2] |= (1 << SPI_CR2_TXEIE);
		// 4. Data transmission will be handled by the ISR code (will implement later).
	}

	return pSPIHandle->txState;

}
uint8_t SPI_ReciveDataAsync(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length)
{
	if(pSPIHandle->rxState != SPI_BUSY_IN_RX)
	{
		// 1.Save RX buffer address & length
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = length;
		// 2.Set busy state, to no code take over same SPI peripheral until transmission is over.
		//pSPIHandle->pSPIx->SR = (1 << SPI_SR_BSY_BIT);

		pSPIHandle->rxState = SPI_BUSY_IN_RX;
		// 3. Enable RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR.
		pSPIHandle->pSPIx->CR[2] |= (1 << SPI_CR2_RXNEIE);
		// 4. Data transmission will be handled by the ISR code (will implement later).
	}

	return pSPIHandle->rxState;
}

void SPI_SSOConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		pSPIx->CR[1] |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR[1] &= ~(1 << SPI_CR2_SSOE);
	}
}


void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(irqNumber < 32)
		{
			// Use ISER0
			*NVIC_ISER0 |= (1 << irqNumber);

		}
		else if(irqNumber > 31 && irqNumber < 64)
		{
			// Use ISER1
			*NVIC_ISER1 |= (1 << (irqNumber % 32));
		}
		else if(irqNumber > 63 && irqNumber < 96)
		{
			// Use ISER2
			*NVIC_ISER2 |= (1 << (irqNumber % 32));
		}
	}
	else
	{
		if(irqNumber < 32)
		{
			// Use ICER0
			*NVIC_ICER0 |= (1 << irqNumber);
		}
		else if(irqNumber > 31 && irqNumber < 64)
		{
			// Use ICER1
			*NVIC_ICER1 |= (1 << (irqNumber % 32));
		}
		else if(irqNumber > 63 && irqNumber < 96)
		{
			// Use ICER2
			*NVIC_ICER2 |= (1 << (irqNumber % 32));
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	uint8_t iprx = irqNumber / 4;
	uint8_t iptx_section = irqNumber % 4;
	uint8_t shift_amount = (iptx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR + iprx) |= (irqPriority << shift_amount);


}

static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pHandle)
{
	if(pHandle->txLen != 0)
	{
		uint16_t data = 0;

		if((pHandle->pSPIx->CR[0] & (1 << SPI_CR1_DFF_BIT)))
		{
			// 16BIT mode
			data = *((uint16_t*)pHandle->pTxBuffer++);
			pHandle->txLen--;
			pHandle->txLen--;
		}
		else
		{
			// 8BIT mode
			data = *(pHandle->pTxBuffer++);
			pHandle->txLen--;
		}


		pHandle->pSPIx->DR = data;
	}
	else
	{
		SPI_CloseReception(pHandle);

		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_COMPLETE);
	}
}

static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pHandle)
{
	if(pHandle->rxLen > 0)
	{
		if(pHandle->pSPIx->CR[0] & (1 << SPI_CR1_DFF_BIT))
		{
			*((uint16_t *)pHandle->pRxBuffer++) = pHandle->pSPIx->DR;
			pHandle->rxLen--;
			pHandle->rxLen--;
		}
		else
		{
			//8BIT
			*(pHandle->pRxBuffer++) = (uint8_t)pHandle->pSPIx->DR;
			pHandle->rxLen--;
		}
	}
	else
	{
		SPI_CloseTransmission(pHandle);

		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_COMPLETE);
	}
}
static void SPI_OVRE_Interrupt_Handle(SPI_Handle_t *pHandle)
{
	if(pHandle->rxState == SPI_BUSY_IN_RX)
	{
		if(pHandle->pSPIx->CR[0] & (1 << SPI_CR1_DFF_BIT))
		{
			*((uint16_t *)pHandle->pRxBuffer++) = pHandle->pSPIx->DR;
			pHandle->rxLen--;
			pHandle->rxLen--;
		}
		else
		{
			//8BIT
			*(pHandle->pRxBuffer++) = (uint8_t)pHandle->pSPIx->DR;
			pHandle->rxLen--;
		}

		pHandle->pSPIx->SR;

		pHandle->rxState = SPI_READY;
	}

	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERROR);
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	if(SPI_GetFlagStatus(pHandle->pSPIx, SPI_TXE_FLAG)
			&& (pHandle->pSPIx->CR[1] & SPI_CR2_TXEIE))
	{
		pHandle->txState = SPI_BUSY_IN_TX;
		SPI_TXE_Interrupt_Handle(pHandle);
	}
	else if(SPI_GetFlagStatus(pHandle->pSPIx, SPI_RXNE_FLAG)
			&& (pHandle->pSPIx->CR[1] & SPI_CR2_RXNEIE))
	{
		pHandle->txState = SPI_BUSY_IN_RX;
		SPI_RXNE_Interrupt_Handle(pHandle);
	}
	else if(SPI_GetFlagStatus(pHandle->pSPIx, SPI_OVR_FLAG)
			&& (pHandle->pSPIx->CR[1] & SPI_CR2_ERRIE))
	{
		// Error
		SPI_OVRE_Interrupt_Handle(pHandle);
	}
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t event)
{

}
