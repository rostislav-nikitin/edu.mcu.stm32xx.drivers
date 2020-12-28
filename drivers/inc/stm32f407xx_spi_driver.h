/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Sep 1, 2020
 *      Author: s0lid
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct
{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		txLen;
	uint32_t		rxLen;
	uint8_t			txState;
	uint8_t			rxState;
} SPI_Handle_t;



#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define	SPI_BUSY_IN_TX				2


#define SPI_DEVICE_MODE_SLAVE			0
#define SPI_DEVICE_MODE_MASTER			1

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	4

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

// CR1 BITS
#define SPI_CR1_CPHA_BIT				0
#define SPI_CR1_CPOL_BIT				1
#define SPI_CR1_MSTR_BIT				2
#define SPI_CR1_BR0_BIT					3
#define SPI_CR1_BR1_BIT					4
#define SPI_CR1_BR2_BIT					5
#define SPI_CR1_SPE_BIT					6
#define SPI_CR1_LSB_FIRST_BIT			7
#define SPI_CR1_SSI_BIT					8
#define SPI_CR1_SSM_BIT					9
#define SPI_CR1_RX_ONLY_BIT				10
#define SPI_CR1_DFF_BIT					11
#define SPI_CR1_CRC_NEXT_BIT			12
#define SPI_CR1_CRC_EN_BIT				13
#define SPI_CR1_BIDIOE_BIT				14
#define SPI_CR1_BIDI_MODE_BIT			15

// CR2 BITs
#define SPI_CR2_RXDMAEN					0
#define SPI_CR2_TXDMAEN					1
#define SPI_CR2_SSOE					2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE					6
#define SPI_CR2_TXEIE					7

//SPI_SR
#define SPI_SR_RXNE_BIT					0
#define SPI_SR_TXE_BIT					1
#define SPI_SR_CHSIDE_BIT				2
#define SPI_SR_UDR_BIT					3
#define SPI_SR_CRCEI2C3 0x4000 5800 - 0x4000 5BFF I2C2 0x4000 5400 - 0x4000 57FF I2C1RR_BIT				4
#define SPI_SR_MODF_BIT					5
#define SPI_SR_OVR_BIT					6
#define SPI_SR_BSY_BIT					7
#define SPI_SR_FRE_BIT					8

// SPI_I2SCFGR
#define SPI_I2SCFGR_CHLEN				0
#define SPI_I2SCFGR_DATLEN				1
#define SPI_I2SCFGR_CKPOL				3
#define SPI_I2SCFGR_I2SSTD				4
#define SPI_I2SCFGR_PCMSYNC				7
#define SPI_I2SCFGR_I2SCFG				8
#define SPI_I2SCFGR_I2SE				10
#define SPI_I2SCFGR_I2SMOD				11

// SPI_I2SPR
#define SPI_I2SPR_I2SDIV				0
#define SPI_I2SPR_ODD					8
#define SPI_I2SPR_MCKOE					9


// FLAGS
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_BIT)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_BIT)
#define SPI_UDR_FLAG					(1 << SPI_SR_UDR_BIT)
#define SPI_CRCERR_FLAG					(1 << SPI_SR_CRCERR_BIT)
#define SPI_MODF_FLAG					(1 << SPI_SR_MODF_BIT)
#define SPI_OVR_FLAG					(1 << SPI_SR_OVR_BIT)
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY_BIT)
#define SPI_FRE_FLAG					(1 << SPI_SR_FRE_BIT)

// EVENTS

#define SPI_EVENT_TX_COMPLETE			1
#define SPI_EVENT_RX_COMPLETE			2
#define SPI_EVENT_OVR_ERROR				3
#define SPI_EVENT_CRC_ERROR				4





void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_PeripherelControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi);
void SPI_SSOConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);
void SPI_ClearOvrFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReciveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);

uint8_t SPI_SendDataAsync(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_ReciveDataAsync(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);

void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t event);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
