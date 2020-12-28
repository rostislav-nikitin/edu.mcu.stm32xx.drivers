/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 5, 2020
 *      Author: s0lid
 */

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// Enable the clock for the I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Enable the ACKing
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/ 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3f);

	// Configure the own address (if slave)
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress & 0x7f) << I2C_OAR1_ADD7_1;
	tempreg |= (1 << I2C_OAR1_ALWAYS1);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// Configure the speed of serial clock (SCL)
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
	}
	else
	{
		// Fast mode
		// Configure mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
		}
	}
	tempreg |= (ccr_value & 0xfff);
	pI2CHandle->pI2Cx->CCR = tempreg;


	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// Fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3f);



	// Configure the rise time for I2C



}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
	uint8_t result;

	if(pI2Cx->SR1 & flagName)
	{
		result = FLAG_SET;
	}
	else
	{
		result = FLAG_RESET;
	}

	return result;
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}
	else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteTransmitAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress)
{
	slaveAddress = ((slaveAddress & 0x7f) << 1);
	slaveAddress &= ~(0x01);


	pI2Cx->DR = (slaveAddress | I2C_OP_WRITE);
}

static void I2C_ExecuteReciveAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress)
{
	slaveAddress = ((slaveAddress & 0x7f) << 1);
	slaveAddress &= ~(0x01);


	pI2Cx->DR = (slaveAddress | I2C_OP_READ);
}

void I2C_ClearACK(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
}

void I2C_SetACK(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		if(pI2CHandle->txRxState == I2C_STATE_BUSY_IN_RX)
		{
			if(pI2CHandle->rxLength == 1)
			{
				// Disable ACK
				I2C_ClearACK(pI2CHandle->pI2Cx);
				// Disable ADDR
				pI2CHandle->pI2Cx->SR1;
				pI2CHandle->pI2Cx->SR2;
			}
		}
		else
		{
			pI2CHandle->pI2Cx->SR1;
			pI2CHandle->pI2Cx->SR2;
		}
	}
	else
	{
		pI2CHandle->pI2Cx->SR1;
		pI2CHandle->pI2Cx->SR2;
	}

}

static void I2C_SendBuffer(I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint32_t length)
{
	while(length > 0)
	{
		while(! I2C_GetFlagStatus(pI2Cx, I2C_TxE_FLAG));
		pI2Cx->DR = *(pTxBuffer++);
		length--;
	}
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t repeatedStart)
{
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that generation is completed
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));
	// 3. Send address byte with last bit = 0 = Write operation
	I2C_ExecuteTransmitAddressPhase(pI2CHandle->pI2Cx, slaveAddress);
	// 4. Confirm that address phase is completed
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));
	// 5. Clear ADDR flag
	I2C_ClearADDRFlag(pI2CHandle);
	// 6. Send data until length becomes 0;
	I2C_SendBuffer(pI2CHandle->pI2Cx, pTxBuffer, length);
	// 7. When data sent wait for TxE = 1 and BTF=1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));

	if(repeatedStart == I2C_RS_OFF)
	{
		// 8. Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_MasterReciveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t repeatedStart)
{
	// 0. Set ACK = 1
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_SetACK(pI2CHandle->pI2Cx);
	}
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that START generation is completed
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));
	// 3. Send the address to the slave with r/rw bit set R(1) (total 8 bits)
	I2C_ExecuteReciveAddressPhase(pI2CHandle->pI2Cx, slaveAddress);
	// 4. Wait until address phase is completed (SR1.ADDR should be 1)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));
	// procedure to read only 1 byte from slave
	if(length == 1)
	{
		// Disable ACKing
		I2C_ClearACK(pI2CHandle->pI2Cx);
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		// Wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG));
		if(repeatedStart == I2C_RS_OFF)
		{
			// generate STOP condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		// read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;


	}
	else if(length > 1)
	{
		// clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		// read data until length becomes zero
		for(int32_t index = length; index > 0; index--)
		{
			// wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG));

			if(index == 2)
			{
				// clear the ACK bit
				I2C_ClearACK(pI2CHandle->pI2Cx);

				if(repeatedStart == I2C_RS_OFF)
				{
					// generate STOP condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			// read data from the register to buffer & increment the buffer address
			*(pRxBuffer++) = pI2CHandle->pI2Cx->DR;
		}
	}

	// re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_SetACK(pI2CHandle->pI2Cx);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}
uint8_t I2C_SlaveReciveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}


uint8_t I2C_MasterSendDataAsync(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t repeatedStart)
{
//	uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)

	uint8_t busy_state = pI2CHandle->txRxState;

	if( (busy_state != I2C_STATE_BUSY_IN_TX) && (busy_state != I2C_STATE_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->txLength = length;
		pI2CHandle->txRxState = I2C_STATE_BUSY_IN_TX;
		pI2CHandle->devAddr = slaveAddress;
		pI2CHandle->repeatedStart = repeatedStart;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busy_state;

}
uint8_t I2C_MasterReciveDataAsync(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t repeatedStart)
{
	uint8_t busy_state = pI2CHandle->txRxState;

	if((busy_state != I2C_STATE_BUSY_IN_TX) && (busy_state != I2C_STATE_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->rxLength = length;
		pI2CHandle->txRxState = I2C_STATE_BUSY_IN_RX;
		pI2CHandle->rxLength = length; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->rxSize = 0;
		pI2CHandle->devAddr = slaveAddress;
		pI2CHandle->repeatedStart = repeatedStart;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busy_state;
}

void I2C_PeripherelControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}



void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t EnOrDi)
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

void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	uint8_t iprx = irqNumber / 4;
	uint8_t iptx_section = irqNumber % 4;
	uint8_t shift_amount = (iptx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR + iprx) |= (irqPriority << shift_amount);
}

void I2C_CloseTx(I2C_Handle_t *pI2CHandle)
{
	// Disable interrupts
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->txRxState = I2C_STATE_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->txLength = 0;

	//I2C_SetACK(pI2CHandle->pI2Cx);
}

void I2C_CloseRx(I2C_Handle_t *pI2CHandle)
{
	// Disable interrupts
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->txRxState = I2C_STATE_READY;
	//pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->rxLength = 0;
	pI2CHandle->rxNeedFinalize = FALSE;
	//pI2CHandle->rxSize = 0;

	I2C_SetACK(pI2CHandle->pI2Cx);
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE))
	{
		// BTF=1, TXE=1
		if(pI2CHandle->txLength == 0)
		{
			if(pI2CHandle->repeatedStart == I2C_RS_OFF)
			{
				// Generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			I2C_CloseTx(pI2CHandle);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_COMPLETE);
		}
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->rxLength == 1)
	{
		*(pI2CHandle->pRxBuffer++) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->rxLength--;
		pI2CHandle->rxSize++;
	}
	else if(pI2CHandle->rxLength > 1)
	{
		if(pI2CHandle->rxLength == 2)
		{
			I2C_ClearACK(pI2CHandle->pI2Cx);
			if(pI2CHandle->repeatedStart == I2C_RS_OFF)
			{
				// generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				pI2CHandle->rxNeedFinalize = TRUE;
			}
		}
		*(pI2CHandle->pRxBuffer++) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->rxLength--;
		pI2CHandle->rxSize++;
	}

	if(pI2CHandle->rxLength == 0)
	{
		I2C_CloseRx(pI2CHandle);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_COMPLETE);
	}
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t evIrqEnabled = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	uint32_t bufIrqEnabled = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	uint32_t sbSet = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if(evIrqEnabled && sbSet)
	{
		if(pI2CHandle->txRxState == I2C_STATE_BUSY_IN_TX)
		{
			I2C_ExecuteTransmitAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->devAddr);
		}
		else if(pI2CHandle->txRxState == I2C_STATE_BUSY_IN_RX)
		{
			I2C_ExecuteReciveAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->devAddr);
		}
		printf("evIrqEnabled && sbSet\n");
	}


	uint32_t addrSet = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(evIrqEnabled &&  addrSet)
	{
		I2C_ClearADDRFlag(pI2CHandle);
		printf("evIrqEnabled &&  addrSet\n");
	}

	uint32_t btfSet = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(evIrqEnabled && btfSet)
	{
		// Make sure that TXE is also set.
		if(pI2CHandle->txRxState == I2C_STATE_BUSY_IN_TX)
		{
			printf("evIrqEnabled &&  btfSet\n");
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
		else if(pI2CHandle->txRxState == I2C_STATE_BUSY_IN_RX)
		{
			//
		}

	}

	uint32_t stopfSet = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(evIrqEnabled &&  stopfSet)
	{
		// Clear procedure is: Read SR1 (already done 2 lines above), Write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOP);
		printf("evIrqEnabled &&  stopfSet\n");
	}

	uint32_t deviceMaster = pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL);

	uint32_t txeSet = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);

	if(evIrqEnabled && bufIrqEnabled && txeSet)
	{
		if(deviceMaster)
		{
			if(pI2CHandle->txRxState == I2C_STATE_BUSY_IN_TX)
			{
				if(pI2CHandle->txLength > 0)
				{
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer++);
					pI2CHandle->txLength--;
				}
			}
			printf("evIrqEnabled && bufIrqEnabled && txeSet\n");
		}
		else
		{
			// make sure that device really in a transmitter mode (not receiver)
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_SLAVE_DATA_TRANSMIT);
			}
		}

	}

	uint32_t rxneSet = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	if(evIrqEnabled && bufIrqEnabled && rxneSet)
	{


		if(deviceMaster || (pI2CHandle->rxNeedFinalize == TRUE))
		{
			if(pI2CHandle->txRxState == I2C_STATE_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
			printf("evIrqEnabled && bufIrqEnabled && rxneSet\n");
		}
		else
		{
			// Check that slave in receiver mode (not transmitter)
			uint8_t flag = !(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA));
			//if((pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			//{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_SLAVE_DATA_RECEIVE);
			//}
		}
	}
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}
/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

		//Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, uint8_t event)
{

}
