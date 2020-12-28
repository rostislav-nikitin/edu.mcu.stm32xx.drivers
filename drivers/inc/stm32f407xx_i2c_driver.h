/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Sep 5, 2020
 *      Author: s0lid
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
} I2C_Config_t;



#define I2C_SCL_SPEED_SM	100000 // Normal Mode 100 KHz
#define I2C_SCL_SPEED_FM2K	200000 // Fast Mode 200 KHz
#define I2C_SCL_SPEED_FM4K	400000 // Fast Mode 400 KHz

#define I2C_ACK_DISABLE		0
#define I2C_ACK_ENABLE		1

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

#define I2C_OP_WRITE		0
#define I2C_OP_WRITE_BIT	0

#define I2C_OP_READ			1
#define I2C_OP_READ_BIT		0

#define I2C_RS_OFF			RESET
#define I2C_RS_ON			SET

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t txLength;
	uint32_t rxLength;
	uint8_t txRxState;
	uint8_t devAddr;
	uint32_t rxSize;
	uint8_t repeatedStart;
	uint8_t rxNeedFinalize;
} I2C_Handle_t;


#define I2C_STATE_READY			0
#define I2C_STATE_BUSY_IN_RX	1
#define I2C_STATE_BUSY_IN_TX	2


#define I2C_EVENT_TX_COMPLETE			0
#define I2C_EVENT_RX_COMPLETE			1
#define I2C_EVENT_STOP					2
#define I2C_ERROR_BERR					3
#define I2C_ERROR_ARLO					4
#define I2C_ERROR_AF					5
#define I2C_ERROR_OVR					6
#define I2C_ERROR_TIMEOUT				7

#define I2C_EVENT_SLAVE_DATA_TRANSMIT	32
#define I2C_EVENT_SLAVE_DATA_RECEIVE	33


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and recive
 */

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_ClearACK(I2C_RegDef_t *pI2Cx);
void I2C_SetACK(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t repeatedStart);
void I2C_MasterReciveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t repeatedStart);

uint8_t I2C_MasterSendDataAsync(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t repeatedStart);
uint8_t I2C_MasterReciveDataAsync(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t repeatedStart);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReciveData(I2C_RegDef_t *pI2Cx);

void I2C_CloseTx(I2C_Handle_t *pI2CHandle);
void I2C_CloseRx(I2C_Handle_t *pI2CHandle);

void I2C_PeripherelControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t enOrDi);

void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, uint8_t event);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
