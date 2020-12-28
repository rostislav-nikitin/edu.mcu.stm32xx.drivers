/*
 * stm32f407xx.h
 *
 *  Created on: Aug 26, 2020
 *      Author: s0lid
 */

#include <stddef.h>
#include <stdint.h>

#define __weak __attribute__((weak))
#define __vo volatile




#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * CPU Specific details
 */
#define NVIC_ISER0 ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0 ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1 ((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2 ((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3 ((__vo uint32_t*)0XE000E18C)

#define NVIC_IPR	((__vo uint32_t*)0xE000E400)

/*
 * MCU Specific details
 */

#define NO_PR_BITS_IMPLEMENTED	4

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM1_SIZE				0x0001c000U
#define SRAM2_BASEADDR			(SRAM1_BASEADDR + SRAM1_SIZE)
#define SRAM__BASEADDR			SRAM1_BASEADDR
#define ROM_BASEADDR			0x1fff0000U

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

// AHB1
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

// APB1
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

// ABP2
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR			(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR			(APB2PERIPH_BASEADDR + 0x5400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)
#define USART7_BASEADDR			(APB2PERIPH_BASEADDR + 0x7800)
#define USART9_BASEADDR			(APB2PERIPH_BASEADDR + 0x7C00)

// GPIO

typedef struct
{
	__vo uint32_t MODER;	/*!<mode register  >*/
	__vo uint32_t OTYPER;	/*!<port output type register>*/
	__vo uint32_t OSPEEDR;	/*!<port output speed register>*/
	__vo uint32_t PUPDR;	/*!<port pull-up/pull-down register>*/
	__vo uint32_t IDR;		/*!<port input data register>*/
	__vo uint32_t ODR;		/*!<port output data register>*/
	__vo uint32_t BSRR;		/*!<port bit set/reset register>*/
	__vo uint32_t LCKR;		/*!<port configuration lock register>*/
	__vo uint32_t AFR[2];	/*!<alternate function low & high registers>*/
} GPIO_RegDef_t;

//RCC

typedef struct
{
	__vo uint32_t CR;			/*!<clock control register>*/
	__vo uint32_t PLLCFGR;		/*!<PLL configuration register>*/
	__vo uint32_t CFGR;			/*!<clock configuration register>*/
	__vo uint32_t CIR;			/*!<clock interrupt register>*/
	__vo uint32_t AHB1RSTR;		/*!<AHB1 peripheral reset register >*/
	__vo uint32_t AHB2RSTR;		/*!<AHB2 peripheral reset register>*/
	__vo uint32_t AHB3RSTR;		/*!<AHB3 peripheral reset register>*/
	uint32_t 	Reserved0;
	__vo uint32_t APB1RSTR;		/*!<APB1 peripheral reset register>*/
	__vo uint32_t APB2RSTR;		/*!<APB2 peripheral reset register>*/
	uint32_t Reserved1;
	uint32_t Reserved2;
	__vo uint32_t AHB1ENR;		/*!<AHB1 peripheral clock register>*/
	__vo uint32_t AHB2ENR;		/*!<AHB2 peripheral clock enable register>*/
	__vo uint32_t AHB3ENR;		/*!<AHB3 peripheral clock enable register>*/
	uint32_t Reserved3;
	__vo uint32_t APB1ENR;		/*!<APB1 peripheral clock enable register>*/
	__vo uint32_t APB2ENR;		/*!<APB2 peripheral clock enable register>*/
	uint32_t Reserved4;
	uint32_t Reserved5;
	__vo uint32_t AHB1LPENR;	/*!<AHB1 peripheral clock enable in low power mode register>*/
	__vo uint32_t AHB2LPENR;	/*!<AHB2 peripheral clock enable in low power mode register>*/
	__vo uint32_t AHB3LPENR;	/*!<AHB3 peripheral clock enable in low power mode register>*/
	uint32_t Reserved6;
	__vo uint32_t APB1LPENR;	/*!<APB1 peripheral clock enable in low power mode register>*/
	__vo uint32_t APB2LPENR;	/*!<APB2 peripheral clock enable in low power mode register>*/
	uint32_t Reserved7;
	uint32_t Reserved8;
	__vo uint32_t BDCR;			/*!<Backup domain control register>*/
	__vo uint32_t CSR;			/*!<clock control & status register>*/
	uint32_t Reserved9;
	uint32_t Reserved10;
	__vo uint32_t SSCGR;		/*!<spread spectrum clock generation register>*/
	__vo uint32_t PLLI2SCFGR;	/*!<PLLI2S configuration register >*/
	__vo uint32_t PLLSAICFGR;	/*!<PLL configuration register>*/
	__vo uint32_t DCKCFGR;		/*!<Dedicated Clock Configuration Register>*/
} RCC_RegDef_t;

// EXTI

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

// SYSCFG
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;

} SYSCFG_RegDef_t;

// SPI
typedef struct
{
	__vo uint32_t CR[2];
	__vo uint32_t SR;
	__vo uint16_t DR;
	uint16_t Reserved;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

} I2C_RegDef_t;

#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15


#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12


#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD7_1			1
#define I2C_OAR1_ADD9_8			8
#define I2C_OAR1_ALWAYS1		14
#define I2C_OAR1_ADDMODE		15


#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RxNE			6
#define I2C_SR1_TxE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8


#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15


// FLAGS
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_BIT)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_BIT)

//
#define I2C_SB_FLAG				(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG			(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG			(1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG			(1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG			(1 << I2C_SR1_STOPF)
#define I2C_RxNE_FLAG			(1 << I2C_SR1_RxNE)
#define I2C_TxE_FLAG			(1 << I2C_SR1_TxE)
#define I2C_BERR_FLAG			(1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG			(1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG				(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG			(1 << I2C_SR1_OV)
#define I2C_PECERR_FLAG			(1 << I2C_SR1_SB)
#define I2C_TIMEOUT_FLAG		(1 << I2C_SR1_SB)
#define I2C_SMBALERT_FLAG		(1 << I2C_SR1_SB)


/*
#define SPI_CR1_BIDIMODE_UNI		0;
#define SPI_CR1_BIDIMODE_BI			#define1;

#define SPI_CR1_BIDIOE_EN			ENABLED;
#define SPI_CR1_BIDIOE_DI			DISABLED;

#define SPI_CR1_CRCEN_DI			DISABLED
#define SPI_CR1_CRCEN_EN			ENABLED

#define SPI_CR1_CRCNEXT#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x0E))_DATA_PH		0
#define SPI_CR1_CRCNEXT_NEXT_TR		1
pHandle->pRxBuffer = NULL;
#define SPI_CR1_DFF_8BIT			0
#define SPI_CR1_DFF_16BIT			1

#define SPI_CR1_RXONLY_FULL_DUPLEX	0
#define SPI_CR1_RXONLY_OUTPUT_DI	1

#define SPI_CR1_SSM_DI				DISABLED
#define SPI_CR1_SSM_EN				ENABLED

#define SPI_CR1_SSI_DI				DISABLED
#define SPI_CR1_SSI_EN				ENABLED

for
#define SPI_CR1_LSBFIRST_DI		// FLAGS
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_BIT)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_BIT)
	DISABLED
#define SPI_CR1_LSBFIRST_EN			ENABLED

#define SPI_CR1_SPE_DI				DISABLED
#define SPI_CR1_SPE_EN				ENABLED

#define SPI_CR1_BR_2				0x01
#define SPI_CR1_BR_4				0x02
#define SPI_CR1_BR_8				0x03
#define SPI_CR1_BR_16				0x04
#define SPI_CR1_BR_32				0x05
#define SPI_CR1_BR_64				0x06
#define SPI_CR1_BR_128				0x07
#define SPI_CR1_BR_256				0x08

#define SPI_CR1_MSTR_DI				DISABLED
#define SPI_CR1_MSTR_EN				ENABLED

#define SPI_CR1_CPOL_ZERO			0
#define SPI_CR1_CPOL_ONE			1
// FLAGS
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_BIT)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_BIT)

#define SPI_CR1_CPHA_FIRST			0
#define SPI_CR1_CPHA_SECOND			1
*/

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
} USART_RegDef_t;


#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9

#define USART_BRR_DIV_FRACTION	0
#define USART_BRR_DIV_MANTISSA	4


#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_RESERVED		14
#define USART_CR1_OVER8			15


#define USART_CR2_ADD			0
#define USART_CR2_RESERVED1		4
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_RESERVED2		7
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14

#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11

#define USART_GTPR_PSC			0
#define USART_GTPR_GT			8

// Peripheral

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ				((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK				((GPIO_RegDef_t*)GPIOK_BASEADDR)


#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5				((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6				((SPI_RegDef_t*)SPI6_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define USART1				((USART_RegDef_t*)USART1_BASEADDR)
#define USART2				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3				((USART_RegDef_t*)USART3_BASEADDR)
#define UART4				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6				((USART_RegDef_t*)USART6_BASEADDR)

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 0x04))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 0x11))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 0x12))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x13))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x14))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 0x05))

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0x04))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 0x11))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 0x12))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x13))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x14))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0x05))

// Enable clock
// FLAGS
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_BIT)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_BIT)


#define USART_FLAG_TXE					(1 << USART_SR_TXE)
#define USART_FLAG_RXNE					(1 << USART_SR_RXNE)
#define USART_FLAG_TC					(1 << USART_SR_TC)


#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x00))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x01))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x02))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x03))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x04))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x05))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x06))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x07))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x08))
#define GPIOJ_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x09))
#define GPIOK_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0x0A))



#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x0E))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x0F))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 0x11))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 0x12))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x13))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x14))
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x15))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x16))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x17))
#define UART7_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x1E))
#define UART8_PCLK_EN()		(RCC->APB1ENR |= (1 << 0x1F))
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_BIT)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_BIT)



#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 0x04))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 0x05))
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 0x0C))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 0x0D))
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 0x0E))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 0x14))
#define SPI6_PCLK_EN()		(RCC->APB2ENR |= (1 << 0x15))


// Disable clock

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x00))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x01))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x02))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x03))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x04))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x05))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x06))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x07))
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_BIT)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_BIT)

#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x08))
#define GPIOJ_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x09))
#define GPIOK_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0x0A))


#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x0E))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x0F))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 0x11))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 0x12))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x13))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x14))
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x15))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x16))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x17))
#define UART7_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x1E))
#define UART8_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0x1F))


#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0x04))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0x05))
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 0x0C))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 0x0D))
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0x0E))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 0x14))
#define SPI6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 0x15))

// Reset
// FLAGS
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_BIT)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_BIT)

#define GPIOA_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x00)); (RCC->AHB1RSTR &= ~(1 << 0x00));}
#define GPIOB_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x01)); (RCC->AHB1RSTR &= ~(1 << 0x01));}
#define GPIOC_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x02)); (RCC->AHB1RSTR &= ~(1 << 0x02));}
#define GPIOD_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x03)); (RCC->AHB1RSTR &= ~(1 << 0x03));}
#define GPIOE_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x04)); (RCC->AHB1RSTR &= ~(1 << 0x04));}
#define GPIOF_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x05)); (RCC->AHB1RSTR &= ~(1 << 0x05));}
#define GPIOG_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x06)); (RCC->AHB1RSTR &= ~(1 << 0x06));}
#define GPIOH_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x07)); (RCC->AHB1RSTR &= ~(1 << 0x07));}
#define GPIOI_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x08)); (RCC->AHB1RSTR &= ~(1 << 0x08));}
#define GPIOJ_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x09)); (RCC->AHB1RSTR &= ~(1 << 0x09));}
#define GPIOK_REG_RESET()	{(RCC->AHB1RSTR |= (1 << 0x0A)); (RCC->AHB1RSTR &= ~(1 << 0x0A));}

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0:\
									(x == GPIOB) ? 1:\
									(x == GPIOC) ? 2:\
									(x == GPIOD) ? 3:\
									(x == GPIOE) ? 4:\
									(x == GPIOF) ? 5:\
									(x == GPIOG) ? 6:\
									(x == GPIOH) ? 7:\
									(x == GPIOI) ? 8:\
									(x == GPIOJ) ? 9:\
									(x == GPIOK) ? 10:-1)

// IRQ NO

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85
#define IRQ_NO_SPI6			86

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73


#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_USART4		52
#define IRQ_NO_USART5		53
#define IRQ_NO_USART6		71



// IRQ PRIORITY

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI15		15

// Generic
#define FALSE	0
#define TRUE	1
#define DISABLE			0
#define ENABLE			1
#define SET 			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define HIGH			1
#define LOW				0
#define FLAG_RESET		RESET
#define FLAG_SET		SET
#define ACK_ERROR		0
#define ACK_OK			1

#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"

#endif /* INC_STM32F407XX_H_ */
