/*
 * stm32f446xx.h
 *
 *  Created on: 10-Dec-2025
 *      Author: HP
 */

/*******************************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-

*/

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#define __vo  volatile

/***************************Processor Specific Details****************************************************************/
/*********Arm Cortex Mx NVIC ISERx Register Boundary Addresses********************************************************/

#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)		//32-bit register from 0-31
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)		//32-bit register from 32-63
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)		//32-bit register from 64-95
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)		//32-bit register from 96-127

/*********Arm Cortex Mx NVIC ICERx Register Boundary Addresses********************************************************/
#define NVIC_ICER0			((__vo uint32_t*)0XE000E180)		//32-bit register from 0-31
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)		//32-bit register from 32-63
#define NVIC_ICER2			((__vo uint32_t*)0XE000E188)		//32-bit register from 64-95
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)		//32-bit register from 96-127

/*********Arm Cortex Mx NVIC IPER Base Address********************************************************/
#define NVIC_IPR_BASE_ADDR	((__vo uint32_t*)0xE000E400)

#define NO_OF_BITS_IMPLEMENTED 	4 //These lower bits are not useful in IPR register sections when settings Priority Configuration

/* Base Address of FLASH and SRAM memory */

#define FLASH_BASEADDR			0x08000000U          /*Base address of flash memory from ref manual */
#define SRAM1_BASEADDR			0x20000000U		  /*Base address of SRAM1 (112KB) memory from ref manual */
#define SRAM2_BASEADDR			0x2001C000U		  /*Base address of SRAM2 (16KB) memory from ref manual */
#define ROM_BASEADDR			0x1FFF0000U		  /*Base address of ROM/System memory from ref manual */
#define SRAM					SRAM1_BASEADDR

/* Base addresses of AHBx and APBx */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define AHB3PERIPH_BASEADDR		0x60000000U

/* Base addresses of peripherals associated with AHBx and APBx */

#define RCC_BASEADDR  			(AHB1PERIPH_BASEADDR + 0x3800)

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

/*****************************GPIO PERIPHERAL REGISTER STRUCTURE**************************************/

typedef struct
{
	__vo uint32_t MODER;         /* GPIO port mode register : Address Offset 0x00 */
	__vo uint32_t OTYPER;        /* GPIO port output type register : Address Offset 0x04 */
	__vo uint32_t OSPEEDER;      /* GPIO port output speed register : Address Offset 0x08 */
	__vo uint32_t PUPDR;         /* GPIO port pull-up/pull-down register : Address Offset 0x0C */
	__vo uint32_t IDR;           /* GPIO port input data register : Address Offset 0x10 */
	__vo uint32_t ODR;           /* GPIO port output data register : Address Offset 0x14 */
	__vo uint32_t BSRR;          /* GPIO port bit set/reset register : Address Offset 0x18 */
	__vo uint32_t LCKR;          /* GPIO port configuration lock register : Address Offset 0x1C */
	__vo uint32_t AFR[2];        /* AFR[0] = AFRL : GPIO alternate function low register : Address Offset 0x20, AFR[1] = AFRH : GPIO alternate function high register : Address Offset 0x24 */
}GPIO_RegDef_t;

/*****************************RCC REGISTER STRUCTURE**************************************/
typedef struct
{
	__vo uint32_t CR;  			/* RCC clock control register : Address Offset 0x00 */
	__vo uint32_t PLLCFGR;  	/* RCC PLL configuration register : Address Offset 0x04 */
	__vo uint32_t CFGR;  		/* RCC clock configuration register : Address Offset 0x08 */
	__vo uint32_t CIR;  	    /* RCC clock interrupt register : Address Offset 0x0C */
	__vo uint32_t AHB1RSTR;  	/* RCC AHB1 peripheral reset register : Address Offset 0x10 */
	__vo uint32_t AHB2RSTR;  	/* RCC AHB2 peripheral reset register : Address Offset 0x14 */
	__vo uint32_t AHB3RSTR;  	/* RCC AHB3 peripheral reset register : Address Offset 0x18 */
	__vo uint32_t RESERVED0;	/* Reserved : Address Offset 0x1C */
	__vo uint32_t APB1RSTR;  	/* RCC APB1 peripheral reset register : Address Offset 0x20 */
	__vo uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register : Address Offset 0x24 */
	__vo uint32_t RESERVED1[2]; /* Reserved : Address Offset 0x28 - 0x2C */
	__vo uint32_t AHB1ENR;  	/* RCC AHB1 peripheral clock enable register : Address Offset 0x30 */
	__vo uint32_t AHB2ENR;  	/* RCC AHB2 peripheral clock enable register : Address Offset 0x34 */
	__vo uint32_t AHB3ENR;  	/* RCC AHB3 peripheral clock enable register : Address Offset 0x38 */
	__vo uint32_t RESERVED2;  	/* Reserved : Address Offset 0x3C */
	__vo uint32_t APB1ENR;  	/* RCC APB1 peripheral clock enable register : Address Offset 0x40 */
	__vo uint32_t APB2ENR;  	/* RCC APB2 peripheral clock enable register : Address Offset 0x44 */
	__vo uint32_t RESERVED3[2]; /* Reserved : Address Offset 0x48 - 0x4C */
	__vo uint32_t AHB1LPENR;  	/* RCC AHB1 peripheral clock enable in low power mode register : Address Offset 0x50 */
	__vo uint32_t AHB2LPENR;  	/* RCC AHB2 peripheral clock enable in low power mode register : Address Offset 0x54 */
	__vo uint32_t AHB3LPENR;  	/* RCC AHB3 peripheral clock enable in low power mode register0x5C */
	__vo uint32_t APB1LPENR;  	/* RCC APB1 peripheral clock enable in low power mode register : Address Offset 0x60 */
	__vo uint32_t APB2LPENR;  	/* RCC APB2 peripheral clock enable in low power mode register : Address Offset 0x64 */
	__vo uint32_t RESERVED5[2]; /* Reserved : Address Offset 0x68 - 0x6C */
	__vo uint32_t BDCR;  	    /* RCC Backup domain control register : Address Offset 0x70 */
	__vo uint32_t CSR;  	    /* RCC clock control and status register : Address Offset 0x74 */
	__vo uint32_t RESERVED6[2]; /* Reserved : Address Offset 0x78 - 0x7C */
	__vo uint32_t SSCGR;  	    /* RCC spread spectrum clock generation register : Address Offset 0x80 */
	__vo uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register : Address Offset 0x84 */
	__vo uint32_t PLLSAICFGR;  	/* RCC PLL configuration register : Address Offset 0x88 */
	__vo uint32_t DCKCFGR;  	/* RCC dedicated clock configuration register : Address Offset 0x8C */
	__vo uint32_t CKGATENR;  	/* RCC clocks gated enable register : Address Offset 0x90 */
	__vo uint32_t DCKCFGR2;  	/* RCC dedicated clocks configuration register 2 : Address Offset 0x94 */
}RCC_RegDef_t;

/*****************************EXTI REGISTER STRUCTURE**************************************/
typedef struct
{
	__vo uint32_t IMR;         /* Interrupt mask register : Address Offset 0x00 */
	__vo uint32_t EMR;         /* Event mask register : Address Offset 0x04 */
	__vo uint32_t RTSR;        /* Rising trigger selection register : Address Offset 0x08 */
	__vo uint32_t FTSR;        /* Falling trigger selection register : Address Offset 0x0C */
	__vo uint32_t SWIER;       /* Software interrupt event register : Address Offset 0x10 */
	__vo uint32_t PR;          /* Pending register : Address Offset 0x14 */
}EXTI_RegDef_t;

/*****************************SYSCFG REGISTER STRUCTURE**************************************/

typedef struct
{
	__vo uint32_t MEMRMP;       /* SYSCFG memory remap register : Address Offset 0x00 */
	__vo uint32_t PMC;          /* SYSCFG peripheral mode configuration register : Address Offset 0x04 */
	__vo uint32_t EXTICR[4];    /* SYSCFG external interrupt configuration register 1-4 : Address Offset 0x08 - 0x14 */
	__vo uint32_t RESERVED1[2]; /* Reserved : Address Offset 0x18 - 0x1C */
	__vo uint32_t CMPCR;        /* Compensation cell control register : Address Offset 0x20 */
	__vo uint32_t RESERVED2[2]; /* Reserved : Address Offset 0x24 - 0x28 */
	__vo uint32_t CFGR;         /* SYSCFG configuration register : Address Offset 0x2C */
}SYSCFG_RegDef_t;

/*****************************SPI PERIPHERAL REGISTER STRUCTURE**************************************/

typedef struct
{
	__vo uint32_t CR1;          /* SPI control register 1 : Address Offset 0x00 */
	__vo uint32_t CR2;          /* SPI control register 2 : Address Offset 0x04 */
	__vo uint32_t SR;           /* SPI status register : Address Offset 0x08 */
	__vo uint32_t DR;           /* SPI data register : Address Offset 0x0C */
	__vo uint32_t CRCPR;        /* SPI CRC polynomial register : Address Offset 0x10 */
	__vo uint32_t RXCRCR;       /* SPI RX CRC register  : Address Offset 0x14 */
	__vo uint32_t TXCRCR;       /* SPI TX CRC register : Address Offset 0x18 */
	__vo uint32_t I2SCFGR;      /* SPI_I2S configuration register : Address Offset 0x1C */
	__vo uint32_t I2SPR;        /* SPI_I2S prescaler register : Address Offset 0x20 : Address Offset 0x24 */
}SPI_RegDef_t;

/*****************************Peripheral Pointer to Structure Definitons****************************************/

#define GPIOA 			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI 			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG 			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)

/****************Clock Enable and Disable Macros********************************************/

/****Clock Enable for GPIOx****/

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))

/****Clock Disable for GPIOX****/

#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))

/****Clock Enable for SPI****/

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))

/****Clock Disable for SPI****/

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))

/****Clock Enable for I2C****/

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))

/****Clock Disable for I2C****/

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))

/****Clock Enable for USART, UART****/

#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))

/****Clock Disable for USART, UART****/

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))

/****Clock Enable & Disable for SYSCFG***/
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))

/****SYSCFG_EXTI PORTCODE for GPIOx***/

#define GPIO_TO_CODE(x)			   ((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
								    (x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
								    (x == GPIOH) ? 7 :0 )
/****GPIO Register Reset Macros****/

#define GPIOA_REG_RESET()				do { RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOB_REG_RESET()				do { RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); } while(0)
#define GPIOC_REG_RESET()				do { RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); } while(0)
#define GPIOD_REG_RESET()				do { RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); } while(0)
#define GPIOE_REG_RESET()				do { RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); } while(0)
#define GPIOF_REG_RESET()				do { RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); } while(0)
#define GPIOG_REG_RESET()				do { RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); } while(0)
#define GPIOH_REG_RESET()				do { RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); } while(0)

/****SPI Register Reset Macros****/
#define SPI1_REG_RESET()				do { RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12); } while(0)
#define SPI2_REG_RESET()				do { RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14); } while(0)
#define SPI3_REG_RESET()				do { RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15); } while(0)
#define SPI4_REG_RESET()				do { RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13); } while(0)

/****IRQ Position Macros****/
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

/****Macros for IRQ Priority****/
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/*** Some Generic Macros ***/
#define ENABLE 			1
#define DISABLE 		0
#define SET   			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET  	SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET

/*****Bit Position Definitions of SPI Peripheral******/

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


#endif /* INC_STM32F446XX_H_ */
