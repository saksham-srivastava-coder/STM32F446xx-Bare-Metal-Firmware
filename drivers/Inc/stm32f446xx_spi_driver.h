/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 18-Dec-2025
 *      Author: HP
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */

/********Configuration Structure of SPI**********/

typedef struct
{
	uint8_t SPI_DeviceMode;					/* @SPI_DeviceMode will have the definitions */
	uint8_t SPI_BusConfig;					/* @SPI_BusConfig will have the definitions */
	uint8_t SPI_SclkSpeed;					/* @SPI_SclkSpeed will have the definitions */
	uint8_t SPI_DFF;						/* @SPI_DFF will have the definitions */
	uint8_t SPI_CPOL;						/* @SPI_CPOL will have the definitions */
	uint8_t SPI_CPHA;						/* @SPI_CPHA will have the definitions */
	uint8_t SPI_SSM;						/* @SPI_SSM will have the definitions */
}SPI_Config_t;

/********Handle Structure of SPI**********/

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t		 *pTxBuffer;				/* To Store the application Tx Buffer Address */
	uint8_t 	 *pRxBuffer;				/* To Store the application Rx Buffer Address */
	uint32_t 	 TxLen;						/* To Store Tx Length */
	uint32_t 	 RxLen;						/* To Store Rx Length */
	uint8_t		 TxState;					/* To Store Tx State */
	uint8_t		 RxState;					/* To Store Rx State */
}SPI_Handle_t;

/****@SPI_DeviceMode****/
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/****@SPI_BusConfig****/
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3			//By default in simplex it is transmit only

/****@SPI_SclkSpeed****/
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/****@SPI_DFF****/
#define SPI_DFF_8BIT			0
#define SPI_DFF_16BIT			1

/****@SPI_CPOL****/
#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1

/****@SPI_CPHA****/
#define SPI_CPHA_LOW			0
#define SPI_CPHA_HIGH			1

/****@SPI_SSM****/
#define SPI_SSM_EN				1
#define SPI_SSM_DI				0

/****SPI_SR_Flags****/
#define SPI_FLAG_RXNE				(1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE				(1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE				(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR				(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR				(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF				(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR				(1 << SPI_SR_OVR)
#define SPI_FLAG_BSY				(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE				(1 << SPI_SR_FRE)

/*****SPI_APPLICATION_STATES****/
#define SPI_READY				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

/* Peripheral Clock Setup*/

void SPI_PeriClockControl(SPI_RegDef_t* pSPIx,uint8_t EnorDi);

/*Init and De-Init*/

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*Data Send and Receive*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*Data Send and Receive with Interrupt*/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*IRQ and ISR Handling*/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
