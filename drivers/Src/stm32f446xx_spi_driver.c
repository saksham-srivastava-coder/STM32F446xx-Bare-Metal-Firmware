/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 18-Dec-2025
 *      Author: HP
 */

#include "stm32f446xx_spi_driver.h"

/*******************************************************************************
 * @fn				-	SPI_PeriClockControl
 *
 * @brief			-	This function will set the clock for SPI.
 *
 * @param[in]		-	Base Address of SPI1 to SPI4
 * @param[in]		-	Enable or Disable Input
 *
 * @return			-	none
 *
 * @Note			-	none

*/

void SPI_PeriClockControl(SPI_RegDef_t* pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx ==  SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx ==  SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx ==  SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx ==  SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx ==  SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx ==  SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx ==  SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx ==  SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*******************************************************************************
 * @fn				-	SPI_Init
 *
 * @brief			-	This function will initialize the SPI with given configurations.
 *
 * @param[in]		-	Handle structure (base address + configuration structure) of SPI.
 *
 * @return			-	none
 *
 * @Note			-	none
*/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0;
	// 1.Configure the Device Mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2.Configure the BusConfig
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// Bidi should be cleared for unidirectional
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Bidi should be set for bidirectional
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// Bidi should be cleared for unidirectional, RXOnly mode should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3.Configure the SPI clock speed
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	// 4.Configure the SPI DataFrame Format
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	// 5.Configure the SPI CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 6.Configure the SPI CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// 7.Configure the SPI SSM
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*******************************************************************************
 * @fn				-	SPI_DeInit
 *
 * @brief			-	This function will deinitialize the SPI with the use of RCC Reset Register.
 *
 * @param[in]		-	Base address of SPI.
 *
 * @return			-	none
 *
 * @Note			-	none
*/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

/*******************************************************************************
 * @fn				-	SPI_SendData
 *
 * @brief			-	This function will send data through SPI Peripheral.
 *
 * @param[in]		-	Base address of SPI.
 * @param[in]		-	Pointer to the TxBuffer, it stores the pointer of the data stored in an array.
 * @param[in]		-	Legnth of data
 *
 * @return			-	none
 *
 * @Note			-	This is blocking mode, this is polling call
*/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16-bit mode
			pSPIx->DR = *((uint16_t *)pTxBuffer);		//2-byte data placed into data register
			(uint16_t *)pTxBuffer++;		//2-byte pointer address shifted
			Len--;
			Len--;
		}
		else
		{
			//8-bit mode
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}
	}
}

/*******************************************************************************
 * @fn				-	SPI_PeripheralControl
 *
 * @brief			-	This function will enable or disable the SPI with given configurations.
 *
 * @param[in]		-	Base address of SPI.
 * @param[in]		-	Enable or Disable Input
 *
 * @return			-	none
 *
 * @Note			-	This is blocking mode, this is polling call
*/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16-bit mode
			*((uint16_t *)pRxBuffer) = pSPIx->DR;		//2-byte data placed into data register
			(uint16_t *)pRxBuffer++;		//2-byte pointer address shifted
			Len--;
			Len--;
		}
		else
		{
			//8-bit mode
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
			Len--;
		}
	}
}

/*******************************************************************************
 * @fn				- SPI_IRQInterruptConfig
 *
 * @brief			- This function sets or clears the interrupt enable register according to IRQ Number and Enable value
 *
 * @param[in]		- IRQ Number
 * @param[in]		- Enable or Disable Pin
 *
 * @return			- none
 *
 * @Note			- none
*/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber < 32)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*******************************************************************************
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			- This function configure the priority of an IRQNumber
 *
 * @param[in]		- IRQ Number
 * @param[in]		- IRQ Priority
 *
 * @return			- none
 *
 * @Note			- none
*/

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;				// For determining register number by dividing by 4 sections per register
	uint8_t iprx_section = IRQNumber % 4;		// For determining the order of section in a 32 bit register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		// 1.Save the TX Buffer address, Length and store it in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2.Mark the SPI State as busy in transmission so that no one can change the SPI State
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3.Enable the Transmit Empty Interrupt bit from Control Register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		// 1.Save the TX Buffer address, Length and store it in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2.Mark the SPI State as busy in transmission so that no one can change the SPI State
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3.Enable the Transmit Empty Interrupt bit from Control Register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

