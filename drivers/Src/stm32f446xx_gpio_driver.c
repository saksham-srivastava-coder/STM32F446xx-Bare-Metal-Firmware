/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 10-Dec-2025
 *      Author: HP
 */

#include "stm32f446xx_gpio_driver.h"

/*******************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables the clock for given GPIO Port
 *
 * @param[in]		- base address of GPIO Peripheral
 * @param[in]		- Enable of Disable macros
 *
 * @return			- none
 *
 * @Note			- none
*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*******************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initialize the GPIO functionalities
 *
 * @param[in]		- Handle structure of GPIO which contains base address and configuration settings of given GPIO Port
 *
 * @return			- none
 *
 * @Note			- none
*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

/*	1. Configure the mode of GPIO Pin  */
	// Non Interrupt mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Mode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing bits
		pGPIOHandle->pGPIOx->MODER |= temp; // setting bits
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_INT_FT)
		{
//			1.Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_INT_RT)
		{
//			1.Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_INT_RFT)
		{
//			1.Configure both RTSR and FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
//			2.Configure GPIO POrt selection in SYSCFG_EXTICR
			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t Portcode = GPIO_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] |= (Portcode << (4 * temp2));


//			3.Enable the EXTI Interrupt delivery using IMR
			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

/*  2. Configure the speed	*/
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

/*  3. Configure the Pull up / Pull down settings */
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

/*  4. Configure the output type */
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

/*  5. Configure the alternate functionality settings */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;           // temp1 = 1 AFRH, temp1 = 0 AFRL
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;			  // temp2 = PinNumber of AFRL/AFRH
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}

/*******************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- This function deinitialize the GPIO functionalities
 *
 * @param[in]		- Base address of given GPIO Port
 *
 * @return			- none
 *
 * @Note			- none
*/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*******************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function reads the value from a specific GPIO Pin
 *
 * @param[in]		- Base address of given GPIO Port
 * @param[in]		- Pin Number of GPIO Port
 *
 * @return			- 0 / 1
 *
 * @Note			- none
*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(( pGPIOx->IDR >> PinNumber ) & 0x00000001 );      // IDR Register value right shifted to LSB by Pin number times , then Masked with 1 to get the value
	return value;
}

/*******************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- This function reads the value from a specific GPIO Port
 *
 * @param[in]		- Base address of given GPIO Port
 *
 * @return			- 0 / 1 from each pin
 *
 * @Note			- none
*/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*******************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- This function writes the value to a specific GPIO Pin
 *
 * @param[in]		- Base address of given GPIO Port
 * @param[in]		- Pin Number of GPIO Port
 * @param[in]		- Value to be written on pin
 *
 * @return			- none
 *
 * @Note			- none
*/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*******************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- This function writes the value to a specific GPIO Port
 *
 * @param[in]		- Base address of given GPIO Port
 * @param[in]		- Value to be written on port
 *
 * @return			- none
 *
 * @Note			- none
*/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/*******************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- This function toggles the value to a specific GPIO Pin
 *
 * @param[in]		- Base address of given GPIO Port
 * @param[in]		- Pin Number to be toggled
 *
 * @return			- none
 *
 * @Note			- none
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);     // For toggling we can use bitwise XOR
}

/*******************************************************************************
 * @fn				- GPIO_IRQInterruptConfig
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

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				- GPIO_IRQPriorityConfig
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;				// For determining register number by dividing by 4 sections per register
	uint8_t iprx_section = IRQNumber % 4;		// For determining the order of section in a 32 bit register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}

/*******************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- This function handles the interrupt by clearing the pending register of a particular pin number
 *
 * @param[in]		- IRQ Number
 * @param[in]		- IRQ Priority
 *
 * @return			- none
 *
 * @Note			- none
*/

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);		// cleared the pending register by programming it to 1
	}
}



