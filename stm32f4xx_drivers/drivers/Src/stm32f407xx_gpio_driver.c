/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 19, 2023
 *      Author: thaithinhtran
 */


#include "stm32f407xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handler_t *pGPIOHandle)
{
	uint32_t temp = 0; //tem. register

	//1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = ((pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode) << (2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));				//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	} else
	{
		//this part will code later. (interrupt mode)
	}
	temp = 0;

	//2. configure the speed
	temp = ((pGPIOHandle->GPIO_Pinconfig.GPIO_pinSpeed)<<(2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));				//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0 ;

	//3. configure the pupd settings
	temp = ((pGPIOHandle->GPIO_Pinconfig.GPIO_PinPuPdControl)<<(2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));					//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0 ;

	//4. configure the optype
	temp = ((pGPIOHandle->GPIO_Pinconfig.GPIO_PinOPType)<<(pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));						//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0 ;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers
		uint8_t temp1 , temp2;

		temp1 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber % 8;
		(pGPIOHandle->pGPIOx->AFR[temp1]) |= ~(0xF<<(4 * temp2));			//clearing
		(pGPIOHandle->pGPIOx->AFR[temp1]) |= ((pGPIOHandle->GPIO_Pinconfig.GPIO_PinAltMode)<<(4 * temp2));
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin Number
 * @param[in]         -
 *
 * @return            -  0 or 1
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) (((pGPIOx->IDR) >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)
{
	uint8_t value;
	value = (uint8_t) pGPIOx->IDR;
	return value;
}

void GPIO_WritetoOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritetoOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig (uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling (uint8_t PinNumber);
