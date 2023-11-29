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
	uint32_t temp = 0; //tem register

	//Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = ((pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode) << (2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	} else
	{
		//this part will code later. (interrupt mode)
		if (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI ->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);

			//Clear the correspondinf RSTR bit
			EXTI ->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI ->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);

			//Clear the correspondinf FSTR bit
			EXTI ->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure the both RTSR and FTSR
			EXTI ->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
			EXTI ->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
		}
			//2. configure the GPIO port selection in SYSCFG_EXITICR
			uint8_t	temp1 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber / 4 ;
			uint8_t	temp2 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber % 4 ;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] &= ~(0xF << (4 * temp2));  // This line is  missing
			SYSCFG->EXTICR[temp1] |= portcode << (4 * temp2);

			//3. enable the exti interrupt delivery using IMR
			EXTI ->EXTI_IMR |= (1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
	}
	temp = 0;

	//2. configure the speed
	temp = ((pGPIOHandle->GPIO_Pinconfig.GPIO_pinSpeed)<<(2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0 ;

	//3. configure the pupd settings
	temp = ((pGPIOHandle->GPIO_Pinconfig.GPIO_PinPuPdControl)<<(2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber)));
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
		(pGPIOHandle->pGPIOx->AFR[temp1]) &= ~(0xF<<(4 * temp2));			//clearing
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

/*********************************************************************
 * @fn      		  - GPIO_WritetoOutputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin Number
 * @param[in]         - value
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WritetoOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ((0x01) << PinNumber);
	}else
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ~((0x01) << PinNumber);
	}
}
void GPIO_WritetoOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = (uint32_t) Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin Number
 * @param[in]         - value
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * IRQ Configuration and ISR handling
 *********************************************************************/
/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <=31 )
		{
			//program ISER0 register
			*NVIC_ISER0 |= 1 << IRQNumber;

		}else if(IRQNumber >31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if (IRQNumber <=31 )
		{
			//program ICER0 register
			*NVIC_ICER0 |= 1 << IRQNumber;

		}else if(IRQNumber >31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}
/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQpriority)
{
	//1. first find out ipr register
	uint8_t prx 		= IRQNumber / 4 ;
	uint8_t prx_section = IRQNumber % 4 ;

	uint8_t shift_amout = (8 * prx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + prx) &= ~(0xF << (shift_amout));
	*(NVIC_PR_BASE_ADDR + prx) |= (IRQpriority << (shift_amout));
}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQHandling (uint8_t PinNumber)
{
	//Clear the exti pr register corressponding to the pin number
	if (EXTI->EXTI_PR & (1<<PinNumber))
	{
		//Clear
		EXTI->EXTI_PR |= (1<<PinNumber);
	}
}
