/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Jan 21, 2024
 *      Author: thaithinhtran
 */

#include "stm32f407xx_usart_driver.h"



/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given USARTs
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_PeriClockControl(USART_RegDef_t *pUSART,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			if (pUSART == USART1)
			{
				USART1_PCLK_EN();
			}
			else if (pUSART == USART2)
			{
				USART2_PCLK_EN();
			}
			else if (pUSART == USART3)
			{
				USART3_PCLK_EN();
			}
			else if (pUSART == UART4)
			{
				UART4_PCLK_EN();
			}
			else if (pUSART == UART5)
			{
				UART5_PCLK_EN();
			}
			else if (pUSART == USART6)
			{
				USART6_PCLK_EN();
			}
		}
		else
		{
			if (pUSART == USART1)
			{
				USART1_PCLK_DI();
			}
			else if (pUSART == USART2)
			{
				USART2_PCLK_DI();
			}
			else if (pUSART == USART3)
			{
				USART3_PCLK_DI();
			}
			else if (pUSART == UART4)
			{
				UART4_PCLK_DI();
			}
			else if (pUSART == UART5)
			{
				UART5_PCLK_DI();
			}
			else if (pUSART == USART6)
			{
				USART6_PCLK_DI();
			}
		}
}


/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 TODO

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << TODO);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << TODO );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << TODO) | ( 1 << TODO) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << TODO ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= ( 1 << TODO);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << TODO);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = TODO;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.TODO << TODO;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->TODO = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << TODO);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= TODO

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		TODO
	}


	pUSARTHandle->pUSARTx->TODO = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here

}

/*********************************************************************
 * IRQ Configuration and ISR handling
 *********************************************************************/
/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first find out ipr register
	uint8_t prx 		= IRQNumber / 4 ;
	uint8_t prx_section = IRQNumber % 4 ;

	uint8_t shift_amout = (8 * prx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + prx) &= ~(0xF << (shift_amout));
	*(NVIC_PR_BASE_ADDR + prx) |= (IRQpriority << (shift_amout));
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - This function Enable USART
 *
 * @param[in]         - The address of Handle structure for USARTx peripheral
 * @param[in]         - Enable or Disable option
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDI == ENABLE)
	{
		pUSARTx->CR1 |= 1<< USART_CR1_UE_Pos;
	}else
	{
		pUSARTx->CR1 &= ~(1<< USART_CR1_UE_Pos);
	}
}

/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - This function to return status of Flags
 *
 * @param[in]         - The address of Definition structure for USARTx peripheral
 * @param[in]         - Flag name
 * @param[in]         -
 *
 * @return            -  Set or Reset
 *
 * @Note              -
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->SR & FLAG_NAME)
		{
			return FLAG_SET;
		}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Applicable to only USART_CTS_FLAG , USART_LBD_FLAG
 * USART_TC_FLAG,USART_TC_FLAG flags
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(1<<StatusFlagName)
}
