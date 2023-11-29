/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 5, 2023
 *      Author: thaithinhtran
 */

#include "stm32f407xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPIs
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			if (pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{
			if (pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if (pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function Init SPix
 *
 * @param[in]         - The address of Handle structure for SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_Init(SPI_Handler_t *pSPIHanler)
{
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHanler->pSPIx, ENABLE);

	//First lets configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= pSPIHanler->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_Pos;

	//2. Configure the bus config
	if (pSPIHanler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE_Pos);
	}else if (pSPIHanler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode shoud be set
		tempreg |= 1<<SPI_CR1_BIDIMODE_Pos;
	}else if (pSPIHanler->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE_Pos);

		//RXONLY bit must be set
		tempreg |= 1<<SPI_CR1_RXONLY_Pos;
	}
	//3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHanler->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_Pos;

	//4. Configure the DFF
	tempreg |= pSPIHanler->SPIConfig.SPI_DFF << SPI_CR1_DFF_Pos;

	//5. configure the CPOL
	tempreg |= pSPIHanler->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_Pos;

	//6. configure the CPHA
	tempreg |= pSPIHanler->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_Pos;

	//7. Configure the SSM
	tempreg |= pSPIHanler->SPIConfig.SPI_SSM << SPI_CR1_SSM_Pos;

	pSPIHanler->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function Send SPix
 *
 * @param[in]         - The address of Handle structure for SPIx peripheral
 * @param[in]         - The transmit buffer
 * @param[in]         - Length of the message
 *
 * @return            -  none
 *
 * @Note              -  Blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. Wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 & (1<<SPI_CR1_DFF_Pos))
		{
			// 16 bit DFF
			//1. Load the data to data register
			pSPIx->DR = *(uint16_t *) pTxBuffer;
			Len--;
			Len--;
			(uint16_t*) pTxBuffer ++;
		}else
		{
			// 8 bit DFF
			//1. Load the data to data register
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer ++;
		}

	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function Receive SPix
 *
 * @param[in]         - The address of Definition structure for SPIx peripheral
 * @param[in]         - The address of Receive buffer
 * @param[in]         - Length of the message
 *
 * @return            -  none
 *
 * @Note              -  Blocking call
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. Wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 & (1<<SPI_CR1_DFF_Pos))
		{
			// 16 bit DFF
			//1. Load the data from data register to Rxbuffer address
			*(uint16_t *) pRxBuffer = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*) pRxBuffer ++;
		}else
		{
			// 8 bit DFF
			//1. Load the data from data register to Rxbuffer address
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer ++;
		}

	}
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function to return status of Flags
 *
 * @param[in]         - The address of Definition structure for SPIx peripheral
 * @param[in]         - Flag name
 * @param[in]         -
 *
 * @return            -  Set or Reset
 *
 * @Note              -
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FLAG_NAME)
{
	if(pSPIx->SR & FLAG_NAME)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function Enable SPI
 *
 * @param[in]         - The address of Handle structure for SPIx peripheral
 * @param[in]         - Enable or Disable option
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDI)
{
	if (EnOrDI == ENABLE)
	{
		pSPIx->CR1 |= 1<<SPI_CR1_SPE_Pos;
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE_Pos);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSI_Config
 *
 * @brief             - This function cofig the SSI bit
 *
 * @param[in]         - The address of Handle structure for SPIx peripheral
 * @param[in]         - Enable or Disable option
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDI)
{
	if (EnOrDI == ENABLE)
	{
		pSPIx->CR1 |= 1<<SPI_CR1_SSI_Pos;
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI_Pos);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOE_Config
 *
 * @brief             - This function cofig the SSOE bit
 *
 * @param[in]         - The address of Handle structure for SPIx peripheral
 * @param[in]         - Enable or Disable option
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 */
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDI)
{
	if (EnOrDI == ENABLE)
	{
		pSPIx->CR2 |= 1<<SPI_CR2_SSOE_Pos;
	}else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE_Pos);
	}
}
/*********************************************************************
 * IRQ Configuration and ISR handling
 *********************************************************************/
/*********************************************************************
 * @fn      		  - SPI_IRQConfig
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
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQpriority)
{
	//1. first find out ipr register
	uint8_t prx 		= IRQNumber / 4 ;
	uint8_t prx_section = IRQNumber % 4 ;

	uint8_t shift_amout = (8 * prx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + prx) &= ~(0xF << (shift_amout));
	*(NVIC_PR_BASE_ADDR + prx) |= (IRQpriority << (shift_amout));
}