/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Dec 3, 2023
 *      Author: thaithinhtran
 */

#include "stm32f407xx_I2C_driver.h"

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given I2Cs
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			if (pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}
		else
		{
			if (pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
}

/*
 * Helper function
 */
uint32_t RCC_GetPLLOutputClock()
{
	//Needs to be fixed
	return 0;
}
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t	clksrc,temp,AHBp,APBp;
	uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
	uint16_t APB_PreScaler[8] = {2,4,8,16};

	clksrc = (RCC->RCC_CFGR >>2) & 0x03;
	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if (clksrc == 1)
	{
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = (RCC->RCC_CFGR >>4) & 0xF;
	if(temp < 8)
	{
		AHBp = 1;
	}else
	{
		AHBp = AHB_PreScaler[temp-8];
	}

	temp = (RCC->RCC_CFGR >>10) & 0x7;
	if(temp < 4)
	{
		APBp = 1;
	}else
	{
		APBp = APB_PreScaler[temp-4];
	}

	pclk1 = (SystemClk/AHBp)/APBp;
	return pclk1;
}
/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - This function Init I2Cx
 *
 * @param[in]         - The address of Handle structure for I2Cx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_Init(I2C_Handler_t *pI2CHandler)
{
	uint32_t temreg = 0;

	//Set the ACK bit
	temreg |= pI2CHandler->I2CConfig.I2C_ACKControl << 10;
}
