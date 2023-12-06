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
	uint16_t APB_PreScaler[4] = {2,4,8,16};

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

	//For AHB
	temp = (RCC->RCC_CFGR >>4) & 0xF;
	if(temp < 8)
	{
		AHBp = 1;
	}else
	{
		AHBp = AHB_PreScaler[temp-8];
	}

	//For APB
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

	//Enable the clock for the I2Cx peripheral
	I2C_PeriClockControl(pI2CHandler->pI2Cx, ENABLE);

	//Set the ACK control bit
	temreg |= pI2CHandler->I2CConfig.I2C_ACKControl << 10;
	pI2CHandler->pI2Cx->CR1 &= ~(1<<10);
	pI2CHandler->pI2Cx->CR1 |= temreg;

	//Configure the FREQ field of CR2
	temreg = 0;
	temreg = RCC_GetPCLK1Value() / 1000000;
	pI2CHandler->pI2Cx->CR2 &= ~(0x3F);
	pI2CHandler->pI2Cx->CR2 |= (temreg & 0x3F);

	//program the device own address
	temreg = 0;
	temreg = pI2CHandler->I2CConfig.I2C_DeviceAddress;
	pI2CHandler->pI2Cx->OAR1 &= ~(0x7F << 1);
	pI2CHandler->pI2Cx->OAR1 |= (temreg << 1);
	pI2CHandler->pI2Cx->OAR1 |= (1<<14); //Don't know while just follow UM

	//CCR calculations
	uint16_t ccr_value = 0;
	temreg = 0;
	if(pI2CHandler->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandler->I2CConfig.I2C_SCLSpeed);
		temreg |= ccr_value & 0xFFF;
	}else
	{
		//mode is fast mode
		temreg |= (1<<15);
		temreg |= (pI2CHandler->I2CConfig.I2C_FMDutyCycle << 14);
		if (pI2CHandler->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandler->I2CConfig.I2C_SCLSpeed);
		}else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandler->I2CConfig.I2C_SCLSpeed);
		}
		temreg |= ccr_value & 0xFFF;
	}
	pI2CHandler->pI2Cx->CCR = temreg;
}
