/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jan 26, 2024
 *      Author: thaithinhtran
 */
#include "stm32f407xx_rcc_driver.h"

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

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2,SystemClk;
	uint8_t	clksrc,temp,AHBp,APBp2;
	uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
	uint16_t APB_PreScaler[4] = {2,4,8,16};

	clksrc = (RCC->RCC_CFGR >>2) & 0x03;
	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if (clksrc == 1)
	{
		SystemClk = 8000000;
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
	temp = (RCC->RCC_CFGR >>13) & 0x7;
	if(temp < 4)
	{
		APBp2 = 1;
	}else
	{
		APBp2 = APB_PreScaler[temp-4];
	}

	pclk2 = (SystemClk/AHBp)/APBp2;
	return pclk2;
}
