/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Dec 3, 2023
 *      Author: thaithinhtran
 */

#include "stm32f407xx_I2C_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx,uint8_t SlaveAddr);
static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ClearADDRFlag(I2C_RegDef_t* pI2Cx);

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
	pI2CHandler->pI2Cx->CCR &= ~(0xFFF);
	pI2CHandler->pI2Cx->CCR |= temreg;

	//TRISE Configuration
	if(pI2CHandler->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		temreg = (RCC_GetPCLK1Value() / 1000000) + 1;
	}else
	{
		//mode is fast mode
		temreg = ((RCC_GetPCLK1Value() * 3) / 10000000) + 1;
	}
	pI2CHandler->pI2Cx->TRISE &= ~(0x3F);
	pI2CHandler->pI2Cx->TRISE |= temreg;
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - This function send data
 *
 * @param[in]         - The address of Handle structure for I2Cx peripheral
 * @param[in]         - The transmit buffer
 * @param[in]         - Length of the message
 * @param[in]         - Slave Address
 *
 * @return            -  none
 *
 * @Note              -  Blocking call
 */
void I2C_MasterSendData(I2C_Handler_t *pI2CHandler,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// 	 Note : Until SB is cleared SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandler->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx,I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandler->pI2Cx);

	//6. Send the data until Len becomes 0
	while (Len > 0 )
	{
		//Wait until TxE is set
		while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx, I2C_FLAG_TxE));
		pI2CHandler->pI2Cx->DR = *(pTxbuffer);
		pTxbuffer++;
		Len--;

	}

	//7. When Len becomes 0 wait for TXE=1 and BTF = 1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1, means that both SR and DR are empty and next transmission should
	//   begin when BTF=1 SCL will be stretched (pull to LOW)
	while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx, I2C_FLAG_TxE));
	while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterRecieveData
 *
 * @brief             - This function receive data
 *
 * @param[in]         - The address of Handle structure for I2Cx peripheral
 * @param[in]         - The receive buffer
 * @param[in]         - Length of the message
 * @param[in]         - Slave Address
 *
 * @return            -  none
 *
 * @Note              -  Blocking call
 */
void I2C_MasterRecieveData(I2C_Handler_t *pI2CHandler,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

	//2. Confirm that start generation was completed
	while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (8bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandler->pI2Cx,SlaveAddr);

	//4. Wait until address is complete by check ADDR flag in SR1
	while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx,I2C_FLAG_ADDR));

	if(Len == 1)
	{
		//Read a single byte from Slave
		//1. Disable ACKing
		I2C_ManageAcking(pI2CHandler->pI2Cx, DISABLE);

		//2. Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandler->pI2Cx);

		//3. wait until RXNE = 1
		while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx, I2C_FLAG_RxNE));

		//4. Generate STOP condition
		if(Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
		}

		//5. Read data in buffer DR
		*pRxBuffer = pI2CHandler->pI2Cx->DR;
	}
	if(Len > 1)
	{
		//Read multiple bytes
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandler->pI2Cx);

		//Read the data until Len is zero
		for(uint32_t i = Len ; i>0 ; i--)
		{
			//Wait until RXNE = 1
			while (!I2C_GetFlagStatusSR1(pI2CHandler->pI2Cx, I2C_FLAG_RxNE));

			if (i == 2)//last 2 bytes remaining
			{
				//Clear the ACKing
				I2C_ManageAcking(pI2CHandler->pI2Cx, DISABLE);

				//Generate STOP condition
				if(Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
				}
			}
			//Read the data from data register DR into buffer
			*pRxBuffer = pI2CHandler->pI2Cx->DR;

			//Increment the buffer address
			pRxBuffer++;
		}
	}
		//Re-Enable ACKing
		if(pI2CHandler->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE)
		{
			I2C_ManageAcking(pI2CHandler->pI2Cx, ENABLE);
		}
}

uint8_t I2C_MasterSendDataIT(I2C_Handler_t *pI2CHandler,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

}

uint8_t I2C_MasterRecieveDataIT(I2C_Handler_t *pI2CHandler,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDI)
{
	if (EnOrDI == ENABLE)
	{
		pI2Cx->CR1 |= 1<<I2C_CR1_PE_Pos;
	}else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE_Pos);
	}
}

/*
 * Helper function
 */
void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_START_Pos);
}

uint8_t I2C_GetFlagStatusSR1(I2C_RegDef_t* pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;

	}
	return FLAG_RESET;
}

void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;

	//The LSB is R/nW bit which must be set to 0 for Write
	SlaveAddr &= ~(1);

	pI2Cx->DR = SlaveAddr;
}

void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;

	//The LSB is R/nW bit which must be set to 1 for Read
	SlaveAddr |= 1;

	pI2Cx->DR = SlaveAddr;
}

void I2C_ManageAcking(I2C_RegDef_t* pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= 1 << I2C_CR1_ACK_Pos;
	} else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK_Pos);
	}
}
void I2C_ClearADDRFlag(I2C_RegDef_t* pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void) dummyRead;
}

void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP_Pos);
}
