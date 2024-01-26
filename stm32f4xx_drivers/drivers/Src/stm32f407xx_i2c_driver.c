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
static void I2C_ClearADDRFlag(I2C_Handler_t* pI2CHandle);
static void I2C_MasterHandlerRXNEInterrupt (I2C_Handler_t* pI2CHandler);
static void I2C_MasterHandlerTXEInterrupt (I2C_Handler_t* pI2CHandle);
static void I2C_CloseReceiveData(I2C_Handler_t* pI2C_Handler);

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
	I2C_ClearADDRFlag(pI2CHandler);

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
		I2C_ClearADDRFlag(pI2CHandler);

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
		I2C_ClearADDRFlag(pI2CHandler);

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
	uint8_t busystate = pI2CHandler->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandler->pTxBuffer =  pTxbuffer;
		pI2CHandler->TxLen = Len;
		pI2CHandler->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandler->DevAddr = SlaveAddr;
		pI2CHandler->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN_Pos);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN_Pos);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN_Pos);
	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handler_t *pI2CHandler,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandler->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandler->pRxBuffer = pRxBuffer;
		pI2CHandler->RxLen = Len;
		pI2CHandler->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandler->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception 
		pI2CHandler->DevAddr = SlaveAddr;
		pI2CHandler->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN_Pos);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN_Pos);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN_Pos);
	}

	return busystate;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t) pI2C->DR;
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

void I2C_ClearADDRFlag(I2C_Handler_t* pI2CHandler)
{
	uint32_t dummy_read;
	//Check for device mode
	if (pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL_Pos))
	{
		//device is in master mode
		if (pI2CHandler->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandler->RxSize ==1)
			{
				//First disable the ack
				I2C_ManageAcking(pI2CHandler->pI2Cx, DISABLE);

				//Clear the ADDR flag (read SR1 , read SR2)
				dummy_read = pI2CHandler->pI2Cx->SR1;
				dummy_read = pI2CHandler->pI2Cx->SR2;
				(void)dummy_read;
			}
		}else
		{
			//Clear the ADDR flag (read SR1 , read SR2)
			dummy_read = pI2CHandler->pI2Cx->SR1;
			dummy_read = pI2CHandler->pI2Cx->SR2;
			(void)dummy_read;
		}

	}else
	{
		//device is in slave mode
		//Clear the ADDR flag (read SR1 , read SR2)
		dummy_read = pI2CHandler->pI2Cx->SR1;
		dummy_read = pI2CHandler->pI2Cx->SR2;
		(void)dummy_read;

	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP_Pos);
}

/*********************************************************************
 * IRQ Configuration and ISR handling
 *********************************************************************/
/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQpriority)
{
	//1. first find out ipr register
	uint8_t prx 		= IRQNumber / 4 ;
	uint8_t prx_section = IRQNumber % 4 ;

	uint8_t shift_amout = (8 * prx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + prx) &= ~(0xF << (shift_amout));
	*(NVIC_PR_BASE_ADDR + prx) |= (IRQpriority << (shift_amout));
}

void I2C_EV_IRQHandling(I2C_Handler_t *pI2CHandler)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandler->pI2Cx->CR2 & (1<<I2C_CR2_ITEVTEN_Pos);
	temp2 = pI2CHandler->pI2Cx->CR2 & (1<<I2C_CR2_ITBUFEN_Pos);

	temp3 = pI2CHandler->pI2Cx->SR1 & (1<<I2C_SR1_SB_Pos);
	//1. Handler for Interrupt generated by SB event
	// Note : SB flag is only applicable in Master mode
	if (temp1 && temp3)
	{
		//SB flag is set
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if (pI2CHandler->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandler->pI2Cx, pI2CHandler->DevAddr);

		}else if(pI2CHandler->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandler->pI2Cx, pI2CHandler->DevAddr);
		}
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1<<I2C_SR1_ADDR_Pos);
	//2. Handler for Interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//	     When slave mode : Address is matched with own address
	if (temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandler);
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1<<I2C_SR1_BTF_Pos);
	//3. Handle for Interrupt generated by BTF (Byte transfer finished) event
	if (temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandler->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set
			if (pI2CHandler->pI2Cx->SR1 & (1<<I2C_SR1_TxE_Pos))
			{
				//BTF , TXR = 1
				if (pI2CHandler->TxLen == 0)
				{
					//1. Generate the STOP condition
					if (pI2CHandler->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
					}
					//2. Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandler);

					//3. Notify that application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandler,I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandler->TxRxState == I2C_BUSY_IN_RX)
		{
			//do nothing
			;
		}
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1<<I2C_SR1_STOPF_Pos);
	//4. Handler for Interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode.For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if (temp1 && temp3)
	{
		//STOPF flag is set
		//Clear the STOPF (read SR1 then Write to CR1 , read SR1 done above)
		pI2CHandler->pI2Cx->CR1 |= 0x0000;

		//Notify that application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandler,I2C_EV_STOP);

	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1<<I2C_SR1_TxE_Pos);
	//5. Handle For interrupt generated by TXE event
	if (temp1 && temp2 && temp3)
	{
		//Check the device mode
		if (pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL_Pos))
		{
			//TxE flag is set
			//We have to do the data transmission
			if(pI2CHandler->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandlerTXEInterrupt(pI2CHandler);
			}
		}else
		{
			//Slave mode
			//make sure that the slave is really in transmitter mode
			if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_TRA_Pos))
			{
				I2C_ApplicationEventCallback(pI2CHandler, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandler->pI2Cx->SR1 & (1<<I2C_SR1_RxNE_Pos);
	//6. Handle For interrupt generated by RXNE event
	if (temp1 && temp2 && temp3)
	{
		//Check the device mode
		if (pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL_Pos))
		{
			//RxNE flag is set
			if(pI2CHandler->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandlerRXNEInterrupt(pI2CHandler);
			}
		}else
		{
			//Slave
			//make sure that the slave is really in transmitter mode
			if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_TRA_Pos))
			{
				I2C_ApplicationEventCallback(pI2CHandler, I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_MasterHandlerTXEInterrupt (I2C_Handler_t* pI2CHandler)
{
		if(pI2CHandler->TxLen > 0)
		{
			//1. Load the data in to DR
			pI2CHandler->pI2Cx->DR = *(pI2CHandler->pTxBuffer);

			//2. Decrement the TxLen
			pI2CHandler->TxLen--;

			//3. Increment the buffer address
			pI2CHandler->pTxBuffer++;
		}
}

void I2C_MasterHandlerRXNEInterrupt (I2C_Handler_t* pI2CHandler)
{
		//We have to do to the data reception
		if(pI2CHandler->RxSize ==1)
		{
			*pI2CHandler->pRxBuffer = pI2CHandler->pI2Cx->DR;
			pI2CHandler->RxLen--;
		}

		if(pI2CHandler->RxSize >1)
		{
			if (pI2CHandler->RxLen == 2)
			{
				//Clear the ACK bit
				I2C_ManageAcking(pI2CHandler->pI2Cx, DISABLE);
			}

			//Read DR
			*pI2CHandler->pRxBuffer = pI2CHandler->pI2Cx->DR;
			pI2CHandler->pRxBuffer++;
			pI2CHandler->RxLen--;
		}
		if(pI2CHandler->RxLen == 0)
		{
			//Close the I2C data reception and notify application
			if (pI2CHandler->Sr == I2C_DISABLE_SR)
			{
				//1. Generate the stop condition
				I2C_GenerateStopCondition(pI2CHandler->pI2Cx);
			}
			//2. Close the I2C rx
			I2C_CloseReceiveData(pI2CHandler);

			//3. Notify the application
			I2C_ApplicationEventCallback(pI2CHandler, I2C_EV_RX_CMPLT);
		}
}

void I2C_CloseSendData(I2C_Handler_t* pI2C_Handler)
{
	//Implement the code to disable ITBUFFEN control bit
	pI2C_Handler->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN_Pos);

	//Implement the code to disable ITEVTEN control bit
	pI2C_Handler->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN_Pos);

	pI2C_Handler->TxRxState = I2C_READY;
	pI2C_Handler->pTxBuffer = NULL;
	pI2C_Handler->TxLen = 0;
}

void I2C_CloseReceiveData(I2C_Handler_t* pI2C_Handler)
{
	//Implement the code to disable ITBUFFEN control bit
	pI2C_Handler->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN_Pos);

	//Implement the code to disable ITEVTEN control bit
	pI2C_Handler->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN_Pos);

	pI2C_Handler->TxRxState = I2C_READY;
	pI2C_Handler->pRxBuffer = NULL;
	pI2C_Handler->RxLen = 0;
	pI2C_Handler->RxSize = 0;

	if (pI2C_Handler->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2C_Handler->pI2Cx, ENABLE);
	}
}

void I2C_ER_IRQHandling(I2C_Handler_t *pI2CHandler)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandler->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN_Pos);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandler->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR_Pos);
	if(temp1  && temp2 )
	{
		//This is Bus error
		//Implement the code to clear the buss error flag
		pI2CHandler->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR_Pos);

		//Implement the code to notify the application about the error
	    I2C_ApplicationEventCallback(pI2CHandler,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandler->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO_Pos );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag

		//Implement the code to notify the application about the error

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandler->pI2Cx->SR1) & ( 1 << I2C_SR1_AF_Pos);
	if(temp1  && temp2)
	{
		//This is ACK failure error
	    //Implement the code to clear the ACK failure error flag
		pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF_Pos);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandler,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandler->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR_Pos);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandler->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT_Pos);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag

		//Implement the code to notify the application about the error
	}

}

void I2C_SlaveControlCallbackEvents(I2C_RegDef_t* pI2Cx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN_Pos);

		//Implement the code to enable ITEVTEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN_Pos);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN_Pos);
	}else
	{
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN_Pos);

		//Implement the code to enable ITEVTEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN_Pos);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN_Pos);
	}
}
