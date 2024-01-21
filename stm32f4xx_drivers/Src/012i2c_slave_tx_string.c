/*
 * 012i2c_slave_tx_string.c
 *
 *  Created on: Jan 10, 2024
 *      Author: thaithinhtran
 */


#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"

#define MY_ADDR		 SLAVEADDR
#define SLAVEADDR	 0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handler_t I2C1Handler;

//tx buffer
uint8_t Tx_buf[32] = "Thinh Tran from RVCHIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII";

/*
 * PB6 --> I2C1_SCL
 * PB9 --> I2C1_SDA
 * ALT function mode : 4
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handler_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_Pinconfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	I2CPins.GPIO_Pinconfig.GPIO_PinAltMode		= 4;
	I2CPins.GPIO_Pinconfig.GPIO_PinOPType		= GPIO_OP_TYPE_OD;
	I2CPins.GPIO_Pinconfig.GPIO_PinPuPdControl	= GPIO_PIN_PU;
	I2CPins.GPIO_Pinconfig.GPIO_pinSpeed		= GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_Pinconfig.GPIO_PinNumber		= GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_Pinconfig.GPIO_PinNumber		= GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1Handler.pI2Cx			 			= I2C1;
	I2C1Handler.I2CConfig.I2C_ACKControl	= I2C_ACK_ENABLE;
	I2C1Handler.I2CConfig.I2C_DeviceAddress = MY_ADDR;
	I2C1Handler.I2CConfig.I2C_FMDutyCycle	= I2C_FM_DUTY_2;
	I2C1Handler.I2CConfig.I2C_SCLSpeed		= I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handler);
}

int main (void)
{
	//I2C Pin inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	I2C_SlaveControlCallbackEvents(I2C1,ENABLE);

	//Enable the I2C peripheral
	I2C_PeripheralControl(I2C1Handler.pI2Cx, ENABLE);

	//Enable ACK
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);

	return 0;
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handler);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handler);
}

void I2C_ApplicationEventCallback(I2C_Handler_t *pI2CHandler,uint8_t ApplicationEvent)
{
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0 ;

	if (ApplicationEvent == I2C_EV_DATA_REQ)
	{
		//Master wants some data, slave has to send it
		if(commandCode == 0x51)
		{
			//Send the length information to the master
			I2C_SlaveSendData(pI2CHandler->pI2Cx,strlen((char*)Tx_buf));
		}else if (commandCode == 0x52)
		{
			//Send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandler->pI2Cx,Tx_buf[Cnt++]);
		}
	}else if (ApplicationEvent == I2C_EV_DATA_RCV)
	{
		//Data is waiting for the slave to read. Slave has to read it
		commandCode = I2C_SlaveReceiveData(pI2CHandler->pI2Cx);

	}else if (ApplicationEvent == I2C_ERROR_AF)
	{
		//This happens only during slave txing
		//Master has sent the NACK. Slave understand that Master receive enough data
		commandCode = 0xff;
		Cnt = 0;
	}else if (ApplicationEvent == I2C_EV_STOP)
	{
		//This happens only during slave rxing
		//Master has ended the I2C communication with slave
	}

}


