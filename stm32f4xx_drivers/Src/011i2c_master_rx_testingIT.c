/*
 * 011i2c_master_rx_testingIT.c
 *
 *  Created on: Jan 2, 2024
 *      Author: thaithinhtran
 */


#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"

#define MY_ADDR		 0x24
#define SLAVEADDR	 0x68

extern void initialise_monitor_handles(void);

//Flag variable
uint8_t rxComplt = RESET;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handler_t I2C1Handler;

//reveive buffer
uint8_t rcv_buf[32];

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
	I2C1Handler.I2CConfig.I2C_DeviceAddress = MY_ADDR; //Does not matter, only device in slave. Choose care check un-accepted value in NXP
	I2C1Handler.I2CConfig.I2C_FMDutyCycle	= I2C_FM_DUTY_2;
	I2C1Handler.I2CConfig.I2C_SCLSpeed		= I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handler);
}

void GPIO_ButtonInit()
{
	GPIO_Handler_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_Pinconfig.GPIO_PinNumber	 	= GPIO_PIN_NO_0;
	GpioBtn.GPIO_Pinconfig.GPIO_PinMode		 	= GPIO_MODE_IN;
	GpioBtn.GPIO_Pinconfig.GPIO_pinSpeed	 	= GPIO_SPEED_FAST;
	GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

int main (void)
{
	extern void initialise_monitor_handles(void);
	initialise_monitor_handles();

	uint8_t command_code;
	uint8_t len;

	//I2C Pin inits
	I2C1_GPIOInits();

	//Button inits
	GPIO_ButtonInit();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	//Enable the I2C peripheral
	I2C_PeripheralControl(I2C1Handler.pI2Cx, ENABLE);

	//Enable ACK
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		//Wait for the Button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//To avoid button de-boucing, related issues 200ms of delay
		delay();

		command_code = 0x51;
		while(I2C_MasterSendDataIT(&I2C1Handler, &command_code,1, SLAVEADDR,I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterRecieveDataIT(&I2C1Handler, &len, 1, SLAVEADDR,I2C_ENABLE_SR) != I2C_READY);

		command_code = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handler, &command_code,1, SLAVEADDR,I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterRecieveDataIT(&I2C1Handler, rcv_buf, len, SLAVEADDR,I2C_DISABLE_SR) != I2C_READY);

		rxComplt = RESET;

		//wait till rx completes
		while (rxComplt != SET);

		rcv_buf[len+1]='\0';
		printf("Data receive is :%s \n",rcv_buf);

		rxComplt = RESET;

	}

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
	if (ApplicationEvent == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");
	}else if(ApplicationEvent == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
		rxComplt = SET;
	}else if(ApplicationEvent == I2C_ERROR_AF)
	{
		printf("Error : Ack failure\n");
		//In master ack failure happens when slave fails to send ack for the byte
		//sent from the master
		I2C_CloseSendData(pI2CHandler);
		I2C_GenerateStopCondition(pI2CHandler->pI2Cx);

		while(1);
	}
}

