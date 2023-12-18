/*
 * 009i2c_master_tx_testing.c
 *
 *  Created on: Dec 9, 2023
 *      Author: thaithinhtran
 */


#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"

#define MY_ADDR		 0x24
#define SLAVEADDR	 0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handler_t I2C1Handler;

//Some data
uint8_t some_data[] = "Hi,Thinh from rvc:)).\n";

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
	//I2C Pin inits
	I2C1_GPIOInits();

	//Button inits
	GPIO_ButtonInit();

	//I2C peripheral configuration
	I2C1_Inits();

	//Enable the I2C peripheral
	I2C_PeripheralControl(I2C1Handler.pI2Cx, ENABLE);

	while(1)
	{
		//Wait for the Button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//To avoid button de-boucing, related issues 200ms of delay
		delay();

		//Send some data to the Slave
		I2C_MasterSendData(&I2C1Handler, some_data, sizeof(some_data), SLAVEADDR);
	}
	while(1);

	return 0;
}
