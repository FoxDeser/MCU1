/*
 * 006spi_txonly_arduino.c
 *
 *  Created on: Nov 21, 2023
 *      Author: thaithinhtran
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

void delay(void)
{
	for (uint32_t t = 0; t <= 500000 ; t++);
}

/*
 * PB14 ---> SPI2_MISO
 * PB15 ---> SPI2_MOSI
 * PB13 ---> SPI2_SCLK
 * PB12 ---> SPI2_NSS
 * ALT function mode : 5
 */
void SPI2_GPIOInits()
{
	GPIO_Handler_t SPIpins;
	SPIpins.pGPIOx = GPIOB;
	SPIpins.GPIO_Pinconfig.GPIO_PinMode			 = GPIO_MODE_ALTFN;
	SPIpins.GPIO_Pinconfig.GPIO_PinAltMode		 = 5;
	SPIpins.GPIO_Pinconfig.GPIO_PinOPType		 = GPIO_OP_TYPE_PP;
	SPIpins.GPIO_Pinconfig.GPIO_PinPuPdControl   = GPIO_NO_PUPD;
	SPIpins.GPIO_Pinconfig.GPIO_pinSpeed		 = GPIO_SPEED_FAST;

	//SCLK
	SPIpins.GPIO_Pinconfig.GPIO_PinNumber	 	 = GPIO_PIN_NO_13;
	GPIO_Init(&SPIpins);

	//MOSI
	SPIpins.GPIO_Pinconfig.GPIO_PinNumber	 	 = GPIO_PIN_NO_15;
	GPIO_Init(&SPIpins);

//	//MISO
//	SPIpins.GPIO_Pinconfig.GPIO_PinNumber	 	 = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIpins);
//
	//NSS
	SPIpins.GPIO_Pinconfig.GPIO_PinNumber	 	 = GPIO_PIN_NO_12;
	GPIO_Init(&SPIpins);
}

void GPIO_ButtonInit()
{
	GPIO_Handler_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_Pinconfig.GPIO_PinNumber	 	= GPIO_PIN_NO_0;
	GpioBtn.GPIO_Pinconfig.GPIO_PinMode		 	= GPIO_MODE_IN;
	GpioBtn.GPIO_Pinconfig.GPIO_pinSpeed	 	= GPIO_SPEED_FAST;
	GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA , ENABLE);
	GPIO_Init(&GpioBtn);
}

void SPI2Init()
{
	SPI_Handler_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig		 = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode 	 = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed		 = SPI_SCLK_SPEED_DIV8; //generates sclk of 2Mhz
	SPI2handle.SPIConfig.SPI_DFF			 = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL			 = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA			 = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM			 = SPI_SSM_DI; //Hardware slave management enbaled for NSS pins

	SPI_Init(&SPI2handle);
}

int main(void)
{
	char user_data[] = "Hello worldm";

	//This function is used to initialize the GPIO pins to behave as SPI2 pins (include Clock)
	SPI2_GPIOInits();

	////This function is used to initialize the GPIO for Button
	GPIO_ButtonInit();

	//This function is used to initialize the SPI2 peripheral parameters (include Clock)
	SPI2Init();

	/*
	 * Making SSOE 1 does NSS output Enable.
	 * The NSS pin is automatically managed by the hardware
	 * i.e When SPE = 1 , NSS will be pulled to low
	 * and NSS pin will be high when SPI =0
	 */
	SPI_SSOE_Config(SPI2, ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-boucing realted issues of delay
		delay();

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//First send length information
		uint8_t	dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, sizeof(uint8_t));

		//To send the data
		SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}

