/*
 * 005spi_tx_testing.c
 *
 *  Created on: Nov 20, 2023
 *      Author: thaithinhtran
 */
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

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
	SPIpins.GPIO_Pinconfig.GPIO_pinSpeed		 = GPIO_SPEED_HIGH;

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
//	//NSS
//	SPIpins.GPIO_Pinconfig.GPIO_PinNumber	 	 = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIpins);
}
void SPI2Init()
{
	SPI_Handler_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig		 = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode 	 = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed		 = SPI_SCLK_SPEED_DIV4; //generates sclk of 8Mhz
	SPI2handle.SPIConfig.SPI_DFF			 = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL			 = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA			 = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM			 = SPI_SSM_EN; //Software slave management enbaled for NSS pins

	SPI_Init(&SPI2handle);
}

int main(void)
{
	char user_data[] = "Hello world";

	//This function is used to initialize the GPIO pins to behave as SPI2 pins (include Clock)
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters (include Clock)
	SPI2Init();

	//this make NSS signal internally high and avoids MODF error
	SPI_SSI_Config(SPI2, ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

	//lets confirm SPI is not busy
	while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);
	while(1);
	return 0;
}
