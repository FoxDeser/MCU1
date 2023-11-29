/*
 * 007spi_cmd_handling.c
 *
 *  Created on: Nov 23, 2023
 *      Author: thaithinhtran
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

//Command codes
#define COMMNAND_LED_CRTL		0x50
#define COMMNAND_SENSOR_READ	0x51
#define COMMNAND_LED_READ		0x52
#define COMMNAND_PRINT			0x53
#define COMMNAND_ID_READ		0x54

#define LED_ON					1
#define LED_OFF					0

//arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3

//arduino led
#define LED_PIN					22

extern void initialise_monitor_handles();
void delay(void)
{
	for (uint32_t t = 0; t <= 500000/2 ; t++);
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

	//MISO
	SPIpins.GPIO_Pinconfig.GPIO_PinNumber	 	 = GPIO_PIN_NO_14;
	GPIO_Init(&SPIpins);

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

	GPIO_Init(&GpioBtn);
}

void SPI2Init()
{
	SPI_Handler_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig		 = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode 	 = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed		 = SPI_SCLK_SPEED_DIV32; //generates sclk of 2Mhz
	SPI2handle.SPIConfig.SPI_DFF			 = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL			 = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA			 = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM			 = SPI_SSM_DI; //Hardware slave management enbaled for NSS pins

	SPI_Init(&SPI2handle);
}
uint8_t SPI_VerifyResponse(uint8_t ack_byte)
{
	if(ack_byte == 0xF5)
	{
		return 1;
	}
	return 0;
}
int main(void)
{
	initialise_monitor_handles();
	printf("Application is running \n");
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read	;

	//This function is used to initialize the GPIO pins to behave as SPI2 pins (include Clock)
	SPI2_GPIOInits();

	////This function is used to initialize the GPIO for Button (include Clock)
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

//1. CMD_LED_CTRL pin no(1)	<value(1)>
		printf("Start COMMAND_LED_CTRL\n");
		uint8_t	commandcode	= COMMNAND_LED_CRTL;
		uint8_t ack_byte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandcode, sizeof(commandcode));
		delay();
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, sizeof(dummy_read));

		//send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, sizeof(dummy_write));

		SPI_ReceiveData(SPI2, &ack_byte, sizeof(ack_byte));

		if (SPI_VerifyResponse(ack_byte))
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, sizeof(args));
			SPI_ReceiveData(SPI2, args, sizeof(args));
			printf("COMMAND_LED_CTRL Executed\n");
		}

//2. CMD_SENSOR_READ 		<analog pin number(1)>

		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		printf("Start CMD_SENSOR_READ\n");

		//to avoid button de-boucing realted issues of delay
		delay();

		commandcode	= COMMNAND_SENSOR_READ;
		SPI_SendData(SPI2, &commandcode, sizeof(commandcode));
		delay();
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, sizeof(dummy_read));

		//send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, sizeof(dummy_write));

		SPI_ReceiveData(SPI2, &ack_byte, sizeof(ack_byte));

		if (SPI_VerifyResponse(ack_byte))
		{
			//send arguments
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, sizeof(args[0]));

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, sizeof(dummy_read));

			//insert some delay so that slave can ready with the data
			delay();

			//send some dummy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, sizeof(dummy_write));

			uint8_t	analog_read;
			SPI_ReceiveData(SPI2, &analog_read, sizeof(analog_read));
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}
//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		printf("Start CMD_LED_READ\n");

		//to avoid button de-bouncing related issues of delay
		delay();

		commandcode = COMMNAND_LED_READ;
		//send command
		SPI_SendData(SPI2,&commandcode,sizeof(commandcode));
		delay();
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,sizeof(dummy_read));

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,sizeof(dummy_write));

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ack_byte,sizeof(ack_byte));

		if( SPI_VerifyResponse(ack_byte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2,args,sizeof(args[0])); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,sizeof(dummy_read));

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,sizeof(dummy_write));

			uint8_t led_status;
			SPI_ReceiveData(SPI2,&led_status,sizeof(led_status));
			printf("COMMAND_READ_LED %d\n",led_status);

		}
//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );
		printf("Start CMD_PRINT\n");
		//to avoid button de-bouncing related issues  of delay
		delay();

		commandcode = COMMNAND_PRINT;

		//send command
		SPI_SendData(SPI2,&commandcode,sizeof(commandcode));
		delay();
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,sizeof(dummy_read));

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,sizeof(dummy_write));

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ack_byte,sizeof(ack_byte));

		uint8_t message[] = "Hello ! i am THINH !";
		if( SPI_VerifyResponse(ack_byte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,args,sizeof(args[0])); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,sizeof(dummy_read));

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI2,&message[i],sizeof(message[0]));
				SPI_ReceiveData(SPI2,&dummy_read,sizeof(dummy_read));
			}

			printf("COMMAND_PRINT Executed \n");

		}

//5. CMD_ID_READ
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );
		printf("Start CMD_ID_READ\n");

		//to avoid button de-bouncing related issues of delay
		delay();

		commandcode = COMMNAND_ID_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,sizeof(commandcode));
		delay();
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,sizeof(dummy_read));

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,sizeof(dummy_write));

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ack_byte,sizeof(ack_byte));

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ack_byte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,sizeof(dummy_write));
				SPI_ReceiveData(SPI2,&id[i],sizeof(id[0]));
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
