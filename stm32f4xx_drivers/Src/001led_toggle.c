/*
 * 001led_toggle.c
 *
 *  Created on: Oct 22, 2023
 *      Author: thaithinhtran
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
	for (uint32_t t = 0; t <= 1000000 ; t++);
}

int main()
{
	GPIO_Handler_t GpioLed ;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_Pinconfig.GPIO_PinNumber	 	= GPIO_PIN_NO_12;
	GpioLed.GPIO_Pinconfig.GPIO_PinMode		 	= GPIO_MODE_OUT;
	GpioLed.GPIO_Pinconfig.GPIO_pinSpeed	 	= GPIO_SPEED_FAST;
	GpioLed.GPIO_Pinconfig.GPIO_PinOPType	 	= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD , ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD , GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}

