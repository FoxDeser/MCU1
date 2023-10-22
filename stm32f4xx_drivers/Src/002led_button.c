/*
 * 002led_button.c
 *
 *  Created on: Oct 22, 2023
 *      Author: thaithinhtran
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define BTN_PRESSED 1

void delay(void)
{
	for (uint32_t t = 0; t <= 500000 ; t++);
}

int main()
{
	GPIO_Handler_t GpioLed , GpioBtn;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_Pinconfig.GPIO_PinNumber	 	= GPIO_PIN_NO_12;
	GpioLed.GPIO_Pinconfig.GPIO_PinMode		 	= GPIO_MODE_OUT;
	GpioLed.GPIO_Pinconfig.GPIO_pinSpeed	 	= GPIO_SPEED_FAST;
	GpioLed.GPIO_Pinconfig.GPIO_PinOPType	 	= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD , ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_Pinconfig.GPIO_PinNumber	 	= GPIO_PIN_NO_0;
	GpioBtn.GPIO_Pinconfig.GPIO_PinMode		 	= GPIO_MODE_IN;
	GpioBtn.GPIO_Pinconfig.GPIO_pinSpeed	 	= GPIO_SPEED_FAST;
//	GpioBtn.GPIO_Pinconfig.GPIO_PinOPType	 	= GPIO_OP_TYPE_OD;
	GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA , ENABLE);
	GPIO_Init(&GpioBtn);
	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD , GPIO_PIN_NO_12);
		}
	}
	return 0;
}
