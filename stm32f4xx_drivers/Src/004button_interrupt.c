/*
 * 004button_interrupt.c
 *
 *  Created on: Oct 28, 2023
 *      Author: thaithinhtran
 */
#include "string.h"
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define BTN_PRESSED 0

void delay(void)
{
	for (uint32_t t = 0; t <= 500000/2 ; t++);
}

int main()
{
	GPIO_Handler_t GpioLed , GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_Pinconfig.GPIO_PinNumber	 	= GPIO_PIN_NO_12;
	GpioLed.GPIO_Pinconfig.GPIO_PinMode		 	= GPIO_MODE_OUT;
	GpioLed.GPIO_Pinconfig.GPIO_pinSpeed	 	= GPIO_SPEED_FAST;
	GpioLed.GPIO_Pinconfig.GPIO_PinOPType	 	= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pinconfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD , ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_Pinconfig.GPIO_PinNumber	 	= GPIO_PIN_NO_5;
	GpioBtn.GPIO_Pinconfig.GPIO_PinMode		 	= GPIO_MODE_IT_FT;
	GpioBtn.GPIO_Pinconfig.GPIO_pinSpeed	 	= GPIO_SPEED_FAST;
//	GpioBtn.GPIO_Pinconfig.GPIO_PinOPType	 	= GPIO_OP_TYPE_OD;
	GpioBtn.GPIO_Pinconfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;

	GPIO_Init(&GpioBtn);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15); //Priority any from 0 -> 15
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

	return 0;
}
void EXTI9_5_IRQHandler(void)
{
	delay(); //400ms
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}
