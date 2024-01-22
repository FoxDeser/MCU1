/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Jan 21, 2024
 *      Author: thaithinhtran
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

/*****************************************************************************************
 * 						USART configuration and handler struct                           *
 * 						USART user configuration                                         *
 ****************************************************************************************/
/*
 * This is a Configuration structure for USARTx peripheral
 */
typedef struct
{
    uint8_t  USART_Mode;
    uint32_t USART_Baud;
    uint8_t  USART_NoOfStopBits;
    uint8_t  USART_WordLength;
    uint8_t  USART_ParityControl;
    uint8_t  USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle structure for USARTx peripheral
 */
typedef struct 
{
    USART_RegDef_t  *pUSARTx;
    USART_Config_t    USART_Config;
}USART_Handle_t;


#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
