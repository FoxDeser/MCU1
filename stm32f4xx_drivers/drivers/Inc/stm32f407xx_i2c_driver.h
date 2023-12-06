/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Dec 3, 2023
 *      Author: thaithinhtran
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t    I2C_SCLSpeed;
    uint8_t     I2C_DeviceAddress;
    uint8_t     I2C_ACKControl;
    uint16_t    I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t*	 pI2Cx;		//This holds the base address of I2Cx peripheral
	I2C_Config_t	 I2CConfig;
}I2C_Handler_t;

/*
 * @I2C_SCLSpeed
*/
#define I2C_SCL_SPEED_SM    100000
#define I2C_SCL_SPEED_FM4K  400000
#define I2C_SCL_SPEED_FM2K  200000

/*
 * @I2C_ACKControl
*/
#define I2C_ACK_ENABLE      1      
#define I2C_ACK_DISABLE     0

/*
 * @I2C_FMDutyCycle
*/
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

/*****************************************************************************************
 * 						APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 *****************************************************************************************/
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handler_t *pI2CHandler);
void I2C_DeInit(I2C_Handler_t *pI2CHandler);

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handler_t *pI2CHandler,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr);

/*
 * IRQ Configuratuon and ISR handling
 */
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQpriority);

/*
 * Other Peripheral Control APIS
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDI);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FLAG_NAME);

/*
 * Application call back
 */
void I2C_ApplicationEventCallback(I2C_Handler_t *pI2CHandler,uint8_t ApplicationEvent);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
