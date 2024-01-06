/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Dec 3, 2023
 *      Author: thaithinhtran
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*****************************************************************************************
 * 						I2C configuration and handler struct
 * 						I2C user configuration
 *****************************************************************************************/
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
    uint8_t          *pTxBuffer; //To store the app. Tx buffer address
    uint8_t          *pRxBuffer; //To store the app. Rx buffer address
    uint32_t         TxLen;      //To store Tx len
    uint32_t         RxLen;      //To store Rx len
    uint8_t          TxRxState;  //To store the communication state
    uint8_t          DevAddr;    //To store slave/device address
    uint8_t          RxSize;     //To store Rx size
    uint8_t          Sr;         //To store repeated start value  
}I2C_Handler_t;

/*
 * I2C application state
*/
#define I2C_READY          0
#define I2C_BUSY_IN_RX     1
#define I2C_BUSY_IN_TX     2

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
 * 		I2C related status flags definitions (in SR register)
 *****************************************************************************************/
/*
 * SR1 register
 */
#define I2C_FLAG_SB			(1<<I2C_SR1_SB_Pos)
#define I2C_FLAG_ADDR		(1<<I2C_SR1_ADDR_Pos)
#define I2C_FLAG_BTF		(1<<I2C_SR1_BTF_Pos)
#define I2C_FLAG_ADD10		(1<<I2C_SR1_ADD10_Pos)
#define I2C_FLAG_STOPF		(1<<I2C_SR1_STOPF_Pos)
#define I2C_FLAG_RxNE		(1<<I2C_SR1_RxNE_Pos)
#define I2C_FLAG_TxE		(1<<I2C_SR1_TxE_Pos)
#define I2C_FLAG_BERR		(1<<I2C_SR1_BERR_Pos)
#define I2C_FLAG_ARLO		(1<<I2C_SR1_ARLO_Pos)
#define I2C_FLAG_AF			(1<<I2C_SR1_AF_Pos)
#define I2C_FLAG_OVR		(1<<I2C_SR1_OVR_Pos)
#define I2C_FLAG_PECERR		(1<<I2C_SR1_PECERR_Pos)
#define I2C_FLAG_TIMEOUT	(1<<I2C_SR1_TIMEOUT_Pos)
#define I2C_FLAG_SMALERT	(1<<I2C_SR1_SMBALERT_Pos)

/*
 * SR2 register
 */
#define I2C_FLAG_MSL			(1<<I2C_SR2_MSL_Pos)
#define I2C_FLAG_BUSY			(1<<I2C_SR2_BUSY_Pos)
#define I2C_FLAG_TRA			(1<<I2C_SR2_TRA_Pos)
#define I2C_FLAG_GENCALL		(1<<I2C_SR2_GENCALL_Pos)
#define I2C_FLAG_SMBDEFAULT		(1<<I2C_SR2_SMBDEFAULT_Pos)
#define I2C_FLAG_SMBHOST		(1<<I2C_SR2_SMBHOST_Pos)
#define I2C_FLAG_DUALF			(1<<I2C_SR2_DUALF_Pos)

#define I2C_DISABLE_SR	RESET
#define I2C_ENABLE_SR	SET

/*
 * Possible SPI Application events
 */
#define I2C_EV_TX_CMPLT 	0
#define I2C_EV_STOP			1
#define I2C_EV_RX_CMPLT		2

#define I2C_ERROR_BERR		3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7

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
void I2C_MasterSendData(I2C_Handler_t *pI2CHandler,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterRecieveData(I2C_Handler_t *pI2CHandler,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handler_t *pI2CHandler,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handler_t *pI2CHandler,uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);

/*
 * IRQ Configuratuon and ISR handling
 */
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQpriority);
void I2C_EV_IRQHandling(I2C_Handler_t *pI2CHandler);
void I2C_ER_IRQHandling(I2C_Handler_t *pI2CHandler);

/*
 * Other Peripheral Control APIS
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDI);
uint8_t I2C_GetFlagStatusSR1(I2C_RegDef_t *pI2Cx, uint32_t FLAG_NAME);
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx,uint8_t EnorDi);

void I2C_CloseSendData(I2C_Handler_t* pI2C_Handler);
void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);
/*
 * Application call back
 */
void I2C_ApplicationEventCallback(I2C_Handler_t *pI2CHandler,uint8_t ApplicationEvent);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
