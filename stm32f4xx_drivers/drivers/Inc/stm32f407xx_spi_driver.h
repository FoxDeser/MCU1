/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Nov 5, 2023
 *      Author: thaithinhtran
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t	 *pSPIx;		//This holds the base address of SPIx peripheral
	SPI_Config_t	 SPIConfig;
}SPI_Handler_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE	0
#define SPI_DEVICE_MODE_MASTER	1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI			0
#define SPI_SSM_EN			1

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG	(1<<SPI_SR_TXE_Pos)
#define SPI_RXNE_FLAG	(1<<SPI_SR_RXNE_Pos)
#define SPI_BUSY_FLAG	(1<<SPI_SR_BSY_Pos)

/*****************************************************************************************
 * 						APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 *****************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handler_t *pSPIHandler);
void SPI_DeInit(SPI_Handler_t *pSPIHandler);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

void SPI_SendDataIT(SPI_Handler_t *pSPIHandler, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveDataIT(SPI_Handler_t *pSPIHandler, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuratuon and ISR handling
 */
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQpriority);
void SPI_IRQHandling (SPI_Handler_t *pSPIHandler);

/*
 * Other Peripheral Control APIS
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDI);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FLAG_NAME);
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDI);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDI);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
