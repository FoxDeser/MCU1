/*
 * stm32f407xx.h
 *
 *  Created on: Oct 10, 2023
 *      Author: thaithinhtran
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include <stdint.h>

#define __vo   volatile

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR 			  0x08000000U	//or (uint32_t) 0x08000000
#define SRAM1_BASEADDR 			  0x20000000U	//112KB to hex is 1C000 (x1024 covert hex)
#define SRAM2_BASEADDR 		      0x20001C00U
#define ROM 		     		  0x1FFF0000U	//System memory
#define RAM 			 		  SRAM1_BASEADDR

  /*
   * AHx and APBx Bus Peripheral base addresses
   */
#define PERIPH_BASE       		 0x40000000U
#define APB1PERIPH_BASE 		 PERIPH_BASE
#define APB2PERIPH_BASE			 0x40010000U
#define AHB1PERIPH_BASE          0x40020000U
#define AHB2PERIPH_BASE          0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */
#define GPIOA_BASEADDR           (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR 			 (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR 			 (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR 			 (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR 			 (AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR 			 (AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR 			 (AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR 			 (AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR 			 (AHB1PERIPH_BASE + 0x2000U)

#define RCC_BASEADDR             (AHB1PERIPH_BASE + 0x3800U)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR    		 (APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR			 (APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR 			 (APB1PERIPH_BASE + 0x5C00U)

#define SPI2_BASEADDR 			 (APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR 			 (APB1PERIPH_BASE + 0x3C00U)

#define USART2_BASEADDR   		 (APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR 	     (APB1PERIPH_BASE + 0x4800U)

#define UART4_BASEADDR           (APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR           (APB1PERIPH_BASE + 0x5000U)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define SPI1_BASEADDR            (APB2PERIPH_BASE + 0x3000U)

#define USART1_BASEADDR 	     (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR 	     (APB2PERIPH_BASE + 0x1400U)

#define EXTI_BASEADDR 			 (APB2PERIPH_BASE + 0x3C00U)
#define SYSCFG_BASEADDR 		 (APB2PERIPH_BASE + 0x3800U)

/*
 ****************peripheral register definition structures******************************************
 *Note : Register of a peripheral are specific to MCU
 *eg : Number of register of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 *Compared to number of register of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 *Please check the your Device RM
 */
typedef struct
{


  __vo uint32_t MODER;		//GPIO port mode register                  Address offset 0x00
  __vo uint32_t OTYPER;		//GPIO port output type register           Address offset 0x04
  __vo uint32_t OSPEEDR;		//GPIO port output speed register          Address offset 0x08
  __vo uint32_t PUPDR;		//GPIO port pull-up/pull-down register     Address offset 0x0C
  __vo uint32_t IDR;		//GPIO port input data register            Address offset 0x10
  __vo uint32_t ODR;		//GPIO port output data register           Address offset 0x14
  __vo uint32_t BSRR;		//GPIO port bit set/reset register         Address offset 0x18
  __vo uint32_t LCKR;		//GPIO port configuration lock register    Address offset 0x1C
  __vo uint32_t AFR[2];		//GPIO alternate function register         Address offset 0x20
} GPIO_RegDef_t;


/*
 * peripheral register definition structure for RCC
 */
typedef struct
{

  __vo uint32_t RCC_CR;       	//RCC clock control register
  __vo uint32_t RCC_PLLCFGR;	//RCC PLL configuration register
  __vo uint32_t RCC_CFGR;   	//RCC clock configuration register
  __vo uint32_t CIR;	    	//RCC clock interrupt register
  __vo uint32_t RCC_AHB1RSTR;	//RCC AHB1 peripheral reset register
  __vo uint32_t RCC_AHB2RSTR;	//RCC AHB2 peripheral reset register
  __vo uint32_t RCC_AHB3RSTR;	//RCC AHB3 peripheral reset register
       uint32_t RESERVED0;

  __vo uint32_t RCC_APB1RSTR;	//RCC APB1 peripheral reset register
  __vo uint32_t RCC_APB2RSTR;	//RCC APB2 peripheral reset register
       uint32_t RESERVED1[2];

  __vo uint32_t RCC_AHB1ENR;	//RCC AHB1 peripheral clock enable register
  __vo uint32_t RCC_AHB2ENR;	//RCC AHB2 peripheral clock enable register
  __vo uint32_t RCC_AHB3ENR;	//RCC AHB3 peripheral clock enable register
       uint32_t RESERVED2;

  __vo uint32_t RCC_APB1ENR;	//RCC APB1 peripheral clock enable register
  __vo uint32_t RCC_APB2ENR;	//RCC APB2 peripheral clock enable register
       uint32_t RESERVED3[2];

  __vo uint32_t RCC_AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register
  __vo uint32_t RCC_AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register
  __vo uint32_t RCC_AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register
       uint32_t RESERVED4;

  __vo uint32_t RCC_APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register
  __vo uint32_t RCC_APB2LPENR;	//RCC APB2 peripheral clock enabled in low power mode register
       uint32_t RESERVED5[2];

  __vo uint32_t RCC_BDCR;   	//RCC Backup domain control register
  __vo uint32_t RCC_CSR;    	//RCC clock control & status register
       uint32_t RESERVED6[2];

  __vo uint32_t RCC_SSCGR;  	//RCC spread spectrum clock generation register
  __vo uint32_t RCC_PLLI2SCFGR;	//RCC PLLI2S configuration register
  __vo uint32_t RCC_PLLSAICFGR;	//RCC PLL configuration register
  __vo uint32_t RCC_DCKCFGR;	//RCC Dedicated Clock Configuration Register
} RCC_RegDef_t;


/*
 * peripheral definition (Peripheral base addresses type casted to xxx_RegDef_t)
 */
#define GPIOA     ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB     ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC     ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD     ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE     ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF     ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG     ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH     ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI     ((GPIO_RegDef_t*)GPIOI_BASEADDR)

/*
 * RCC definition (RCC base addresses type casted to RCC_RegDef_t)
 */
#define RCC       ((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<0))   //Remember PCLK is Peripheral Clock, EN is Enbale , DI is Disable
#define GPIOB_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<1))
#define GPIOC_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<2))
#define GPIOD_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<3))
#define GPIOE_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<4))
#define GPIOF_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<5))
#define GPIOG_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<6))
#define GPIOH_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<7))
#define GPIOI_PCLK_EN() ((RCC->RCC_AHB1ENR) |= (1<<8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()  ((RCC->RCC_APB1ENR) |= (1<<21))
#define I2C2_PCLK_EN()  ((RCC->RCC_APB1ENR) |= (1<<22))
#define I2C3_PCLK_EN()  ((RCC->RCC_APB1ENR) |= (1<<23))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()	((RCC->RCC_APB2ENR) |= (1<<12))
#define SPI2_PCLK_EN()	((RCC->RCC_APB1ENR) |= (1<<14))
#define SPI3_PCLK_EN()	((RCC->RCC_APB1ENR) |= (1<<15))
#define SPI4_PCLK_EN()	((RCC->RCC_APB2ENR) |= (1<<13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() ((RCC->RCC_APB2ENR) |= (1<<4))
#define USART6_PCLK_EN() ((RCC->RCC_APB2ENR) |= (1<<5))
#define USART2_PCLK_EN() ((RCC->RCC_APB1ENR) |= (1<<17))
#define USART3_PCLK_EN() ((RCC->RCC_APB1ENR) |= (1<<18))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN() ((RCC->RCC_APB2ENR) |= (1<<14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<0))
#define GPIOB_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<1))
#define GPIOC_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<2))
#define GPIOD_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<3))
#define GPIOE_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<4))
#define GPIOF_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<5))
#define GPIOG_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<6))
#define GPIOH_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<7))
#define GPIOI_PCLK_DI() ((RCC->RCC_AHB1ENR) &= ~(1<<8))

/*
 * Clock Disable Marcos for I2Cx peripherals
 */
#define I2C1_PCLK_DI()  ((RCC->RCC_APB1ENR) &= ~(1<<21))
#define I2C2_PCLK_DI()  ((RCC->RCC_APB1ENR) &= ~(1<<22))
#define I2C3_PCLK_DI()  ((RCC->RCC_APB1ENR) &= ~(1<<23))

/*
 * Clock Disable Marcos for SPIx peripherals
 */
#define SPI1_PCLK_DI()	((RCC->RCC_APB2ENR) &= ~(1<<12))
#define SPI2_PCLK_DI()	((RCC->RCC_APB1ENR) &= ~(1<<14))
#define SPI3_PCLK_DI()	((RCC->RCC_APB1ENR) &= ~(1<<15))
#define SPI4_PCLK_DI()	((RCC->RCC_APB2ENR) &= ~(1<<13))

/*
 * Clock Disable Marcos for USARTx peripherals
 */
#define USART1_PCLK_DI() ((RCC->RCC_APB2ENR) &= ~(1<<4))
#define USART6_PCLK_DI() ((RCC->RCC_APB2ENR) &= ~(1<<5))
#define USART2_PCLK_DI() ((RCC->RCC_APB1ENR) &= ~(1<<17))
#define USART3_PCLK_DI() ((RCC->RCC_APB1ENR) &= ~(1<<18))

/*
 * Clock Disable Marcos for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI() ((RCC->RCC_APB2ENR) &= ~(1<<14))

/*
 * Macros to seset GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<0);  (RCC->RCC_AHB1RSTR) |= ~(1<<0);} while(0)
#define GPIOB_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<1);  (RCC->RCC_AHB1RSTR) |= ~(1<<1);} while(0)
#define GPIOC_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<2);  (RCC->RCC_AHB1RSTR) |= ~(1<<2);} while(0)
#define GPIOD_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<3);  (RCC->RCC_AHB1RSTR) |= ~(1<<3);} while(0)
#define GPIOE_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<4);  (RCC->RCC_AHB1RSTR) |= ~(1<<4);} while(0)
#define GPIOF_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<5);  (RCC->RCC_AHB1RSTR) |= ~(1<<5);} while(0)
#define GPIOG_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<6);  (RCC->RCC_AHB1RSTR) |= ~(1<<6);} while(0)
#define GPIOH_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<7);  (RCC->RCC_AHB1RSTR) |= ~(1<<7);} while(0)
#define GPIOI_REG_RESET()		do{	(RCC->RCC_AHB1RSTR) |= (1<<8);  (RCC->RCC_AHB1RSTR) |= ~(1<<8);} while(0)

/*
 * Some others macros
 */
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

#endif	/* STM32F407XX_H_ */
