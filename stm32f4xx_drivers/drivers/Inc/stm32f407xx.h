/*
 * stm32f407xx.h
 *
 *  Created on: Oct 10, 2023
 *      Author: thaithinhtran
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo   volatile
#define _weak __attribute__((weak))

/************************************START:Processor Specific Details**********************
 *
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define 	NVIC_ISER0		((__vo uint32_t*)0xE000E100)
#define 	NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define 	NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define 	NVIC_ISER3		((__vo uint32_t*)0xE000E10C)
#define 	NVIC_ISER4		((__vo uint32_t*)0xE000E110)
#define 	NVIC_ISER5		((__vo uint32_t*)0xE000E114)
#define 	NVIC_ISER6		((__vo uint32_t*)0xE000E118)
#define 	NVIC_ISER7		((__vo uint32_t*)0xE000E11C)
 /*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define 	NVIC_ICER0		((__vo uint32_t*)0XE000E180)
#define 	NVIC_ICER1		((__vo uint32_t*)0XE000E184)
#define 	NVIC_ICER2		((__vo uint32_t*)0XE000E188)
#define 	NVIC_ICER3		((__vo uint32_t*)0XE000E18C)
#define 	NVIC_ICER4		((__vo uint32_t*)0XE000E190)
#define 	NVIC_ICER5		((__vo uint32_t*)0XE000E194)
#define 	NVIC_ICER6		((__vo uint32_t*)0XE000E198)
#define 	NVIC_ICER7		((__vo uint32_t*)0XE000E19C)

 /*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define 	NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)
#define 	NO_PR_BITS_IMPLEMENTED	4

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
 */
#define SPI1_BASEADDR            (APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASEADDR            (APB2PERIPH_BASE + 0x3400U)

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
  __vo uint32_t OSPEEDR;	//GPIO port output speed register          Address offset 0x08
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
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t	EXTI_IMR;	//Interrupt mask register
	__vo uint32_t	EXTI_EMR;	//Event mask register
	__vo uint32_t	EXTI_RTSR;	//Rising trigger selection register
	__vo uint32_t	EXTI_FTSR;	//Falling trigger selection register
	__vo uint32_t	EXTI_SWIER;	//Software interrupt event register
	__vo uint32_t	EXTI_PR;	//Pending register
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t 	MEMRMP;		 //SYSCFG memory remap register
	__vo uint32_t	PMC;		 //SYSCFG peripheral mode configuration register
	__vo uint32_t	EXTICR[4];	 //SYSCFG external interrupt configuration register 1234
	uint32_t 		RESERVED[2];
	__vo uint32_t	CMPCR;		 //Compensation cell control register
}SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t	CR1;		//SPI control register 1
	__vo uint32_t	CR2;		//SPI control register 2
	__vo uint32_t	SR;			//SPI status register
	__vo uint32_t	DR;			//SPI data register
	__vo uint32_t	CRCPR;		//SPI CRC polynomial register
	__vo uint32_t	RXCRCR;		//SPI RX CRC register
	__vo uint32_t	TXCRCR;		//SPI TX CRC register
	__vo uint32_t	I2SCFGR;		//SPI_I2S configuration register
	__vo uint32_t	I2SPR;			//SPI_I2S prescaler register
}SPI_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
     __vo uint32_t CR1;       //I2C Control register 1       0x00
     __vo uint32_t CR2;       //I2C Control register 2       0x04
     __vo uint32_t OAR1;      //I2C Own address register 1   0x08
     __vo uint32_t OAR2;      //I2C Own address register 2   0x0C
     __vo uint32_t DR;        //I2C Data register            0x10
     __vo uint32_t SR1;       //I2C Status register 1        0x14
     __vo uint32_t SR2;       //I2C Status register 1        0x18
     __vo uint32_t CCR;       //I2C Clock control register   0x1C
     __vo uint32_t TRISE;     //I2C TRISE register           0x20
     __vo uint32_t FLTR;      //I2C FLTR register            0x24
}I2C_RegDef_t;


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
 * SPI definition (SPI base addresses type casted to SPI_RegDef_t)
 */
#define SPI1	  ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	  ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	  ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4	  ((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * I2C definition (I2C base addresses type casted to I2C_RegDef_t)
 */
#define I2C1        ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2        ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3        ((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * RCC definition (RCC base addresses type casted to RCC_RegDef_t)
 */
#define RCC       ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI	  ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG 	  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

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
 * Return port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x) 	((x == GPIOA) ? 0 :\
									(x == GPIOB)? 1 :\
									(x == GPIOC)? 2 :\
									(x == GPIOD)? 3 :\
									(x == GPIOE)? 4 :\
									(x == GPIOF)? 5 :\
									(x == GPIOG)? 6 :\
									(x == GPIOH)? 7 :8)

/*
 * IRQ(Interrupt request) Numbers of STM32F407x MCU
 * NOTE : update these macros with valid values according to your MCU
 */
#define	IRQ_NO_EXTI0		6
#define	IRQ_NO_EXTI1		7
#define	IRQ_NO_EXTI2		8
#define	IRQ_NO_EXTI3		9
#define	IRQ_NO_EXTI4		10
#define	IRQ_NO_EXTI9_5		23
#define	IRQ_NO_EXTI015_10	40
#define IRQ_NO_SPI1         35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_I2C2_EV      33
#define IRQ_NO_I2C2_ER      34
#define IRQ_NO_I2C3_EV      72
#define IRQ_NO_I2C3_ER      73

#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15

/*
 * Some others macros
 */
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/*
 *********************************************
 *Bit position definition of SPI peripheral***
 *********************************************
 */
// SPI_CR1 Register
#define		 SPI_CR1_CPHA_Pos			0
#define		 SPI_CR1_CPOL_Pos			1
#define		 SPI_CR1_MSTR_Pos			2
#define		 SPI_CR1_BR_Pos				3
#define		 SPI_CR1_SPE_Pos			6
#define		 SPI_CR1_LSBFIRST_Pos		7
#define		 SPI_CR1_SSI_Pos			8
#define		 SPI_CR1_SSM_Pos			9
#define		 SPI_CR1_RXONLY_Pos			10
#define		 SPI_CR1_DFF_Pos			11
#define		 SPI_CR1_CRCNEXT_Pos		12
#define		 SPI_CR1_CRCEN_Pos			13
#define		 SPI_CR1_BIDIOE_Pos			14
#define		 SPI_CR1_BIDIMODE_Pos		15

// SPI_CR2 Register
#define		 SPI_CR2_RXDMAEN_Pos			0
#define		 SPI_CR2_TXDMAEN_Pos			1
#define		 SPI_CR2_SSOE_Pos				2
#define		 SPI_CR2_FRF_Pos				4
#define		 SPI_CR2_ERRIE_Pos				5
#define		 SPI_CR2_RXNEIE_Pos				6
#define		 SPI_CR2_TXEIE_Pos				7

// SPI_SR Register
#define		 SPI_SR_RXNE_Pos				0
#define		 SPI_SR_TXE_Pos					1
#define		 SPI_SR_CHSIDE_Pos				2
#define		 SPI_SR_UDR_Pos					3
#define		 SPI_SR_CRCERR_Pos				4
#define		 SPI_SR_MODF_Pos				5
#define		 SPI_SR_OVR_Pos					6
#define		 SPI_SR_BSY_Pos					7
#define		 SPI_SR_FRE_Pos					8

/*
 *********************************************
 *Bit position definition of I2C peripheral***
 *********************************************
 */
// I2C Control register 1 (I2C_CR1)
#define        I2C_CR1_PE_Pos                0                
#define        I2C_CR1_SMBUS_Pos             1                
#define        I2C_CR1_SMBTYPE_Pos           3                
#define        I2C_CR1_ENARP_Pos             4                
#define        I2C_CR1_ENPEC_Pos             5                
#define        I2C_CR1_ENGC_Pos              6                
#define        I2C_CR1_NOSTRETCH_Pos         7                
#define        I2C_CR1_START_Pos             8                
#define        I2C_CR1_STOP_Pos              9                
#define        I2C_CR1_ACK_Pos               10                
#define        I2C_CR1_POS_Pos               11                
#define        I2C_CR1_PEC_Pos               12                
#define        I2C_CR1_ALERT_Pos             13                
#define        I2C_CR1_SWRST_Pos             15

// I2C Control register 2 (I2C_CR2)
#define        I2C_CR2_FREQ_Pos              0
#define        I2C_CR2_ITERREN_Pos           8
#define        I2C_CR2_ITEVTEN_Pos           9
#define        I2C_CR2_ITBUFEN_Pos           10
#define        I2C_CR2_DMAEN_Pos             11
#define        I2C_CR2_LAST_Pos              12

// I2C Status register 1 (I2C_SR1)
#define        I2C_SR1_SB_Pos                0
#define        I2C_SR1_ADDR_Pos              1
#define        I2C_SR1_BTF_Pos               2
#define        I2C_SR1_ADD10_Pos             3
#define        I2C_SR1_STOPF_Pos             4
#define        I2C_SR1_RxNE_Pos              6
#define        I2C_SR1_TxE_Pos               7
#define        I2C_SR1_BERR_Pos              8
#define        I2C_SR1_ARLO_Pos              9
#define        I2C_SR1_AF_Pos                10
#define        I2C_SR1_OVR_Pos               11
#define        I2C_SR1_PECERR_Pos            12
#define        I2C_SR1_TIMEOUT_Pos           14
#define        I2C_SR1_SMBALERT_Pos          15

// I2C Status register 2 (I2C_SR2)
#define        I2C_SR2_MSL_Pos               0
#define        I2C_SR2_BUSY_Pos              1
#define        I2C_SR2_TRA_Pos               2
#define        I2C_SR2_GENCALL_Pos           4
#define        I2C_SR2_SMBDEFAULT_Pos        5
#define        I2C_SR2_SMBHOST_Pos           6
#define        I2C_SR2_DUALF_Pos             7
#define        I2C_SR2_PEC_Pos               8

//I2C Clock control register (I2C_CCR)
#define        I2C_CCR_CCR_Pos               0
#define        I2C_CCR_DUTY_Pos              14
#define        I2C_CCR_FS                    15

#endif	/* STM32F407XX_H_ */
