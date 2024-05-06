/*
 * stm31fxxhal.h
 *
 *  Created on: Mar 2, 2023
 *      Author: if420_01
 */

#ifndef STM32FXXHAL_H_
#define STM32FXXHAL_H_

#include <stdint.h>
#include <stddef.h>

#define HSI_CLOCK_SPEED  16000000
#define HSE_CLOCK_SPEED  4000000

#define NOP() asm("NOP")
#define __weak	__attribute__((weak))

#define FLASH_BASE_ADDR		0x08000000U
#define SRAM_BASE_ADDR		0x20000000U

#define APB1_BASE_ADDR		0x40000000U
#define APB2_BASE_ADDR		0x40010000U
#define AHB1_BASE_ADDR		0x40020000U
#define AHB2_BASE_ADDR		0x50000000U

#define USB_OTG_FS_BASE_ADDR		(AHB2_BASE_ADDR + 0x0000U)

#define RCC_BASE_ADDR		(AHB1_BASE_ADDR + 0x3800U)
#define GPIOH_BASE_ADDR		(AHB1_BASE_ADDR + 0x1C00U)
#define GPIOE_BASE_ADDR		(AHB1_BASE_ADDR + 0x1000U)
#define GPIOD_BASE_ADDR		(AHB1_BASE_ADDR + 0x0C00U)
#define GPIOC_BASE_ADDR		(AHB1_BASE_ADDR + 0x0800U)
#define GPIOB_BASE_ADDR		(AHB1_BASE_ADDR + 0x0400U)
#define GPIOA_BASE_ADDR		(AHB1_BASE_ADDR + 0x0000U)


#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define CLEAR			DISABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET	
#define FLAG_RESET		RESET
#define I2C_WIRE		0
#define I2C_READ		1

typedef struct
{	
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t reserved0;
	volatile uint32_t reserved1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t reserved2;
	volatile uint32_t reserved3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t reserved4;
	volatile uint32_t reserved5;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t reserved6;
	volatile uint32_t reserved7;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t reserved8;
	volatile uint32_t reserved9;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t reserved10;
	volatile uint32_t reserved11;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t reserved12;
	volatile uint32_t reserved13;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t reserved14;
	volatile uint32_t DCKCFGR;
} RCC_RegDef_t;

#define RCC		((RCC_RegDef_t *) RCC_BASE_ADDR)

#define RCC_AHB1ENR_GPIOA_EN		0
#define RCC_AHB1ENR_GPIOB_EN		1
#define RCC_AHB1ENR_GPIOC_EN		2
#define RCC_AHB1ENR_GPIOD_EN		3
#define RCC_AHB1ENR_GPIOE_EN		4
#define RCC_AHB1ENR_GPIOH_EN		7
#define RCC_AHB1ENR_CRCEN			12
#define RCC_AHB1ENR_DMA1_EN			21
#define RCC_AHB1ENR_DMA2_EN			22

typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
	
} GPIOx_RegDef_t;

#define GPIOA		((GPIOx_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB		((GPIOx_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC		((GPIOx_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD		((GPIOx_RegDef_t *) GPIOD_BASE_ADDR)
#define GPIOE		((GPIOx_RegDef_t *) GPIOE_BASE_ADDR)
#define GPIOH		((GPIOx_RegDef_t *) GPIOH_BASE_ADDR)

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3

#define GPIO_OTYPE_PUSHPULL		0
#define GPIO_OTYPE_OPENDRAIN	1

#define GPIO_OSPEEDR_LOW		0
#define GPIO_OSPEEDR_MEDIUM		1
#define GPIO_OSPEEDR_FAST		2
#define GPIO_OSPEEDR_HIGH		3

#define GPIO_PUPDR_NOTHING		0
#define GPIO_PUPDR_PULLUP		1
#define GPIO_PUPDR_PULLDOWN		2
#define GPIO_PUPDR_RESERVED		3

#define PIN_0		0
#define PIN_1		1
#define PIN_2		2
#define PIN_3		3
#define PIN_4		4
#define PIN_5		5
#define PIN_6		6
#define PIN_7		7
#define PIN_8		8
#define PIN_9		9
#define PIN_10		10
#define PIN_11		11
#define PIN_12		12
#define PIN_13		13
#define PIN_14		14
#define PIN_15		15

#define AF0		0b0000
#define AF1		0b0001
#define AF2		0b0010
#define AF3		0b0011
#define AF4		0b0100
#define AF5		0b0101
#define AF6		0b0110
#define AF7		0b0111
#define AF8		0b1000
#define AF9		0b1001
#define AF10	0b1010
#define AF11	0b1011
#define AF12	0b1100
#define AF13	0b1101
#define AF14	0b1110
#define AF15	0b1111


#endif /* STM32FXXHAL_H_ */
