/*
 * PLLDriver.c
 *
 *  Created on: 27.5.2023
 *    Author: Sentry
 */

#include "stm32f4xx.h"
#include "PLLDriver.h"

void configPLL (void){

		//APB1 = max Freq 50MHz
		//Se configura el prescaler para que divida por 2
		RCC->CFGR &= ~RCC_CFGR_PPRE1;
		RCC->CFGR |= RCC_CFGR_PPRE1_2;

		//APB2 = max Freq 100MHz
		//No es necesario configurar prescaler en el APB2
		RCC->CFGR &= ~RCC_CFGR_PPRE2_0;
		RCC->CFGR &= ~RCC_CFGR_PPRE2_1;
		RCC->CFGR &= ~RCC_CFGR_PPRE2_2;

		//Activar el HSI como alimentación del PLL
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;
		RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSI);

		//Apagar el PLL
		RCC->CR &= ~ RCC_CR_PLLON;

		//Limpiar los bit M del registro PLLCFGR
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_0;
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_1;
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_2;
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_3;
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_4;
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_5;

		//PLLM = 10 -- Para configurar a 80 MHz
		//RCC->PLLCFGR |= (0b1010 << RCC_PLLCFGR_PLLM_Pos);

		//PLLM = 8 -- Para configurar a 100 MHz
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_3;

		//Limpiar los bit N del registro PLLCFGR
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_0;
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_1;
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_2;
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_3;
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_4;
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_5;
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_6;
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_7;
		RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_8;

		//PLLN = 100
		RCC->PLLCFGR |= (0b001100100 << RCC_PLLCFGR_PLLN_Pos);

		//PLLP = 2
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);

		//Limpiamos el registro de la FLASH
		FLASH->ACR &= ~(FLASH_ACR_LATENCY);

		//Configurar 2 WS para el reloj
		FLASH->ACR |= (0x3 << FLASH_ACR_LATENCY_Pos);


		//Habilitar el PLL
		RCC->CR |= RCC_CR_PLLON;
		while (!(RCC->CR & (RCC_CR_PLLRDY)));

		//PLL como reloj del sistema
		RCC->CFGR &= ~RCC_CFGR_SW_0;
		RCC->CFGR |= RCC_CFGR_SW_1;

		//Activar el MCO1 para realizar la medida del HSI
		RCC->CFGR &= ~RCC_CFGR_MCO1;

		//Configuración del prescaler
		RCC->CFGR &= ~RCC_CFGR_MCO1PRE;

		//Activar el MCO2 para realizar la medida del PLL
		RCC->CFGR &= ~RCC_CFGR_MCO2;
		RCC->CFGR |= RCC_CFGR_MCO2;

		//Configuración del prescaler
		RCC->CFGR &= ~RCC_CFGR_MCO2PRE;
		RCC->CFGR |= (0b110 << RCC_CFGR_MCO2PRE_Pos);
}

uint16_t getConfigPLL(void){

	//Definición de variables
	uint16_t auxVar = 0;			//Variable que guarda la configuración del equipo

	uint8_t PLLM = 0;				//Se guarda el valor de PLLM
	uint16_t PLLN = 0;				//Se guarda el valor de PLLN
	uint8_t PLLP = 0;				//Se guarda el valor de PLLP

	uint8_t auxFreqIn = 0;			//Variable para guardar la frecuencia de entrada del PLL

	auxFreqIn = 16;					//Frecuencia de entrada al PLL

	//Se guardan los valores de PLLM, PLLN y PLLP respectivamente
	//PLLM
	PLLM = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLM_Pos);
	PLLM &= (RCC_PLLCFGR_PLLM);

	//PLLN
	PLLN = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLN_Pos);
	PLLN &= (RCC_PLLCFGR_PLLN);

	//PLLP
	PLLP = (RCC->PLLCFGR >> RCC_PLLCFGR_PLLP_Pos);
	PLLP &= (RCC_PLLCFGR_PLLP);

	//Se guarda el valor de PLLP dependiendo de la configuración
	switch(PLLP){
	case 0:{
		PLLP = 2;
		break;
	}
	case 1:{
		PLLP = 4;
		break;
	}
	case 2:{
		PLLP = 6;
		break;
	}
	case 3:{
		PLLP = 8;
		break;
	}
}

	//VCO clock
	auxVar = auxFreqIn * (PLLN / (uint16_t) PLLM);

	//PLL general clock output
	auxVar = auxVar / PLLP;

	return auxVar;
}
