/*
 * SysTickDriver.c
 *
 *  Created on: May 11, 2023
 *      Author: ingfisica
 */

#include <stm32f4xx.h>
#include "SysTickDriver.h"

uint64_t ticks = 0;
uint64_t ticks_start = 0;
uint64_t ticks_counting = 0;

void config_SysTick_ms(uint8_t systemClock){
	//Reiniciamos el valor de la variable que cuenta tiempo
	ticks = 0;

	//Cargando el valor del limite de incrementos que representan 1 ms
	switch(systemClock){

	//Caso para el reloj HSI -> 16MHz
	case 0:
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;

	//Caso para el reloj HSE
	case 1:
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;

	//Caso para el reloj PLL a 100 MHz
	case 2:
		SysTick->LOAD = SYSTICK_LOAD_VALUE_100MHz_1ms;
		break;

	//Caso para el reloj PLL a 80 MHz
	case 3:
		SysTick->LOAD = SYSTICK_LOAD_VALUE_80MHz_1ms;
		break;

	//Caso en el que se ingresa un valor diferente
	default:
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;
	}

	//Limpiamos el valor actual del SysTick
	SysTick->VAL = 0;

	//Configuramos el reloj interno como el reloj para el timer
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	//Desactivamos las interrupciones globales
	__disable_irq();

	//Matriculamos la interrupcion en el NVIC
	NVIC_EnableIRQ(SysTick_IRQn);

	//Activamos la interrupcion debida al conteo a cero del SysTick
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

	//Activamos el timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

	//Activamos nuevamente las interrupciones globales
	__enable_irq();
}

uint64_t getTicks_ms(void){
	return ticks;
}

void delay_ms(uint32_t wait_time_ms){
	//Captura el primer valor del tiempo para comparar
	ticks_start = getTicks_ms();

	//Captura el segundo valor del tiempo para comparar
	ticks_counting = getTicks_ms();

	/*Compara: si el valor "counting" es menor que el "start + wait"
	 *actualiza el valor del "counting".
	 *Repite esta operacion hasta que counting sea mayor
	 *(Se cumple el tiempo de espera)
	 */
	while(ticks_counting < (ticks_start + (uint64_t)wait_time_ms)){

		//Actualiza el valor
		ticks_counting = getTicks_ms();
	}
}

void SysTick_Handler(void){

	//Verificamos que la interrupcion se lanzo
	if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk){

		//Limpiamos la bandera
		SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;

		//Incrementamos en 1 el contador
		ticks++;
	}
}

