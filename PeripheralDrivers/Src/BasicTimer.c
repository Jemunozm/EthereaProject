/*
 * BasicTimer.c
 *
 *  Created on: 23/03/2023
 *      Author: if420_01
 */

#include "BasicTimer.h"

//Variable que guarda la referencia del periferico que se está utilizando

TIM_TypeDef		*ptrTimerUsed;
TIM_TypeDef 	*ptrTIM2 = TIM2;
TIM_TypeDef 	*ptrTIM3 = TIM3;
TIM_TypeDef 	*ptrTIM4 = TIM4;
TIM_TypeDef 	*ptrTIM5 = TIM5;

/*Funcion en la que cargamos la configuracion del Timer
 * Recordar que siempre se debe comenzar con activar la señal de reloj del periferico
 * que se está utilizando.
 * en este caso debemos ser cuidadosos al momento de utilizar las interrupciones.
 * Los Timers están conectados directamente al elemento NVIC del cortex _ Mx
 * Debemos configurar y/o utilizar:
 * 		-TIMx_CR1 (control register 1)
 * 		-TIMx_SMCR (slave mode control register) -> mantener en 0 para modo Timer Basico
 * 		-TIMx_DIER (DMA and Interrupt enable register)
 * 		-TIMx_SR (status register)
 * 		-TIMx_CNT (counter)
 * 		-TIMx_PSC (pre scaler)
 * 		-TIMx_ARR (AutoReload register)
 *
 *Como vamos a trabajar con interrupciones, antes de configurar una nueva, debemos desactivar
 *el sistema global de interrupciones, activar la IRQ especifica y luego volver a encender
 *el sistema
 */

void BasicTimer_Config(BasicTimer_Handler_t *ptrBTimerHandler){

	//Guardamos una referencia al periferico que estamos utilizando

	ptrTimerUsed = ptrBTimerHandler->ptrTIMx;

	/*0. Desactivamos las interrupciones globales mientras configuramos el sistema.*/

	__disable_irq();

	/*1.Activar la señal de reloj del periferico requerido*/

	if(ptrBTimerHandler->ptrTIMx == TIM2){

		//Registro del RCC que nos activa la señal de reloj para el TIM2
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	}
	else if(ptrBTimerHandler->ptrTIMx == TIM3){

		//Registro del RCC que nos activa la señal de reloj para el TIM3
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	}
	else if(ptrBTimerHandler->ptrTIMx == TIM4){

		//Registro del RCC que nos activa la señal de reloj para el TIM4
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}
	else if(ptrBTimerHandler->ptrTIMx == TIM5){

		//Registro del RCC que nos activa la señal de reloj para el TIM5
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	}
	else{
		__NOP();

	}

	/*2.Configuramos el pre-escaler
	 * Recordar que el pre-escaler nos indica la velocidad a la que se incrementa el counter,
	 * de forma que periodo_incremento * veces_incremento_counter = periodo_update
	 * Modificar el valor del registro PSC en el TIM utilizado
	 */

	ptrBTimerHandler->ptrTIMx->PSC = ptrBTimerHandler->TIMx_Config.TIMx_speed;

	/*3.Configuramos la direccion del counter (up/down)*/

	if(ptrBTimerHandler->TIMx_Config.TIMx_mode == BTIMER_MODE_UP){

		/*3a. Estamos en UP_Mode, el limite se carga en ARR y se comienza en 0 */
		//Configurar el registro que nos controla el modo up or down

		ptrBTimerHandler->ptrTIMx->ARR &= ~TIM_CR1_DIR;

		/*3b. Configuramos el Autoreload. Este es el "limite" hasta donde el CNT va a contar */

		ptrBTimerHandler->ptrTIMx->ARR = ptrBTimerHandler->TIMx_Config.TIMx_period - 1;

		/*3c. Reiniciamos el registro counter*/

		ptrBTimerHandler->ptrTIMx->CNT = RESET;

	}else{
		/*3a. Estamos en DOWN_Mode, el limite se carga en ARR (0) y se comienza con un valor alto
		 * Trabaja contando en direccion descendente
		 */
		ptrBTimerHandler->ptrTIMx->ARR |= TIM_CR1_DIR;

		/*3b. Configuramos el autoreload. Este es el limite hasta donde el CNT va a contar
		 * En modo descendente, con numeros positivos, cual es el minimo valor que el ARR puede tomar*/

		ptrBTimerHandler->ptrTIMx->CNT = 0;

		/*3c. Reiniciamos el registro counter
		 * Este es el valor con el que el counter comienza */

		ptrBTimerHandler->ptrTIMx->CNT = ptrBTimerHandler->TIMx_Config.TIMx_period - 1;

	}

	/*4. Activamos el Timer (el CNT debe comenzar a contar */

	ptrBTimerHandler->ptrTIMx->CR1 |= TIM_CR1_CEN;

	/*5. Activamos la interrupcion debida al Timerx Utilizado
	 * Modificar el registro encargado de activar la interrupcion generada por el TIMx */

	if (ptrBTimerHandler->TIMx_Config.TIMx_interruptEnable == 1){
		ptrBTimerHandler->ptrTIMx->DIER |= TIM_DIER_UIE;
	}
	else{
		ptrBTimerHandler->ptrTIMx->DIER &= ~TIM_DIER_UIE;
	}

	/*6. Activamos el canal del sistema NVIC para que lea la interrupcion */

	if (ptrBTimerHandler->ptrTIMx == TIM2){

		//Activando el NVIC para la interrupcion del TIM2


		NVIC_EnableIRQ(TIM2_IRQn);
		__NVIC_SetPriority(TIM2_IRQn, 3);
	}
	else if (ptrBTimerHandler->ptrTIMx == TIM3){

		//Activando el NVIC para la interrupcion del TIM3

		NVIC_EnableIRQ(TIM3_IRQn);
	}
	else if(ptrTimerUsed == TIM4){

			// Activando en NVIC para la interrupción del TIM3

		NVIC_EnableIRQ(TIM4_IRQn);
	}
	else if(ptrTimerUsed == TIM5){

			// Activando en NVIC para la interrupción del TIM3

		NVIC_EnableIRQ(TIM5_IRQn);
		__NVIC_SetPriority(TIM5_IRQn, 5);
	}
	else{
		__NOP();
	}

	/*7. Volvemos a activar las interrupciones del sistema*/

	__enable_irq();
}

__attribute__((weak)) void BasicTimer2_Callback(void){

	/*NOTE: This function should not be modified, when the callback id needed
	 * the BasicTimer2_Callback could be implemented in the main file */
	__NOP();
}

__attribute__((weak)) void BasicTimer3_Callback(void){

	/*NOTE: This function should not be modified, when the callback id needed
	 * the BasicTimer3_Callback could be implemented in the main file */
	__NOP();
}

__attribute__((weak)) void BasicTimer4_Callback(void){

	/*NOTE: This function should not be modified, when the callback id needed
	 * the BasicTimer4_Callback could be implemented in the main file */
	__NOP();
}

__attribute__((weak)) void BasicTimer5_Callback(void){

	/*NOTE: This function should not be modified, when the callback id needed
	 * the BasicTimer5_Callback could be implemented in the main file */
	__NOP();
}

/*Esta es la funcion a la que apunta el ssema en el vector de interrupciones.
 * Se debe utilizar usando el mismo nombre definido en el vector de interrupciones.
 * Al hacerlo correctamente, el sistema apunta a esta funcion y cuando la interrupcion se lanza
 * el sistema inmediatamente salta a este lugar en la memoria */

void TIM2_IRQHandler(void){

	/*Limpiamos la bandera que indica que la interrupcion se ha generado */

	ptrTIM2->SR &= ~TIM_SR_UIF;

	/*Lamamos a la funcion que se debe encargar de hacer algo con esta interrupcion */

	BasicTimer2_Callback();
}

void TIM3_IRQHandler(void){

	/*Limpiamos la bandera que indica que la interrupcion se ha generado */

	ptrTIM3->SR &= ~TIM_SR_UIF;

	/*Lamamos a la funcion que se debe encargar de hacer algo con esta interrupcion */

	BasicTimer3_Callback();
}

void TIM4_IRQHandler(void){

	/*Limpiamos la bandera que indica que la interrupcion se ha generado */

	ptrTIM4->SR &= ~TIM_SR_UIF;

	/*Lamamos a la funcion que se debe encargar de hacer algo con esta interrupcion */

	BasicTimer4_Callback();
}

void TIM5_IRQHandler(void){

	/*Limpiamos la bandera que indica que la interrupcion se ha generado */

	ptrTIM5->SR &= ~TIM_SR_UIF;

	/*Lamamos a la funcion que se debe encargar de hacer algo con esta interrupcion */

	BasicTimer5_Callback();
}
