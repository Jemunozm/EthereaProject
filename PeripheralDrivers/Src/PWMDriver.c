/*
 * PWMDriver.c
 *
 *  Created on: 21/05/2023
 *      Author: Sentry
 */

#include "PWMDriver.h"

void pwm_Config(PWM_Handler_t *ptrPwmHandler){

	/*1. Activar la señal de reloj del periférico requerido*/
	if(ptrPwmHandler->ptrTIMx == TIM2){

		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM3){
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM4){
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM5){
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	}
	else{
		__NOP();
	}

	/*1. Cargamos la frecuencia deseada */
	setFrequency(ptrPwmHandler);

	/*2. Cargamos el valor del Dutty-Cycle */
	setDuttyCycle(ptrPwmHandler);

	/*2a. Estamos en UP mode, el límite se carga al ARR y se comienza en 0 */
	ptrPwmHandler->ptrTIMx->CR1 &= ~TIM_CR1_DIR;

	/*3. Configuramos los bits CCxS del registro TIMy_CCMR1,
	 *de forma que sea modo salida.
	 *(Para cada canal hay un conjunto CCxS)
	 *
	 *4. Además, en el mismo "case" podemos configurar el modo del
	 *PWM, su polaridad...
	 *
	 *5. Y adempas activamos el preload bit, para que cada vez que exista un
	 *update-event el valor cargado en el CCRx será recargado en el registro
	 *"shadow" del PWM
	 */

	switch(ptrPwmHandler->config.channel){
		case PWM_CHANNEL_1:{

			// Seleccionamos el canal como salida
			ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_CC1S;

			// Configuramos el canal como PWM modo 1
			ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_OC1M;

			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1M_1;
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1M_2;

			// Configuración del pre-load
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1PE;

			//Activamos el fast enable
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1FE;
			ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_OC1CE;

			break;
		}

		case PWM_CHANNEL_2:{

			// Seleccionamos el canal como salida
			ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_CC2S;

			// Configuramos el canal como PWM modo 1
			ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_OC2M;

			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2M_1;
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2M_2;

			// Configuración del pre-load
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2PE;

			//Activamos el fast enable
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2FE;
			ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_OC2CE;

			break;
		}

		//Para los canales 3 y 4 usamos las mismas mascaras de los dos anteriores
		//ya que los registros son iguales

		case PWM_CHANNEL_3:{

			// Seleccionamos el canal como salida
			ptrPwmHandler->ptrTIMx->CCMR2 &= ~TIM_CCMR1_CC1S;

			// Configuramos el canal como PWM modo 1
			ptrPwmHandler->ptrTIMx->CCMR2 &= ~TIM_CCMR1_OC1M;

			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR1_OC1M_1;
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR1_OC1M_2;

			// Configuración del pre-load
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR1_OC1PE;

			//Activamos el fast enable
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR1_OC1FE;
			ptrPwmHandler->ptrTIMx->CCMR2 &= ~TIM_CCMR1_OC1CE;

			break;
		}

		case PWM_CHANNEL_4:{

			// Seleccionamos el canal como salida
			ptrPwmHandler->ptrTIMx->CCMR2 &= ~TIM_CCMR1_CC2S;

			// Configuramos el canal como PWM modo 1
			ptrPwmHandler->ptrTIMx->CCMR2 &= ~TIM_CCMR1_OC2M;

			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR1_OC2M_1;
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR1_OC2M_2;

			// Configuración del pre-load
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR1_OC2PE;

			//Activamos el fast enable
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR1_OC2FE;
			ptrPwmHandler->ptrTIMx->CCMR2 &= ~TIM_CCMR1_OC2CE_Msk;

			break;
		}

		default:{
			break;
		}
	}// fin del switch-case

	/*6. Activamos la salida seleccionada */
	enableOutput(ptrPwmHandler);
}

/*Función para activar el Timer y activar todo el módulo PWM */
void startPwmSignal(PWM_Handler_t *ptrPwmHandler){
	ptrPwmHandler->ptrTIMx->CR1 |= TIM_CR1_ARPE;
	ptrPwmHandler->ptrTIMx->CR1 |= TIM_CR1_CEN;
}

/* Funcion para desactivar el Timer y detener todo el módulo PWM */
void stopPwmSignal(PWM_Handler_t *ptrPwmHandler){
		ptrPwmHandler->ptrTIMx->CR1 &= ~TIM_CR1_CEN;
}

/* Función encargada de activar cada uno de los canales con los que cuenta el TimerX */
void enableOutput(PWM_Handler_t *ptrPwmHandler){
	switch (ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1:{

		//Activamos la salida del canal 1
		ptrPwmHandler->ptrTIMx->CCER &= (0b0 << 0);
		ptrPwmHandler->ptrTIMx->CCER |= (0b1 << 0);
		break;
	}
	case PWM_CHANNEL_2:{

		//Activamos la salida del canal 2
		ptrPwmHandler->ptrTIMx->CCER &= (0b0 << 4);
		ptrPwmHandler->ptrTIMx->CCER |= (0b1 << 4);
	}
	case PWM_CHANNEL_3:{

		//Activamos la salida del canal 3
		ptrPwmHandler->ptrTIMx->CCER &= (0b0 << 8);
		ptrPwmHandler->ptrTIMx->CCER |= (0b1 << 8);
	}
	case PWM_CHANNEL_4:{

		//Activamos la salida del canal 4
		ptrPwmHandler->ptrTIMx->CCER &= (0b0 << 12);
		ptrPwmHandler->ptrTIMx->CCER |= (0b1 << 12);
	}
	default:{
		break;
	}
	}
}

/*
 * La frecuencia es definida por el conjunto formado por el preescaler
 * (PSC) y el valor límite al que llega el Timer (ARR), con estos dos se establece
 * la frecuencia
 */
void setFrequency(PWM_Handler_t *ptrPwmHandler){

	/* Cargamos el valor del prescaler, nos define la velocidad (ns)
	 * a la cual se incrementa el Timer
	 */
	ptrPwmHandler->ptrTIMx->PSC = ptrPwmHandler->config.prescaler;

	/* Cargamos el valor del ARR, el cual es el límite
	 * de incrementos del timer antes de hacer un update y reload
	 */
	ptrPwmHandler->ptrTIMx->ARR = ptrPwmHandler->config.periodo;
}

/*Función para actualizar la frecuencia, funciona de la mano con setFrecuency */
void updateFrequency(PWM_Handler_t *ptrPwmHandler, uint16_t newFreq){

	//Actualizamos el registro que manipula el periodo
	ptrPwmHandler->config.periodo = newFreq;

	//Llamamos la función que cambia la frecuencia
	setFrequency(&*ptrPwmHandler);
}

/* El valor del Dutty debe estar dado en %, entre el 0% y el 100% */
void setDuttyCycle(PWM_Handler_t *ptrPwmHandler){

	//Seleccionamos el canal para configurar su Dutty
	switch(ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1:{
		ptrPwmHandler->ptrTIMx->CCR1 = ptrPwmHandler->config.duttyCicle;

		break;
	}
	case PWM_CHANNEL_2:{
		ptrPwmHandler->ptrTIMx->CCR2 = ptrPwmHandler->config.duttyCicle;

		break;
	}
	case PWM_CHANNEL_3:{
		ptrPwmHandler->ptrTIMx->CCR3 = ptrPwmHandler->config.duttyCicle;

		break;
	}
	case PWM_CHANNEL_4:{
		ptrPwmHandler->ptrTIMx->CCR4 = ptrPwmHandler->config.duttyCicle;
	}
	default:{
		break;
	}
	}//Fin del switch-case
}

/* Función para actualizar el Dutty, funciona de la mano con setDuttyCycle */
void updateDuttyCycle(PWM_Handler_t *ptrPwmHandler, uint16_t newDutty){

	//Actualizamos el registro que manipula el dutty
	ptrPwmHandler->config.duttyCicle = newDutty;

	//Llamamos la función que cambia el Dutty y cargamos el nuevo valor
	setDuttyCycle(ptrPwmHandler);
}
