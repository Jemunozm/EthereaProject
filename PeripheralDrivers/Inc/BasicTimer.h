/*
 * BasicTimer.h
 *
 *  Created on: 23/03/2023
 *      Author: if420_01
 */

#ifndef BASICTIMER_H_
#define BASICTIMER_H_

#include "stm32f4xx.h"

#define BTIMER_MODE_UP				0
#define BTIMER_MODE_DOWN			1

#define BTIMER_SPEED_10us					160
#define BTIMER_SPEED_100us					1600
#define BTIMER_SPEED_16000us				16000
#define BTIMER_SPEED_PLL_80_MHz_100us		8000
#define BTIMER_SPEED_PLL_100_MHz_100us		10000


/*Estructura que contiene la configuracion minima necesaria para el manejo del timer*/

typedef struct
{
	uint8_t 	TIMx_mode;				//Up or Down
	uint32_t 	TIMx_speed;				//A qué velocidad se incrementa el timer
	uint32_t 	TIMx_period;			//Valor en ms del periodo del timer
	uint8_t		TIMx_interruptEnable;	//Activa o desactiva el modo interrupcion del timer

}BasicTimer_Config_t;

/*Handler para el timer*/

typedef struct
{
	TIM_TypeDef				*ptrTIMx;
	BasicTimer_Config_t		TIMx_Config;

}BasicTimer_Handler_t;

void BasicTimer_Config(BasicTimer_Handler_t *ptrBTimerHandler);
void BasicTimer2_Callback(void); 		//Esta funcion debe ser sobreescrita en el main para que el sistema funcione
void BasicTimer3_Callback(void);
void BasicTimer4_Callback(void);
void BasicTimer5_Callback(void);

#endif /* BASICTIMER_H_ */
