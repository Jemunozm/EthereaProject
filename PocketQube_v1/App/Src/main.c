// Primera version de satelite ETHEREA 2024 autores: Manuel M Jesus Munoz//



#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"
#include "USARTxDriver.h"
#include "I2CxDriver.h"
#include "PWMDriver.h"
#include "PLLDriver.h"
#include "SysTickDriver.h"
#include "AdcDriver.h"
#include "RTCxDriver.h"
#include "arm_math.h"

ADC_Config_t temp = {0};
BasicTimer_Handler_t timerMuestreo = {0};
void initSystem(void);


uint8_t ADCComplete = 0;

uint16_t tempValue = 0;

int main (){

	initSystem();

	while (1){


	}

	return 0;
}
void initSystem(void){

	timerMuestreo.ptrTIMx = TIM2;
	timerMuestreo.TIMx_Config.TIMx_speed = 16000;
	timerMuestreo.TIMx_Config.TIMx_period = 5000;
	timerMuestreo.TIMx_Config.TIMx_mode = BTIMER_MODE_UP;
	timerMuestreo.TIMx_Config.TIMx_interruptEnable = 1;

	BasicTimer_Config(&timerMuestreo);



	temp.channel = ADC_CHANNEL_0;
	temp.resolution = ADC_RESOLUTION_12_BIT;
	temp.dataAlignment = ADC_ALIGNMENT_RIGHT;
	temp.samplingPeriod = ADC_SAMPLING_PERIOD_84_CYCLES; // preguntar ciclos optimos
	temp.eventType = EXTERNAL_EVENT_ENABLE;
	configAnalogPin(ADC_CHANNEL_0);



	adc_Config(&temp);

}

void BasicTimer2_Callback(void){
	startSingleADC();
}

void adcComplete_Callback(void){

	tempValue = getADC();
	tempValue= (tempValue * 3300) /4096;
}
