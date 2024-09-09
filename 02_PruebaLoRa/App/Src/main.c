// Primera version de satelite ETHEREA 2024 autores: Manuel M Jesus Munoz//



#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "PLLDriver.h"
#include "AdcDriver.h"

ADC_Config_t temp = {0};
BasicTimer_Handler_t timerMuestreo = {0};
void initSystem(void);
USART_Handler_t usart6 = {0};

GPIO_Handler_t usartRX = {0};
GPIO_Handler_t usartTX = {0};
GPIO_Handler_t blinky  = {0};

uint8_t ADCComplete = 0;

uint16_t tempValue = 0;

uint8_t sendMsg = 0;
uint8_t flag_data = 0;

uint8_t usartData = 0;

uint8_t buffer[128];

int main (){

	initSystem();

	while (1){
		if(flag_data){
			writeMsg(&usart6,"hola mundo \n");
			flag_data = 0;
		}




	}

	return 0;
}
void initSystem(void){

	blinky.pGPIOx								= GPIOA;
	blinky.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	blinky.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_OUT;
	blinky.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	blinky.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEEDR_FAST;

	GPIO_Config(&blinky);

	timerMuestreo.ptrTIMx = TIM2;
	timerMuestreo.TIMx_Config.TIMx_speed = 16000;
	timerMuestreo.TIMx_Config.TIMx_period = 500;
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

	usartRX.pGPIOx								= GPIOC;
	usartRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_7;
	usartRX.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFN;
	usartRX.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	usartRX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEEDR_FAST;
	usartRX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF8;

	GPIO_Config(&usartRX);

	usartTX.pGPIOx								= GPIOC;
	usartTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_6;
	usartTX.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFN;
	usartTX.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	usartTX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEEDR_FAST;
	usartTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF8;

	GPIO_Config(&usartTX);

	usart6.ptrUSARTx						= USART6;
	usart6.USART_Config.USART_mode			= USART_MODE_RXTX;
	usart6.USART_Config.USART_baudrate		= USART_BAUDRATE_9600;
	usart6.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usart6.USART_Config.USART_parity		= USART_PARITY_NONE;
	usart6.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usart6.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usart6.USART_Config.USART_PLL_EN		= PLL_DISABLE;

	USART_Config(&usart6);



}

void BasicTimer2_Callback(void){
	GPIOxTooglePin(&blinky);
	flag_data = 1;
	startSingleADC();
}

void adcComplete_Callback(void){
	tempValue = getADC();
	tempValue= (tempValue * 3300) /4096;
}

void usart6Rx_Callback(void){
	usartData = getRxData();
}
