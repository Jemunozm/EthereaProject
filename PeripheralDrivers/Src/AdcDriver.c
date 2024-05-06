/*
 * AdcDriver.c
 *
 *  Created on: June 1, 2023
 *      Author: Sentry
 */
#include "AdcDriver.h"
#include "GPIOxDriver.h"

GPIO_Handler_t handlerAdcPin = {0};
uint16_t	adcRawData = 0;

void adc_Config(ADC_Config_t *adcConfig){

	/* 1. Configuramos el PinX para que cumpla la función de canal análogo deseado. */

	configAnalogPin(adcConfig->channel);

	/* 2. Activamos la señal de reloj para el periférico ADC1 (bus APB2)*/

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Limpiamos los registros antes de comenzar a configurar

	ADC1->CR1 = 0;
	ADC1->CR2 = 0;

	/* Comenzamos la configuración del ADC1 */
	/* 3. Resolución del ADC */

	switch(adcConfig->resolution){
	case ADC_RESOLUTION_12_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		break;
	}

	case ADC_RESOLUTION_10_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		ADC1->CR1 |= ADC_CR1_RES_0;
		break;
	}

	case ADC_RESOLUTION_8_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		ADC1->CR1 |= ADC_CR1_RES_1;
		break;
	}

	case ADC_RESOLUTION_6_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		ADC1->CR1 |= ADC_CR1_RES;
		break;
	}

	default:
	{
		break;
	}
	}

	/* 4. Configuramos el modo Scan como desactivado */
	ADC1->CR1 &= ~ADC_CR1_SCAN;

	/* 5. Configuramos la alineación de los datos (derecha o izquierda) */
	if(adcConfig->dataAlignment == ADC_ALIGNMENT_RIGHT){
		// Alineación a la derecha (esta es la forma "natural")
		ADC1->CR2 &= ~ADC_CR2_ALIGN;
	}
	else{
		// Alineación a la izquierda (para algunos cálculos matemáticos)
		ADC1->CR2 &= ADC_CR2_ALIGN;
	}

	/* 6. Desactivamos el "continuos mode" */
	ADC1->CR2 &= ~ADC_CR2_CONT;

	/* 7. Acá se debería configurar el sampling...*/
	if(adcConfig->channel < ADC_CHANNEL_10){
		ADC1->SMPR2 |= (adcConfig->samplingPeriod << 3*adcConfig->channel);
	}
	else{
		ADC1->SMPR1 |= (adcConfig->samplingPeriod << 3*(adcConfig->channel%10));
	}

	/* 8. Configuramos la secuencia y cuantos elementos hay en la secuencia */
	// Al hacerlo todo 0, estamos seleccionando solo 1 elemento en el conteo de la secuencia
	ADC1->SQR1 = 0;

	// Asignamos el canal de la conversión a la primera posición en la secuencia
	ADC1->SQR3 |= (adcConfig->channel << 0);

	/* 9. Configuramos el preescaler del ADC en 2:1 (el mas rápido que se puede tener */
	ADC->CCR &= ~ADC_CCR_ADCPRE;
	ADC->CCR |= ADC_CCR_ADCPRE;

	/* 10. Desactivamos las interrupciones globales */
	__disable_irq();

	/* 11. Activamos la interrupción debida a la finalización de una conversión EOC (CR1)*/
	ADC1->CR1 &= ~ADC_CR1_EOCIE;
	ADC1->CR1 |= ADC_CR1_EOCIE;

	/* 11a. Matriculamos la interrupción en el NVIC*/
	NVIC_EnableIRQ(ADC_IRQn);

	/* 11b. Configuramos la prioridad para la interrupción ADC */
	NVIC_SetPriority(ADC_IRQn, 6);

	/* 12. Activamos el modulo ADC */
	ADC1->CR2 &= ~ADC_CR2_ADON;
	ADC1->CR2 |= ADC_CR2_ADON;

	/* 13. Activamos las interrupciones globales */
	__enable_irq();
}

/*
 * Esta función lanza la conversion ADC y si la configuración es la correcta, solo se hace
 * una conversion ADC.
 * Al terminar la conversion, el sistema lanza una interrupción y el dato es leido en la
 * función callback, utilizando la funciona getADC().
 *
 * */
void startSingleADC(void){

	/* Desactivamos el modo continuo de ADC */
	ADC1->CR2 &= ~ADC_CR2_CONT;

	/* Limpiamos el bit del overrun (CR1) */
	ADC1->CR1 &= ~ADC_CR1_OVRIE;

	/* Iniciamos un ciclo de conversión ADC (CR2)*/
	ADC1->CR2 &= ~ADC_CR2_SWSTART;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC_ConfigMultichannel (ADC_Config_t *adcConfig, uint8_t numeroDeCanales){

	/* 1. Configuramos el PinX para que cumpla la función de canal análogo deseado. */

	for(uint8_t i = 0; i < numeroDeCanales; i++){
		configAnalogPin(adcConfig->multiChannel[i]);
	}

	/* 2. Activamos la señal de reloj para el periférico ADC1 (bus APB2)*/

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Limpiamos los registros antes de comenzar a configurar

	ADC1->CR1 = 0;
	ADC1->CR2 = 0;

	/* Comenzamos la configuración del ADC1 */
	/* 3. Resolución del ADC */

	switch(adcConfig->resolution){
	case ADC_RESOLUTION_12_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		break;
	}

	case ADC_RESOLUTION_10_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		ADC1->CR1 |= ADC_CR1_RES_0;
		break;
	}

	case ADC_RESOLUTION_8_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		ADC1->CR1 |= ADC_CR1_RES_1;
		break;
	}

	case ADC_RESOLUTION_6_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		ADC1->CR1 |= ADC_CR1_RES;
		break;
	}

	default:
	{
		break;
	}
	}

	/* 4. Configuramos el modo Scan como desactivado */
	ADC1->CR1 &= ~ADC_CR1_SCAN;
	ADC1->CR1 |= ADC_CR1_SCAN;

	/* 5. Configuramos la alineación de los datos (derecha o izquierda) */
	if(adcConfig->dataAlignment == ADC_ALIGNMENT_RIGHT){
		// Alineación a la derecha (esta es la forma "natural")
		ADC1->CR2 &= ~ADC_CR2_ALIGN;
	}
	else{
		// Alineación a la izquierda (para algunos cálculos matemáticos)
		ADC1->CR2 &= ADC_CR2_ALIGN;
	}

	/* 6. Desactivamos el "continuos mode" */
	ADC1->CR2 &= ~ADC_CR2_CONT;

	/* 7. Acá se debería configurar el sampling...*/

	for(uint8_t j=0; j < numeroDeCanales; j++){

		if(adcConfig->channel < ADC_CHANNEL_10){
			ADC1->SMPR2 |= (adcConfig->multiSampling[j] << 3*adcConfig->multiChannel[j]);
		}
		else{
			ADC1->SMPR1 |= (adcConfig->multiSampling[j] << 3*(adcConfig->multiChannel[j]%10));
	    }
	}

	/* 8. Configuramos la secuencia y cuantos elementos hay en la secuencia */
	// Al hacerlo todo 0, estamos seleccionando solo 1 elemento en el conteo de la secuencia
	ADC1->SQR1 = ((numeroDeCanales - 1) << ADC_SQR1_L_Pos);

	for(uint8_t k=0; k < numeroDeCanales; k++){
		if(k <= 5){

			ADC1->SQR3 |= (adcConfig->multiChannel[k] << 5 * k);
		}
		else if((k > 5) & (k <= 11)){

			ADC1->SQR2 |= (adcConfig->multiChannel[k] << 5 * (k-6));
		}
		else if(k < 16){

			ADC1->SQR1 |= (adcConfig->multiChannel[k] << 5 * (k-12));
		}
	}

	/* 9. Configuramos el preescaler del ADC en 2:1 (el mas rápido que se puede tener */
	ADC->CCR &= ~ADC_CCR_ADCPRE;
	ADC->CCR |= ADC_CCR_ADCPRE;

	/* 10. Desactivamos las interrupciones globales */
	__disable_irq();

	/* 11. Activamos la interrupción debida a la finalización de una conversión EOC (CR1)*/
	ADC1->CR1 &= ~ADC_CR1_EOCIE;
	ADC1->CR1 |= ADC_CR1_EOCIE;

	//Activar el EOCS (End of convertion selection)
	ADC1->CR2 &= ~ADC_CR2_EOCS;
	ADC1->CR2 |= ADC_CR2_EOCS;

	/* 11a. Matriculamos la interrupción en el NVIC*/
	NVIC_EnableIRQ(ADC_IRQn);

	/* 11b. Configuramos la prioridad para la interrupción ADC */
	NVIC_SetPriority(ADC_IRQn, 6);

	/* 12. Activamos el modulo ADC */
	ADC1->CR2 &= ~ADC_CR2_ADON;
	ADC1->CR2 |= ADC_CR2_ADON;

	/* 13. Activamos las interrupciones globales */
	__enable_irq();
}

/*
 * Esta función habilita la conversion ADC de forma continua.
 * Una vez ejecutada esta función, el sistema lanza una nueva conversion ADC cada vez que
 * termina, sin necesidad de utilizar para cada conversion el bit SWSTART del registro CR2.
 * Solo es necesario activar una sola vez dicho bit y las conversiones posteriores se lanzan
 * automaticamente.
 * */
void startContinousADC(void){

	/* Activamos el modo continuo de ADC */
	ADC1->CR2 &= ~ADC_CR2_CONT;
	ADC1->CR2 |= ADC_CR2_CONT;

	/* Iniciamos un ciclo de conversión ADC */
	// Escriba su código acá
	ADC1->CR2 &= ~ADC_CR2_SWSTART;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

/*
 * Función que retorna el ultimo dato adquirido por la ADC
 * La idea es que esta función es llamada desde la función callback, de forma que
 * siempre se obtiene el valor mas actual de la conversión ADC.
 * */
uint16_t getADC(void){

	// Esta variable es actualizada en la ISR de la conversión, cada vez que se obtiene
	// un nuevo valor.
	return adcRawData;
}

/*
 * Esta es la ISR de la interrupción por conversión ADC
 */
void ADC_IRQHandler(void){

	// Evaluamos que se dio la interrupción por conversión ADC
	if(ADC1->SR & ADC_SR_EOC){
		// Leemos el resultado de la conversión ADC y lo cargamos en una variale auxiliar
		// la cual es utilizada en la función getADC()

		adcRawData = ADC1->DR;

		// Hacemos el llamado a la función que se ejecutará en el main
		adcComplete_Callback();
	}

}

/* Función debil, que debe ser sobreescrita en el main. */
__attribute__ ((weak)) void adcComplete_Callback(void){
	__NOP();
}

/*
 * Con esta función configuramos que pin deseamos que funcione como canal ADC
 * Esta funcion trabaja con el GPIOxDriver, por lo cual requiere que se incluya
 * dicho driver.
 */
void configAnalogPin(uint8_t adcChannel) {

	// Con este switch seleccionamos el canal y lo configuramos como análogo.
	switch (adcChannel) {

	case ADC_CHANNEL_0: {
		//PA0
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_0;
		break;
	}

	case ADC_CHANNEL_1: {
		//PA1
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_1;
		break;
	}

	case ADC_CHANNEL_2: {
		//PA2
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_2;
		break;
	}

	case ADC_CHANNEL_3: {
		//PA3
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_3;
		break;
	}

	case ADC_CHANNEL_4: {
		//PA4
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_4;
		break;
	}

	case ADC_CHANNEL_5: {
		//PA5
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
		break;
	}
	case ADC_CHANNEL_6: {
		//PA6
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_6;
		break;
	}
	case ADC_CHANNEL_7: {
		//PA7
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_7;
		break;
	}
	case ADC_CHANNEL_8: {
		//PB0
		handlerAdcPin.pGPIOx 						= GPIOB;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_0;
		break;
	}
	case ADC_CHANNEL_9: {
		//PB1
		handlerAdcPin.pGPIOx 						= GPIOB;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_1;
		break;
	}
	case ADC_CHANNEL_10: {
		//C0
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_0;
		break;
	}
	case ADC_CHANNEL_11: {
		//PC1
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_1;
		break;
	}
	case ADC_CHANNEL_12: {
		//PC2
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_2;
		break;
	}
	case ADC_CHANNEL_13: {
		//PC3
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_3;
		break;
	}
	case ADC_CHANNEL_14: {
		//PC4
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_4;
		break;
	}
	case ADC_CHANNEL_15: {
		//PC5
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
		break;
	}
	default: {
		break;
	}
}
	// Despues de configurar el canal adecuadamente, se define este pin como Analogo y se
	// carga la configuración con el driver del GPIOx
	handlerAdcPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GPIO_Config(&handlerAdcPin);
}

void ADC_Channel_Interrupt(ADC_Config_t *adcConfig){

	//Limpiar y configurar el EXTEN
	ADC1->CR2 &= ~ADC_CR2_EXTEN;
	ADC1->CR2 |= ADC_CR2_EXTEN_0;

	if(adcConfig->eventType == EXTERNAL_EVENT_ENABLE){

		//Limpiar el registro
		ADC1->CR2 &= ~ADC_CR2_EXTSEL;

		switch(adcConfig->AdcEvent){
		case 1:{

			//Timer 1 CC1 event
			ADC1->CR2 &= ~ADC_CR2_EXTSEL;
			break;
		}
		case 2:{

			//Timer 1 CC2 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_0;
			break;
		}
		case 3:{

			//Timer 1 CC3 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_1;
			break;
		}
		case 4:{

			//Timer 2 CC2 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_0;
			ADC1->CR2 |= ADC_CR2_EXTSEL_1;
			break;
		}
		case 5:{

			//Timer 2 CC3 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_2;
			break;
		}
		case 6:{

			//Timer 2 CC4 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_0;
			ADC1->CR2 |= ADC_CR2_EXTSEL_2;
			break;
		}
		case 7:{

			//Timer 2 TRGO event
			ADC1->CR2 |= ADC_CR2_EXTSEL_1;
			ADC1->CR2 |= ADC_CR2_EXTSEL_2;
			break;
		}
		case 8:{

			//Timer 3 CC1 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_0;
			ADC1->CR2 |= ADC_CR2_EXTSEL_1;
			ADC1->CR2 |= ADC_CR2_EXTSEL_2;
			break;
		}
		case 9:{

			//Timer 3 TRGO event
			ADC1->CR2 |= ADC_CR2_EXTSEL_3;
			break;
		}
		case 10:{

			//Timer 4 CC4 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_0;
			ADC1->CR2 |= ADC_CR2_EXTSEL_3;
			break;
		}
		case 11:{

			//Timer 5 CC1 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_1;
			ADC1->CR2 |= ADC_CR2_EXTSEL_3;
			break;
		}
		case 12:{

			//Timer 5 CC2 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_0;
			ADC1->CR2 |= ADC_CR2_EXTSEL_1;
			ADC1->CR2 |= ADC_CR2_EXTSEL_3;
			break;
		}
		case 13:{

			//Timer 5 CC3 event
			ADC1->CR2 |= ADC_CR2_EXTSEL_2;
			ADC1->CR2 |= ADC_CR2_EXTSEL_3;
			break;
		}
		case 14:{

			//EXTI line11
			ADC1->CR2 |= ADC_CR2_EXTSEL;
			break;
		}
		default:{
			break;
		}
		}
	}
}
