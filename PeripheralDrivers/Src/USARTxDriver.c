/*
 * USARTxDriver.c
 *
 *  Created on: Apr 19, 2023
 *      Author: if420_01
 */

#include <stm32f4xx.h>
#include "USARTxDriver.h"

/**
 * Configurando el puerto Serial...
 * Recordar que siempre se debe comenzar con activar la señal de reloj
 * del periferico que se está utilizando.
 */

uint8_t auxRxData;
uint8_t auxFun = 0;
char dataTxSend = 0;
char dataToSend = 0;
char msgTxSend = 0;
char auxArray[100];
volatile uint16_t counterTx = 0;
volatile uint16_t dataTx = 0;


 void USART_Config(USART_Handler_t *ptrUsartHandler){
	/* 1. Activamos la señal de reloj que viene desde el BUS al que pertenece el periferico */
	/* Lo debemos hacer para cada uno de las pisbles opciones que tengamos (USART1, USART2, USART6) */

	if(ptrUsartHandler->ptrUSARTx == USART1){

		RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);

	}else if(ptrUsartHandler->ptrUSARTx == USART2){

		RCC->APB1ENR |= (RCC_APB1ENR_USART2EN);

	}else if(ptrUsartHandler->ptrUSARTx == USART6){

		RCC->APB2ENR |= (RCC_APB2ENR_USART6EN);
	}

	/* 2. Configuramos el tamaño del dato, la paridad y los bit de parada */
	/* En el CR1 estan parity (PCE y PS) y tamaño del dato (M) */
	/* Mientras que en CR2 estan los stopbit (STOP)*/
	/* Configuracion del Baudrate (registro BRR) */
	/* Configuramos el modo: only TX, only RX, o RXTX */
	/* Por ultimo activamos el modulo USART cuando todo esta correctamente configurado */

	// 2.1 Comienzo por limpiar los registros, para cargar la configuración desde cero
	ptrUsartHandler->ptrUSARTx->CR1 = 0;
	ptrUsartHandler->ptrUSARTx->CR2 = 0;

	// 2.2 Configuracion del Parity:
	// Verificamos si el parity esta activado o no
    // Tenga cuidado, el parity hace parte del tamaño de los datos...
	if(ptrUsartHandler->USART_Config.USART_parity != USART_PARITY_NONE){

		ptrUsartHandler->ptrUSARTx->CR1 |= (USART_CR1_PCE);

		// Verificamos si se ha seleccionado ODD or EVEN
		if(ptrUsartHandler->USART_Config.USART_parity == USART_PARITY_EVEN){
			// Es even, entonces cargamos la configuracion adecuada
			ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_PS);

		}else{
			// Si es "else" significa que la paridad seleccionada es ODD, y cargamos esta configuracion
			ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_PS);
			ptrUsartHandler->ptrUSARTx->CR1 |= (USART_CR1_PS);

		}
	}else{
		// Si llegamos aca, es porque no deseamos tener el parity-check
		ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_PCE);
	}

	// 2.3 Configuramos el tamaño del dato
    if(ptrUsartHandler->USART_Config.USART_datasize == USART_DATASIZE_8BIT){
    	ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_M);
    }
    else{
    	ptrUsartHandler->ptrUSARTx->CR1 |= (USART_CR1_M);
    }

	// 2.4 Configuramos los stop bits (SFR USART_CR2)
	switch(ptrUsartHandler->USART_Config.USART_stopbits){
	case USART_STOPBIT_1: {
		// Debemos cargar el valor 0b00 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 &= ~(USART_CR2_STOP);
		break;
	}
	case USART_STOPBIT_0_5: {
		// Debemo scargar el valor 0b01 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 &= ~(USART_CR2_STOP);
		ptrUsartHandler->ptrUSARTx->CR2 |= (USART_CR2_STOP_0);
		break;
	}
	case USART_STOPBIT_2: {
		// Debemos cargar el valor 0b10 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 &= ~(USART_CR2_STOP);
		ptrUsartHandler->ptrUSARTx->CR2 |= (USART_CR2_STOP_1);
		break;
	}
	case USART_STOPBIT_1_5: {
		// Debemos cargar el valor 0b11 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 &= ~(USART_CR2_STOP);
		ptrUsartHandler->ptrUSARTx->CR2 |= (USART_CR2_STOP);
		break;
	}
	default: {
		// En el caso por defecto seleccionamos 1 bit de parada
		ptrUsartHandler->ptrUSARTx->CR2 &= ~(USART_CR2_STOP);
		break;
	}
	}

	// 2.5 Configuracion del Baudrate (SFR USART_BRR)
	// Ver tabla de valores (Tabla 73), Frec = 16MHz, overr = 0;
	if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_9600){
		if(ptrUsartHandler->USART_Config.USART_PLL_EN == PLL_DISABLE){

			// El valor a cargar es 104.1875 -> Mantiza = 104,fraction = 0.1875
			// Mantiza = 104 = 0x68, fraction = 16 * 0.1875 = 3
			// Valor a cargar 0x0683
			// Configurando el Baudrate generator para una velocidad de 9600bps

			ptrUsartHandler->ptrUSARTx->BRR = 0x0683;

		}else if(ptrUsartHandler->USART_Config.USART_PLL_EN == PLL_ENABLE){

			//Se configura el baudrate cuando el PLL está activado a 80MHz
			//ptrUsartHandler->ptrUSARTx->BRR = 0x208D;

			//Se configura el baudrate cuando el PLL está activado a 100MHz
			ptrUsartHandler->ptrUSARTx->BRR = 0x28B1;

		}else{
			//Se configura el baudrate por defecto a 16MHz
			ptrUsartHandler->ptrUSARTx->BRR = 0x0683;
		}
	}

	else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_19200) {
		if(ptrUsartHandler->USART_Config.USART_PLL_EN == PLL_DISABLE){

			// El valor a cargar es 52.0625 -> Mantiza = 52,fraction = 0.0625
			// Mantiza = 52 = 0x34, fraction = 16 * 0.0625 = 1
			// Valor a cargar 0x0341
			// Configurando el Baudrate generator para una velocidad de 19200bps

			ptrUsartHandler->ptrUSARTx->BRR = 0x0341;

		}else if(ptrUsartHandler->USART_Config.USART_PLL_EN == PLL_ENABLE){

			//Se configura el baudrate cuando el PLL está activado a 80MHz
			//ptrUsartHandler->ptrUSARTx->BRR = 0x1046;

			//Se configura el baudrate cuando el PLL está activado a 100MHz
			ptrUsartHandler->ptrUSARTx->BRR = 0x1458;

		}else{
			//Se configura el baudrate por defecto a 16MHz
			ptrUsartHandler->ptrUSARTx->BRR = 0x0341;
		}
	}

	else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_115200){
		if(ptrUsartHandler->USART_Config.USART_PLL_EN == PLL_DISABLE){

			// El valor a cargar es 8.6875 -> Mantiza = 8, fraction = 0.6875
			// Mantiza = 8 = 0x08, fraction = 16 * 0.6875 = 11 = B
			// Valor a cargar 0x08B
			// Configurando el Baudrate generator para una velocidad de 115200bps

			ptrUsartHandler->ptrUSARTx->BRR = 0x008B;

		}else if(ptrUsartHandler->USART_Config.USART_PLL_EN == PLL_ENABLE){

			//Se configura el baudrate cuando el PLL está activado a 80MHz
			//ptrUsartHandler->ptrUSARTx->BRR = 0x02B6;

			//Se configura el baudrate cuando el PLL está activado a 100MHz
			ptrUsartHandler->ptrUSARTx->BRR = 0x0364;

		}else{

			//Se configura el baudrate por defecto a 16MHz
			ptrUsartHandler->ptrUSARTx->BRR = 0x008B;
		}
	}

	// 2.6 Configuramos el modo: TX only, RX only, RXTX, disable
	switch(ptrUsartHandler->USART_Config.USART_mode){
	case USART_MODE_TX:
	{
		// Activamos la parte del sistema encargada de enviar
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TE;
		break;
	}
	case USART_MODE_RX:
	{
		// Activamos la parte del sistema encargada de recibir
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RE;
		break;
	}
	case USART_MODE_RXTX:
	{
		// Activamos ambas partes, tanto transmision como recepcion
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TE;
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RE;
		break;
	}
	case USART_MODE_DISABLE:
	{
		// Desactivamos ambos canales
		ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_TE);
		ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_RE);
		ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_UE);
		break;
	}

	default:
	{
		// Actuando por defecto, desactivamos ambos canales
		ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_RE);
		ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_TE);
		break;
	}
	}

	// 2.7 Activamos el modulo serial.
	if(ptrUsartHandler->USART_Config.USART_mode != USART_MODE_DISABLE){
		ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_UE);
		ptrUsartHandler->ptrUSARTx->CR1 |= (USART_CR1_UE);
	}

// 3. Activamos las interrupciones para cuando algun puerto de USART recibe un dato

	//3.1 Desactivamos las interrupciones globales
	__disable_irq();

	//3.2 Activar las interrupciones por recepcion
	if(ptrUsartHandler->USART_Config.USART_enableIntRX == USART_RX_INTERRUP_ENABLE ){
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RXNEIE;

	}else if(ptrUsartHandler->USART_Config.USART_enableIntRX == USART_RX_INTERRUP_DISABLE){
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RXNEIE;

	}else{
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RXNEIE;
	}

	//3.3 Activar las interrupciones por transmision
	if(ptrUsartHandler->USART_Config.USART_enableIntTX == USART_TX_INTERRUP_ENABLE ){
			ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TXEIE;
			if(ptrUsartHandler->ptrUSARTx == USART1){
				__NVIC_EnableIRQ(USART1_IRQn);
			}
			else if(ptrUsartHandler->ptrUSARTx == USART2){
				__NVIC_EnableIRQ(USART2_IRQn);
			}
			else if(ptrUsartHandler->ptrUSARTx == USART6){
				__NVIC_EnableIRQ(USART6_IRQn);
			}

		}else if(ptrUsartHandler->USART_Config.USART_enableIntTX == USART_TX_INTERRUP_DISABLE){
			ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TXEIE;

		}else{
			ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TXEIE;
		}

	//3.4 Matricular las interrupciones en NVIC
	//USART1
	if(ptrUsartHandler->ptrUSARTx == USART1){
		NVIC_EnableIRQ(USART1_IRQn);

	//USART2
	}else if(ptrUsartHandler->ptrUSARTx == USART2){
		NVIC_EnableIRQ(USART2_IRQn);

	//USART6
	}else if(ptrUsartHandler->ptrUSARTx == USART6){
		NVIC_EnableIRQ(USART6_IRQn);
	}


	//3.5 Activar las interrupciones globales
	__enable_irq();
}

/* Funcion para escribir un solo char */
int writeChar(USART_Handler_t *ptrUsartHandler, char dataToSend){
	while( !(ptrUsartHandler->ptrUSARTx->SR & USART_SR_TXE)){
		__NOP();
	}

	ptrUsartHandler->ptrUSARTx->DR = dataToSend;

	auxFun = 0;
	return dataToSend;
}

/*Funcion para leer un mensaje*/
uint8_t readChar(USART_Handler_t *ptrUsartHandler){
	uint8_t dataToReceive;
	while( !(ptrUsartHandler->ptrUSARTx->SR & USART_SR_RXNE)){
		__NOP();
	}

	dataToReceive = ptrUsartHandler->ptrUSARTx->DR;

	return dataToReceive;

}

/* Función para escribir un mensaje */
void writeMsg(USART_Handler_t *ptrUsartHandler, char *msgToSend){
	while(*msgToSend != '\0'){
		writeChar(ptrUsartHandler, *msgToSend);
		msgToSend++;
	}
}

//Lectura del caracter que llega por la interfase serial
uint8_t getRxData(void){
	return auxRxData;
}

//Función para enviar un Char por transmisión
int writeCharTX(USART_Handler_t *ptrUsartHandler, int dataToSend){

	//Activar la transmisión por interrupción
	ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;

	//Se carga la variable en la nueva variable global
	dataTxSend = dataToSend;

	auxFun = 0;
	return dataToSend;
}

void writeMsgTX(USART_Handler_t *ptrUsartHandler, char *msgToSend){

	//Valor de la variable global
	strncpy(auxArray, msgToSend, 100);
	dataTx = strlen(msgToSend);
	counterTx = 0;

	//Activar las interrupciones por transmisión
	ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;

	auxFun = 1;
}

void USART1_IRQHandler(void){
	//Evaluamos si la interrupcion que se dio es por RX
	if(USART1->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART1->DR;
		usart1Rx_Callback();
	}
	//Evaluamos si la interrupción que se dió es por TX
	else if(USART1->SR & USART_SR_TXE){

		//Usando la función writeCharTx
		if(auxFun == 0){
			//Guardar en el DR
			USART1->DR = dataTxSend;
			//Bajar la bandera de la interrupción
			USART1->CR1 &= ~(USART_CR1_TXEIE);
		}
		//Usando la función writeMsg
		else if(auxFun == 1){
			if (auxArray[counterTx] != '\0'){
				if(counterTx < dataTx){
					USART1->DR = auxArray[counterTx];
					while(!(USART1->SR & USART_SR_TC)){
						__NOP();
					}
					counterTx++;
				}
			}
			else if(auxArray[counterTx] == '\0'){

				//Bajar la bandera de la interrupción
				USART1->CR1 &= ~(USART_CR1_TXEIE);
			}
		}
	}
}

void USART2_IRQHandler(void){
	//Evaluamos si la interrupcion que se dio es por RX
	if(USART2->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART2->DR;
		usart2Rx_Callback();
	}
	//Evaluamos si la interrupción que se dió es por TX
	else if(USART2->SR & USART_SR_TXE){

		//Usando la función writeCharTx
		if(auxFun == 0){
			//Guardar en el DR
			USART2->DR = dataTxSend;
			//Bajar la bandera de la interrupción
			USART2->CR1 &= ~(USART_CR1_TXEIE);
		}
		//Usando la función writeMsg
		else if(auxFun == 1){
			if (auxArray[counterTx] != '\0'){
				if(counterTx < dataTx){
					USART2->DR = auxArray[counterTx];
					while(!(USART2->SR & USART_SR_TC)){
						__NOP();
					}
					counterTx++;
				}
			}
			else if(auxArray[counterTx] == '\0'){

				//Bajar la bandera de la interrupción
				USART2->CR1 &= ~(USART_CR1_TXEIE);
			}
		}
	}
}

void USART6_IRQHandler(void){
	//Evaluamos si la interrupcion que se dio es por RX
	if(USART6->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART6->DR;
		usart6Rx_Callback();
	}
	//Evaluamos si la interrupción que se dió es por TX
	else if(USART6->SR & USART_SR_TXE){

		//Usando la función writeCharTx
		if(auxFun == 0){
			//Guardar en el DR
			USART6->DR = dataTxSend;
			//Bajar la bandera de la interrupción
			USART6->CR1 &= ~(USART_CR1_TXEIE);
		}
		//Usando la función writeMsg
		else if(auxFun == 1){
			if (auxArray[counterTx] != '\0'){
				if(counterTx < dataTx){
					USART6->DR = auxArray[counterTx];
					while(!(USART6->SR & USART_SR_TC)){
						__NOP();
					}
					counterTx++;
				}
			}
			else if(auxArray[counterTx] == '\0'){

				//Bajar la bandera de la interrupción
				USART6->CR1 &= ~(USART_CR1_TXEIE);
			}
		}
	}
}

__attribute__((weak)) void usart1Rx_Callback(void){

	__NOP();
}

__attribute__((weak)) void usart2Rx_Callback(void){

	__NOP();
}

__attribute__((weak)) void usart6Rx_Callback(void){

	__NOP();
}
