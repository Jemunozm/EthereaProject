/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Santiago Valencia Roldan 
 * @brief          : Programa principal proyecto final
 ******************************************************************************
 */

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

//Definición de handlers GPIO
GPIO_Handler_t handlerBlinky 				= {0};		//Handler Blinky
GPIO_Handler_t handlerPinTX1				= {0};		//Handler Transmisión USART1
GPIO_Handler_t handlerPinRX1				= {0};		//Handler Recepción USART1

//Definición de handlers TIM
BasicTimer_Handler_t handlerTimerBlinky 	= {0};		//Handler Timer del blinky
BasicTimer_Handler_t handlerTimerData		= {0};		//Handler Timer de la muestra de datos

//Handlers comunicación serial USART
USART_Handler_t handlerUsart1 				= {0};		//Handler USART1

//Handlers I2C
GPIO_Handler_t handlerI2cSDA				= {0};		//Handler SDA I2C acelerómetro
GPIO_Handler_t handlerI2cSCL 				= {0};		//Handler SCL I2C acelerómetro
I2C_Handler_t handlerBarometer				= {0};		//Handler de la configuración I2C barómetro
I2C_Handler_t handlerTemperature			= {0};		//Hanlder de la configuración I2C SI7021

//Definición de variables
uint8_t rxData = 0;
uint8_t flagData = 0;
uint8_t dataOn = 0;

//Variables relacionadas con la comunicación I2C del Accel
uint8_t i2cBuffer = 0;
char bufferData[64] = {0};

//Variables relacionadas con el uso de los comandos en terminal
bool stringComplete;
uint16_t counterReception = 0;
char cmd[256] = {0};
char bufferReception[256] = {0};

uint16_t Temp_Code = 0;
long T = 0;
float Temp = 0;

//Definiciones para comunicación i2C (Temp si7021)
#define TEMP_ADDRESS	0x40		//ID Device
#define TEMP_VALUE		0xE0		//Read temperature value from Previous RH measurement

//Definición de funciones
void initSystem(void);
void SI7021(void);
long getTemp(void);
void parseCommands(char *ptrBufferReception);

int main(void){

	//Activación del coprocesador matemático
	SCB->CPACR |= (0xF << 20);

	initSystem();

	/* Se imprimen los mensajes de inicio para dar info al usuario
	 * sobre el manejo del dispositivo
	 */

	writeMsg(&handlerUsart1, "\nPress . \n");

	/*Loop forever*/
	while(1){

		//Creamos una cadena de caracteres con los datos que llegan por el puerto serial
		//El caracter '@' nos indica que es el final de la cadena

		if ((rxData != '\0') && (rxData != '.')){
			bufferReception[counterReception] = rxData;
			counterReception++;

			//Se define el siguiente caracter para indicar que el string está completo
			if(rxData == ','){

				stringComplete = true;

				//Agrego esta línea para crear el string con null al final
				bufferReception[counterReception-1] = '\0';

				counterReception = 0;
			}
			//Para que no vuelva a entrar, Solo cambia debido a la interrupción
			rxData = '\0';
		}
		if (rxData == '.'){
			writeMsg(&handlerUsart1, "\n~Iniciando Sistema~\n");
			writeMsg(&handlerUsart1, "\nacc  -->  Calibración del Accel-Gyro \n");
			writeMsg(&handlerUsart1, "\nshow  -->  Presenta los datos actuales capturados por los sensores \n");
			writeMsg(&handlerUsart1, "\ndata  -->  Muestra de manera periodica los datos tomados por los sesores \n");
			writeMsg(&handlerUsart1, "\nstop  -->  Detiene la muestra de datos \n");
			writeMsg(&handlerUsart1, "\nvalve  -->  Alto o bajo para cerrar o abrir la valvula de combustible \n");
			rxData = '\0';
		}

		//Hacemos un análisis de la cadena de datos obtenida
		if(stringComplete){

			parseCommands(bufferReception);
			stringComplete = false;
		}

		if(dataOn == 1){

			if(flagData == 1){

				SI7021();

				//Temperatura
				sprintf(bufferData, "\nTemperatura: %.2f°C", (float)Temp);
				writeMsg(&handlerUsart1, bufferData);

				writeMsg(&handlerUsart1, "\n--------------------------------------------------------------------------------------------");
				rxData = '\0';

				flagData = 0;
			}
		}
	}
	return 0;
}

void parseCommands(char *ptrBufferReception){

	sscanf(ptrBufferReception, "%s", cmd);


	if(strcmp(cmd, "show") == 0){

		SI7021();

		//Temperatura
		sprintf(bufferData, "\nLa temperatura es: %.2f°C \n", (float)Temp);
		writeMsg(&handlerUsart1, bufferData);

		writeMsg(&handlerUsart1, "\n--------------------------------------------------------------------------------------------\n");
		rxData = '\0';
	}
	else if(strcmp(cmd, "data") == 0){

		dataOn = 1;
	}
	else if(strcmp(cmd, "stop") == 0){
		dataOn = 0;
	}
	else{
		writeMsg(&handlerUsart1, "\nError!: Wrong command \n");
	}
}

void initSystem(void){

	//Configuración del Blinky
	handlerBlinky.pGPIOx 									= GPIOC;
	handlerBlinky.GPIO_PinConfig.GPIO_PinNumber 			= PIN_13;
	handlerBlinky.GPIO_PinConfig.GPIO_PinMode 				= GPIO_MODE_OUT;
	handlerBlinky.GPIO_PinConfig.GPIO_PinOPType 			= GPIO_OTYPE_PUSHPULL;
	handlerBlinky.GPIO_PinConfig.GPIO_PinSpeed 				= GPIO_OSPEEDR_FAST;
	handlerBlinky.GPIO_PinConfig.GPIO_PinPuPdControl 		= GPIO_PUPDR_NOTHING;
	GPIO_Config(&handlerBlinky);

	//Configuración del TIM2 (Blinky)
	handlerTimerBlinky.ptrTIMx 								= TIM2;
	handlerTimerBlinky.TIMx_Config.TIMx_mode 				= BTIMER_MODE_UP;
	handlerTimerBlinky.TIMx_Config.TIMx_speed 				= BTIMER_SPEED_100us;
	handlerTimerBlinky.TIMx_Config.TIMx_period 				= 2500;
	handlerTimerBlinky.TIMx_Config.TIMx_interruptEnable 	= 1;
	BasicTimer_Config(&handlerTimerBlinky);

	//Configuración del TIM5 (Data)
	handlerTimerData.ptrTIMx 								= TIM5;
	handlerTimerData.TIMx_Config.TIMx_mode 					= BTIMER_MODE_UP;
	handlerTimerData.TIMx_Config.TIMx_speed 				= BTIMER_SPEED_100us;
	handlerTimerData.TIMx_Config.TIMx_period 				= 30000;
	handlerTimerData.TIMx_Config.TIMx_interruptEnable 		= 1;
	BasicTimer_Config(&handlerTimerData);

	//Configuración comunicación I2C (ACCEL)
	//SDA
	handlerI2cSDA.pGPIOx									= GPIOB;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinNumber				= PIN_9;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OTYPE_OPENDRAIN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEEDR_FAST;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinAltFunMode			= AF4;
	GPIO_Config(&handlerI2cSDA);

	//SCL
	handlerI2cSCL.pGPIOx									= GPIOB;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinNumber				= PIN_8;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OTYPE_OPENDRAIN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEEDR_FAST;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinAltFunMode			= AF4;
	GPIO_Config(&handlerI2cSCL);

	//Configuraciòn I2C (BAR)
	handlerTemperature.PLL_ON								= PLL_DISABLE;
	handlerTemperature.ptrI2Cx								= I2C1;
	handlerTemperature.slaveAddress							= TEMP_ADDRESS;
	handlerTemperature.modeI2C								= I2C_MODE_FM;
	i2c_Config(&handlerTemperature);

	//Configuración de pines para USART2
	//TX Pin (USART2)
	handlerPinTX1.pGPIOx									= GPIOA;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinNumber				= PIN_2;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinAltFunMode			= AF7;
	GPIO_Config(&handlerPinTX1);

	//RX Pin (USART2)
	handlerPinRX1.pGPIOx									= GPIOA;
	handlerPinRX1.GPIO_PinConfig.GPIO_PinNumber				= PIN_3;
	handlerPinRX1.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinRX1.GPIO_PinConfig.GPIO_PinAltFunMode			= AF7;
	GPIO_Config(&handlerPinRX1);

	//Configuración de la comunicación serial USART1
	handlerUsart1.ptrUSARTx	 								= USART2;
	handlerUsart1.USART_Config.USART_baudrate				= USART_BAUDRATE_9600;
	handlerUsart1.USART_Config.USART_PLL_EN					= PLL_DISABLE;
	handlerUsart1.USART_Config.USART_datasize				= USART_DATASIZE_8BIT;
	handlerUsart1.USART_Config.USART_parity					= USART_PARITY_NONE;
	handlerUsart1.USART_Config.USART_stopbits				= USART_STOPBIT_1;
	handlerUsart1.USART_Config.USART_mode					= USART_MODE_RXTX;
	handlerUsart1.USART_Config.USART_enableIntTX			= USART_TX_INTERRUP_DISABLE;
	handlerUsart1.USART_Config.USART_enableIntRX			= USART_RX_INTERRUP_ENABLE;
	USART_Config(&handlerUsart1);
}

void SI7021(void){

	config_SysTick_ms(0);

	Temp_Code = i2c_readSingleRegister(&handlerTemperature, TEMP_VALUE);

	Temp = getTemp();


}

//Función que entrega la temperatura final ya calibrada
long getTemp(void){

	T = ((175*Temp_Code)/65536) - 46;
	return T;
}


void usart2Rx_Callback(void){
	//Leemos el valor del registro DR, donde se almacena el dato que llega.
	rxData = getRxData();
}

void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinky);
}

void BasicTimer5_Callback(void){

	//Bandera que se levanta cada medio segundo para mostrar los datos en consola
	flagData = 1;
}

