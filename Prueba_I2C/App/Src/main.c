/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Santiago Valencia Roldan 
 * @brief          : Main program body
 ******************************************************************************
*/
/*
 * Descomentar la siguiente linea y agregar al main si se requiere el uso
 * de la FPU.
 * SCB->CPACR |= (0xF << 20);
 * Activacion del coprocesador matematico
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"
#include "USARTxDriver.h"
#include "I2CxDriver.h"
#include "PWMDriver.h"
#include "PLLDriver.h"
#include "SysTickDriver.h"

//Definiciòn de variables
GPIO_Handler_t handlerLedOK = {0};
GPIO_Handler_t handlerPinTX = {0};
GPIO_Handler_t handlerPinRX = {0};

BasicTimer_Handler_t handlerStateOKTimer = {0};

USART_Handler_t handlerCommTerminal = {0};
uint8_t rxData = 0;
char bufferData[64] = "Accel MP-6050 testing...";

uint32_t systemTicks = 0;
uint32_t systemTicksStart = 0;
uint32_t systemticksEnd = 0;

//Configuraciòn para el I2C
GPIO_Handler_t handlerI2cSDA = {0};
GPIO_Handler_t handlerI2cSCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
uint8_t i2cBuffer = 0;

#define ACCEL_ADDRESS 	0b1101001;		//0xD2 --> Direcciòn del Accel con Logic_1
#define ACCEL_XOUT_H	59 				//0x3B
#define ACCEL_XOUT_L 	60 				//0x3C
#define ACCEL_YOUT_H	61				//0x3D
#define ACCEL_YOUT_L	62 				//0x3E
#define ACCEL_ZOUT_H	63				//0x3F
#define ACCEL_ZOUT_L	64				//0x40

#define PWR_MGMT_1		107
#define WHO_AM_I		117

//Definiciòn de los prototipos de funciones
void initSystem(void);

int main(void){

	//Llamamos la funciòn que inicializa el hardware del sistema
	initSystem();

	//Imprimir un mensaje de inicio
	writeMsg(&handlerCommTerminal, bufferData);

	/*Loop forever*/
	while(1){

		//Hacemos un eco con el valor que llega por el serial
		if(rxData != '\0'){
			writeChar(&handlerCommTerminal, rxData);

			if(rxData == 'w'){
				sprintf(bufferData, "WHO_AM_I? (r)\n");
				writeMsg(&handlerCommTerminal, bufferData);

				i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
				sprintf(bufferData, "dataRead = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			else if(rxData == 'p'){
				sprintf(bufferData, "PWR_MGMT_1 state (r)\n");
				writeMsg(&handlerCommTerminal, bufferData);

				i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, PWR_MGMT_1);
				sprintf(bufferData, "dataRead = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			else if(rxData == 'r'){
				sprintf(bufferData, "PWR_MGMT_1 reset (w)\n");
				writeMsg(&handlerCommTerminal, bufferData);

				i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
				rxData = '\0';
			}
			else if(rxData == 'x'){
				sprintf(bufferData, "Axis X data (r)\n");
				writeMsg(&handlerCommTerminal, bufferData);

				uint8_t AccelX_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_L);
				uint8_t AccelX_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_H);
				int16_t AccelX = AccelX_high << 8 | AccelX_low;

				sprintf(bufferData, "AccelX = %d \n", (int) AccelX);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			else if(rxData == 'y'){
				sprintf(bufferData, "Axis Y data (r)\n");
				writeMsg(&handlerCommTerminal, bufferData);

				uint8_t AccelY_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_L);
				uint8_t AccelY_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_H);
				int16_t AccelY = AccelY_high << 8 | AccelY_low;
				sprintf(bufferData, "AccelY = %d \n", (int) AccelY);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			else if(rxData == 'z'){
				sprintf(bufferData, "Axis Z data (r)\n");
				writeMsg(&handlerCommTerminal, bufferData);

				uint8_t AccelZ_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_L);
				uint8_t AccelZ_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_H);
				int16_t AccelZ = AccelZ_high << 8 | AccelZ_low;
				sprintf(bufferData, "AccelZ = %d \n", (int) AccelZ);
				writeMsg(&handlerCommTerminal, bufferData);
				rxData = '\0';
			}
			else{
				rxData = '\0';
			}
		}
	}
	return 0;
}

void initSystem(void){

	//Configuraciòn del Blinky
	handlerLedOK.pGPIOx									= GPIOC;
	handlerLedOK.GPIO_PinConfig.GPIO_PinNumber			= PIN_0;
	handlerLedOK.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerLedOK.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerLedOK.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEEDR_FAST;
	handlerLedOK.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	GPIO_Config(&handlerLedOK);

	//Llevamos el led a un estado de encendido
	GPIO_WritePin(&handlerLedOK, SET);

	//Configuraciòn de los pines para USART
	handlerPinTX.pGPIOx										= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber				= PIN_2;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OTYPE_PUSHPULL;
	handlerPinTX.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerPinTX.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEEDR_FAST;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode			= AF7;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx										= GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber				= PIN_3;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OTYPE_PUSHPULL;
	handlerPinRX.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	handlerPinRX.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEEDR_FAST;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode			= AF7;
	GPIO_Config(&handlerPinRX);

	//Configuraciòn del puerto serial
	handlerCommTerminal.ptrUSARTx	 								= USART2;
	handlerCommTerminal.USART_Config.USART_baudrate					= USART_BAUDRATE_115200;
	handlerCommTerminal.USART_Config.USART_datasize					= USART_DATASIZE_8BIT;
	handlerCommTerminal.USART_Config.USART_parity					= USART_PARITY_NONE;
	handlerCommTerminal.USART_Config.USART_stopbits					= USART_STOPBIT_1;
	handlerCommTerminal.USART_Config.USART_mode						= USART_MODE_RXTX;
	handlerCommTerminal.USART_Config.USART_enableIntTX				= USART_TX_INTERRUP_DISABLE;
	handlerCommTerminal.USART_Config.USART_enableIntRX				= USART_RX_INTERRUP_ENABLE;
	USART_Config(&handlerCommTerminal);

	//Configuraciòn del TIM
	handlerStateOKTimer.ptrTIMx 								= TIM2;
	handlerStateOKTimer.TIMx_Config.TIMx_mode					= BTIMER_MODE_UP;
	handlerStateOKTimer.TIMx_Config.TIMx_speed					= BTIMER_SPEED_100us;
	handlerStateOKTimer.TIMx_Config.TIMx_period					= 2500;
	handlerStateOKTimer.TIMx_Config.TIMx_interruptEnable		= 1;
	BasicTimer_Config(&handlerStateOKTimer);

	//Configuraciònes de los pines para SDA y SCL
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

	//Configuraciòn I2C
	handlerAccelerometer.ptrI2Cx		= I2C1;
	handlerAccelerometer.modeI2C		= I2C_MODE_FM;
	handlerAccelerometer.slaveAddress	= ACCEL_ADDRESS;
	i2c_Config(&handlerAccelerometer);
}

void usart2Rx_Callback(void){
	//Leemos elvalor del registro DR, donde se almacena el dato que llega.
	//Esto ademàs debe bajar la bandera de la interrupciòn
	rxData = getRxData();
}

void BasicTimer2_Callback(void){
	//Hacemos un Blinky para indicar que el equipo està funcionando correctamente
	GPIOxTooglePin(&handlerLedOK);
}


