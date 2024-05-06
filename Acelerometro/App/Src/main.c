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
GPIO_Handler_t handlerPinTX6				= {0};		//Handler Transmisión USART6
GPIO_Handler_t handlerPinRX6				= {0};		//HAndler Recepción USART6
GPIO_Handler_t handlerValvePin				= {0};		//Handler Válvula

//Definición de handlers TIM
BasicTimer_Handler_t handlerTimerBlinky 	= {0};		//Handler Timer del blinky
BasicTimer_Handler_t handlerTimerData		= {0};		//Handler Timer de la muestra de datos

//Handlers comunicación serial USART
USART_Handler_t handlerUsart1 				= {0};		//Handler USART1
USART_Handler_t handlerUsart6				= {0};		//Handler USART6

//Handlers I2C
GPIO_Handler_t handlerI2cSDA				= {0};		//Handler SDA I2C acelerómetro
GPIO_Handler_t handlerI2cSCL 				= {0};		//Handler SCL I2C acelerómetro
I2C_Handler_t handlerAccelerometer 			= {0};		//Handler de la configuración I2C acelerómetro
I2C_Handler_t handlerBarometer				= {0};		//Handler de la configuración I2C barómetro

//Definición de variables
uint8_t rxData = 0;
uint8_t flagData = 0;
uint8_t dataOn = 0;

//Definiciónes para el módulo GPS
uint8_t rxData6 = 0;

//Variables relacionadas con la comunicación I2C del Accel
uint8_t i2cBuffer = 0;
char bufferData[64] = {0};

//Variables relacionadas con el uso de los comandos en terminal
bool stringComplete;
uint16_t counterReception = 0;
char cmd[256] = {0};
char bufferReception[256] = {0};

//Deifiniciones y variables del accel
float converFactAcc = ((2/32767.0)*(9.8));			//Factor de conversión para los datos del accel
float converFactGyr = ((2/32767.0)*(250));
int16_t AccelX = 0;
int16_t AccelY = 0;
int16_t AccelZ = 0;
int16_t GyrX = 0;
int16_t GyrY = 0;
int16_t GyrZ = 0;


//Definiciones para comunicación I2C (ACCEL)
#define ACCEL_ADDRESS 	0x68;		//ID Device
#define ACCEL_XOUT_H	59 				//0x3B
#define ACCEL_XOUT_L 	60 				//0x3C
#define ACCEL_YOUT_H	61				//0x3D
#define ACCEL_YOUT_L	62 				//0x3E
#define ACCEL_ZOUT_H	63				//0x3F
#define ACCEL_ZOUT_L	64				//0x40

#define PWR_MGMT_1		107				//0x40
#define WHO_AM_I		117				//0x68

//Definición de funciones
void initSystem(void);
void MPU6050(void);
void dataAcelerometro(void);
void parseCommands(char *ptrBufferReception);

int main(void){

	//Activación del coprocesador matemático
	SCB->CPACR |= (0xF << 20);

	initSystem();

	/* Como la válvula es normalmente abierta, se requiere que inmediatamente
	 * el sistema inicie la válvula se cierre para que sea posible la conexión con
	 * el tanque de combustible
	 */
	GPIO_WritePin(&handlerValvePin, SET);

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

				//Se cargan los datos de calibración correspondientes al Accel
				dataAcelerometro();

				//Accel X
				sprintf(bufferData, "\nAceleración X: %.2f m/s²", ((float)AccelX*converFactAcc)-0.54);
				writeMsg(&handlerUsart1, bufferData);

				//Accel Y
				sprintf(bufferData, "\nAceleración Y: %.2f m/s²", ((float)AccelY*converFactAcc)+0.15);
				writeMsg(&handlerUsart1, bufferData);

				//Accel Z
				sprintf(bufferData, "\nAceleración Z: %.2f m/s²", ((float)AccelZ*converFactAcc)+0.47);
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

	if(strcmp(cmd, "acc") == 0){

		//Inicialización del MPU6050
		MPU6050();
	}
	else if(strcmp(cmd, "show") == 0){

		//Se cargan los datos de calibración correspondientes al Accel
		dataAcelerometro();

		/* Se calcula la altura (que posteriormente se mostrará) según el valor de
		 * la presión atmosférica medida ppor el Bar
		 */

		//Se debe buscar una fórmula para la altura que sí funcione

		/* Presentación de los datos correspondientes a aceleración, ángulo de rotación,
		 * presión atmosférica, temperatura y altura del dispositivo
		 */

		//Accel X
		sprintf(bufferData, "\nLa aceleración en X es: %.2f m/s² \n", ((float)AccelX*converFactAcc)-0.54);
		writeMsg(&handlerUsart1, bufferData);

		//Accel Y
		sprintf(bufferData, "\nLa aceleración en Y es: %.2f m/s² \n", ((float)AccelY*converFactAcc)+0.15);
		writeMsg(&handlerUsart1, bufferData);

		//Accel Z
		sprintf(bufferData, "\nLa aceleración en Z es: %.2f m/s² \n", ((float)AccelZ*converFactAcc)+0.47);
		writeMsg(&handlerUsart1, bufferData);

	}
	else if(strcmp(cmd, "data") == 0){

		dataOn = 1;
	}
	else if(strcmp(cmd, "stop") == 0){
		dataOn = 0;
	}
	else if(strcmp(cmd, "valve") == 0){

		//Se cambia el estado el estado del pin
		GPIOxTooglePin(&handlerValvePin);

		if(GPIO_ReadPin(&handlerValvePin) == 1){
			writeMsg(&handlerUsart1, "\nVálvula cerrada \n");
		}
		else{
			writeMsg(&handlerUsart1, "\nVálvula abierta \n");
		}
	}
	else{
		writeMsg(&handlerUsart1, "\nError!: Wrong command \n");
	}
}

void initSystem(void){

	//Configuración del Blinky
	handlerBlinky.pGPIOx 									= GPIOC;
	handlerBlinky.GPIO_PinConfig.GPIO_PinNumber 			= PIN_0;
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

	//Configuraciòn I2C (ACCEL)
	handlerAccelerometer.PLL_ON								= PLL_DISABLE;
	handlerAccelerometer.ptrI2Cx							= I2C1;
	handlerAccelerometer.slaveAddress						= ACCEL_ADDRESS;
	handlerAccelerometer.modeI2C							= I2C_MODE_FM;
	i2c_Config(&handlerAccelerometer);

	//Configuración de pines para USART2
	//TX Pin (USART2)
	handlerPinTX1.pGPIOx									= GPIOA;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinNumber				= PIN_2;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinAltFunMode			= AF7;
	GPIO_Config(&handlerPinTX1);

	//RX Pin (USART1)
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

void MPU6050(void){

	//Se lee el dispositivo para realizar la comunicación
	sprintf(bufferData, "\nWHO_AM_I? (r)\n");
	writeMsgTX(&handlerUsart1, bufferData);

	i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
	sprintf(bufferData, "\ndataRead = 0x%x \n", (unsigned int) i2cBuffer);
	writeMsgTX(&handlerUsart1, bufferData);
	rxData = '\0';

	//Estado por defecto del Accel
	sprintf(bufferData, "\nPWR_MGMT_1 state (r)\n");
	writeMsgTX(&handlerUsart1, bufferData);

	i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, PWR_MGMT_1);
	sprintf(bufferData, "\ndataRead = 0x%x \n", (unsigned int) i2cBuffer);
	writeMsgTX(&handlerUsart1, bufferData);
	rxData = '\0';

	//Se escriben los registros de calibración para el Accel
	//Se reinician las rutas de las señales del Accel y el Gyro
	i2c_writeSingleRegister(&handlerAccelerometer, 0x68, 0b11 << 1);

	//Se selecciona el modo de ultra alta resolución para el Accel
	i2c_writeSingleRegister(&handlerAccelerometer, 0x1C, 0b00 << 4);

	//Se selecciona el modo de ultra alta resolución para el Gyro
	i2c_writeSingleRegister(&handlerAccelerometer, 0x1B, 0b00 << 4);

	//Desactivamos el sensor de temperatura integrado en el MPU6050
	i2c_writeSingleRegister(&handlerAccelerometer, 0x6B, 0b01 << 3);

	//Se reinicia la configuración del accel
	i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
	rxData = '\0';

	sprintf(bufferData, "\nPWR_MGMT_1 reset (w)\n");
	writeMsgTX(&handlerUsart1, bufferData);

	writeMsg(&handlerUsart1, "\nAccel calibrado correctamente \n");
}

/* Función para guardar en datos que se puedan manipular toda la información de calibración
 * necesaria que se encuentra en los registros del dispositivo.
 */

void dataAcelerometro(void){

	uint8_t AccelX_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_L);
	uint8_t AccelX_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_H);
	AccelX = AccelX_high << 8 | AccelX_low;

	uint8_t AccelY_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_L);
	uint8_t AccelY_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_H);
	AccelY = AccelY_high << 8 | AccelY_low;

	uint8_t AccelZ_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_L);
	uint8_t AccelZ_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_H);
	AccelZ = AccelZ_high << 8 | AccelZ_low;
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

