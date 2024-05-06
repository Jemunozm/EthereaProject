/*
 * RTCDriver.c
 *
 *  Created on: 8/6/2023
 *      Author: Sentry
 */
#include "RTCxDriver.h"

// Variables para leer la fecha y la hora
uint8_t tiempo[7] = { 0 };
uint8_t fecha[7] = { 0 };

//Configuración del RTC
void RTC_Config(RTC_Config_t *ptrHandlerRTC){

	//Se activa el PWR para modificar los registros del RTC
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_DBP;

	//Habilitamos el RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;

	//Activar la señal de reloj del LSE
	RCC->BDCR |= RCC_BDCR_LSEON;

	//Configurar el LSE como reloj del RTC
	RCC->BDCR |= RCC_BDCR_RTCSEL_0;

	//Tiempo de espera
	while (!(RCC->BDCR & RCC_BDCR_LSERDY)){
		__NOP();
	}

	//Posterior a encender el RTC se modifican los bits para realizar la configuración
	RTC->WPR |= 0xCA;
	RTC->WPR |= 0x53;

	//Inicializar el RTC para la fecha, hora y prescaler
	RTC->ISR |= RTC_ISR_INIT;

	//Tiempo de espera
	while (!(RTC->ISR & RTC_ISR_INITF)) {
		__NOP();
	}

	//Configuración del Preescaler

	// Dividimos la frecuencia real del RTC configurado con el LSE ( 32768 Hz), por un primer factor de prescaler
	// sumado 1, que se programa en los bits del 16 al 22 del RTC_PRER.

	//Programamos un valor del 127. Para que asi la frecuencia de salida sea de 256 Khz
	RTC->PRER |= 127 << RTC_PRER_PREDIV_A_Pos;

	// Ahora programamos un segundo prescaler, con el cual llegaremos a nuestra salida deseada que es de 1Khz,
	//en el prescaler anterior llegamos a una salida de 256Khz, por lo cual para este prescaler lo configuramos e
	// con 255(a este tambien se le suma 1 en el registro). Teniendo ya el prescaler configurado para tener una
	// frecuencia de 1Khz
	RTC->PRER |= 255 << RTC_PRER_PREDIV_S_Pos;

	//Configuramos el RTC para que tome valores desde los contadores del calendario
	RTC->CR |= RTC_CR_BYPSHAD;

	//Inicializar los contadores del calendario
	RTC->TR = 0;
	RTC->DR = 0;

	//Configuración para la hora
	RTC->CR &= ~RTC_CR_FMT;
	if (ptrHandlerRTC->format == MODE_24H){
		RTC->CR &= ~RTC_CR_FMT;
	}
	else if(ptrHandlerRTC->format == MODE_12H){
		RTC->CR &= ~RTC_CR_FMT;
		RTC->CR |= RTC_CR_FMT;
	}

	//Se carga la configuracion de la hora y el formato
	if (ptrHandlerRTC->format == MODE_24H){
		RTC->TR &= ~RTC_TR_PM;
	}
	else if(ptrHandlerRTC->AM_PM == MODE_AM){
		RTC->TR &= ~RTC_TR_PM;
	}
	else if(ptrHandlerRTC->AM_PM == MODE_PM){
		RTC->TR &= ~RTC_TR_PM;
		RTC->TR |= RTC_TR_PM;
	}

	//Conversión de formato decimal a binario
	RTC->ISR = 0;

	//Inicializamos el RTC para poder programarlo
	RTC->ISR |= RTC_ISR_INIT;

	//Esperamos hasta que el RTC este listo para ser actualizado
	while(!(RTC->ISR & RTC_ISR_INITF)){
		__NOP();
	}
	//Configuración de la hora
	//Configuramos el HT
	RTC->TR |= (ptrHandlerRTC->hours / 10) << RTC_TR_HT_Pos;

	//Configuramos el HU
	RTC->TR |= (ptrHandlerRTC->hours % 10) << RTC_TSTR_HU_Pos;

	//Configuración de los minutos
	//Configuramos el MNT
	RTC->TR |= (ptrHandlerRTC->minuts / 10) << RTC_TR_MNT_Pos;

	//Configuramos el MNU
	RTC->TR |= (ptrHandlerRTC->minuts % 10) << RTC_TR_MNU_Pos;

	//Configuración de los segundos
	RTC->TR |= ptrHandlerRTC->seconds / 10 << RTC_TR_ST_Pos;
	RTC->TR |= ptrHandlerRTC->seconds % 10 << RTC_TR_SU_Pos;

	//Configuración de la fecha
	//Configuración de los días
	RTC->DR |= ptrHandlerRTC->days / 10 << RTC_DR_DT_Pos;
	RTC->DR |= ptrHandlerRTC->days  % 10 << RTC_DR_DU_Pos;

	//Configuración del mes
	RTC->DR |= ptrHandlerRTC->month  % 10 << RTC_DR_MU_Pos;
	RTC->DR |= ptrHandlerRTC->month  / 10 << RTC_DR_MT_Pos;

	//Se configuran las unidades y decenas
	RTC->DR |= ((ptrHandlerRTC->year  - 2000) % 10) << RTC_DR_YU_Pos;
	RTC->DR |= (ptrHandlerRTC->year  - 2000) / 10 << RTC_DR_YT_Pos;

	//Activamos la señal de reloj del RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;

	//Se desactiva el modo inicializacion
	RTC->ISR &= ~RTC_ISR_INIT;

	//Se desactiva el acceso a los registros RTC
	PWR->CR &= ~ PWR_CR_DBP;

	//Se escribe una contraseña aleatorea para proteger nuevamente los registros
	RTC->WPR = (0xFF);
}

/*Funcion para obtener la hora de los registros del RTC*/
uint8_t* read_time(void) {

	//Definición de variables para guardar los datos
	uint8_t RTC_HoraUnits = 0;
	uint8_t RTC_HoraTens = 0;
	uint8_t RTC_MinutosUnits = 0;
	uint8_t RTC_MinutosTens = 0;
	uint8_t RTC_SegundosUnits = 0;
	uint8_t RTC_SegundosTens = 0;
	uint8_t RTC_AmPm = 0;

	//Se guadan los valores de los registros en sus respectivas variables
	RTC_HoraUnits = ((RTC->TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos);
	RTC_HoraTens = ((RTC->TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos);
	RTC_MinutosUnits = ((RTC->TR & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos);
	RTC_MinutosTens = ((RTC->TR & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos);
	RTC_SegundosUnits = ((RTC->TR & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos);
	RTC_SegundosTens = ((RTC->TR & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos);
	RTC_AmPm = ((RTC->TR) >> RTC_TR_PM_Pos);

	//Se guardan las unidades y decenas
	tiempo[0] = RTC_SegundosUnits;
	tiempo[1] = RTC_SegundosTens;
	tiempo[2] = RTC_MinutosUnits;
	tiempo[3] = RTC_MinutosTens;
	tiempo[4] = RTC_HoraUnits;
	tiempo[5] = RTC_HoraTens;
	tiempo[6] = RTC_AmPm;

	return tiempo;
}

/*Funcion para obtener la fecha de los registros del RTC*/
uint8_t* read_date(void) {

	uint8_t RTC_dayUnits = 0;
	uint8_t RTC_dayTens = 0;
	uint8_t RTC_month = 0;
	uint8_t RTC_yearUnits = 0;
	uint8_t RTC_yearTens = 0;

	RTC_dayTens = ((RTC->DR & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos);
	RTC_dayUnits = ((RTC->DR & RTC_DR_DU_Msk) >> RTC_DR_DU_Pos);
	RTC_yearUnits = ((RTC->DR & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos);
	RTC_yearTens = ((RTC->DR & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos);

	// Se organiza el mes para entregarlo en un solo arreglo
	RTC_month = (((RTC->DR & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos) * 10) + (((RTC->DR & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos) % 10);

	fecha[0] = RTC_dayUnits;
	fecha[1] = RTC_dayTens;
	fecha[2] = RTC_month;
	fecha[3] = RTC_yearUnits;
	fecha[4] = RTC_yearTens;

	return fecha;
}

