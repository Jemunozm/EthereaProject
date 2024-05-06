/*
 * I2CxDriver.h
 *
 *  Created on: 27.5.2023
 *      Author: Sentry
 */

#ifndef I2CxDRIVER
#define I2CxDRIVER

#include <stm32f4xx.h>

#define I2C_WRITE_DATA		0
#define I2C_READ_DATA		1

#define MAIN_CLOCK_4_MHz_FOR_I2C		4
#define MAIN_CLOCK_16_MHz_FOR_I2C		16
#define MAIN_CLOCK_20_MHz_FOR_I2C		20
#define MAIN_CLOCK_40_MHz_FOR_I2C		40
#define MAIN_CLOCK_50_MHz_FOR_I2C		50


#define I2C_MODE_SM			0
#define I2C_MODE_FM			1

#define I2C_MODE_SM_SPEED_100KHz		250
#define I2C_MODE_FM_SPEED_400KHz		42

#define I2C_MODE_SM_SPEED_100KHz_CLOCK_40MHz	200
#define I2C_MODE_FM_SPEED_400KHz_CLOCK_40MHz	33

#define I2C_MAX_RISE_TIME_SM			17
#define I2C_MAX_RISE_TIME_FM			5
#define I2C_MAX_RISE_TIME_SM_40MHz		41
#define I2C_MAX_RISE_TIME_FM_40MHz		13

#define PLL_DISABLE		0
#define PLL_ENABLE		1

typedef struct
{
	I2C_TypeDef		*ptrI2Cx;
	uint8_t			slaveAddress;
	uint8_t			modeI2C;
	uint8_t			dataI2C;
	uint8_t			PLL_ON;
}I2C_Handler_t;

/*Prototipos de las funciones publicas*/
void i2c_Config(I2C_Handler_t *ptrHandlerI2C);
void i2c_startTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_reStartTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendSlaveAddressRW(I2C_Handler_t *ptrHandlerI2C, uint8_t slaveAddress, uint8_t readOrWrite);
void i2c_sendMemoryaddress(I2C_Handler_t *ptrHandlerI2C, uint8_t memAddr);
void i2c_sendDataByte(I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite);
uint8_t i2c_readDataByte(I2C_Handler_t *ptrHandlerI2C);
void i2c_stopTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendAck(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendNoAck(I2C_Handler_t *ptrHandlerI2C);

uint8_t i2c_readSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead);
void i2c_writeSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead, uint8_t newValue);


#endif /* I2CDRIVER_H_ */
