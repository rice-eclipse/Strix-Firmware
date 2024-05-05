/*
 * KX134.h
 *
 *  Created on: May 3, 2024
 *      Author: max
 */
#ifndef KX134_IMU_H
#define KX134_IMU_H

#include "stm32l4xx_hal.h"

/* Register defines */
#define KX_CHIP_ID			0x13
#define KX_DATA				0x08
#define KX_CNTL_1			0x1b // note: to change any values, PC1 bit in CNTL1 register must first be set to “0”
#define KX_CNTL_2			0x1c
#define KX_ODCNTL			0x21
#define KX_INC1				0x22 // interrupt control for pin 1
#define KX_INC4				0x25 // interrupt control for pin 1
//command testing
#define KX_COTR				0x12 // controlled by bit in CNTL_2


/*
 * Define a structure to hold all of the necessary variables
 */
typedef struct {

	/* SPI */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csPinBank;
	uint16_t 		   csPin;

	/* DMA */
	uint8_t reading;
	uint8_t txBuf[8];
	volatile uint8_t rxBuf[8];

	/* Conversion constants (raw to m/s^2) */
	float accConversion;

	/* x-y-z measurements */
	float acc_mps2[3];

} KX134;

/*
 *
 * INITIALISATION
 *
 */
uint8_t KX134_Init(KX134 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin);

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t KX134_ReadRegister(KX134 *imu, uint8_t regAddr, uint8_t *data);
uint8_t KX134_WriteRegister(KX134 *imu, uint8_t regAddr, uint8_t data);

/*
 *
 * POLLING
 *
 */
uint8_t KX134_Read(KX134 *imu);

/*
 *
 * DMA
 *
 */
uint8_t KX134_ReadAccelerometerDMA(KX134 *imu);
void 	KX134_ReadAccelerometerDMA_Complete(KX134 *imu);

#endif


