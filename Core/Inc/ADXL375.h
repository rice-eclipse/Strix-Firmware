#ifndef ADXL375_IMU_H
#define ADXL375_IMU_H

#include "stm32l4xx_hal.h"

/* register defines */
#define ADXL375_DEVID			0x00
#define ADXL375_BW_RATE			0x2C //default = 0x0A (100Hz)
#define ADXL375_PWR_CTL			0x2D
#define ADXL375_INT_ENABLE		0x2E
#define ADXL375_INT_MAP			0x2F
#define ADXL375_DATA_FORMAT		0x31
#define ADXL375_DATA			0x32


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
	uint8_t txBuf[7];
	volatile uint8_t rxBuf[7];

	/* Conversion constants (raw to m/s^2) */
	float accConversion;

	/* x-y-z measurement */
	float acc_mps2[3];
} ADXL375;

/*
 *
 * INITIALISATION
 *
 */
uint8_t ADXL375_Init(ADXL375 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin);

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t ADXL375_ReadRegister(ADXL375 *imu, uint8_t regAddr, uint8_t *data);
uint8_t ADXL375_WriteRegister(ADXL375 *imu, uint8_t regAddr, uint8_t data);

/*
 *
 * POLLING
 *
 */
uint8_t ADXL375_Read(ADXL375 *imu);

/*
 *
 * DMA
 *
 */
uint8_t ADXL375_ReadDMA(ADXL375 *imu);
void 	ADXL375_ReadDMA_Complete(ADXL375 *imu);

#endif
