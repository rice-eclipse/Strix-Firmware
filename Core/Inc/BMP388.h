#ifndef BMP388_ALT_H
#define BMP388_ALT_H

#include "stm32l4xx_hal.h"

/* register definitions */
#define BMP388_CHIP_ID			0x00
#define BMP388_STATUS			0x03
#define BMP388_DATA				0x04
#define BMP388_INT_CTL			0x19
#define BMP388_IF_CONFIG		0x1A
#define BMP388_PWR_CTL			0x1B
#define BMP388_OSR				0x1C
#define BMP388_ODR				0x1D
#define BMP388_CALIBRATION		0x31
#define BMP388_CMD				0x7E



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

	/* measurements */
	float pressure;
	float temperature;

	uint8_t data[6];

	/* Temperature coefficients */
	uint16_t par_t1;
	uint16_t par_t2;
	int8_t par_t3;

	/* Pressure coefficients */
	int16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int8_t par_p4;
	uint16_t par_p5;
	uint16_t par_p6;
	int8_t par_p7;
	int8_t par_p8;
	int16_t par_p9;
	int8_t par_p10;
	int8_t par_p11;

} BMP388;





/*
 *
 * INITIALISATION
 *
 */
uint8_t BMP388_Init(BMP388 *alt,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin);

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t BMP388_ReadRegister(BMP388 *alt, uint8_t regAddr, uint8_t *data);
uint8_t BMP388_WriteRegister(BMP388 *alt, uint8_t regAddr, uint8_t data);

/*
 *
 * CALIBRATION
 *
 */
uint8_t BMP388_ReadCalibrationData(BMP388 *alt);

/*
 *
 * POLLING
 *
 */
uint8_t BMP388_Read(BMP388 *alt);

/*
 *
 * DMA
 *
 */
uint8_t BMP388_ReadDMA(BMP388 *alt);
void 	BMP388_ReadDMA_Complete(BMP388 *alt);

#endif
