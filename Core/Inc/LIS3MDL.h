/*
 * Header file for LIS3MDL driver
 *
 * Defines a structure for dealing with variables related to the sensor,
 * and function headers for communicating with the sensor
 *
 */

#ifndef LIS3MDL_MAG_H
#define LIS3MDL_MAG_H


#define LIS3MDL_ID			0x0F
#define LIS3MDL_CTRL1		0x20
#define LIS3MDL_CTRL2		0x21
#define LIS3MDL_CTRL3		0x22
#define LIS3MDL_STATUS		0x27
#define LIS3MDL_DATA		0x28

#include "stm32l4xx_hal.h"

/*
 * Define a structure to hold all of the necessary variables
 */
typedef struct {

	/* SPI */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csPinBank;
	uint16_t 		   csPin;

	/* x-y-z measurements */
	float mag[3];

} LIS3MDL;

/*
 *
 * INITIALISATION
 *
 */
uint8_t LIS3MDL_Init(LIS3MDL *mag,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin);


/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t LIS3MDL_ReadRegister(LIS3MDL *mag, uint8_t regAddr, uint8_t *data);
uint8_t LIS3MDL_WriteRegister(LIS3MDL *mag, uint8_t regAddr, uint8_t data);


/*
 *
 * POLLING
 *
 */
uint8_t LIS3MDL_DataReady(LIS3MDL *mag);
uint8_t LIS3MDL_Read(LIS3MDL *mag);

#endif
