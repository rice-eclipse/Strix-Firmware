/*
 * Driver for LIS3MDL 3 axis Magnetometer
 */

#include "LIS3MDL.h"


/*
 * Setup
 *
 */
uint8_t LIS3MDL_Init(LIS3MDL *mag,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin) {

	/* enter values into the structure */
	mag->spiHandle = spiHandle;
	mag->csPin = csPin;
	mag->csPinBank = csPinBank;

	/* define a status for function success */
	uint8_t status = 0;

	/* check the device id */
	uint8_t device_id;
	status += LIS3MDL_ReadRegister(mag, LIS3MDL_ID, &device_id);

	if (device_id != 0x3D) {
		return 0;
	}

	/* set the first control register; x and y axes -> high performance; ODR -> 80Hz */
	status += LIS3MDL_WriteRegister(mag, LIS3MDL_CTRL1, 0b01011100);

	/* set the second control register; ±4 gauss */
	status += LIS3MDL_WriteRegister(mag, LIS3MDL_CTRL2, 0x00);

	/* set the third control register; disable low power mode; 4-wire SPI; single conversion mode */
	status += LIS3MDL_WriteRegister(mag, LIS3MDL_CTRL3, 0x01);

	return status;
}


/*
 * Low-level reading and writing registers
 *
 */
uint8_t LIS3MDL_ReadRegister(LIS3MDL *mag, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80 , 0x00};
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(mag->csPinBank, mag->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(mag->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(mag->csPinBank, mag->csPin, GPIO_PIN_SET);

	if (status == 1) {
		*data = rxBuf[2];
	}

	return status;
}


uint8_t LIS3MDL_WriteRegister(LIS3MDL *mag, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(mag->csPinBank, mag->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(mag->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(mag->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(mag->csPinBank, mag->csPin, GPIO_PIN_SET);

	return status;
}

/*
 *
 * POLLING
 *
 */
uint8_t LIS3MDL_DataReady(LIS3MDL *mag) {

	/* read the status register */
	uint8_t status;
	uint8_t status_register;

	status = LIS3MDL_ReadRegister(mag, LIS3MDL_STATUS, &status_register);

	if ((status_register | 0xF7) == 0xFF) {
		return 0;
	}

	return status;
}

uint8_t LIS3MDL_Read(LIS3MDL *mag) {

	/* check data availability */
	if(LIS3MDL_DataReady(mag) == 0) {
		return 0;
	}

	/* buffers for burst reading the six data registers */
	uint8_t txBuf[7] = {(LIS3MDL_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[7];

	/* use the buffers to SPI read */
	HAL_GPIO_WritePin(mag->csPinBank, mag->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(mag->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(mag->csPinBank, mag->csPin, GPIO_PIN_SET);

	/* get the raw data */
	uint16_t x_gauss = (rxBuf[2] << 8) + rxBuf[1];
	uint16_t y_gauss = (rxBuf[4] << 8) + rxBuf[3];
	uint16_t z_gauss = (rxBuf[6] << 8) + rxBuf[5];

	/* use conversion value specified for ±4 gauss on datasheet pg. 4 */
	mag->mag[0] = x_gauss/6842;
	mag->mag[1] = y_gauss/6842;
	mag->mag[2] = z_gauss/6842;

	/* return the status of the SPI operation, 1 for success */
	return status;
}
