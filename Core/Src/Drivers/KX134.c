/*
 * KX134.c
 *
 *  Created on: May 3, 2024
 *      Author: max
 */


#include "KX134.h"

/*
 *
 * INITIALISATION
 *
 */
uint8_t KX134_Init(KX134 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csAccBank, uint16_t csPin) {

	/* Store interface parameters in structure */
	imu->spiHandle 		= spiHandle;
	imu->csPinBank 		= csPinBank;
	imu->csPin 			= csPin;

	/* Clear DMA flags */
	imu->reading = 0;

	uint8_t status = 0;

	/* Check chip ID */
	uint8_t chipID_accel;
	status += KX134_ReadRegister(imu, KX_CHIP_ID, &chipID_accel);

	if (chipID_accel != 0x46) {
		status+=50;
	}
	HAL_Delay(10);

	/* use the COTR register to test if it works */
	status += KX134_WriteRegister(imu, KX_CNTL2, 0x40);
	uint8_t cotr;
	status += KX134_ReadRegister(imu, KX_COTR, &cotr);

	if (cotr != 0xAA) {
		status += 70;
	}
	HAL_Delay(1);

	/* turn off device to enable changes to configuration */
	status += KX134_WriteRegister(imu, KX_CNTL1, 0x68); // everything is correct except for bit 7 (1st), which is 0 instead of 1

	/* set output data rate to 1.6kHz, disable IIR filter */
	status += KX134_WriteRegister(imu, KX_ODCNTL, 0x8B);

	/* interrupt control */
	status += KX134_WriteRegister(imu, KX_INC1, 0xA8); // set interrupt to real time non-latched, enable pin 1, active high

	/* other interrupt control */
	status += KX134_WriteRegister(imu, KX_INC4, 0x10); // enable DRDY interrupt

	/* turn device back on */
	status += KX134_WriteRegister(imu, KX_CNTL1, 0xE8); // now turn things on, ±16G

	/* Pre-compute accelerometer conversion constant (raw to m/s^2) */
	imu->accConversion =  (9.81f / 32768.0f) * 16.0f;

	/* Set accelerometer TX buffer for DMA */
	imu->accTxBuf[0] = KX_DATA | 0x80;

	//return status;
	return chipID_accel;

}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t KX134_ReadRegister(KX134 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[3] = {regAddr | 0x80, 0x00};
	uint8_t rxBuf[3];

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 3, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[2];

	}

	return status;

}

uint8_t KX134_WriteRegister(KX134 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	//wait for the SPI bus to finish being used
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	return status;

}


/*
 *
 * POLLING
 *
 */
uint8_t KX134_Read(KX134 *imu) {

	/* Read raw accelerometer data */
	uint8_t txBuf[8] = {(BMI_ACC_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 1 byte dummy, 6 bytes data */
	uint8_t rxBuf[8];

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
	int16_t accY = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);
	int16_t accZ = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

	return status;

}

/*
 *
 * DMA
 *
 */
uint8_t KX134_ReadDMA(KX134 *imu) {

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->txBuf, (uint8_t *) imu->rxBuf, 8) == HAL_OK) {

		imu->reading = 1;
		return 1;

	} else {

		HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
		return 0;

	}

}

void KX134_ReadDMA_Complete(BMI088 *imu) {

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
	imu->reading = 0;

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((imu->accRxBuf[3] << 8) | imu->accRxBuf[2]);
	int16_t accY = (int16_t) ((imu->accRxBuf[5] << 8) | imu->accRxBuf[4]);
	int16_t accZ = (int16_t) ((imu->accRxBuf[7] << 8) | imu->accRxBuf[6]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

}
