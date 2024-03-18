#include "ADXL375.h"

/*
 *
 * INITIALIZATION
 *
 * HAL_Delay used because this is intended to be run before freeRTOS is started
 *
 */
uint8_t ADXL375_Init(ADXL375 *imu, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPinBank, uint16_t csPin) {

	/* store everything in the ADXL375 struct that was passed in */
	imu->spiHandle = spiHandle;
	imu->csPinBank = csPinBank;
	imu->csPin = csPin;

	/* clear DMA flag */
	imu->reading = 0;

	/* stores the result of each operation (success or failure) */
	uint8_t status = 0;

	/* enable SPI mode with a falling edge */
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
	HAL_Delay(10);

	/* check the device ID */
	uint8_t chipID;
	status += ADXL375_ReadRegister(imu, ADXL375_DEVID, &chipID);

	//if the chip ID is not what we are expecting, return 0 (no successes)
	if (chipID != 0xE5) {
		status+=100;
	}

	HAL_Delay(1);

	/* set the DATA_FORMAT register to use 4-wire SPI */
	status += ADXL375_WriteRegister(imu, ADXL375_DATA_FORMAT, 0x4F); // no self-test; 4-wire SPI; interrupt active high; MSB first

	/* set the BW_RATE register to control data rate and band limiting */
	status += ADXL375_WriteRegister(imu, ADXL375_BW_RATE, 0x0A); // 100 Hz, 50 Hz band-limited

	/* map the data-ready interrupt to pin 1 */
	status += ADXL375_WriteRegister(imu, ADXL375_INT_MAP, 0x00); //map all interrupts to 1

	/* enable the data ready interrupt */
	status += ADXL375_WriteRegister(imu, ADXL375_INT_ENABLE, 0x80);

	/* turn on the device */
	status += ADXL375_WriteRegister(imu, ADXL375_PWR_CTL, 0x08); //link off, auto-sleep off, measure on, sleep off, sleep data rate 8Hz

	/* Pre-compute conversion to m/s from raw Accelerometer data */
	imu->accConversion = 9.81f / 20.5f; //20.5 LSB per G

	/* set up Accelerometer txBuf for DMA */
	imu->txBuf[0] = ADXL375_DATA | 0xC0; //set write bit and multiple bytes bit

	return status;
}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

uint8_t ADXL375_ReadRegister(ADXL375 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80 , 0x00}; //or with 0x80 to set the 'read' bit
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	if (status == 1) {
		*data = rxBuf[2];
	}

	return status;
}

uint8_t ADXL375_WriteRegister(ADXL375 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	return status;

}


/*
 *
 * POLLING
 *
 */
uint8_t ADXL375_Read(ADXL375 *imu) {

	/* Read raw accelerometer data */
	uint8_t txBuf[7] = {(ADXL375_DATA | 0xC0), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[7];

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((rxBuf[2] << 8) | rxBuf[1]);
	int16_t accY = (int16_t) ((rxBuf[4] << 8) | rxBuf[3]);
	int16_t accZ = (int16_t) ((rxBuf[6] << 8) | rxBuf[5]);

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
uint8_t ADXL375_ReadDMA(ADXL375 *imu) {

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);

	if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->txBuf, (uint8_t *) imu->rxBuf, 7) == HAL_OK) {

		//if the result is OK, then it will automatically call the dma_complete method, so don't need to disable the cs

		//set the flag to reading so the spi bus isn't stolen
		imu->reading = 1;
		return 1;

	} else {

		//if the operation failed, disconnect the device
		HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
		return 0;

	}
}

void ADXL375_ReadDMA_Complete(ADXL375 *imu) {

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
	imu->reading = 0;

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((imu->rxBuf[2] << 8) | imu->rxBuf[1]);
	int16_t accY = (int16_t) ((imu->rxBuf[4] << 8) | imu->rxBuf[3]);
	int16_t accZ = (int16_t) ((imu->rxBuf[6] << 8) | imu->rxBuf[5]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;



}
