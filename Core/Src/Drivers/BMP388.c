#include "BMP388.h"

/*
 *
 * INITIALIZATION
 *
 */
uint8_t BMP388_Init(BMP388 *alt,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin) {

	/* define a check variable */
	uint8_t status = 0;

	/* requires a falling edge on chip select pin to enable SPI mode */
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_RESET);
	HAL_Delay(1);

	/* soft reset */
	status += BMP388_WriteRegister(alt, BMP388_CMD, 0xB6); //four-wire SPI mode is default

	/* check chip ID */
	uint8_t chipID;
	status += BMP388_ReadRegister(alt, BMP388_CHIP_ID, &chipID);

	if (chipID != 0x50) {
		return 0;
	}

	/* configure OSR (over-sampling) suggested for drones: osr_p = x8; osr_t = x2 */
	status += BMP388_WriteRegister(alt, BMP388_OSR, 0x09); //x2 for both pressure and temperature to allow for 100Hz ODR

	/* configure ODR (output data rate) */
	status += BMP388_WriteRegister(alt, BMP388_ODR, 0x01); //pre-scaler 2, ODR 100Hz

	/* configure interrupts */
	status += BMP388_WriteRegister(alt, BMP388_INT_CTL, 0x20); //enable the data-ready interrupt

	/* get calibration data from the device */
	status += BMP388_ReadCalibrationData(alt);

	/* turn on measurement */
	status += BMP388_WriteRegister(alt, BMP388_PWR_CTL, 0x03); //turn on both the pressure sensor and temperature sensor

	/* set up txBuf for DMA */
	alt->txBuf[0] = BMP388_DATA | 0x80; //set read bit

	return status;
}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t BMP388_ReadRegister(BMP388 *alt, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80 , 0x00};
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(alt->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_SET);

	if (status == 1) {
		*data = rxBuf[2];
	}

	return status;
}


uint8_t BMP388_WriteRegister(BMP388 *alt, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(alt->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(alt->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_SET);

	return status;
}

/*
 *
 * CALIBRATION DATA READING
 *
 */
uint8_t BMP388_ReadCalibrationData(BMP388 *alt) {

	uint8_t txBuf[22];
	txBuf[0] = (BMP388_CALIBRATION | 0x80);
	uint8_t rxBuf[22];

	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(alt->spiHandle, txBuf, rxBuf, 22, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_SET);

	if (status != 1) {
		return 0;
	}

	/* temperature calibration data */
	alt->par_t1 = (uint16_t) ((rxBuf[2] << 8) | rxBuf[1]);
	alt->par_t2 = (uint16_t) ((rxBuf[4] << 8) | rxBuf[3]);
	alt->par_t3 = (int8_t) rxBuf[5];

	/* pressure calibration data */
	alt->par_p1 = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);
	alt->par_p2 = (int16_t) ((rxBuf[9] << 8) | rxBuf[8]);
	alt->par_p3 = (int8_t) rxBuf[10];
	alt->par_p4 = (int8_t) rxBuf[11];
	alt->par_p5 = (uint16_t) ((rxBuf[13] << 8) | rxBuf[12]);
	alt->par_p6 = (uint16_t) ((rxBuf[15] << 8) | rxBuf[14]);
	alt->par_p7 = (int8_t) rxBuf[16];
	alt->par_p8 = (int8_t) rxBuf[17];
	alt->par_p1 = (int16_t) ((rxBuf[18] << 8) | rxBuf[19]);
	alt->par_p10 = (int8_t) rxBuf[20];
	alt->par_p11 = (int8_t) rxBuf[21];

	return 1;
}

/* function provided in data sheet to convert temperature */
float BMP388_CompensateTemperature(uint32_t uncomp_temp, BMP388 *calib_data) {
	float partial_data1;
	float partial_data2;

	partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
	partial_data2 = (float)(partial_data1 * calib_data->par_t2);
	/* update the compensated temperature in calibration structure since this is needed for pressure calculation */
	calib_data->temperature = partial_data2 + (partial_data1*partial_data1) * calib_data->par_t3;

	/* return the compensated temperature */
	return calib_data->temperature;
}

/*
 * function provided in data sheet to convert pressure
 * must be run after compensate temperature, because it uses the temperature in the calculation
 */
float BMP388_CompensatePressure(uint32_t uncomp_pressure, BMP388 *calib_data) {
	/* variable to store the compensated pressure */
	float comp_pressure;
	/* temporary variable used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;

	/* calibration data */
	partial_data1 = calib_data->par_p6 * calib_data->temperature;
	partial_data2 = calib_data->par_p7 * (calib_data->temperature * calib_data->temperature);
	partial_data3 = calib_data->par_p8 * (calib_data->temperature * calib_data->temperature * calib_data->temperature);
	partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = calib_data->par_p2 * calib_data->temperature;
	partial_data2 = calib_data->par_p3 * (calib_data->temperature * calib_data->temperature);
	partial_data3 = calib_data->par_p4 * (calib_data->temperature * calib_data->temperature * calib_data->temperature);

	partial_out2 = (float)uncomp_pressure * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = (float)uncomp_pressure * (float)uncomp_pressure;
	partial_data2 = calib_data->par_p9 + calib_data->par_p10 + calib_data->temperature;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_pressure * (float)uncomp_pressure * (float)uncomp_pressure) * calib_data->par_p11;
	comp_pressure = partial_out1 + partial_out2 + partial_data4;

	calib_data->pressure = comp_pressure;
	return comp_pressure;
}


/*
 *
 * POLLING
 *
 */
uint8_t BMP388_Read(BMP388 *alt) {

	/* buffers for burst reading the six data registers */
	uint8_t txBuf[7] = {(BMP388_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[7];

	/* use the buffers to SPI read */
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(alt->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_SET);

	/* get the raw data */
	uint32_t uncomp_pressure = (uint32_t) ((rxBuf[3] << 16) | (rxBuf[2] << 8) | rxBuf[1]);
	uint32_t uncomp_temperature = (uint32_t) ((rxBuf[6] << 16) | (rxBuf[5] << 8) | rxBuf[4]);

	/* use the provided conversion functions to get the actual values */
	alt->temperature = BMP388_CompensateTemperature(uncomp_temperature, alt);
	alt->pressure = BMP388_CompensatePressure(uncomp_pressure, alt);

	return status;
}

/*
 *
 * DMA
 *
 */
uint8_t BMP388_ReadDMA(BMP388 *alt) {
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_RESET);

	if (HAL_SPI_TransmitReceive_DMA(alt->spiHandle, alt->txBuf, (uint8_t *) alt->rxBuf, 7) == HAL_OK) {

		//if the result is OK, then it will automatically call the dma_complete method, so don't need to disable the cs

		//set the flag to reading so the spi bus isn't stolen
		alt->reading = 1;
		return 1;

	} else {

		//if the operation failed, disconnect the device
		HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_SET);
		return 0;

	}
}

void BMP388_ReadDMA_Complete(BMP388 *alt) {
	/* set the reading flag to false, and disable chip select */
	HAL_GPIO_WritePin(alt->csPinBank, alt->csPin, GPIO_PIN_SET);
	alt->reading = 0;

	/* get the raw data */
	uint32_t uncomp_pressure = (uint32_t) ((alt->rxBuf[3] << 16) | (alt->rxBuf[2] << 8) | alt->rxBuf[1]);
	uint32_t uncomp_temperature = (uint32_t) ((alt->rxBuf[6] << 16) | (alt->rxBuf[5] << 8) | alt->rxBuf[4]);

	/* use the provided conversion functions to get the actual values */
	alt->temperature = BMP388_CompensateTemperature(uncomp_temperature, alt);
	alt->pressure = BMP388_CompensatePressure(uncomp_pressure, alt);
}
