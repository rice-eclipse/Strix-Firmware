/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "quadspi.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "BMI088.h"
#include "BMP388.h"
#include "LIS3MDL.h"
#include "KX134.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_RECORDING_LENGTH  0x400000 // 4 MByte => 32 Bytes per sample => 131,072 samples => 200Hz => ~11 min
#define DATA_CONFIG_LOCATION 0x00 // start of the memory chip
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//USB transmission code
uint8_t txBuffer[64];
uint8_t rxBuffer[64];
uint32_t txDataTransmitTick;
uint8_t usbStatus;
/*define the status of the USB transmission--
 * 		0 is nothing
 * 		1 is waiting for a recording choice
 * 		2 is waiting for confirmation to clear flash
 * 		3 is sending data ticks
 */

//QSPI Memory code
uint8_t flashWriteBuffer[256];
uint8_t flashReadBuffer[64];
uint8_t dataWritten;

uint32_t writeHead;
uint32_t writeHeadEnd;

uint8_t recordingData;
uint8_t newData; //store which sensors have new data

//status LED and Buzzer code
uint32_t ledTick;
uint16_t ledTimeout;
uint32_t buzzerTick;
uint8_t buzzerEnable;

//devices using hspi1
BMI088 bmi088;
BMP388 bmp388;
uint32_t bmiTick;
uint32_t bmpTick;

//devices using hspi3
LIS3MDL lis3mdl;
KX134 kx134;
uint32_t spi3Tick;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */

  /*
   *
   * SEND STARTING COMMUNICATION TO LAPTOP
   *
   *
   */

  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Waiting for connection..."));
  HAL_Delay(4000); //wait ten seconds so user has time to connect serial port viewer

  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n# ------------------------------ \n#\n"));
  HAL_Delay(1); //this is a shitty fucking solution but whatever I'm not getting paid
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "# StrixFlight v0.0\n#\t\t\tRunning on Strix v3.0 \n#\n#\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "# ------------------------------ \n# Initializing sensors...\n"));
  HAL_Delay(1);


  /*
   *
   * INITIALIZE SENSORS
   *
   *
   */

  /*
   * devices that use hspi1
   */

  /* Initialize BMI088 */
  uint8_t bmi_status = BMI088_Init(&bmi088, &hspi1, BMI088_Accel_NCS_GPIO_Port, BMI088_Accel_NCS_Pin, BMI088_Gyro_NCS_GPIO_Port, BMI088_Gyro_NCS_Pin);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t\tStatus of BMI088: %i\n", bmi_status));
  HAL_Delay(1);

  /* Initialize BMP388 */
  /*
  uint8_t bmp_status = BMP388_Init(&bmp388, &hspi1, BMP388_NCS_GPIO_Port, BMP388_NCS_Pin);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "%i\n", bmp_status));
  HAL_Delay(1);
  */


  /*
   * devices that use hspi3
   */

  uint8_t kx_status = KX134_Init(&kx134, &hspi3, KX134_NCS_GPIO_Port, KX134_NCS_Pin);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t\tStatus of KX134: %i\n", kx_status));
  HAL_Delay(1);
  /* Initialize LIS3MDL */
  uint8_t lis3mdl_status = LIS3MDL_Init(&lis3mdl, &hspi3, LIS3MDL_NCS_GPIO_Port, LIS3MDL_NCS_Pin);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t\tStatus of LIS3MDL: %i\n", lis3mdl_status));
  HAL_Delay(1);


  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "# Finished Sensor Initialization\n"));
  HAL_Delay(1);

  /*
   *
   * QSPI Memory Status
   *
   */
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "# Reading Status from Flash Memory\n"));
  HAL_Delay(10);
  if (CSP_QUADSPI_Init() != HAL_OK) {
	  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t\tError Configuring QSPI Flash Chip:\n#\t\t\tW25Q128JVSIQ\n#\n"));
	  HAL_Delay(1);
  }

  //get 16 bytes, which hold configuration information
  CSP_QSPI_Read(flashReadBuffer, DATA_CONFIG_LOCATION, 16);

  //calculate the number of recordings
  uint8_t l = 7;
  while (flashReadBuffer[0] >>= 1) { --l; }
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t\tNumber of Recordings: %i out of 7 possible\n", l));
  HAL_Delay(1);
  if (flashReadBuffer[0] > 0x4) {
	  HAL_GPIO_WritePin(Continuity_LED_E_GPIO_Port, Continuity_LED_E_Pin, GPIO_PIN_SET);
  }

  /*
   * TELL THE USER HOW TO COMMUNICATE
   */
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\n#\t1 to test USB functionality\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t2 to access a recording\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t3 to delete data (confirmation required)\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t4 to toggle live sensor data reporting\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t5 to test writing to the flash memory\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t6 to read the cofiguration bytes back\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t7 to write one data set to 256 in mem\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t8 to clear the first sector\n#\n"));
  HAL_Delay(1);

  //set the usbStatus to 0 to confirm nothing has been requested yet
  usbStatus = 0;
  //set the ledTimeout to 500ms for nominal state
  ledTimeout = 500;
  //enable the buzzer
  buzzerEnable = 1;
  //set the writeHead
  writeHead = MEMORY_SECTOR_SIZE + flashReadBuffer[0]*DATA_RECORDING_LENGTH;
  //set the data recording end
  writeHeadEnd = MEMORY_SECTOR_SIZE + (flashReadBuffer[0]+1)*DATA_RECORDING_LENGTH;
  //set recordingData to 0 bc we haven't detected a launch yet
  recordingData = 0;
  //set the newData variable to 0 to prepare for recording data
  newData = 0;





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	/*
	 *
	 * COMMUNICATION WITH HOST COMPUTER (IF CONNECTED VIA USB)
	 *
	 *
	 */

	/*
	 *
	 * USBSTATUS--
	 * 	0 = normal
	 * 	3 = transmitting data ticks
	 * 	2 = reading stored data off of flash chip
	 * 	4 = deleting flash chip
	 *
	 */

	//check if there are any commands from the user
	if (rxBuffer[0] != 0) {
		if (usbStatus == 0 || usbStatus == 3) {
			//check what was sent
			if (rxBuffer[0] == '1') {
				//usb transmission test
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nUSB transmission confirmed functional\n"));
			}
			if (rxBuffer[0] == '2') {
				//check status of the flash memory chip
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nGetting Flash Memory Data...\n"));
				CSP_QSPI_Read(flashReadBuffer, DATA_CONFIG_LOCATION, 16);
				//calculate the number of recordings
				uint8_t l = 7;
				while (flashReadBuffer[0] >>= 1) { --l; }
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n\tThere are %i recordings\n", l));
				HAL_Delay(1);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n\tChoose a recording number (starting at 0)\n"));
				usbStatus = 1;
			}
			if (rxBuffer[0] == '3') {
				//send the confirmation message and change current state
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nYou have requested to delete all flight recordings\n"));
				HAL_Delay(1);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n\t\t\tEnter 'y' to proceed\n"));
				HAL_Delay(1);
				ledTimeout = 250;
				usbStatus = 2;
			}
			if (rxBuffer[0] == '4') {
				if (usbStatus != 3) {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nStarting Data Transmission\n"));
					usbStatus = 3;
				} else {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nStopping Data Transmission\n"));
					usbStatus = 0;
				}
			}
			if (rxBuffer[0] == '5') {
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nWriting config bytes\n"));
				//write new data
				for (uint8_t i = 0; i < 16; i++) {
					flashWriteBuffer[i] = 64+i;
				}
				CSP_QSPI_WriteMemory(flashWriteBuffer, DATA_CONFIG_LOCATION, 16);
				//reading new configuration
				CSP_QSPI_Read(flashReadBuffer, DATA_CONFIG_LOCATION, 16);
				//transmit to the host computer
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", flashReadBuffer[0], flashReadBuffer[1],
						flashReadBuffer[2], flashReadBuffer[3], flashReadBuffer[4], flashReadBuffer[5], flashReadBuffer[6], flashReadBuffer[7], flashReadBuffer[8],
						flashReadBuffer[9], flashReadBuffer[10], flashReadBuffer[11], flashReadBuffer[12], flashReadBuffer[13], flashReadBuffer[14], flashReadBuffer[15]));
				HAL_Delay(1);
			}
			if (rxBuffer[0] == '6') {
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nReading Config Bytes\n"));
				//reading new configuration
				CSP_QSPI_Read(flashReadBuffer, DATA_CONFIG_LOCATION, 16);
				//transmit to the host computer
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", flashReadBuffer[0], flashReadBuffer[1],
						flashReadBuffer[2], flashReadBuffer[3], flashReadBuffer[4], flashReadBuffer[5], flashReadBuffer[6], flashReadBuffer[7], flashReadBuffer[8],
						flashReadBuffer[9], flashReadBuffer[10], flashReadBuffer[11], flashReadBuffer[12], flashReadBuffer[13], flashReadBuffer[14], flashReadBuffer[15]));
				HAL_Delay(1);
			}
			if (rxBuffer[0] == '7') {
				//store data from the gyroscope into the first 32 bytes of the flashWriteBuffer
				uint32_t timeValue = HAL_GetTick();
				uint8_t timeArray[4];
				memcpy(timeArray, &timeValue, sizeof(uint32_t));
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nTime: %02X %02X %02X %02X\n", timeArray[0], timeArray[1], timeArray[2], timeArray[3]));

				for (uint8_t i = 0; i < 4; i++) {
					flashWriteBuffer[i] = timeArray[i];
				}
				for (uint8_t i = 0; i < 6; i++) {
					flashWriteBuffer[i+4] = bmi088.gyr_data[i];
				}
				for (uint8_t i = 0; i < 6; i++) {
					flashWriteBuffer[i+10] = bmi088.acc_data[i];
				}
				for (uint8_t i = 0; i < 6; i++) {
					flashWriteBuffer[i+16] = lis3mdl.data[i];
				}
				for (uint8_t i = 0; i < 6; i++) {
					flashWriteBuffer[i+22] = kx134.data[i];
				}
				for (uint8_t i = 0; i < 3; i++) {
					flashWriteBuffer[i+28] = bmp388.data[i];
				}
				flashWriteBuffer[31] = 0xFF;
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n%02X %02X", flashWriteBuffer[0], flashWriteBuffer[1]));
				HAL_Delay(1);

				//write data to the flash chip
				CSP_QSPI_WriteMemory(flashWriteBuffer, 256, 32);

				CSP_QSPI_Read(flashReadBuffer, 256, 32);
				//transmit to the host computer
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n%02X %02X", flashReadBuffer[0], flashReadBuffer[1]));
				HAL_Delay(1);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n%02X %02X %02X %02X %02X %02X", flashReadBuffer[2], flashReadBuffer[3],
						flashReadBuffer[4], flashReadBuffer[5], flashReadBuffer[6], flashReadBuffer[7]));
				HAL_Delay(1);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n%02X %02X %02X %02X %02X %02X", flashReadBuffer[8], flashReadBuffer[9],
						flashReadBuffer[10], flashReadBuffer[11], flashReadBuffer[12], flashReadBuffer[13]));
				HAL_Delay(1);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n%02X %02X %02X %02X %02X %02X", flashReadBuffer[14], flashReadBuffer[15],
						flashReadBuffer[16], flashReadBuffer[17], flashReadBuffer[18], flashReadBuffer[19]));
				HAL_Delay(1);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n%02X %02X %02X %02X %02X %02X", flashReadBuffer[20], flashReadBuffer[21],
						flashReadBuffer[22], flashReadBuffer[23], flashReadBuffer[24], flashReadBuffer[25]));
				HAL_Delay(1);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n%02X %02X %02X %02X %02X %02X\n", flashReadBuffer[26], flashReadBuffer[27],
						flashReadBuffer[28], flashReadBuffer[29], flashReadBuffer[30], flashReadBuffer[31]));
				HAL_Delay(1);
			}
			if (rxBuffer[0] == '8') {
				//need to erase flash memory before stuff can be re-written
				if(CSP_QSPI_EraseSector(0, MEMORY_SECTOR_SIZE) == HAL_OK) {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nFirst flash memory sector rewritten\n"));
				}
			}
		} else if (usbStatus == 1) {
			//usbStatus of 1 corresponds to the user requesting a recording
			//the number that was transmitted must be the desired recording
			if (rxBuffer[0] > 47 && rxBuffer[0] < 58) { //ASCII values for 0-9
				//calculate starting address
				uint32_t start_address = (DATA_RECORDING_LENGTH * (rxBuffer[0]-48)) + MEMORY_SECTOR_SIZE; //add sector b.c. writeHead starts after the first sector (config bytes)
				uint32_t address = start_address;
				//read data page by page and transmit through serial port
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nTime,\t\t\tBMI088 Gyro,\t\t\tBMI088 Accel,\t\t\t"));
				HAL_Delay(1);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "LIS3MDL,\t\t\tKX134,\t\t\tBMP388\n\n"));
				do {
					//read 32 bytes at a time
					CSP_QSPI_Read(flashReadBuffer, address, 32);
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n"));
					HAL_Delay(1);
					//print out the 8 bytes to the computer
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "%02X%02X%02X%02X,    %02X%02X, %02X%02X, %02X%02X,    %02X%02X, %02X%02X, %02X%02X,     %02X%02X", flashReadBuffer[0], flashReadBuffer[1],
												flashReadBuffer[2], flashReadBuffer[3], flashReadBuffer[4], flashReadBuffer[5], flashReadBuffer[6], flashReadBuffer[7], flashReadBuffer[8],
												flashReadBuffer[9], flashReadBuffer[10], flashReadBuffer[11], flashReadBuffer[12], flashReadBuffer[13], flashReadBuffer[14], flashReadBuffer[15], flashReadBuffer[16], flashReadBuffer[17]));
					HAL_Delay(1);
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, ", %02X%02X, %02X%02X,     %02X%02X, %02X%02X, %02X%02X,     %02X%02X%02X", flashReadBuffer[18], flashReadBuffer[19], flashReadBuffer[20], flashReadBuffer[21],
																	flashReadBuffer[22], flashReadBuffer[23], flashReadBuffer[24], flashReadBuffer[25], flashReadBuffer[26], flashReadBuffer[27],
																	flashReadBuffer[28], flashReadBuffer[29], flashReadBuffer[30]));
					address+=32;
				} while (address < start_address+DATA_RECORDING_LENGTH);
			} else {
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nExit confirmed\n"));
			}
			usbStatus = 0;
			ledTimeout = 500;
		} else if (usbStatus == 2) {
			//check if we have confirmation to delete the entire chip
			if (rxBuffer[0] == 'y') {

				CSP_QSPI_Erase_Chip();
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nChip Rewritten\n"));
				for (uint8_t i = 0; i < 16; i++) {
					flashWriteBuffer[i] = i;
				}
				flashWriteBuffer[0] = 255;
				CSP_QSPI_WriteMemory(flashWriteBuffer, DATA_CONFIG_LOCATION, 16);
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nFlash config rewritten\n"));
			} else {
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\nExit Confirmed\n"));
			}
			usbStatus = 0;
			ledTimeout = 500;
		}
		//reset the rxBuffer so we don't accidentally do things multiple times
		memset (rxBuffer, '\0', 64);
	}

	//check if it's time to transmit the next data tick yet
	if (HAL_GetTick() > txDataTransmitTick + 1000 && usbStatus == 3) {
		//update the last time data was transmitted
		txDataTransmitTick = HAL_GetTick();

		//transmit the current data from the sensor structures
		CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "deg/sec: x: %.3f   y: %.3f   z: %.3f\n",
					bmi088.gyr_rps[0],bmi088.gyr_rps[1],bmi088.gyr_rps[2]));
		HAL_Delay(1);
		CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "BMI m/s^2: x: %.3f   y: %.3f   z: %.3f\n", bmi088.acc_mps2[0],bmi088.acc_mps2[1],bmi088.acc_mps2[2]));
		HAL_Delay(1);
		//CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Pressure: %.5f; Temperature: %.5f\n", bmp388.pressure,bmp388.temperature));
		//HAL_Delay(1);
		CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "KX134 m/s^2: x: %.3f   y: %.3f   z: %.3f\n", kx134.acc_mps2[0],kx134.acc_mps2[1],kx134.acc_mps2[2]));
		HAL_Delay(1);
		CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "LIS3MDL: x: %.3f   y: %.3f   z: %.3f\n", lis3mdl.mag[0],lis3mdl.mag[1],lis3mdl.mag[2]));
		HAL_Delay(1);
	}

	/*
	 *
	 * COMMUNICATION WITH SENSORS
	 *
	 *
	 */

	/*
	 * devices using hspi1
	 */
	//BMI088_ReadAccelerometer(&bmi088);

	/*
	 * TODO:
	 * 	Use fucking DMA next time
	 */
	if (HAL_GetTick() >= bmiTick + 5) { //interrupts fucking suck-- this should trigger ~200Hz
		bmiTick = HAL_GetTick();
		newData = newData | 0x1;
		BMI088_ReadGyroscope(&bmi088);
		BMI088_ReadAccelerometer(&bmi088);
	}

	/*
	if (HAL_GetTick() >= bmpTick + 10) { //interrupts fucking suck-- this should trigger ~100Hz
		bmpTick = HAL_GetTick();
		newData = newData | 0x2;
		//BMP388_Read(&bmp388);
	}
	*/

	/*
	 *devices using hspi3
	 */
	if (HAL_GetTick() >= spi3Tick + 5) { //interrupts fucking suck-- this should trigger ~200Hz
		spi3Tick = HAL_GetTick();
		newData = newData | 0x4;
		KX134_Read(&kx134);
		LIS3MDL_Read(&lis3mdl);
	}

	/*
	 *
	 * DATA RECORDING
	 *
	 *
	 */

	if (recordingData == 1) {
		//check if there is new data from the gyroscope
		if ((newData & 0x01) == 0x01) {
			//store data from the gyroscope into the first 32 bytes of the flashWriteBuffer
			uint8_t *timeArray;
			uint16_t timeValue = HAL_GetTick() & 0xFFFF;
			timeArray = (uint8_t*)(&timeValue);

			for (uint8_t i = 0; i < 4; i++) {
				flashWriteBuffer[i] = timeArray[i];
			}
			for (uint8_t i = 0; i < 6; i++) {
				flashWriteBuffer[i+4] = bmi088.gyr_data[i];
			}
			for (uint8_t i = 0; i < 6; i++) {
				flashWriteBuffer[i+10] = bmi088.acc_data[i];
			}
			for (uint8_t i = 0; i < 6; i++) {
				flashWriteBuffer[i+16] = lis3mdl.data[i];
			}
			for (uint8_t i = 0; i < 6; i++) {
				flashWriteBuffer[i+22] = kx134.data[i];
			}
			for (uint8_t i = 0; i < 3; i++) {
				flashWriteBuffer[i+28] = bmp388.data[i];
			}
			flashWriteBuffer[31] = 0xFF;

			if (writeHead % 256 == 0) {
				//write data to the flash chip
				CSP_QSPI_WriteMemory(flashWriteBuffer, writeHead-256, 256);
			}

			if (writeHead > writeHeadEnd) {
				//if we reached the end of the data recording length, turn off the function to record data
				recordingData = 0;
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Done Recording Data\n"));
				HAL_GPIO_WritePin(Continuity_LED_A_GPIO_Port, Continuity_LED_A_Pin, GPIO_PIN_RESET); //reset the gpio pin A to show no longer recording data
			}
			newData = 0;
		}
	} else {
		if (kx134.acc_mps2[0] > 30.f || kx134.acc_mps2[0] < -30.f || kx134.acc_mps2[1] > 30.f || kx134.acc_mps2[1] < -30.f || kx134.acc_mps2[2] > 30.f || kx134.acc_mps2[2] < -30.f) {
			HAL_GPIO_WritePin(Continuity_LED_B_GPIO_Port, Continuity_LED_B_Pin, GPIO_PIN_SET);
			//if we detect movement, check if we haven't already recorded a flight
			//get the current config information
			CSP_QSPI_Read(flashReadBuffer, DATA_CONFIG_LOCATION, 16);
			if (dataWritten == 0 && flashReadBuffer[0] > 31) { //we only want to record up to four times
				//copy the data read in from the flashReadBuffer to the flashWriteBuffer
				for (uint8_t i = 0; i < 16; i++) {
					flashWriteBuffer[i] = flashReadBuffer[i];
				}
				/*
				 * Increment the location of the flash storage-- because of the way NOR memory works,
				 * once a byte has been written 0, it cannot be changed without erasing the sector--
				 * this means we obtain a counter from 1 to 8 by bit shifting down, setting each consecutive
				 * MSB to 0 to indicate another recording
				 */
				flashWriteBuffer[0] = flashWriteBuffer[0] >> 1;

				//need to erase flash memory before stuff can be re-written
				CSP_QSPI_EraseSector(0, MEMORY_SECTOR_SIZE);
				CSP_QSPI_WriteMemory(flashWriteBuffer, DATA_CONFIG_LOCATION, 16);

				CSP_QSPI_Read(flashReadBuffer, DATA_CONFIG_LOCATION, 16);
				//transmit to the host computer
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", flashReadBuffer[0], flashReadBuffer[1],
						flashReadBuffer[2], flashReadBuffer[3], flashReadBuffer[4], flashReadBuffer[5], flashReadBuffer[6], flashReadBuffer[7], flashReadBuffer[8],
						flashReadBuffer[9], flashReadBuffer[10], flashReadBuffer[11], flashReadBuffer[12], flashReadBuffer[13], flashReadBuffer[14], flashReadBuffer[15]));
				HAL_Delay(1);


				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Recording Data...%i\n", flashWriteBuffer[0]));
				HAL_Delay(1);

				//if recording starts, LED A goes high, if movement is detected but no recording is started, LED B goes high
				HAL_GPIO_WritePin(Continuity_LED_B_GPIO_Port, Continuity_LED_B_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Continuity_LED_A_GPIO_Port, Continuity_LED_A_Pin, GPIO_PIN_SET);

				//set recording flags
				recordingData = 1;
				dataWritten = 1;
			}
		}
	}

	/*
	 *
	 * LED AND BUZZER FEEDBACK
	 *
	 *
	 */
	if (HAL_GetTick() > ledTick + ledTimeout) {
		ledTick = HAL_GetTick();
		HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
		if (recordingData == 1) {
			HAL_GPIO_TogglePin(Continuity_LED_C_GPIO_Port, Continuity_LED_C_Pin);
		}
		//ternary expression to toggle buzzerEnable on and off
		buzzerEnable = (buzzerEnable == 0) ? 1 : 0;
	}
	//TODO: use a timer and actual PWM b.c. tones other than 1kHz are nice and more efficient computationally
	if (HAL_GetTick() > buzzerTick && buzzerEnable && buzzerTick < 0xFFFFF) {
		HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
		buzzerTick = HAL_GetTick();
	} else if (buzzerTick > 0xFFFF) {
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(Pyro_A_Trigger_GPIO_Port, Pyro_A_Trigger_Pin, GPIO_PIN_SET);
  while (1)
  {
	  HAL_GPIO_TogglePin(Pyro_F_Trigger_GPIO_Port, Pyro_F_Trigger_Pin);
	  HAL_Delay(250);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
