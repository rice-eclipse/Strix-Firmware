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
#define DATA_RECORDING_LENGTH  0x400000 // 4MByte = 16 Bytes per data write * 400 Hz * ~10 minutes
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
uint8_t flashWriteBuffer[65]; //because sprintf sucks
uint8_t flashReadBuffer[64];
uint8_t config[16]; //this is to store the configuration right at the beginning of the program
uint32_t writeHead; //to store how many recordings we've made
uint8_t recordingData;

//status LED and Buzzer code
uint32_t ledTick;
uint16_t ledTimeout;

//devices using hspi1
BMI088 bmi088;
//BMP388 bmp388;

//devices using hspi3
//LIS3MDL lis3mdl;
//KX134 kx134;


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

  HAL_Delay(3000);

  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\n# ------------------------------ \n#\n#\n#\n#\n"));
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
  CDC_Transmit_FS(buffer, sprintf((char *)txBuffer, "%i\n", bmp_status));
  */

  /*
   * devices that use hspi3
   */
  /*
  uint8_t kx_status = KX134_Init(&kx134, &hspi3, KX134_NCS_GPIO_Port, KX134_NCS_Pin);
  CDC_Transmit_FS(buffer, sprintf((char *)txBuffer, "Status of KX134: %i\n", kx_status));
  */

  /* Initialize LIS3MDL */
  /*
  uint8_t lis3mdl_status = LIS3MDL_Init(&lis3mdl, &hspi3, LIS3MDL_NCS_GPIO_Port, LIS3MDL_NCS_Pin);
  if(lis3mdl_status != 4) {
  	  HAL_GPIO_WritePin(Pyro_C_Trigger_GPIO_Port, Pyro_C_Trigger_Pin, GPIO_PIN_SET);
  }
  */

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

  //get the first 64 bytes, which hold configuration information
  CSP_QSPI_Read(flashReadBuffer, 0, 16);

  //copy the configuration into the configuration array
  memcpy(&config, &flashReadBuffer, 16);

  //read the number of recordings currently on the chip, and write to the USB port
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "# Number of Recordings: %i out of 4 possible\n", config[0]));
  HAL_Delay(1);

  /*
   * TELL THE USER HOW TO COMMUNICATE
   */
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\n#\n#\t1 to test USB functionality\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t2 to access a recording\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t3 to delete data (confirmation required)\n"));
  HAL_Delay(1);
  CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "#\t4 to toggle live sensor data reporting\n"));
  HAL_Delay(1);

  //set the usbStatus to 0 to confirm nothing has been requested yet
  usbStatus = 0;
  //set the ledTimeout to 500ms for nominal state
  ledTimeout = 500;
  //set the writeHead to 256 bc there hasn't been any data recorded, but the first 256 bytes are reserved for config
  writeHead = 256;
  //set recordingData to 0 bc we haven't detected a launch yet
  recordingData = 0;



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

	//check if there are any commands from the user
	if (rxBuffer[0] != 0) {
		if (usbStatus == 0 || usbStatus == 3) {
			//check what was sent
			if (rxBuffer[0] == '1') {
				//usb transmission test
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "USB transmission confirmed functional"));
			}
			if (rxBuffer[0] == '2') {
				//check status of the flash memory chip
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Getting Flash Memory Data"));
				if (config[0] == 1) {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "There are no recordings to access"));
				} else {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Choose one of the %i recordings", config[0]));
				}
			}
			if (rxBuffer[0] == '3') {
				//send the confirmation message and change current state
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "You have requested to delete all flight recordings"));
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "\t\t\tEnter 'yes' to proceed"));
				ledTimeout = 250;
				usbStatus = 2;
			}
			if (rxBuffer[0] == '4') {
				if (usbStatus != 4) {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Starting Data Transmission"));
					usbStatus = 3;
				} else {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Stopping Data Transmission"));
					usbStatus = 0;
				}
			}
		} else if (usbStatus == 1) {
			//the number that was transmitted must be the desired recording
			if (rxBuffer[0] > config[0]) {
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Invalid Number Entered"));
			}
			//report the desired results...
			//calculate starting address
			uint32_t start_address = DATA_RECORDING_LENGTH * rxBuffer[0] + 256; //add 256 b.c. the first page is reserved for config
			uint32_t address = start_address;
			//read data page by page and transmit through serial port
			do {

				address+=256;
			} while (address < start_address+DATA_RECORDING_LENGTH);

			usbStatus = 0;
			ledTimeout = 500;
		} else if (usbStatus == 2) {
			//check if we have confirmation to delete the entire chip
			if (rxBuffer[0] == 'y' && rxBuffer[1] == 'e' && rxBuffer[2] == 's') {
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Deleting entire flash chip and rewriting config"));
				//now we actually have to do it
				if (CSP_QSPI_Erase_Chip() != HAL_OK) {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Error while deleting data"));
				}
				//copy over the config array (64 bytes)
				memcpy (flashWriteBuffer, config, 64);
				if (CSP_QSPI_WriteMemory(flashWriteBuffer, 0, 64) != HAL_OK) {
					CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Error while copying config data"));
				}
				//reset the counter of how many flights have been recorded
				CSP_QSPI_WriteMemory((uint8_t)0, 0, 1);
			} else {
				CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "Exit Confirmed"));
			}
			usbStatus = 0;
			ledTimeout = 500;
		}
		//reset the rxBuffer so we don't accidentally do things multiple times
		memset (rxBuffer, '\0', 64);
	}

	//check if it's time to transmit the next data tick yet
	if (HAL_GetTick() > txDataTransmitTick + 5000 && usbStatus == 3) {
		//update the last time data was transmitted
		txDataTransmitTick = HAL_GetTick();

		//transmit the current data from the sensor structures
		CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "deg/sec: x: %.3f   y: %.3f   z: %.3f\n",
					bmi088.gyr_rps[0],bmi088.gyr_rps[1],bmi088.gyr_rps[2]));
		//CDC_Transmit_FS(buffer, sprintf((char *)txBuffer, "BMI m/s^2: x: %.3f   y: %.3f   z: %.3f\n", bmi088.acc_mps2[0],bmi088.acc_mps2[1],bmi088.acc_mps2[2]));
		//CDC_Transmit_FS(buffer, sprintf((char *)txBuffer, "Pressure: %.5f; Temperature: %.5f\n", bmp388.pressure,bmp388.temperature));
		//CDC_Transmit_FS(buffer, sprintf((char *)txBuffer, "KX134 m/s^2: x: %.3f   y: %.3f   z: %.3f\n", kx134.acc_mps2[0],kx134.acc_mps2[1],kx134.acc_mps2[2]));
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
	BMI088_ReadGyroscope(&bmi088);
	//BMP388_Read(&bmp388);

	/*
	 *devices using hspi3
	 */
	//KX134_Read(&kx134);
	//LIS3MDL_Read(&lis3mdl);

	/*
	 *
	 * DATA RECORDING
	 *
	 *
	 */

	if (recordingData == 1) {
		//store data from the gyroscope into the first 64 bytes of the flashWriteBuffer (bc I'm lazy and sprintf sucks so its actually 65 bytes)
		uint8_t *timeArray;
		uint32_t time = HAL_GetTick();
		timeArray = (uint8_t*)(&time);
		uint8_t *xarray;
		xarray = (uint8_t*)(&bmi088.gyr_rps[0]);
		uint8_t *yarray;
		yarray = (uint8_t*)(&bmi088.gyr_rps[1]);
		uint8_t *zarray;
		zarray = (uint8_t*)(&bmi088.gyr_rps[2]);

		for (uint8_t i = 0; i < 4; i++) {
			flashWriteBuffer[writeHead % 64] = timeArray[i];
			writeHead++;
		}
		for (uint8_t i = 0; i < 4; i++) {
			flashWriteBuffer[writeHead % 64] = xarray[i];
			writeHead++;
		}
		for (uint8_t i = 0; i < 4; i++) {
			flashWriteBuffer[writeHead % 64] = yarray[i];
			writeHead++;
		}
		for (uint8_t i = 0; i < 4; i++) {
			flashWriteBuffer[writeHead % 64] = zarray[i];
			writeHead++;
		}

		//check if we should write data to the flash chip
		if (writeHead % 64 == 0) {
			//write the data from flashWriteBuffer
			CSP_QSPI_WriteMemory(flashWriteBuffer, writeHead, 64);
		}
		if ((writeHead - 256) % DATA_RECORDING_LENGTH == 0 || writeHead > 0x1000000) {
			//if we reached the end of the data recording length or the chip, turn off the function to record data
			recordingData = 0;
		}
	} else {
		if (bmi088.gyr_rps[0] > 1.f) {
			//if we detect movement, check if we haven't already recorded a flight
			//get the current config information
			CSP_QSPI_Read(flashReadBuffer, 0, 16);
			if (flashReadBuffer[0] == config[0]) {
				recordingData = 1;
				flashWriteBuffer[0] = config[0]+1;
				//update the status inside the config bytes in the flash chip
				CSP_QSPI_WriteMemory(flashWriteBuffer, 0, 1);
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
			HAL_GPIO_TogglePin(Continuity_LED_A_GPIO_Port, Continuity_LED_A_Pin);
		}
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
