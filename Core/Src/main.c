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
#include "usb_device.h"

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */

//USB transmission code
uint8_t buffer[64];

//devices using hspi1
BMI088 bmi088;
//BMP388 bmp388;

//devices using hspi3
//LIS3MDL lis3mdl;
KX134 kx134;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
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
  /* USER CODE BEGIN 2 */


  	/* devices that use hspi1 */

  	/* Initialize BMI088 */
  	uint8_t bmi_status = BMI088_Init(&bmi088, &hspi1, BMI088_Accel_NCS_GPIO_Port, BMI088_Accel_NCS_Pin, BMI088_Gyro_NCS_GPIO_Port, BMI088_Gyro_NCS_Pin);
  	CDC_Transmit_FS(buffer, sprintf((char *)buffer, "Status of BMI088: %i\n", bmi_status));

  	HAL_Delay(500);


  	/* Initialize BMP388 */
  	/*
  	uint8_t bmp_status = BMP388_Init(&bmp388, &hspi1, BMP388_NCS_GPIO_Port, BMP388_NCS_Pin);
  	CDC_Transmit_FS(buffer, sprintf((char *)buffer, "%i\n", bmp_status));
  	*/


  	/* devices that use hspi3 */
    uint8_t kx_status = KX134_Init(&kx134, &hspi3, KX134_NCS_GPIO_Port, KX134_NCS_Pin);
  	CDC_Transmit_FS(buffer, sprintf((char *)buffer, "Status of KX134: %i\n", kx_status));

  	/* Initialize LIS3MDL */
  	/*
	uint8_t lis3mdl_status = LIS3MDL_Init(&lis3mdl, &hspi3, LIS3MDL_NCS_GPIO_Port, LIS3MDL_NCS_Pin);
	if(lis3mdl_status != 4) {
		HAL_GPIO_WritePin(Pyro_C_Trigger_GPIO_Port, Pyro_C_Trigger_Pin, GPIO_PIN_SET);
	}
	*/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	uint8_t status;

	//devices using hspi1

	/*
	status = BMI088_ReadAccelerometer(&bmi088);

	CDC_Transmit_FS(buffer, sprintf((char *)buffer, "m/s^2: x: %.3f   y: %.3f   z: %.3f\n",
			bmi088.acc_mps2[0],bmi088.acc_mps2[1],bmi088.acc_mps2[2]));
	*/

	status = BMI088_ReadGyroscope(&bmi088);

	CDC_Transmit_FS(buffer, sprintf((char *)buffer, "rad/sec: x: %.3f   y: %.3f   z: %.3f\n",
			bmi088.gyr_rps[0],bmi088.gyr_rps[1],bmi088.gyr_rps[2]));


	/*
	status = BMP388_Read(&bmp388);

	CDC_Transmit_FS(buffer, sprintf((char *)buffer, "pressure: %.5f; temperature: %.5f\n",
					bmp388.pressure,bmp388.temperature));
	*/


	//devices using hspi3

	status = KX134_Read(&kx134);

	CDC_Transmit_FS(buffer, sprintf((char *)buffer, "m/s^2: x: %.3f   y: %.3f   z: %.3f\n",
			kx134.acc_mps2[0],kx134.acc_mps2[1],kx134.acc_mps2[2]));

	/*
	status = LIS3MDL_Read(&lis3mdl);
	*/


	CDC_Transmit_FS(buffer, sprintf((char *)buffer, "\n\n\n"));
	HAL_Delay(1000);

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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Pyro_A_Trigger_Pin|Continuity_LED_D_Pin|Continuity_LED_C_Pin|BMI088_Gyro_Int_Pin
                          |BMI088_Gyro_NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Pyro_B_Trigger_Pin|Pyro_C_Trigger_Pin|Pyro_D_Trigger_Pin|Pyro_E_Trigger_Pin
                          |Pyro_F_Trigger_Pin|BMP388_NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Continuity_LED_B_Pin|Continuity_LED_A_Pin|Continuity_LED_E_Pin|Continuity_LED_F_Pin
                          |Status_LED_Pin|KX134_NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIS3MDL_NCS_GPIO_Port, LIS3MDL_NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Pyro_A_Trigger_Pin Continuity_LED_D_Pin Continuity_LED_C_Pin BMI088_Gyro_Int_Pin
                           BMI088_Gyro_NCS_Pin */
  GPIO_InitStruct.Pin = Pyro_A_Trigger_Pin|Continuity_LED_D_Pin|Continuity_LED_C_Pin|BMI088_Gyro_Int_Pin
                          |BMI088_Gyro_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Pyro_B_Trigger_Pin Pyro_C_Trigger_Pin Pyro_D_Trigger_Pin Pyro_E_Trigger_Pin
                           Pyro_F_Trigger_Pin BMP388_NCS_Pin */
  GPIO_InitStruct.Pin = Pyro_B_Trigger_Pin|Pyro_C_Trigger_Pin|Pyro_D_Trigger_Pin|Pyro_E_Trigger_Pin
                          |Pyro_F_Trigger_Pin|BMP388_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Continuity_LED_B_Pin Continuity_LED_A_Pin Continuity_LED_E_Pin Continuity_LED_F_Pin
                           Status_LED_Pin KX134_NCS_Pin */
  GPIO_InitStruct.Pin = Continuity_LED_B_Pin|Continuity_LED_A_Pin|Continuity_LED_E_Pin|Continuity_LED_F_Pin
                          |Status_LED_Pin|KX134_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LIS3MDL_NCS_Pin */
  GPIO_InitStruct.Pin = LIS3MDL_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIS3MDL_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIS3MDL_Int_Pin KX134_Int_Pin */
  GPIO_InitStruct.Pin = LIS3MDL_Int_Pin|KX134_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI088_Accel_NCS_Pin BMI088_Accel_Int_Pin */
  GPIO_InitStruct.Pin = BMI088_Accel_NCS_Pin|BMI088_Accel_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP388_Int_Pin */
  GPIO_InitStruct.Pin = BMP388_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMP388_Int_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
