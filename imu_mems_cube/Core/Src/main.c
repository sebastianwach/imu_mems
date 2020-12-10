/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iks01a2_motion_sensors.h"
#include "motion_ac.h"
#include <stdio.h>
#include "motion_mc_cm0p.h"
#include "motion_mc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VERSION_STR_LENG 35
#define REPORT_INTERVAL 20
#define SAMPLE_TIME 100
#define PI 3.14159265
#define MAX_BUF_SIZE 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern uint8_t overflow_flag_tim7;
volatile uint32_t timestamp =0;
char dataOutUART[MAX_BUF_SIZE];

char lib_version[VERSION_STR_LENG];
MAC_knobs_t Knobs;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void Init_Motion_Sensors();
void Init_MotionAC_Calibration();
void Init_MotionMC_Calibration();
void Read_Accelero_Sensor(uint32_t Instance);
void Read_Gyro_Sensor(uint32_t Instance);
void Read_Magneto_Sensor(uint32_t Instance);

void PrintMEMSValues ( char message[20], float x, float y, float z);
void PrintMEMSError ( char message[50], int32_t errorNumber);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if((htim->Instance==TIM7)&&(overflow_flag_tim7)){

//		Read_Accelero_Sensor(IKS01A2_LSM6DSL_0);
//		Read_Gyro_Sensor(IKS01A2_LSM6DSL_0);
		Read_Magneto_Sensor(IKS01A2_LSM303AGR_MAG_0);

	}
	return;
}


void Init_Motion_Sensors()
{
	  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
	  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
	  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);

}

void Init_MotionAC_Calibration()
{
	MotionAC_Initialize(1);
	MotionAC_GetKnobs(&Knobs);
	Knobs.Sample_ms = REPORT_INTERVAL;
	(void)MotionAC_SetKnobs(&Knobs);

}

void Init_MotionMC_Calibration()
{
	MotionMC_Initialize(SAMPLE_TIME, 1);
}



void Read_Accelero_Sensor(uint32_t Instance)
{

	/*ODR = 104 Hz*/
	IKS01A2_MOTION_SENSOR_Axes_t acceleration;

	int scale = 1000;
	float acc[3];



	if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration))
	{

		int32_t errorNumber = IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration);
		PrintMEMSError("Acc", errorNumber);
	}
	else
	{
		acc[0] = acceleration.x;
		acc[1] = acceleration.y;
		acc[2] = acceleration.z;

		for (int i = 0; i <3; i++)
		{
			acc[i]/=scale;
		}

		PrintMEMSValues("Accelerometer", acc[0], acc[1], acc[2]);



		uint8_t isCalibrated;

		MAC_input_t input_cal_acc;
		memcpy(input_cal_acc.Acc, acc,sizeof(acc));
		memcpy(input_cal_acc.TimeStamp, 1000, sizeof(int));


		/*START CALIBRATION*/

		MAC_input_t data_in;
		MAC_output_t data_out;;
		float acc_cal_x, acc_cal_y, acc_cal_z;
		uint8_t is_calibrated;

		data_in.Acc[0]=acc[0];
		data_in.Acc[1]=acc[1];
		data_in.Acc[2]=acc[2];

		MotionAC_Update(&data_in, &is_calibrated);

		/* Get Calibration coeficients */
		MotionAC_GetCalParams(&data_out);

		/* Apply correction */
//		acc_cal_x = (data_in.Acc[0] - data_out.AccBias[0])* data_out.SF_Matrix[0][0];
//		acc_cal_y = (data_in.Acc[1] - data_out.AccBias[1])* data_out.SF_Matrix[1][1];
//		acc_cal_z = (data_in.Acc[2] - data_out.AccBias[2])* data_out.SF_Matrix[2][2];

		acc_cal_x = acc[0];
		acc_cal_y = acc[1];
		acc_cal_z = acc[2];


		PrintMEMSValues("Calibrated accelerometer", acc_cal_x, acc_cal_y, acc_cal_z);

		PrintMEMSValues("Acc bias", data_out.AccBias[0], data_out.AccBias[1], data_out.AccBias[2]);

		/*Finish calibration*/

		/*Yaw, roll, pitch*/

		double pitch = 180 * atan (acc_cal_x/sqrt(acc_cal_y*acc_cal_y + acc_cal_z*acc_cal_z))/PI;
		double roll = 180 * atan (acc_cal_y/sqrt(acc_cal_x*acc_cal_x + acc_cal_z*acc_cal_z))/PI;
		double yaw = 180 * atan (acc_cal_z/sqrt(acc_cal_x*acc_cal_x + acc_cal_z*acc_cal_z))/PI;

		snprintf(dataOutUART, MAX_BUF_SIZE, "Pitch: %.3f\t Roll: %.3f\t Yaw: %.3f\r\n",
				pitch, roll, yaw);

		HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 10);


	}



}

void Read_Gyro_Sensor(uint32_t Instance)
{

	/*ODR = 104 Hz, DPS= 2000  address = 0x6a*/

	IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;

	if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity))
	{
		int32_t errorNumber = IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity);
		PrintMEMSError("Gyroscope", errorNumber);
	}
	else
	{
		PrintMEMSValues("Gyroscope", angular_velocity.x, angular_velocity.y, angular_velocity.z);
	}





}

void Read_Magneto_Sensor(uint32_t Instance)
{

	IKS01A2_MOTION_SENSOR_Axes_t magnetic_field;

	if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field))
	{
		int32_t errorNumber = IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field);
		PrintMEMSError("Magnetometer", errorNumber);
	}
	else
	{
		PrintMEMSValues("Magnetometer", magnetic_field.x, magnetic_field.y, magnetic_field.z);
	//Data to txt
	//    snprintf(dataOutUART, MAX_BUF_SIZE, "%d\t%d\t%d\r\n",
	//             (int)magnetic_field.x, (int)magnetic_field.y, (int)magnetic_field.z);



		/*Start Calibration*/
		float mag[3];
		float mag_cal_x, mag_cal_y, mag_cal_z;
		MMC_Input_t data_in;
		MMC_Output_t data_out;

		mag[0] = (float) magnetic_field.x;
		mag[1] = (float) magnetic_field.y;
		mag[2] = (float) magnetic_field.z;


		memcpy(data_in.Mag, mag,sizeof(mag));
		data_in.TimeStamp = timestamp*SAMPLE_TIME;
		MotionMC_Update(&data_in);
		MotionMC_GetCalParams(&data_out);

		mag_cal_x = (int) (( data_in.Mag[0] - data_out.HI_Bias[0]) * data_out.SF_Matrix[0][0]
						+ ( data_in.Mag[1] - data_out.HI_Bias[1]) * data_out.SF_Matrix[0][1]
						+ ( data_in.Mag[2] - data_out.HI_Bias[2]) * data_out.SF_Matrix[0][2] );

		mag_cal_y = (int) (( data_in.Mag[0] - data_out.HI_Bias[0]) * data_out.SF_Matrix[1][0]
						+ ( data_in.Mag[1] - data_out.HI_Bias[1]) * data_out.SF_Matrix[1][1]
						+ ( data_in.Mag[2] - data_out.HI_Bias[2]) * data_out.SF_Matrix[1][2] );

		mag_cal_z = (int) (( data_in.Mag[0] - data_out.HI_Bias[0]) * data_out.SF_Matrix[2][0]
						+ ( data_in.Mag[1] - data_out.HI_Bias[1]) * data_out.SF_Matrix[2][1]
						+ ( data_in.Mag[2] - data_out.HI_Bias[2]) * data_out.SF_Matrix[2][2] );


		PrintMEMSValues("CMagnetometer", mag_cal_x, mag_cal_y, mag_cal_z);




//		double pitch = 180 * atan (acc_cal_x/sqrt(acc_cal_y*acc_cal_y + acc_cal_z*acc_cal_z))/PI;
//		double roll = 180 * atan (acc_cal_y/sqrt(acc_cal_x*acc_cal_x + acc_cal_z*acc_cal_z))/PI;
//		double yaw = 180 * atan (acc_cal_z/sqrt(acc_cal_x*acc_cal_x + acc_cal_z*acc_cal_z))/PI;


//		float yaw =  atan2(-mag[1],mag[0])*180/PI;
//
//		snprintf(dataOutUART, MAX_BUF_SIZE, "Yaw: %.3f\r\n", yaw);
//
//		HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 10);



	}





}

void PrintMEMSValues ( char message[50], float x, float y, float z)
{
	char dest[MAX_BUF_SIZE];

	strcpy( dest, message );
	strcat( dest, ": X: %.3f\t Y: %.3f\t Z: %.3f\r\n" );
    snprintf(dataOutUART, MAX_BUF_SIZE, dest, x, y, z);

    HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 10);

}

void PrintMEMSError ( char message[50], int32_t errorNumber)
{
	char dest[MAX_BUF_SIZE];

	strcpy( dest, message );
	strcat( dest, ": Error number: %d\r\n");
    snprintf(dataOutUART, MAX_BUF_SIZE, dest, errorNumber);

	HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 10);

}


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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

	Init_Motion_Sensors();
//	MotionMC_Initialize(SAMPLE_TIME, 1);
	Init_MotionAC_Calibration();


	HAL_TIM_Base_Init(&htim7);
	HAL_TIM_Base_Start_IT(&htim7);
	htim7.Instance->CNT=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(500);


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 319;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
