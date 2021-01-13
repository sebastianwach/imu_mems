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
#include <stdbool.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VERSION_STR_LENG 35
#define REPORT_INTERVAL 20
#define SAMPLE_TIME 10
#define PI 3.14159265
#define MAX_BUF_SIZE 200


// LOGS DEFINES
//#define PRINT_ACC_LOGS 1
//#define PRINT_GYRO_RAW_LOGS 1
//#define PRINT_GYRO_LOGS 1
//#define PRINT_MAG_mG_LOGS 1
//#define PRINT_MAG_uT_LOGS


//Madgwick
#define sampleFreq	100.0f		// sample frequency in Hz
#define betaDef		1.0f		// 2 * proportional gain    The best for now is 1.0

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
volatile uint32_t timestamp = 0;
char dataOutUART[MAX_BUF_SIZE];

MAC_knobs_t Knobs;

float mag_max[3];
float mag_min[3];
float mag_bias[3];

float gyr_bias[3];
bool areAssignedFirstExtremeValues = false;
bool isAssignedGyroscopeBias = false;
uint16_t gyroscopeCounter = 0;
uint16_t logCounter = 0;
const uint16_t dividerLogs = 5;	//20 Hz printing

float acc[3];
float gyr[3];
float mag[3];

//Madgwick
float beta = betaDef;								// 2 * proportional gain (Kp)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
float roll, pitch, yaw;


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
void CheckMagExtremeValues (float x, float y, float z);
void CalculateMagBias();

void PrintMEMSValues ( char message[20], float x, float y, float z);
void PrintMEMSError ( char message[50], int32_t errorNumber);
void PrintEulerAngles(float roll, float pitch, float yaw);
void PrintQuaternions();
void PrintToPyTeapot(float qw, float qa, float qb, float qc, float yaw, float pitch, float roll);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//Madgwick
float invSqrt(float x);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void ToEulerAngles( float q0, float q1, float q2, float q3);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if((htim->Instance==TIM7)&&(overflow_flag_tim7)){

		logCounter ++;

		Read_Accelero_Sensor(IKS01A2_LSM6DSL_0);
		Read_Gyro_Sensor(IKS01A2_LSM6DSL_0);
//		Read_Magneto_Sensor(IKS01A2_LSM303AGR_MAG_0);

		MadgwickAHRSupdateIMU(gyr[0], gyr[1], gyr[2], acc[0],acc[1], acc[2]);
//		MadgwickAHRSupdate(gyr[0], gyr[1], gyr[2], acc[0],acc[1], acc[2], mag[0], mag[1], mag[2]);

		ToEulerAngles(q0, q1, q2, q3);

		if(logCounter%dividerLogs==0)
		{
			PrintToPyTeapot(q0, q1,q2,q3, yaw, pitch, roll);
//			PrintEulerAngles(roll*180/PI, pitch*180/PI, yaw*360/PI);
//			PrintQuaternions();
		}


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

	IKS01A2_MOTION_SENSOR_Axes_t acceleration;

	int scale = 1000;
	float acc_raw[3];



	if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration))
	{

		int32_t errorNumber = IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration);
		PrintMEMSError("Acc", errorNumber);
	}
	else
	{
		acc_raw[0] = acceleration.x;
		acc_raw[1] = acceleration.y;
		acc_raw[2] = acceleration.z;

		for (int i = 0; i <3; i++)
		{
			acc_raw[i]/=scale;
		}

		/*Calibration isnt necessary, just converted from mg to g*/
		memcpy(acc, acc_raw ,sizeof(acc_raw));

		#ifdef PRINT_ACC_LOGS
		if(logCounter%dividerLogs==0)
		{
			PrintMEMSValues("Accelerometer[g]", acc[0], acc[1], acc[2]);
		}
		#endif

	}
}

void Read_Gyro_Sensor(uint32_t Instance)
{

	IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;
	float gyr_raw[3];

	if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity))
	{
		int32_t errorNumber = IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity);
		PrintMEMSError("Gyroscope", errorNumber);
	}
	else
	{

		gyr_raw[0] = angular_velocity.x;
		gyr_raw[1] = angular_velocity.y;
		gyr_raw[2] = angular_velocity.z;

		#ifdef PRINT_GYRO_RAW_LOGS
		if(logCounter%dividerLogs==0)
		{
			PrintMEMSValues("Gyrosc_raw[mdps]", gyr_raw[0], gyr_raw[1], gyr_raw[2] );
		}
		#endif

		/*Change mdps to dps*/
		/*There was problem with units!!! I had to divide by 100k, not 1k*/
		for ( int i = 0 ; i < 3; i++ )
		{
			gyr_raw[i] /= 100000;
		}

		/*Assign gyroscope bias as a first proper readed values;*/
		if(!isAssignedGyroscopeBias)
		{
			gyroscopeCounter++;
			memcpy(gyr_bias, gyr_raw, sizeof(gyr_raw));

			/*There was problem with first readed values, because they were equal 0*/
			if(gyroscopeCounter >10 )
			{
				isAssignedGyroscopeBias = true;
			}
		}

		/*Substract bias from raw measure gyroscope value*/
		for( int i = 0; i < 3; i++ )
		{
			gyr_raw[i]-=gyr_bias[i];
		}

		memcpy(gyr, gyr_raw, sizeof(gyr_raw));

		/*Checking if values are near 0 and assign 0*/
		for ( int i = 0; i < 3; i++ )
		{
			if( gyr[i] < 0.1 && gyr[i] > -0.1)
			{
				gyr[i] = 0;
			}
		}

		#ifdef PRINT_GYRO_LOGS
		if(logCounter%dividerLogs==0)
		{
			PrintMEMSValues("Gyroscope[dps]", gyr[0], gyr[1], gyr[2] );
		}
		#endif
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


		float mag_raw[3];
		mag_raw[0] = (float) magnetic_field.x;
		mag_raw[1] = (float) magnetic_field.y;
		mag_raw[2] = (float) magnetic_field.z;
		PrintMEMSValues("Magnetometer_raw[mG]", mag_raw[0], mag_raw[1], mag_raw[2]);


		/*Custom Calibration Start HERE*/
		CheckMagExtremeValues(mag_raw[0], mag_raw[1], mag_raw[2]);
		CalculateMagBias();
		PrintMEMSValues("Bias val Mag[mG]", mag_bias[0], mag_bias[1], mag_bias[2]);

		/*Substract bias from value*/
		for ( int i = 0; i < 3; i++)
		{
			mag_raw[i] = mag_raw[i] - mag_bias[i];
		}

		#ifdef PRINT_MAG_mG_LOGS
		if(logCounter%dividerLogs==0)
		{
			PrintMEMSValues("Magnetometer[mG]", mag_raw[0], mag_raw[1], mag_raw[2]);
		}
		#endif

		/*Convert from mG to uT*/
		for ( int i = 0; i < 3; i++)
		{
			mag_raw[i] /= 10;
		}

		memcpy(mag, mag_raw, sizeof(mag_raw));

		#ifdef PRINT_MAG_uT_LOGS
		if(logCounter%dividerLogs==0)
		{
		PrintMEMSValues("Magnetometer[uT]", mag[0], mag[1], mag[2]);
		}
		#endif

		/*Custom Calibration End HERE*/


	}
}

/*It should be changed to float array*/
void CheckMagExtremeValues (float x, float y, float z)
{

	if (!areAssignedFirstExtremeValues)
	{

		mag_min[0] = x;
		mag_min[1] = y;
		mag_min[2] = z;

		mag_max[0] = x;
		mag_max[1] = y;
		mag_max[2] = z;

		areAssignedFirstExtremeValues = true;
	}

	if( x < mag_min[0])
	{
		mag_min[0] = x;
	}
	if( y < mag_min[1])
	{
		mag_min[1] = y;
	}
	if( z < mag_min[2])
	{
		mag_min[2] = z;
	}

	if( x > mag_max[0])
	{
		mag_max[0] = x;
	}
	if( y > mag_max[1])
	{
		mag_max[1] = y;
	}
	if( z > mag_max[2])
	{
		mag_max[2] = z;
	}

}

void CalculateMagBias()
{
	mag_bias[0] = (mag_max[0]+mag_min[0])/2;
	mag_bias[1] = (mag_max[1]+mag_min[1])/2;
	mag_bias[2] = (mag_max[2]+mag_min[2])/2;
}
void PrintMEMSValues ( char message[50], float x, float y, float z)
{
	char dest[MAX_BUF_SIZE];

	strcpy( dest, message );
	strcat( dest, ": X: %.3f\t Y: %.3f\t Z: %.3f\r\n" );
    snprintf(dataOutUART, MAX_BUF_SIZE, dest, x, y, z);

    HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 1);

}

void PrintMEMSError ( char message[50], int32_t errorNumber)
{
	char dest[MAX_BUF_SIZE];

	strcpy( dest, message );
	strcat( dest, ": Error number: %d\r\n");
    snprintf(dataOutUART, MAX_BUF_SIZE, dest, errorNumber);

	HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 1);

}

void PrintEulerAngles(float roll, float pitch, float yaw)
{

	snprintf(dataOutUART, MAX_BUF_SIZE, "Roll: %.3f\t Pitch: %.3f\t Yaw: %.3f\r\n",roll, pitch, yaw);

	/*Raw format to csv/txt*/
//	snprintf(dataOutUART, MAX_BUF_SIZE, "%.3f\t%.3f\t%.3f\r\n", roll, pitch, yaw);

	HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 1);
}

void PrintQuaternions()
{
		snprintf(dataOutUART, MAX_BUF_SIZE, "q0: %.3f\t q1: %.3f q2: %.3f q3: %.3f\r\n", q0,q1,q2,q3);

		HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 1);
}

void PrintToPyTeapot(float qw, float qa, float qb, float qc, float yaw, float pitch, float roll  )
{

	snprintf(dataOutUART, MAX_BUF_SIZE, "w%.3fwa%.3fab%.3fbc%.3fcy%.3fyp%.3fpr%.3fr\r\n",qw,qa,qb,qc,yaw,pitch,roll);

	HAL_UART_Transmit(&huart2, dataOutUART, strlen(dataOutUART), 1);
}

//Madgwick functions
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}



float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void ToEulerAngles( float q0, float q1, float q2, float q3)
{
	float test = q1*q2 + q3*q0;
	if (test > 0.499 && test < 0.501) { // singularity at north pole

		roll = 0;
		pitch = 2 * atan2(q1,q0);
		yaw = PI/2;
		return;
	}
	if (test < -0.499 && test > -0.501) { // singularity at south pole

		roll = 0;
		pitch = -2 * atan2(q1,q0);
		yaw = - PI/2;
		return;
	}
    float sqx = q1*q1;
    float sqy = q2*q2;
    float sqz = q3*q3;
    roll = atan2(2*q1*q0-2*q2*q3 , 1 - 2*sqx - 2*sqz);
    pitch = atan2(2*q2*q0-2*q1*q3 , 1 - 2*sqy - 2*sqz);
    yaw = asin(2*test);

    // Multiplying to print in degrees

    roll*=180/PI;
    pitch*= 180/PI;
    yaw*= 360/PI;


    //Reverse pitch to visualise better cuboid
    pitch*=-1;




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
//	Init_MotionAC_Calibration();


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
  htim7.Init.Prescaler = 31;
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
