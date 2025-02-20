/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include <math.h>
#include "usbd_cdc_if.h"

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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

// IMMU Addresses & Commands
#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

// Constants
#define RAD_TO_DEG 57.2957795131f
#define DEG_TO_RAD 0.0174532925f
#define G_MPU 9.81000000f
#define COMP_FILTER_ALPHA 0.0500f

// IMU Variables
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t IMUstate = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

float phiHat_accel_rad = 0;
float thetaHat_accel_rad = 0;
float psiHat_accel_rad = 0;

float phiDot_rps = 0;
float thetaDot_rps = 0;
float psiDot_rps = 0;

float phiHat_rad = 0;
float thetaHat_rad = 0;
float psiHat_rad = 0;



uint8_t LEDstate = 0;
uint32_t LEDtimer = 500;


uint8_t roll_plus = 0;
uint8_t roll_minus = 0;

int bootTime;

// Connectivity
#define SAMPLE_TIME_USB_MS 20



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



int16_t MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
	if (check == 0x68){  // 0x68 will be returned by the sensor if everything goes well

		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		Data = 0x00;  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 �?/s
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		Data = 0x00;  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &Data, 1, 1000);

	}

	return check;

}


void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);



	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

//	Ax = (float)Accel_X_RAW/16384.0;
//	Ay = (float)Accel_Y_RAW/16384.0;
//	Az = (float)Accel_Z_RAW/16384.0;

	Ay = -1*(float)Accel_X_RAW/16384.0;
	Ax = -1*(float)Accel_Y_RAW/16384.0;
	Az = -1*(float)Accel_Z_RAW/16384.0;
}

void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);


	/*** convert the RAW values into dps (ｰ/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

//	Gx = DEG_TO_RAD*(float)Gyro_X_RAW/131.0;
//	Gy = DEG_TO_RAD*(float)Gyro_Y_RAW/131.0;
//	Gz = DEG_TO_RAD*(float)Gyro_Z_RAW/131.0;

	Gy = -DEG_TO_RAD*(float)Gyro_X_RAW/131.0;
	Gx = -DEG_TO_RAD*(float)Gyro_Y_RAW/131.0;
	Gz = -DEG_TO_RAD*(float)Gyro_Z_RAW/131.0;
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  char logBuf[256];
  uint32_t USB_Timer = 0;

  // Blue LED
  GPIOC->MODER |= 1U<<26; //PC13 as output
  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW

  // Solenoid MOSFET GPIOs, one of these pins fucks up USB
//  GPIOA->MODER |= 1U<<16; //PA8 as output, ROLL +
//  GPIOA->MODER |= 1U<<18; //PA9 as output, ROLL -
//
//  GPIOA->MODER |= 1U<<20; //PA10 as output
//  GPIOA->MODER |= 1U<<22; //PA11 as output
//  GPIOA->MODER |= 1U<<24; //PA12 as output
//  GPIOA->MODER |= 1U<<30; //PA15 as output

  IMUstate = MPU6050_Init();

  if(IMUstate == 0x68){
	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
	  HAL_Delay(150);
	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
	  HAL_Delay(150);
	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
	  HAL_Delay(150);
	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
	  HAL_Delay(150);
	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
	  HAL_Delay(150);
	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
  } else {
	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
	  HAL_Delay(750);
	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
	  HAL_Delay(750);
	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
	  HAL_Delay(750);
	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Time
	  bootTime = HAL_GetTick();

	  // Blink Blue LED
	  if( (HAL_GetTick() - LEDtimer) >= 500){
		  LEDtimer = HAL_GetTick();
		  if(LEDstate){
			  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
			  LEDstate = 0;
		  } else {
			  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
			  LEDstate = 1;
		  }
	  }

	  // Send Data through USB, according to sample time
	  if( (HAL_GetTick() - USB_Timer) >= SAMPLE_TIME_USB_MS){

		  	  // Read IMU and obtain angular velocity & acceleration
		  MPU6050_Read_Accel();
		  MPU6050_Read_Gyro();

	//	  // Roll Angle
	//	  phiHat_deg = atanf( Ay / Az ) * RAD_TO_DEG;
	//
	//	  // Pitch Angle
	//	  thetaHat_deg = asinf( Ax / G_MPU ) * RAD_TO_DEG;

		  // Roll & Pitch Angle from Accelerometer
		  phiHat_accel_rad = atanf( Ax / Az );
		  thetaHat_accel_rad = asinf( Ay / G_MPU )*(RAD_TO_DEG/4);

		  // Transform body rates to Euler Rates
		  phiDot_rps = Gx + (tanf(thetaHat_rad) * (Gy * sinf(phiHat_rad) + ( Gz * cosf(phiHat_rad))));
		  thetaDot_rps = cosf(phiHat_rad) * Gy - sinf(phiHat_rad) * Gz;


		  // Combine Accel & Gyro data
		  phiHat_rad = (COMP_FILTER_ALPHA*phiHat_accel_rad) + (1.0f - COMP_FILTER_ALPHA)
				  * (phiHat_rad + (SAMPLE_TIME_USB_MS/1000.0f) * phiDot_rps);

		  thetaHat_rad = (COMP_FILTER_ALPHA*thetaHat_accel_rad) + (1.0f - COMP_FILTER_ALPHA)
		  				  * (thetaHat_rad + (SAMPLE_TIME_USB_MS/1000.0f) * thetaDot_rps);


		  //sprintf(logBuf,"%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n",Ax,Ay,Az,Gx* RAD_TO_DEG,Gy* RAD_TO_DEG,Gz* RAD_TO_DEG);
		  sprintf(logBuf,"%0.3f,%0.3f\r\n",phiHat_rad* RAD_TO_DEG,thetaHat_rad* RAD_TO_DEG);
		  CDC_Transmit_FS((uint8_t *) logBuf, strlen(logBuf));
		  USB_Timer = HAL_GetTick();

	  }


	 // Control
//	  if(roll_plus){
//		  // Actuate Roll +, PA8
//		  GPIOA->ODR |= 1U<<8; //Set PA13 as HIGH
//		  roll_plus = 0;
//
//	  } else {
//		  GPIOA->ODR = ~(~(GPIOC->ODR) | 1U<<8); //Set PC13 as LOW
//		  roll_plus = 1;
//	  }
//
//	  if(roll_minus){
//		  // Actuate Roll -, PA9
//		  GPIOA->ODR |= 1U<<9; //Set PA13 as HIGH
//		  roll_minus = 0;
//	  } else {
//		  GPIOA->ODR = ~(~(GPIOC->ODR) | 1U<<9); //Set PC13 as LOW
//		  roll_minus = 1;
//	  }



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
