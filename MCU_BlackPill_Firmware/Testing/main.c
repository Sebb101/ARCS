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
#include "MPU9250.h"

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

// Solenoid Registars

//MPU9250
MPU9255_t MPU9255;


// MPU6050 Addresses & Commands
#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75



// IMU Variables
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t IMUstate = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

// Constants
#define RAD_TO_DEG 57.2957795131f
#define DEG_TO_RAD 0.0174532925f
#define G_MPU 9.81000000f
#define COMP_FILTER_ALPHA 0.0500f

// Filter Variables
float phiHat_accel_rad = 0;
float thetaHat_accel_rad = 0;
float psiHat_accel_rad = 0;
float phiDot_rps = 0;
float thetaDot_rps = 0;
float psiDot_rps = 0;
float phiHat_rad = 0;
float thetaHat_rad = 0;
float psiHat_rad = 0;

float Gx_bias, Gy_bias, Gz_bias = 0;
float Gx_sum, Gy_sum, Gz_sum = 0;


// Control/Indication Variables
uint8_t LEDstate = 0;
uint32_t LEDtimer = 500;

uint8_t roll_plus = 0;
uint8_t roll_minus = 0;

// test Sequence variables
#define Roll_plus_test 1
#define Roll_minus_test 2
#define Pitch_plus_test 3
#define Pitch_minus_test 4
#define Yaw_plus_test 5
#define Yaw_minus_test 6
#define SpeedTest 7
#define stop_roll 8

#define test_interval_ms 1000.0f
int hertz = 1;

int TestState = 0;
int Solenoid_timer = 0;
int Speedtest_timer = 0;
uint8_t Solenoid_toggleState = 0;


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

//void solenoidIndFire(GPIO_TypeDef port,uint8_t pin,int timeFire,int boot){
//
//	// ROLL +
//	if(boot<timeFire){
//		port->ODR |= 1U<<pin; //Set PB9 as HIGH
//	} else{
//		port->ODR = ~(~(port->ODR) | 1U<<pin); //Set PB9 as LOW
//	}
//
//}

void SolenoidChecker(void){
	// ROLL +
	GPIOB->ODR |= 1U<<9; //Set PB9 as HIGH
	HAL_Delay(150);
	GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<9); //Set PB9 as LOW

	// ROLL -
	HAL_Delay(150);
	GPIOB->ODR |= 1U<<8; //Set PB8 as HIGH
	HAL_Delay(150);
	GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<8); //Set PB8 as LOW

	// Pitch +
	HAL_Delay(150);
	GPIOB->ODR |= 1U<<5; //Set PB5 as HIGH
	HAL_Delay(150);
	GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<5); //Set PB5 as LOW

	// Pitch -
	GPIOB->ODR |= 1U<<4; //Set PB4 as HIGH
	HAL_Delay(150);
	GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<4); //Set PB4 as LOW

	// Yaw +
	HAL_Delay(150);
	GPIOB->ODR |= 1U<<3; //Set PB3 as HIGH
	HAL_Delay(150);
	GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<3); //Set PB3 as LOW

	// Yaw -
	HAL_Delay(150);
	GPIOA->ODR |= 1U<<15; //Set PA15 as HIGH
	HAL_Delay(150);
	GPIOA->ODR = ~(~(GPIOA->ODR) | 1U<<15); //Set PA15 as LOW

}


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

	Ax = (float)Accel_X_RAW/16384.0;
	Ay = (float)Accel_Y_RAW/16384.0;
	Az = (float)Accel_Z_RAW/16384.0;
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

	Gx = DEG_TO_RAD*(float)Gyro_X_RAW/131.0;
	Gy = DEG_TO_RAD*(float)Gyro_Y_RAW/131.0;
	Gz = DEG_TO_RAD*(float)Gyro_Z_RAW/131.0;
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


  // Solenoid MOSFET GPIOs
  GPIOB->MODER |= 1U<<16; //PB8 as output, ROLL -
  GPIOB->MODER |= 1U<<18; //PB9 as output, ROLL +

  GPIOB->MODER |= 1U<<10; //PB5 as output, Pitch +
  GPIOB->MODER |= 1U<<8; //PB4 as output, Pitch -

  GPIOB->MODER |= 1U<<6; //PB3 as output, Yaw +
  GPIOA->MODER |= 1U<<30; //PA15 as output, Yaw -


  // Solenoid Check
  // Runs thro every MOSFET

  //SolenoidChecker();

  //uint8_t readData1;
  //uint8_t writeData1;

  //IMUstate = MPU6050_Init();
  //HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &readData, 1, i2c_timeout);
  //IMUstate = readData;

//  writeData1 = 0x22;
//  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_PIN_CFG, 1, &writeData1, 1, i2c_timeout);
//
//  HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &readData1, 1, i2c_timeout);
//  IMUstate = readData1;

  while (MPU9255_Init(&hi2c1) == 1);


//  if( (MPU_begin(&hi2c1, AD0_LOW, AFSR_4G, GFSR_500DPS, 0.98, 0.004)) == 1){
//	  IMUstate = 1;
//  }

  // MPU6050 Code
//  if(IMUstate == 0x68){
//	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//	  HAL_Delay(150);
//	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//	  HAL_Delay(150);
//	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//	  HAL_Delay(150);
//	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//	  HAL_Delay(150);
//	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//	  HAL_Delay(150);
//	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//
//	  for (int i=0;i<200;i++){
//
//	  	  MPU6050_Read_Accel();
//	  	  MPU6050_Read_Gyro();
//
//	  	  Gx_sum = (Gx*RAD_TO_DEG)+Gx_sum;
//	  	  Gy_sum = (Gy*RAD_TO_DEG)+Gy_sum;
//	  	  Gz_sum = (Gz*RAD_TO_DEG)+Gz_sum;
//
//	  	  HAL_Delay(10);
//
//	    }
//
//	    Gx_bias = Gx_sum / 200;
//	    Gy_bias = Gy_sum / 200;
//	    Gz_bias = Gz_sum / 200;
//
//	    GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//	    HAL_Delay(150);
//	    GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//	    HAL_Delay(150);
//	    GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//	    HAL_Delay(150);
//	    GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//	    HAL_Delay(150);
//	    GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//	    HAL_Delay(150);
//	    GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//
//  } else {
//	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//	  HAL_Delay(750);
//	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//	  HAL_Delay(750);
//	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//	  HAL_Delay(750);
//	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//  }

  // MPU9250
//  if(IMUstate == 1){
//	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//  	  HAL_Delay(150);
//  	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//  	  HAL_Delay(150);
//  	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//  	  HAL_Delay(150);
//  	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//  	  HAL_Delay(150);
//  	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//  	  HAL_Delay(150);
//  	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//
//
//
//    } else {
//  	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//  	  HAL_Delay(750);
//  	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//  	  HAL_Delay(750);
//  	  GPIOC->ODR |= 1U<<13; //Set PC13 as HIGH
//  	  HAL_Delay(750);
//  	  GPIOC->ODR = ~(~(GPIOC->ODR) | 1U<<13); //Set PC13 as LOW
//    }


  Solenoid_timer = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
		  //MPU6050_Read_Accel();
		  //MPU6050_Read_Gyro();
		  readAll(&hi2c1, &MPU9255);

//		  sprintf(logBuf,"%0.3f,%0.3f,%0.3f,%d,%d,%0.3f,%0.3f,%0.3f\r\n",(Gx* RAD_TO_DEG) - Gx_bias,(Gy* RAD_TO_DEG) - Gy_bias,(Gz* RAD_TO_DEG) - Gz_bias,
//				  roll_plus,roll_minus,Gx_bias,Gy_bias,Gz_bias);
		  //sprintf(logBuf,"%0.3f,%0.3f,%0.1f,%0.1f\r\n",phiHat_rad* RAD_TO_DEG,thetaHat_rad* RAD_TO_DEG,(float)roll_plus,(float)roll_minus);
		  sprintf(logBuf,"%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n",ax,ay,az,gx,gy,gz);
		  CDC_Transmit_FS((uint8_t *) logBuf, strlen(logBuf));
		  USB_Timer = HAL_GetTick();
	  }

	  // Test Sequence
	  switch(TestState){
		  case 0:

			  if( (HAL_GetTick() - Solenoid_timer) >= test_interval_ms){
				  Solenoid_timer = HAL_GetTick();
				  TestState++;
			  }
			  break;

		  case Roll_plus_test:

			  GPIOB->ODR |= 1U<<9; //Set PB9 as HIGH

			  if( (HAL_GetTick() - Solenoid_timer) >= test_interval_ms){
				  Solenoid_timer = HAL_GetTick();
				  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<9); //Set PB9 as LOW
				  TestState++;
			  }
			  break;

		case Roll_minus_test:

			  GPIOB->ODR |= 1U<<8; //Set PB8 as HIGH

			  if( (HAL_GetTick() - Solenoid_timer) >= test_interval_ms){
				  Solenoid_timer = HAL_GetTick();
				  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<8); //Set PB8 as LOW
				  TestState++;
			  }
			  break;

		case Pitch_plus_test:

			  GPIOB->ODR |= 1U<<5; //Set PB5 as HIGH

			  if( (HAL_GetTick() - Solenoid_timer) >= test_interval_ms){
				  Solenoid_timer = HAL_GetTick();
				  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<5); //Set PB5 as LOW
				  TestState++;
			  }
			  break;

		case Pitch_minus_test:

			  GPIOB->ODR |= 1U<<4; //Set PB4 as HIGH

			  if( (HAL_GetTick() - Solenoid_timer) >= test_interval_ms){
				  Solenoid_timer = HAL_GetTick();
				  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<4); //Set PB4 as LOW
				  TestState++;
			  }
			  break;

		case Yaw_plus_test:

			  GPIOB->ODR |= 1U<<3; //Set PB3 as HIGH

			  if( (HAL_GetTick() - Solenoid_timer) >= test_interval_ms){
				  Solenoid_timer = HAL_GetTick();
				  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<3); //Set PB3 as LOW
				  TestState++;
			  }
			  break;

		case Yaw_minus_test:

			  GPIOA->ODR |= 1U<<15; //Set PA15 as HIGH

			  if( (HAL_GetTick() - Solenoid_timer) >= test_interval_ms){
				  Solenoid_timer = HAL_GetTick();
				  Speedtest_timer = HAL_GetTick();
				  GPIOA->ODR = ~(~(GPIOA->ODR) | 1U<<15); //Set PA15 as LOW
				  TestState++;
			  }
			  break;

		case SpeedTest:

				if(Solenoid_toggleState){
					GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<9); //Set PB9 as LOW
				} else{
					GPIOB->ODR |= 1U<<9; //Set PB9 as HIGH
				}

			  if( (HAL_GetTick() - Speedtest_timer) >= (test_interval_ms/hertz) ){
				  Speedtest_timer = HAL_GetTick();
				  if(Solenoid_toggleState){
					  Solenoid_toggleState = 0;
				  } else {
					  Solenoid_toggleState = 1;
				  }
			  }


			  if( (HAL_GetTick() - Solenoid_timer) >= test_interval_ms){
				  Solenoid_timer = HAL_GetTick();
				  hertz++;
			  }

			  if(hertz>=6){
				  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<9); //Set PB9 as LOW
				  TestState++;
			  }

			  break;

		case stop_roll:

			  GPIOB->ODR |= 1U<<8; //Set PB8 as HIGH

			  if( (HAL_GetTick() - Solenoid_timer) >= (test_interval_ms*4)){
				  Solenoid_timer = HAL_GetTick();
				  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<8); //Set PB8 as LOW
				  TestState++;
			  }
			  break;

		default:
			TestState = 10;
			break;

	  }





	 // Control

	 // Current Plan
	 // P - Controller
	 // w_e = r * T
	 // Torque_input = K_qu * delta_qu + K_w * delta_w
	 // delta_w = w_d (desired angular rate) - w_e (estimated angular rate)
	 // delta_qu = (4x1) (matrix), explains required rotation (angle change) [qx , qy, qz, q4], [axis, angle]
	 //

	  // Rate of IC's change?
	  // Average IC measurements (right now 50Hz)

	  // EKF
	  // I_r*w_e_dot = -w_e*I_r*w_e + T_t (from actuated or assumed to be zero if below min thrust/torque)


//	  if( ( (Gz* RAD_TO_DEG) - Gz_bias) > 15){
//		  // Actuate Roll +, PB3
//		  GPIOB->ODR |= 1U<<3; //Set PB3 as HIGH
//		  roll_plus = 1;
//
//	  } else {
//		  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<3); //Set PB3 as LOW
//		  roll_plus = 0;
//	  }
//
//	  if( ((Gz* RAD_TO_DEG) - Gz_bias) < -15){
//		  // Actuate Roll -, PB4
//		  GPIOB->ODR |= 1U<<4; //Set PB4 as HIGH
//		  roll_minus = 1;
//	  } else {
//		  GPIOB->ODR = ~(~(GPIOB->ODR) | 1U<<4); //Set PB3 as LOW
//		  roll_minus = 0;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
