/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define adxl_address 0x53<<1

uint8_t rxData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

uint8_t data_rec[6];

void adxl_write(uint8_t reg,uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, adxl_address, data, 2, 10);
}


void adxl_read(uint8_t reg,uint8_t numberofbytes)
{
	HAL_I2C_Mem_Read(&hi2c1,adxl_address, reg, 1, data_rec, numberofbytes, 100);
}

void adxl_init (void)
{
	adxl_read (0x00, 1);

	adxl_write (0x2d, 0);
	adxl_write (0x2d, 0x08);
	adxl_write(0x31, 0x01);

}



	void usDelay(uint32_t uSec);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const float speedOfSound = 0.0343/2;
float distance;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  int counterValue = 0;
  int pastCounterValue = 0;
  float angleValue = 0;
  char printMessage[200] = {'\0'};

  int counterValue3 = 0;
    int pastCounterValue3 = 0;
    float angleValue3 = 0;
    char printMessage3[200] = {'\0'};
    char printMessage_akc[200] = {'\0'};
    char printMessage_US[200] = {'\0'};
    float x,y,z,xg,yg,zg;
    float distVarStop = 10;
    uint32_t numTicks;
    //TIM1->CCR1 = 400;
    //TIM1->CCR2 = 400;


  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


  adxl_init();

  HAL_UART_Receive_IT(&huart1,&rxData,1);

  HAL_GPIO_WritePin(AT_Mode_GPIO_Port, AT_Mode_Pin, 0);
/*
  void adxl_read_values (uint8_t reg)
  {
     HAL_I2C_Mem_Read (&hi2c1, adxl_address, reg, 1, (uint8_t *)data_rec, 6, 100);
  }
*/
 // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //BT

	  if (distVarStop >= 5)
	  	  {
	  		  HAL_UART_RxCpltCallback(&huart1);
	  	  } else if(distVarStop < 5) {
	  		 HAL_Delay(10);
	  	 while (distVarStop < 5){

	  		  TIM1->CCR1 = 100;
	  		  TIM1->CCR2 = 150;

	  			HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
	  			HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 0); //prosto lewe


	  			HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 1); // tyl prawe
	  			HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 1); // tyl lewe
	  		  HAL_Delay(1000);
	  			HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
	  			HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 0); //prosto lewe


	  			HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); // tyl prawe
	  			HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe
	  		  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
	  		  usDelay(3);

	  		  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
	  		  usDelay(10);
	  		  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);


	  		  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET);

	  		  numTicks = 0;

	  		  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET)
	  		  {
	  			  numTicks++;
	  			  usDelay(2);
	  		  }

	  		  distance = (numTicks + 0.0f)*2.8*speedOfSound;
	  		  distVarStop = distance;
	  	  }
	  	  }

	  //HAL_UART_RxCpltCallback(&huart1);
	  HAL_GPIO_WritePin(AT_Mode_GPIO_Port, AT_Mode_Pin, 1);


	  //Czujnik ultradzwiekowy
	  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
	  usDelay(3);

	  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
	  usDelay(10);
	  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);


	  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET);

	  numTicks = 0;

	  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET)
	  {
		  numTicks++;
		  usDelay(2);
	  }

	  distance = (numTicks + 0.0f)*2.8*speedOfSound;
	  distVarStop = distance;

	  sprintf(printMessage_US, "Odległość: %f cm\n\r", distance);
	  	  HAL_UART_Transmit(&huart2, (uint8_t*)printMessage_US, sizeof(printMessage_US), 300);
/*
	  if (distance >= 5)
	  {
		  HAL_UART_RxCpltCallback(&huart1);
	  } else {
		  TIM1->CCR1 = 200;
		  TIM1->CCR2 = 200;
	  }
	  */
	  //akcelerometr
	  adxl_read (0x32,6);
	  x = ((data_rec[1]<<8)|data_rec[0]);
	  y = ((data_rec[3]<<8)|data_rec[2]);
	  z = ((data_rec[5]<<8)|data_rec[4]);


	  xg = x*.0078;
	  yg = y*.0078;
	  zg = z*.0078;




	  sprintf(printMessage_akc, "x = %f, y = %f, z = %f \n\n\r", xg, yg, zg);
	  HAL_UART_Transmit(&huart2, (uint8_t*)printMessage_akc, sizeof(printMessage_akc), 300);

	 //enkoder
	 counterValue = TIM2->CNT;
	 	 if (counterValue != pastCounterValue)
	 	 {
	 		 angleValue = (360.0/2400.0)*((float)counterValue);
	 		 sprintf(printMessage, "Current angle is motor A: %f\n\r", angleValue);
	 		 HAL_UART_Transmit(&huart2, (uint8_t*)printMessage, sizeof(printMessage), 300);
	 	 }
	 	 pastCounterValue = counterValue;

	 	counterValue3 = TIM3->CNT;
	 		 	 if (counterValue3 != pastCounterValue3)
	 		 	 {
	 		 		 angleValue3 = (360.0/2400.0)*((float)counterValue3);
	 		 		 sprintf(printMessage3, "Current angle is motor B: %f\n\r", angleValue3);
	 		 		 HAL_UART_Transmit(&huart2, (uint8_t*)printMessage3, sizeof(printMessage3), 300);
	 		 	 }
	 		 	 pastCounterValue3 = counterValue3;




	//HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); // tyl prawe
	//HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe


	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	 //HAL_Delay(100);

	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	 //HAL_Delay(1000);
	 /*
	 HAL_Delay(3000);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	 TIM1->CCR1 = 200;
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	 HAL_Delay(3000);
 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AT_Mode_Pin|V_1MotorB_Pin|V_2MotorB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, V_1MotorA_Pin|V_2MotorA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Trigger_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AT_Mode_Pin V_1MotorB_Pin V_2MotorB_Pin */
  GPIO_InitStruct.Pin = AT_Mode_Pin|V_1MotorB_Pin|V_2MotorB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : V_1MotorA_Pin V_2MotorA_Pin */
  GPIO_InitStruct.Pin = V_1MotorA_Pin|V_2MotorA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void usDelay(uint32_t uSec)
{
	if(uSec<2) uSec = 2;
	TIM4->ARR = uSec - 1;
	TIM4->EGR = 1;
	TIM4->SR &= ~1;
	TIM4->CR1 |= 1;
	while((TIM4->SR&0x0001) != 1);
	TIM4->SR &= ~(0x0001);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
	// HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 1); //prosto lewe


	//HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); // tyl prawe
	// HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe
  if(huart->Instance==USART1)
  {
    if(rxData==78) // Ascii value of 'N' is 78 (N for NO)
    {
    	HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); // tyl prawe
    	HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
    	HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 0); // prosto lewe
    	HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe

    }
    else if (rxData==89) // Ascii value of 'Y' is 89 (Y for YES)
    {
   	 HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 1); //prosto prawe
   	 HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 1); //prosto lewe

   	 HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); // tyl prawe
   	 HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe
    	TIM1-> CCR1 = 100;
    	TIM1-> CCR2 = 100;
    }
    else if (rxData == 49) //1 skret w prawo
    {
    	HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 1); //tyl prawe
    	HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
    	HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 1); //prosto lewe
    	HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe
    	TIM1-> CCR1 = 75;
    	TIM1-> CCR2 = 75;
    }
    else if (rxData == 50) //2 stop skret w prawo
        {
    	HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); //tyl prawe
    	HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
    	HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 0); //prosto lewe
    	HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe

        }


  else if (rxData == 51) //3 skret w lewo
  {
  	HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); //tyl prawe
  	HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 1); //prosto prawe
  	HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 0); //prosto lewe
  	HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 1); // tyl lewe
	      	TIM1-> CCR1 = 75;
	      	TIM1-> CCR2 = 75;
  }
  else if (rxData == 52) //4 skret w lewo stop
   {
  	HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); //tyl prawe
  	HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
  	HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 0); //prosto lewe
  	HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe

   }

  else if (rxData == 53) //5 tyl
   {
  	HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 1); //tyl prawe
  	HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
  	HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 0); //prosto lewe
  	HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 1); // tyl lewe
 	      	TIM1-> CCR1 = 60;
 	      	TIM1-> CCR2 = 60;
   }
  else if (rxData == 54) //6 tyl stop
   {
  	HAL_GPIO_WritePin(V_2MotorA_GPIO_Port, V_2MotorA_Pin, 0); //tyl prawe
  	HAL_GPIO_WritePin(V_1MotorA_GPIO_Port, V_1MotorA_Pin, 0); //prosto prawe
  	HAL_GPIO_WritePin(V_2MotorB_GPIO_Port, V_2MotorB_Pin, 0); //prosto lewe
  	HAL_GPIO_WritePin(V_1MotorB_GPIO_Port, V_1MotorB_Pin, 0); // tyl lewe

   }
  }
    HAL_UART_Receive_IT(&huart1,&rxData,1); // Enabling interrupt receive again
  }

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
