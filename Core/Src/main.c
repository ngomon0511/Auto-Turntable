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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "delay.h"
#include "lcd.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"
 
 // To process button event 
#define DEBOUNCE_DELAY_MS 500 
uint32_t lastButtonPressTime;
int runStatus;
float setAngleArr[] = {30, 45, 60, 90, 120, 180};
int setAngleArrLen = sizeof(setAngleArr) / sizeof(setAngleArr[0]);
 
 // To read rotary encoder 
int encoderPos;
#define x4PulsePerEncoderRnd 10000

// To process the motion of step motor
const int rpm = 375;
float setAngle = 90;
float rotateAngle = 90;
#define stepPerRnd 200
#define microSecondPerMinute 60000000
const float transmitRatio = 1.25;
const float anglePerStep = 1.8;
const uint16_t seqHalfDrive[8] = {0x0800, 0x0C00, 0x0400, 0x0600, 0x0200, 0x0300, 0x0100, 0x0900};

/////////////////////////////////////////  Process button event  ///////////////////////////////////////////
float findNext(float arr[], int arrSize, float cur){
    if (arr[arrSize - 1] == cur) {
        return arr[0];
    }
    else{
        for (int i = 0; i < arrSize; i++) {
            if (arr[i] == cur) {
                return arr[i + 1];
            }
        }
    }
    return -1; 
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// Denoise for button
	uint32_t currentTime = HAL_GetTick();
	if (currentTime - lastButtonPressTime < DEBOUNCE_DELAY_MS ) { return; }
	lastButtonPressTime = currentTime;
	
	// Handle button press event
	// <Start/Stop> button
	if (GPIO_Pin == GPIO_PIN_0){
		runStatus++;
	}
	// <Modify Rotate Angle> button
	else if (GPIO_Pin == GPIO_PIN_2){
		if (setAngle > 0){ setAngle = findNext(setAngleArr, setAngleArrLen, setAngle); }
		else{ setAngle = -findNext(setAngleArr, setAngleArrLen, fabs(setAngle)); }
		rotateAngle = setAngle;
	}
	// <Reverse Rotate Direction> button
	else if (GPIO_Pin == GPIO_PIN_3){
		setAngle = -setAngle;
		rotateAngle = setAngle;
	}
	
	// Update LCD
	if (runStatus % 2 == 1){
		LCD_GotoXY(0, 1);
		LCD_Puts("RUN - ANGLE:");
	} else if (runStatus % 2 == 0){
		LCD_GotoXY(0, 1);
		LCD_Puts(" STOP-ANGLE:");
	}
	char setAngleChar[5];
	sprintf(setAngleChar, "%d   ", (int)setAngle);
	LCD_GotoXY(12, 1);
	LCD_Puts(setAngleChar);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////  Read rotary encoder and control rotate angle of step motor  ////////////////////////
void makePosArr(float angle, float *posArr, int size) {
    for (int i = 0; i < size; i++) {
        posArr[i] = i * angle;
    }
}

float findNearest(float arr[], int arrSize, float num){
    int diff = fabs(arr[0] - num);
    int index = 0;
    for (int i = 1; i < arrSize; i++) {
        if (fabs(arr[i] - num) < diff) {
            diff = fabs(arr[i] - num);
            index = i;
        }
    }
    return arr[index];
}

void readEncoder(){
	// Read rotary encoder
	int cntValue = TIM4 -> CNT;
	encoderPos = round((360.0f / x4PulsePerEncoderRnd) * (cntValue % x4PulsePerEncoderRnd));
	
	// Control rotate angle of step motor
	int checkError = encoderPos % (int)setAngle;
	if (checkError!= 0){
		// Create the desired positions
		int posArrLen = (int) (360 / fabs(setAngle)) + 1;
		float posArr[posArrLen];
		makePosArr(fabs(setAngle), posArr, posArrLen);
		int nearestPos = findNearest(posArr, posArrLen, encoderPos);
		
		// Increase or decrease the steps to reach the desired position
		if (setAngle > 0){
			if (nearestPos > encoderPos){ rotateAngle = setAngle + (nearestPos - encoderPos); }
			else{ rotateAngle = setAngle - (encoderPos - nearestPos); }
		}
		else{
			if (nearestPos < encoderPos){ rotateAngle = -(fabs(setAngle) - (nearestPos - encoderPos)); }
			else{ rotateAngle = -(fabs(setAngle) + (encoderPos - nearestPos)); }
		}
	}
	else{ rotateAngle = setAngle; }
	
	// Update LCD
	LCD_GotoXY(0, 0);
	LCD_Puts(" POSITION: ");
	LCD_GotoXY(11, 0);
	char encoderPosChar[6];
	sprintf(encoderPosChar, "%d    ", encoderPos);
	LCD_Puts(encoderPosChar);
}
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////  Process the motion of step motor  //////////////////////////////
void runStep(int time, int step){
	GPIOA->ODR &= ~(0xF << 8);
	GPIOA->ODR |= seqHalfDrive[step % 8];
	Delay_Us(time);
}

void runAngle(float angle) {
	if (angle == 0) { return; }
	else{
		// Set parameters to run step motor
		angle = angle / transmitRatio;
		int timePerStep = microSecondPerMinute / rpm / stepPerRnd;
		int stepNumber = (int)( round( fabs(angle) / anglePerStep ) ) * 2;
		
		// Run step motor
		if (angle >= 0){
			for(int step = stepNumber; step >= 0; step--) { runStep(timePerStep, step); }
		} else if (angle < 0) {
			for (int step = 0; step <= stepNumber; step++) { runStep(timePerStep, step); }
		}
	}
}

void runMotor(){	
	if (runStatus % 2 == 1) {
		HAL_GPIO_WritePin(GPIOA, LED_Pin, 1);
		runAngle(rotateAngle);
		readEncoder();
		Delay_Ms(1000);
	} else if (runStatus % 2 == 0){
		HAL_GPIO_WritePin(GPIOA, LED_Pin, 0);
		runAngle(0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////

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
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  Delay_Init();
  LCD_Init();
  LCD_Puts(" AUTO TURNTABLE ");
  LCD_GotoXY(0, 1);
  LCD_Puts(" STOP - ANGLE:0 ");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	runMotor();
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|IN1_Pin|IN2_Pin|IN3_Pin
                          |IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW_BTN_Pin UP_ANGLE_BTN_Pin REVERSE_BTN_Pin */
  GPIO_InitStruct.Pin = SW_BTN_Pin|UP_ANGLE_BTN_Pin|REVERSE_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin IN1_Pin IN2_Pin IN3_Pin
                           IN4_Pin */
  GPIO_InitStruct.Pin = LED_Pin|IN1_Pin|IN2_Pin|IN3_Pin
                          |IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_SPI1_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
