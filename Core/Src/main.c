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
#include "stdlib.h"
#include "cmsis_os.h"

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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

osThreadId task1Handle;
osThreadId task2Handle;
osThreadId task3Handle;
osThreadId task4Handle;
/* USER CODE BEGIN PV */

// PA8(CH1) - PB4(EXTI4) - OUT3 OUT4
// PA9(CH2) - PB5(EXTI5) - OUT1 OUT2
// PA11(CH3) - PB6(EXTI6) - OUT3 OUT4
// PA10(CH4) - PB7(EXTI7) - OUT1 OUT2

uint8_t c[8] = {0};

volatile int count_timer1;
volatile int count_timer3;
volatile int count_EXTI4 = 0;
volatile int count_EXTI5 = 0;
volatile int count_EXTI6 = 0;
volatile int count_EXTI7 = 0;


volatile int previous_count_encoder1 = 0;
volatile int previous_count_encoder2 = 0;
volatile int previous_count_encoder3 = 0;
volatile int previous_count_encoder4 = 0;

volatile uint32_t lastUpdateTime = 0;

float speed_1 = 0.0;
float speed_2 = 0.0;
float speed_3 = 0.0;
float speed_4 = 0.0;

// PID variables
float kp = 0.003;
float ki = 0.05;
float kd = 0.001;
float setpoint = 600.0;
float input1, output1;
float lastInput1;
float errSum1, lastErr1;

float input2, output2;
float lastInput2;
float errSum2, lastErr2;

float input3, output3;
float lastInput3;
float errSum3, lastErr3;

float input4, output4;
float lastInput4;
float errSum4, lastErr4;


uint16_t pwm_value;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
void task1_function(void const * argument);
void task2_function(void const * argument);
void task3_function(void const * argument);
void task4_function(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TransUart2(uint8_t t[])
{
	for(int i =0;i<100;i++)
	{
		if(t[i] == NULL) break;
		else HAL_UART_Transmit(&huart2,&t[i],1,500);
	}
}
void tangtoc()
{
	for(int i = 0;i<100;i++)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,i);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,i);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,i);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,i);
		HAL_Delay(30);
	}

}

void giamtoc()
{
	for(int i = 100;i>0;i--)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,i);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,i);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,i);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,i);
		HAL_Delay(30);
	}

}

void dung()
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5, 1);
}
void tien()
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4, 0);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5, 1);
}

void lui()
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4, 0);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5, 1);
}
void trai()
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5, 0);
}

void phai()
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, 0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, 0);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4, 0);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5, 1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4)
	{
		count_EXTI4++;
	}
	if(GPIO_Pin == GPIO_PIN_5)
	{
		count_EXTI5++;
	}
	if(GPIO_Pin == GPIO_PIN_6)
	{
		count_EXTI6++;
	}
	if(GPIO_Pin == GPIO_PIN_7)
	{
		count_EXTI7++;
	}
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  //HAL_TIM_Base_Start(&htim1);
  //HAL_TIM_Base_Start(&htim3);
  TransUart2("Start \n");
  TransUart2("Nhap toc do 100 - 900 RPM: ");
  int flag = 1;
  uint8_t c[4];
  while(flag)
  {
	  HAL_UART_Receive(&huart2,c,3,500);
	  c[3]= '\0';
	  int number = atoi((char*)c); // Chuyển chuỗi thành số nguyên
	  setpoint = number;
	  if(setpoint >= 50 && setpoint <= 900)
	  {
		  flag = 0;
		  break;
	  }
  }
  tien();
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,20);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,20);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,20);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,20);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */


  /* definition and creation of task1 */
  osThreadDef(task1, task1_function, 1, 0, 128);
  task1Handle = osThreadCreate(osThread(task1), NULL);

  /* definition and creation of task2 */
  osThreadDef(task2, task2_function,1, 0, 128);
  task2Handle = osThreadCreate(osThread(task2), NULL);

  /* definition and creation of task3 */
  osThreadDef(task3, task3_function, 1, 0, 128);
  task3Handle = osThreadCreate(osThread(task3), NULL);

  /* definition and creation of task4 */
  osThreadDef(task4, task4_function, 1, 0, 128);
  task4Handle = osThreadCreate(osThread(task4), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 480;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


/* USER CODE BEGIN Header_task1_function */
/**
* @brief Function implementing the task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task1_function */
void task1_function(void const * argument)
{
  /* USER CODE BEGIN task1_function */

  /* Infinite loop */
	for(;;)
	{
	//HAL_UART_Receive(&huart2,&c[0],8,500);
	__enable_irq();
	HAL_Delay(1000);
	__disable_irq();
	speed_1 = (count_EXTI4/210)*60.0;
	count_EXTI4 = 0;
	input1 = speed_1;
	float err = setpoint - input1;
	errSum1 += err;
	float dErr = (input1 - lastInput1);
	output1 = kp * err + ki * errSum1 - kd * dErr;
	lastErr1 = err;
	lastInput1 = input1;
	if(output1 < 0) output1 = 0;
	else if(output1 > 100) output1 = 100;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,output1);
	uint32_t output_Tx = (uint32_t)output1;
	char str[4];
	sprintf(str, "%d", output_Tx);
 	TransUart2("OUTPUT1: ");
	HAL_UART_Transmit(&huart2, &str, 3, 500);
	TransUart2("\n");
	}
  /* USER CODE END task1_function */
}

/* USER CODE BEGIN Header_task2_function */
/**
* @brief Function implementing the task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task2_function */
void task2_function(void const * argument)
{
  /* USER CODE BEGIN task2_function */
  /* Infinite loop */
  for(;;)
  {
	  __enable_irq();
	  HAL_Delay(1000);
	  __disable_irq();
	  speed_2 = (count_EXTI5/210)*60.0;
	  count_EXTI5 = 0;
 	  input2 = speed_2;
	  float err = setpoint - input2;
	  errSum2 += err;
	  float dErr = (input2 - lastInput2);
	  output2 = kp * err + ki * errSum2 - kd * dErr;
	  lastErr2 = err;
	  lastInput2 = input2;
	  if(output2 < 0) output2 = 0;
	  else if(output2 > 100) output2 = 100;
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,output2);
	  uint32_t output_Tx = (uint32_t)output2;
	  char str[4]; // Chuỗi để lưu giá trị uint8_t dưới dạng chuỗi
	  sprintf(str, "%d", output_Tx); // Chuyển giá trị uint8_t thành chuỗi
	  TransUart2("OUTPUT2: ");
	  HAL_UART_Transmit(&huart2, &str, 3, 500);
	  TransUart2("\n");
  }
  /* USER CODE END task2_function */
}

/* USER CODE BEGIN Header_task3_function */
/**
* @brief Function implementing the task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task3_function */
void task3_function(void const * argument)
{
  /* USER CODE BEGIN task3_function */
  /* Infinite loop */
  for(;;)
  {
	  __enable_irq();
	  HAL_Delay(1000);
	  __disable_irq();
	  speed_3 = (count_EXTI6/210)*60.0;
	  count_EXTI6 = 0;
	  input3 = speed_3;
	  float err = setpoint - input3;
	  errSum3 += err;
	  float dErr = (input3 - lastInput3);
	  output3 = kp * err + ki * errSum3 - kd * dErr;
	  lastErr3 = err;
	  lastInput3 = input3;
	  if(output3 < 0) output3 = 0;
	  else if(output3 > 100) output3 = 100;
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,output3);
	  uint32_t output_Tx = (uint32_t)output3;
	  char str[4]; // Chuỗi để lưu giá trị uint8_t dưới dạng chuỗi
	  sprintf(str, "%d", output_Tx); // Chuyển giá trị uint8_t thành chuỗi
	  TransUart2("OUTPUT3: ");
	  HAL_UART_Transmit(&huart2, &str, 3, 500);
	  TransUart2("\n");
  }
  /* USER CODE END task3_function */
}

/* USER CODE BEGIN Header_task4_function */
/**
* @brief Function implementing the task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task4_function */
void task4_function(void const * argument)
{
  /* USER CODE BEGIN task4_function */
  /* Infinite loop */
  for(;;)
  {
	  __enable_irq();
	  HAL_Delay(1000);
	  __disable_irq();
	  speed_4 = (count_EXTI7/210)*60.0;
	  count_EXTI7 = 0;
	  input4 = speed_4;
	  float err = setpoint - input4;
	  errSum4 += err;
	  float dErr = (input4 - lastInput4);
	  output4 = kp * err + ki * errSum4 - kd * dErr;
	  lastErr4 = err;
	  lastInput4 = input4;
	  if(output4 < 0) output4 = 0;
	  else if(output4 > 100) output4 = 100;
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,output4);
	  uint32_t output_Tx = (uint32_t)output4;
	  char str[4]; // Chuỗi để lưu giá trị uint8_t dưới dạng chuỗi
	  sprintf(str, "%d", output_Tx); // Chuyển giá trị uint8_t thành chuỗi
	  TransUart2("OUTPUT4: ");
	  HAL_UART_Transmit(&huart2, &str, 3, 500);
	  TransUart2("\n");
  }
  /* USER CODE END task4_function */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
