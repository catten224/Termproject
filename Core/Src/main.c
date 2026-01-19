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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UART.h"
#include "B_MOTOR.h"
#include "STEP_MOTOR.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t num = 0;
// 통신 변수 (1회성)
uint8_t temp = 1;
// 블루투스 통신 변수
uint8_t rxbuffer[20];
uint8_t txbuffer[200];

// 거리 센서 변수
uint8_t uart[9];		// 거리 센서 데이터
uint16_t dist;			// 현재 거리 값
uint16_t goaldist;	// 목표 거리 값
int32_t d_error;		// 현재 거리 - 목표 거리
int32_t ticks;			// 이동 엔코더 값
uint16_t strength;
uint8_t check;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void routine_move(bool encoder_use);
void routine_slide();
bool routine_wait_for_data(uint8_t data_size);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7) {
		B_MOTOR_ReadEncoder();
		T_MOTOR_ReadEncoder();
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM9) {
		slide.motor.current_steps++;  // 현재 스텝 증가

		if (slide.motor.current_steps >= slide.motor.total_steps) {
			STEP_MOTOR_STOP();       		// 목표 도달 → 모터 정지
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	// 블루투스 함ㅅ
	if (huart->Instance == USART6)
	{
		switch(rxbuffer[0]){
			case '0':
				sprintf((char *)txbuffer, "currentStep: %d\r\n", currentStep);
				HAL_UART_Transmit_IT(&huart6, (uint8_t *)txbuffer, strlen((char *)txbuffer));
				break;

			case '1':
				UART_Transmit_EPS32("2");	// 로봇 리프트 상층 중 신호 송신
				sprintf((char *)txbuffer, "send 2\r\n");
				HAL_UART_Transmit_IT(&huart6, (uint8_t *)txbuffer, strlen((char *)txbuffer));
				break;

			case '2':
				UART_Transmit_EPS32("3");	// 로봇 리프트 상층 중 신호 송신
				sprintf((char *)txbuffer, "send 3\r\n");
				HAL_UART_Transmit_IT(&huart6, (uint8_t *)txbuffer, strlen((char *)txbuffer));
				break;

			case '3':
				UART_Transmit_EPS32("4");	// 로봇 리프트 상층 중 신호 송신
				sprintf((char *)txbuffer, "send 4\r\n");
				HAL_UART_Transmit_IT(&huart6, (uint8_t *)txbuffer, strlen((char *)txbuffer));
				break;

			case '4':
				currentStep++;
				sprintf((char *)txbuffer, "next\r\n");
				HAL_UART_Transmit_IT(&huart6, (uint8_t *)txbuffer, strlen((char *)txbuffer));
				break;

			case '5':
				break;

			case '6':
				sprintf((char *)txbuffer, "dist: %3d\r\n", dist);
				HAL_UART_Transmit_IT(&huart6, (uint8_t *)txbuffer, strlen((char *)txbuffer));
				break;

			default:
				HAL_UART_Receive_IT(&huart6, rxbuffer, 3);
				break;
		}

		HAL_UART_Receive_IT(&huart6, rxbuffer, 3);
	}

	// 거리 센서 함수
	if (huart->Instance == USART3){
		if (uart[0] == 0x59 && uart[1] == 0x59){
			check = 0;
			for (int i=0; i<8; i++) check += uart[i];

			if (uart[8] == (check & 0xFF))
			{
				dist = uart[2] + (uart[3]<<8);
				strength = uart[4] + (uart[5]<<8);
			}
		}

		HAL_UART_Receive_IT(&huart3, uart, 9);
	}
}
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
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart6, rxbuffer, 3);	// 블루투스 통신 시작
  HAL_UART_Receive_IT(&huart3, uart, 9);			// 거리 센서 시작

  HAL_TIM_Base_Start_IT(&htim7);	// 엔코더 카운터 타이머 시작

	// 이동 모터 타이머 초기화 및 변수 초기화
	B_MOTOR_Init();
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // 1차 슬라이드 (스텝 모터)
  STEP_MOTOR_HOLDOFF_OFF();

  // 2차 슬라이드 타이머 시작
  T_MOTOR_Init();
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // 슬라이드 그리퍼 (서보 모터)
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 6);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 6);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if 1
    switch (currentStep) {
/* ******************************************************************************************************************* */
			case STEP_WAIT_FOR_COMMAND:
				// 최초 1회 실행
				if (temp == 1){
					memset(command, 0, COMMAND_MAX_SIZE+2);	// 통신 변수 초기화
					temp = 2;
				}

		    if (routine_wait_for_data(1)) {
		        UART_Command_Parse();

		        if (currentPosition.io_type == 0)	SERVO_DOWN();

		        currentStep = STEP_MOVE_INIT_TO_LIFT;
		    }

				break;
/* ******************************************************************************************************************* */
			case STEP_MOVE_INIT_TO_LIFT:
				// 최초 1회 실행
				if (temp == 2){
					goaldist = CELL_LIFT;
					if (currentPosition.floor == 1) temp = 4;
					else temp = 3;
				}

				routine_move(false);

				break;
/* ******************************************************************************************************************* */
			case STEP_WAIT_LIFT_UP:
				// 최초 1회 실행
				if (temp == 3){
					UART_Transmit_EPS32("2");	// 로봇 리프트 상층 중 신호 송신
					temp = 4;
				}

		    if (routine_wait_for_data(1)) {
		        currentStep = STEP_MOVE_LIFT_TO_SLOT;
		    }
				break;
/* ******************************************************************************************************************* */
			case STEP_MOVE_LIFT_TO_SLOT:
				// 최초 1회 실행
				if (temp == 4){
					if (currentPosition.cell == 1) goaldist = CELL_1;
					else if (currentPosition.cell == 2) goaldist = CELL_2;
					else if (currentPosition.cell == 3) goaldist = CELL_3;
					temp = 5;
				}

				routine_move(true);

				break;
/* ******************************************************************************************************************* */
			case STEP_PERFORM_SLIDE:
				// 최초 1회 실행
				if (temp == 5){
					temp = 6;
				}

				routine_slide();

				break;
/* ******************************************************************************************************************* */
			case STEP_MOVE_SLOT_TO_LIFT:
				// 최초 1회 실행
				if (temp == 6){
					goaldist = CELL_LIFT;
					if (currentPosition.floor == 1) temp = 8;
					else temp = 7;
				}

				routine_move(true);

				break;
/* ******************************************************************************************************************* */
			case STEP_WAIT_LIFT_DOWN:
				// 최초 1회 실행
				if (temp == 7){
					UART_Transmit_EPS32("3");	// 로봇 리프트 하강 중 신호 송신
					temp = 8;
				}

		    if (routine_wait_for_data(1)) {
		        currentStep = STEP_MOVE_LIFT_TO_INIT;
		    }

				break;
/* ******************************************************************************************************************* */
			case STEP_MOVE_LIFT_TO_INIT:
				// 최초 1회 실행
				if (temp == 8){
					goaldist = CELL_INIT;
					temp = 9;
				}

				routine_move(false);

				break;
/* ******************************************************************************************************************* */
			case STEP_OPERATION_COMPLETE:
				// 최초 1회 실행
				if (temp == 9){
					UART_Transmit_EPS32("4");	// 로봇 동작 확인 완료 신호 송신

					if (currentPosition.io_type == 1) SERVO_UP();
					temp = 1;
				}

        currentStep = STEP_WAIT_FOR_COMMAND;

				break;
/* ******************************************************************************************************************* */
			case STEP_OPERATION_INCOMPLETE:
				// 최초 1회 실행
				if (temp == 9){
					temp = 1;
				}

        currentStep = STEP_WAIT_FOR_COMMAND;

				break;
/* ******************************************************************************************************************* */
    }
    HAL_Delay(10);
#endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 21000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 80-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 6;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  htim7.Init.Prescaler = 84-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 168-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 180-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  huart2.Init.BaudRate = 57600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE7 PE8
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ******************************************************************************************************************* */
bool routine_wait_for_data(uint8_t data_size)
{
  UART_Receive_ESP32();                     				// 데이터 수신 대기
  num++;
  UART_Command_Vaildate(data_size, currentStep);    // 유효성 검사

  if (Rx_flag) {
      Rx_flag = false;                    				  // 수신 플래그 초기화
      return true;
  }
  return false;
}

void routine_move(bool use_encoder)
{
  switch (robot_state) {
/* ******************************************************************************************************************* */
      case ROBOT_STATE_INIT:

      	HAL_Delay(10);
      	B_MOTOR_Init();

      	robot_state = ROBOT_STATE_DISTANCE_CALC;
        break;

/* ******************************************************************************************************************* */
      case ROBOT_STATE_DISTANCE_CALC:

				d_error = dist - goaldist;
				ticks = ENCODER_COUNTS_PER_CM * ((abs(d_error)-10));

      	robot_state = ROBOT_STATE_MOTOR_START;
        break;

/* ******************************************************************************************************************* */
      case ROBOT_STATE_MOTOR_START:

				if (use_encoder) {
					if (d_error >= 0) {
						L_B_Motor.PID.Target32 = -ticks;
						R_B_Motor.PID.Target32 = ticks;
						B_MOTOR_L_DIR_CCW();
						B_MOTOR_R_DIR_CW();
						HAL_Delay(10);
					}
					else {
						L_B_Motor.PID.Target32 = ticks;
						R_B_Motor.PID.Target32 = -ticks;
						B_MOTOR_L_DIR_CW();
						B_MOTOR_R_DIR_CCW();
						HAL_Delay(10);
					}

					robot_state = ROBOT_STATE_ENCODER_APPROACH;
				}
				else{
					if (d_error >= 0) {
						B_MOTOR_L_DIR_CCW();
						B_MOTOR_R_DIR_CW();
						HAL_Delay(10);
					}
					else{
						B_MOTOR_L_DIR_CW();
						B_MOTOR_R_DIR_CCW();
						HAL_Delay(10);
					}

					B_MOTOR_SETPWM(&L_B_Motor, L_B_PWM_MIN);
					B_MOTOR_SETPWM(&R_B_Motor, R_B_PWM_MIN);

					robot_state = ROBOT_STATE_SENSOR_APPROACH;
				}

				B_MOTOR_L_ENABLE();
				B_MOTOR_R_ENABLE();

				break;

/* ******************************************************************************************************************* */
      case ROBOT_STATE_ENCODER_APPROACH:

      	B_MOTOR_PidControl();	// PID 제어 반복

      	if (L_B_Motor.flag && R_B_Motor.flag){
      		HAL_Delay(200);
      		robot_state = ROBOT_STATE_SENSOR_APPROACH;
      	}

				break;

/* ******************************************************************************************************************* */
      case ROBOT_STATE_SENSOR_APPROACH:

				B_MOTOR_L_ENABLE();
				B_MOTOR_R_ENABLE();

      	if (abs(dist - goaldist) <= 1){
					B_MOTOR_L_DISABLE();
					B_MOTOR_R_DISABLE();
					robot_state = ROBOT_STATE_DONE;
      	}

				break;

/* ******************************************************************************************************************* */
      case ROBOT_STATE_DONE:

      	robot_state = ROBOT_STATE_INIT;

      	if (currentStep == STEP_MOVE_INIT_TO_LIFT) {
      		if (currentPosition.floor == 1) currentStep = STEP_MOVE_LIFT_TO_SLOT;
      		else currentStep = STEP_WAIT_LIFT_UP;
      	}
      	else if (currentStep == STEP_MOVE_LIFT_TO_SLOT)	{
      		currentStep = STEP_PERFORM_SLIDE;
      	}
      	else if (currentStep == STEP_MOVE_SLOT_TO_LIFT) {
      		if (currentPosition.floor == 1) currentStep = STEP_MOVE_LIFT_TO_INIT;
      		else currentStep = STEP_WAIT_LIFT_DOWN;
      	}
      	else if (currentStep == STEP_MOVE_LIFT_TO_INIT) {
      		currentStep = STEP_OPERATION_COMPLETE;
      	}

      	HAL_Delay(100);
				break;
/* ******************************************************************************************************************* */
  }
}

void routine_slide()
{
  switch (slide_state) {
/* ******************************************************************************************************************* */
      case SLIDE_STATE_INIT:

      	HAL_Delay(10);

      	if (currentPosition.io_type == 0) {
      		slide.mode = SLIDE_MODE_IN;
      	} else {
      		slide.mode = SLIDE_MODE_OUT;
      	}
      	if (currentPosition.side == 0) {
      		slide.direction = SLIDE_DIR_LEFT;
      	} else {
      		slide.direction = SLIDE_DIR_RIGHT;
      	}

      	T_MOTOR_Init();
      	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 60);
      	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 10);

  			L_T_Motor.PID.Target32 = 6000;	// 6000
  			R_T_Motor.PID.Target32 = 6500;	// 6500

      	slide_state = SLIDE_STATE_MOVE_FORWARD;
        break;

/* ******************************************************************************************************************* */
      case SLIDE_STATE_MOVE_FORWARD:

        if (slide.direction == SLIDE_DIR_LEFT)
        {
					T_MOTOR_L_DIR_CW();
          STEP_MOTOR_DIR_CW();
					HAL_Delay(10);
    			T_MOTOR_L_ENABLE();
					T_MOTOR_R_DIR_CCW();
          STEP_MOTOR_START();
        }
        else if (slide.direction == SLIDE_DIR_RIGHT)
        {
        	T_MOTOR_L_DIR_CCW();
          STEP_MOTOR_DIR_CCW();
          HAL_Delay(10);
    			T_MOTOR_L_ENABLE();
					T_MOTOR_R_DIR_CW();
          STEP_MOTOR_START();
        }

      	slide_state = SLIDE_STATE_WAIT_FORWARD_DONE;
        break;

/* ******************************************************************************************************************* */
      case SLIDE_STATE_WAIT_FORWARD_DONE:

        T_MOTOR_Control();

        if (slide.motor.step_flag && L_T_Motor.flag && R_T_Motor.flag){
        	slide_state = SLIDE_STATE_SERVO_ACTION;
        }
				break;

/* ******************************************************************************************************************* */
      case SLIDE_STATE_SERVO_ACTION:

      	HAL_Delay(500);

        if (slide.mode == SLIDE_MODE_IN)
          SERVO_UP();
        else if (slide.mode == SLIDE_MODE_OUT)
          SERVO_DOWN();

        HAL_Delay(500);

      	slide_state = SLIDE_STATE_MOVE_BACKWARD;
				break;

/* ******************************************************************************************************************* */
      case SLIDE_STATE_MOVE_BACKWARD:

      	T_MOTOR_Init();
      	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 60);
      	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 10);

        if (slide.direction == SLIDE_DIR_LEFT)
        {
					T_MOTOR_L_DIR_CCW();
          STEP_MOTOR_DIR_CCW();
					HAL_Delay(10);
    			T_MOTOR_L_ENABLE();
					T_MOTOR_R_DIR_CW();
          STEP_MOTOR_START();
        }
        else if (slide.direction == SLIDE_DIR_RIGHT)
        {
					T_MOTOR_L_DIR_CW();
          STEP_MOTOR_DIR_CW();
					HAL_Delay(10);
    			T_MOTOR_L_ENABLE();
					T_MOTOR_R_DIR_CCW();
          STEP_MOTOR_START();
        }

      	slide_state = SLIDE_STATE_WAIT_BACKWARD_DONE;
				break;

/* ******************************************************************************************************************* */
      case SLIDE_STATE_WAIT_BACKWARD_DONE:

        T_MOTOR_Control();

        if (slide.motor.step_flag && L_T_Motor.flag && R_T_Motor.flag){
        	slide_state = SLIDE_STATE_DONE;
        }
				break;
/* ******************************************************************************************************************* */
			case SLIDE_STATE_DONE:

				slide_state = SLIDE_STATE_INIT;
				currentStep = STEP_MOVE_SLOT_TO_LIFT;

				HAL_Delay(100);
				break;
/* ******************************************************************************************************************* */
  }
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
