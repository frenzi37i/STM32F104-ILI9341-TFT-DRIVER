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
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t SPI_transfer_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void LCD_demo_OLD(void);
void new_LCD_demo(void);
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
  MX_DMA_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//LCD_demo_OLD();
	  static int cycles = 0;
	  if(!cycles){
		  new_LCD_demo();
		  cycles++;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = WIRE_SPEED_COUNTER_TIMER_PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = WIRE_SPEED_COUNTER_TIMER_ARR;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = PWM_TIMER_PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PWM_AUTORELOAD_REG_ARR;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.Mode = UART_MODE_TX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, TFT_DC_Pin|TFT_RESET_Pin|TFT_CS_Pin|ACTIVATE_ARC_OUT_Pin
                          |GAS_SOLENOID_OUT_Pin|EEPROM_WRITE_ENABLE_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, DEBUG_OUT_Pin|DC_MOTOR_RELAY_OUT_Pin);

  /**/
  GPIO_InitStruct.Pin = OPT_SELECT_PUSH_BUTTON_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(OPT_SELECT_PUSH_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MIG_BUTTON_IN_Pin|OPT_SELECTOR_2T_Pin|PID_DIP_SW_BIT2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TFT_DC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(TFT_DC_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TFT_RESET_Pin|ACTIVATE_ARC_OUT_Pin|GAS_SOLENOID_OUT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TFT_CS_Pin|EEPROM_WRITE_ENABLE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OPTO_IN_Pin|PID_DIP_SW_BIT1_Pin|OPT_SELECTOR_WIRE_TEST_Pin|OPT_SELECTOR_GAS_TEST_Pin
                          |OPT_SELECTOR_4T_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DEBUG_OUT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(DEBUG_OUT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DC_MOTOR_RELAY_OUT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(DC_MOTOR_RELAY_OUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void new_LCD_demo(void){
	   //debug_blink(3);
	   ILI9341_Init();

	   // Simple Text writing (Text, Font, X, Y, Color, BackColor)
	   // Available Fonts are FONT1, FONT2, FONT3 and FONT4
	   ILI9341_FillScreen(BLACK);
	   ILI9341_SetRotation(SCREEN_VERTICAL_1);
	   ILI9341_DrawText("MIDI BUSSBARS ORGAN", FONT4, 10, 20, BLACK, GREEN);
	   ILI9341_DrawText("TFT DEBUG", FONT4, 10, 60, BLACK, GREEN);

	   // LCD TEST SCREENS

	   ILI9341_DrawText("HELLO WORLD", FONT4, 90, 110, BLACK, WHITE);
	   HAL_Delay(1000);

	   //debug_blink(5);

	   //Writing numbers
	   ILI9341_FillScreen(WHITE);
	   static char BufferText[30];
	   for(uint8_t i = 0; i <= 5; i++)
	   {
	     sprintf(BufferText, "COUNT : %d", i);
	     ILI9341_DrawText(BufferText, FONT3, 10, 10, BLACK, WHITE);
	     ILI9341_DrawText(BufferText, FONT3, 10, 30, BLUE, WHITE);
	     ILI9341_DrawText(BufferText, FONT3, 10, 50, RED, WHITE);
	     ILI9341_DrawText(BufferText, FONT3, 10, 70, GREEN, WHITE);
	     ILI9341_DrawText(BufferText, FONT3, 10, 90, YELLOW, WHITE);
	     ILI9341_DrawText(BufferText, FONT3, 10, 110, PURPLE, WHITE);
	     ILI9341_DrawText(BufferText, FONT3, 10, 130, ORANGE, WHITE);
	     ILI9341_DrawText(BufferText, FONT3, 10, 150, MAROON, WHITE);
	     ILI9341_DrawText(BufferText, FONT3, 10, 170, WHITE, BLACK);
	     ILI9341_DrawText(BufferText, FONT3, 10, 190, BLUE, BLACK);
	   }

	   // Horizontal Line (X, Y, Length, Color)
	   ILI9341_FillScreen(WHITE);
	   ILI9341_DrawHLine(50, 120, 200, NAVY);
	   HAL_Delay(1000);

	   // Vertical Line (X, Y, Length, Color)
	   ILI9341_FillScreen(WHITE);
	   ILI9341_DrawVLine(160, 40, 150, DARKGREEN);
	   HAL_Delay(1000);

	   // Hollow Circle (Centre X, Centre Y, Radius, Color)
	   ILI9341_FillScreen(WHITE);
	   ILI9341_DrawHollowCircle(160, 120, 80, PINK);
	   HAL_Delay(1000);

	   // Filled Circle (Centre X, Centre Y, Radius, Color)
	   ILI9341_FillScreen(WHITE);
	   ILI9341_DrawFilledCircle(160, 120, 50, CYAN);
	   HAL_Delay(1000);

	   // Filled Rectangle (Start X, Start Y, Length X, Length Y)
	   ILI9341_FillScreen(WHITE);
	   ILI9341_DrawRectangle(50, 50, 220, 140, GREENYELLOW);
	   HAL_Delay(1000);

	   // Hollow Rectangle (Start X, Start Y, End X, End Y)
	   ILI9341_FillScreen(WHITE);
	   ILI9341_DrawHollowRectangleCoord(50, 50, 270, 190, DARKCYAN);
	   HAL_Delay(1000);

	   // Simple Pixel Only (X, Y, Color)
	   ILI9341_FillScreen(WHITE);
	   ILI9341_DrawPixel(100, 100, BLACK);
	   HAL_Delay(1000);
}
//void LCD_demo_OLD(void){
//	//----------------------------------------------------------COUNTING MULTIPLE SEGMENTS
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Counting multiple segments at once", 10, 10, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//	char Temp_Buffer_text[14];
//	for(uint16_t i = 0; i <= 10; i++)
//	{
//	sprintf(Temp_Buffer_text, "Counting: %d", i);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 2, WHITE);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 30, BLUE, 2, WHITE);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 50, RED, 2, WHITE);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 70, GREEN, 2, WHITE);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 90, BLACK, 2, WHITE);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 110, BLUE, 2, WHITE);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 130, RED, 2, WHITE);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, GREEN, 2, WHITE);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 170, WHITE, 2, BLACK);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 190, BLUE, 2, BLACK);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 210, RED, 2, BLACK);
//	}
//
//	HAL_Delay(1000);
//
////----------------------------------------------------------COUNTING SINGLE SEGMENT
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Counting single segment", 10, 10, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//	for(uint16_t i = 0; i <= 100; i++)
//	{
//	sprintf(Temp_Buffer_text, "Counting: %d", i);
//	ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 3, WHITE);
//	}
//
//	HAL_Delay(1000);
//
////----------------------------------------------------------ALIGNMENT TEST
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Rectangle alignment check", 10, 10, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//	ILI9341_Draw_Hollow_Rectangle_Coord(50, 50, 100, 100, BLACK);
//	ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 50, 50, BLACK);
//	ILI9341_Draw_Hollow_Rectangle_Coord(10, 10, 19, 19, BLACK);
//	HAL_Delay(1000);
//
//
//
//
//
////----------------------------------------------------------INDIVIDUAL PIXEL EXAMPLE
//
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Slow draw by selecting", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("and adressing pixels", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//	ILI9341_Fill_Screen(WHITE);
//
//
//	int x = 0;
//	int y = 0;
//	while (y < 240)
//	{
//	while ((x < 320) && (y < 240))
//	{
//
//		if(x % 2)
//		{
//			ILI9341_Draw_Pixel(x, y, BLACK);
//		}
//
//		x++;
//	}
//
//		y++;
//		x = 0;
//	}
//
//	x = 0;
//	y = 0;
//
//
//	while (y < 240)
//	{
//	while ((x < 320) && (y < 240))
//	{
//
//		if(y % 2)
//		{
//			ILI9341_Draw_Pixel(x, y, BLACK);
//		}
//
//		x++;
//	}
//
//		y++;
//		x = 0;
//	}
//	HAL_Delay(2000);
//
//
////----------------------------------------------------------565 COLOUR EXAMPLE, Grayscale
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//	ILI9341_Draw_Text("Colour gradient", 10, 10, BLACK, 1, WHITE);
//	ILI9341_Draw_Text("Grayscale", 10, 20, BLACK, 1, WHITE);
//	HAL_Delay(2000);
//
//
//	for(uint16_t i = 0; i <= (320); i++)
//	{
//		uint16_t Red = 0;
//		uint16_t Green = 0;
//		uint16_t Blue = 0;
//
//		Red = i/(10);
//		Red <<= 11;
//		Green = i/(5);
//		Green <<= 5;
//		Blue = i/(10);
//
//
//
//		uint16_t RGB_color = Red + Green + Blue;
//		ILI9341_Draw_Rectangle(i, x, 1, 240, RGB_color);
//
//	}
//	HAL_Delay(2000);
//}

void pulse_debug_out(int n_pulses){
	for(int i=0; i<n_pulses; i++){
		LL_GPIO_SetOutputPin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
		LL_GPIO_ResetOutputPin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
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
