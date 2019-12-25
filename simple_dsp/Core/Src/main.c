/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint16_t sine_table[]= {
    2047, 2059, 2072, 2084, 2097, 2109, 2122, 2134, 2146, 2159,
    2171, 2183, 2195, 2207, 2219, 2231, 2242, 2254, 2265, 2277,
    2288, 2299, 2310, 2320, 2331, 2341, 2351, 2361, 2371, 2381,
    2390, 2400, 2409, 2417, 2426, 2434, 2442, 2450, 2458, 2465,
    2472, 2479, 2486, 2492, 2498, 2504, 2509, 2515, 2520, 2524,
    2529, 2533, 2536, 2540, 2543, 2546, 2549, 2551, 2553, 2555,
    2556, 2557, 2558, 2558, 2559, 2558, 2558, 2557, 2556, 2555,
    2553, 2551, 2549, 2546, 2543, 2540, 2536, 2533, 2529, 2524,
    2520, 2515, 2509, 2504, 2498, 2492, 2486, 2479, 2472, 2465,
    2458, 2450, 2442, 2434, 2426, 2417, 2409, 2400, 2390, 2381,
    2371, 2361, 2351, 2341, 2331, 2320, 2310, 2299, 2288, 2277,
    2265, 2254, 2242, 2231, 2219, 2207, 2195, 2183, 2171, 2159,
    2146, 2134, 2122, 2109, 2097, 2084, 2072, 2059, 2047, 2034,
    2021, 2009, 1996, 1984, 1971, 1959, 1947, 1934, 1922, 1910,
    1898, 1886, 1874, 1862, 1851, 1839, 1828, 1816, 1805, 1794,
    1783, 1773, 1762, 1752, 1742, 1732, 1722, 1712, 1703, 1693,
    1684, 1676, 1667, 1659, 1651, 1643, 1635, 1628, 1621, 1614,
    1607, 1601, 1595, 1589, 1584, 1578, 1573, 1569, 1564, 1560,
    1557, 1553, 1550, 1547, 1544, 1542, 1540, 1538, 1537, 1536,
    1535, 1535, 1535, 1535, 1535, 1536, 1537, 1538, 1540, 1542,
    1544, 1547, 1550, 1553, 1557, 1560, 1564, 1569, 1573, 1578,
    1584, 1589, 1595, 1601, 1607, 1614, 1621, 1628, 1635, 1643,
    1651, 1659, 1667, 1676, 1684, 1693, 1703, 1712, 1722, 1732,
    1742, 1752, 1762, 1773, 1783, 1794, 1805, 1816, 1828, 1839,
    1851, 1862, 1874, 1886, 1898, 1910, 1922, 1934, 1947, 1959,
    1971, 1984, 1996, 2009, 2021, 2034
};
static uint16_t sine_table_size = sizeof(sine_table)/sizeof(sine_table[0]);

static void TEST_DAC_DMA_HAL(void){


  // Enable peripherals: GPIOA, DMA, DAC, TIM6, SYSCFG.
  RCC->AHBENR   |= ( RCC_AHBENR_GPIOAEN |
                     RCC_AHBENR_DMA1EN );
  RCC->APB1ENR  |= ( RCC_APB1ENR_DAC1EN |
                     RCC_APB1ENR_TIM6EN );
  RCC->APB2ENR  |= RCC_APB2ENR_SYSCFGEN;
  // Pin A4: analog mode. (PA4 = DAC1, Channel 1)
  GPIOA->MODER    &= ~( 0x3 << ( 4 * 2 ) );
  GPIOA->MODER    |=  ( 0x3 << ( 4 * 2 ) );
  // Set the 'TIM6/DAC1 remap' bit in SYSCFG_CFGR1,
  // so that DAC1_Ch1 maps to DMA1_Ch3 instead of DMA2_Ch3.
  // (Not all STM32F303 chips have a DMA2 peripheral)
  SYSCFG->CFGR1 |=  ( SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP );
  // DMA configuration (DMA1, channel 3).
  // CCR register:
  // - Memory-to-peripheral
  // - Circular mode enabled.
  // - Increment memory ptr, don't increment periph ptr.
  // - 16-bit data size for both source and destination.
  // - High priority (2/3).
  DMA1_Channel3->CCR &= ~( DMA_CCR_MEM2MEM |
                          DMA_CCR_PL |
                          DMA_CCR_MSIZE |
                          DMA_CCR_PSIZE |
                          DMA_CCR_PINC |
                          DMA_CCR_EN );
  DMA1_Channel3->CCR |=  ( ( 0x2 << DMA_CCR_PL_Pos ) |
                           ( 0x1 << DMA_CCR_MSIZE_Pos ) |
                           ( 0x1 << DMA_CCR_PSIZE_Pos ) |
                           DMA_CCR_MINC |
                           DMA_CCR_CIRC |
                           DMA_CCR_DIR );
  // Set DMA source and destination addresses.
  // Source: Address of the sine wave buffer in memory.
  DMA1_Channel3->CMAR  = ( uint32_t )&sine_table;
  // Dest.: DAC1 Ch1 '12-bit right-aligned data' register.
  DMA1_Channel3->CPAR  = ( uint32_t )&( DAC1->DHR12R1 );
  // Set DMA data transfer length (# of sine wave samples).
  DMA1_Channel3->CNDTR = ( uint16_t )sine_table_size;
  // Enable DMA1 Channel 1.
  // Note: the transfer won't actually start here, because
  // the DAC peripheral is not sending DMA requests yet.
  DMA1_Channel3->CCR |= ( DMA_CCR_EN );
  // TIM6 configuration. This timer will set the frequency
  // at which the DAC peripheral requests DMA transfers.
  // Set prescaler and autoreload for a 440Hz sine wave.
  TIM6->PSC  =  ( 0x0000 );
//  TIM6->ARR  =  ( SystemCoreClock / ( 440 * SINE_SAMPLES ) );
  // Enable trigger output on timer update events.
  TIM6->CR2 &= ~( TIM_CR2_MMS );
  TIM6->CR2 |=  ( 0x2 << TIM_CR2_MMS_Pos );
  // Start the timer.
  TIM6->CR1 |=  ( TIM_CR1_CEN );
  // DAC configuration.
  // Set trigger sources to TIM6 TRGO (TRiGger Output).
  DAC1->CR  &= ~( DAC_CR_TSEL1 );
  // Enable DAC DMA requests for channel 1.
  DAC1->CR  |=  ( DAC_CR_DMAEN1 );
  // Enable DAC channel 1.
  DAC1->CR  |=  ( DAC_CR_EN1 );
  // Delay briefly to allow sampling to stabilize.
//  delay_cycles( 1000 );
  HAL_Delay(1);
  // Enable DAC channel trigger.
  // The DMA channel and timer are both already on, so the
  // DMA transfer will start as soon as the DAC peripheral
  // starts making requests. The DAC peripheral will make a
  // request every time that TIM6 ticks over, but only after
  // this 'trigger enable' bit is set.
  DAC1->CR  |=  ( DAC_CR_TEN1 );

while(1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //uint8_t n = 0;
//  volatile uint32_t delay;
//  uint32_t delay_reload = 1000000;

//  uint16_t adcvalue;
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
  MX_DAC1_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

//  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//  HAL_ADC_Start(&hadc1);
  TEST_DAC_DMA_HAL();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    HAL_ADC_Start(&hadc1);
//    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
//    {
//        adcvalue = HAL_ADC_GetValue(&hadc1);
//        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, adcvalue);
//        HAL_ADC_Stop(&hadc1);
//    }

//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (sine_table[n]>>2) + 2047);
//    n++;

//    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (sine_table[n]>>2) + 2047);
//    n++;
//    if(n >= sine_table_size){
//      n = 0;
//    }
    //LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	//HAL_Delay(1);
//	delay = delay_reload;
//	while(delay--);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**DAC1 GPIO Configuration  
  PA4   ------> DAC1_OUT1 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* DAC1 DMA Init */
  
  /* DAC1_CH1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_WORD);

  LL_SYSCFG_SetRemapDMA_DAC(LL_SYSCFG_DAC1_CH1_RMP_DMA1_CH3);

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC channel OUT1 config 
  */
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_EXT_TIM6_TRGO;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC1, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1499;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
