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

COM_InitTypeDef BspCOMInit;

PSSI_HandleTypeDef hpssi;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
#define RIVI 128
#define SARAKE 128
static uint16_t kuva_buffer[RIVI][SARAKE]; //rxbufferi saapuvalle datalle
static uint32_t kuva_summa[10][RIVI]; //summamuuttuja rivinsummaukselle
volatile uint8_t Transfer_Ready = 0; //RDY lippu
volatile uint8_t rivisummat = 0; //Summatut rivit
volatile uint32_t start_time = 0; // Kokonais aloitusaika millisekunneissa
volatile uint32_t elapsed_time = 0; // Kulunut aika millisekunneissa
volatile uint16_t start_time_summaus; // Aloitusaika summaukselle
volatile uint16_t end_time_summaus;   // Lopetusaika summaukselle
volatile uint16_t end_time_siirto; // Lopetusaika siirrolle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPDMA1_Init(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM16_Init(void);
static void MX_PSSI_Init(void);
/* USER CODE BEGIN PFP */
void HAL_PSSI_RxCpltCallback(PSSI_HandleTypeDef *hpssi);
static void RIVIEN_SUMMAUS(uint16_t kuva_buffer[RIVI][SARAKE], uint32_t sum[10][RIVI]);
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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPDMA1_Init();
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_TIM16_Init();
  MX_PSSI_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_PSSI_Receive_DMA(&hpssi, (uint32_t*)kuva_buffer, 32768) != HAL_OK) //Käynnistetään PSSI vastaanotto sensorilta
    	           {
    	               Error_Handler(); // Virheen käsittely
    	           }
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    HAL_TIM_Base_Start(&htim16); //Ajastin päälle summauksen ja siirron mittaamiseen
    start_time = __HAL_TIM_GET_COUNTER(&htim16); //Ajastin päälle koko prosessin mittaamiseen
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET); //Asetetaan RDY-pinni ylös
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Aktivoituu kun kuva on saapunut
	  	  if (Transfer_Ready == 1) {
	  	          Transfer_Ready = 0;  // Nollataan odotustila
	  	          if (rivisummat < 10){ //Vastaanotetaan 10 kuvaa

	  	          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); //asetetaan RDY nollaan
	  	          HAL_Delay(1);
	  	          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);

	  	          }
	  	          else {
	  	        	  elapsed_time = (__HAL_TIM_GET_COUNTER(&htim16) - start_time); //Kun 10 kuvaa saapunut mitataan kulunut aika
	  	        	  float fps = (1000000.0 / elapsed_time) * 10; //lasketaan kuvien määrä sekunnissa
	  	        	  uint32_t tarkistus = 0;
	  	        	  for (int i = 0; i < 128; i++) {
	  	        	      tarkistus += kuva_summa[0][i]; // varmistus optimoinnille
	  	        	  }
	  	        	  printf(" 10 kuvaa vastaanotettu ajassa: %lu us\n\r FPS: %.2f\n\r Summauksen kesto: %u us\n\r PSSI siirron kesto: %u us\n\r tarkistus %lu\r\n\n", elapsed_time, fps, end_time_summaus, end_time_siirto, tarkistus);
	  	        	  rivisummat = 0;
	  	        	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); //asetetaan RDY nollaan
	  	        	  HAL_Delay(1);
	  	        	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);

	  	          }
	  	      }

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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief PSSI Initialization Function
  * @param None
  * @retval None
  */
static void MX_PSSI_Init(void)
{

  /* USER CODE BEGIN PSSI_Init 0 */

  /* USER CODE END PSSI_Init 0 */

  /* USER CODE BEGIN PSSI_Init 1 */

  /* USER CODE END PSSI_Init 1 */
  hpssi.Instance = PSSI;
  hpssi.Init.DataWidth = HAL_PSSI_16BITS;
  hpssi.Init.BusWidth = HAL_PSSI_16LINES;
  hpssi.Init.ControlSignal = HAL_PSSI_DE_RDY_DISABLE;
  hpssi.Init.ClockPolarity = HAL_PSSI_FALLING_EDGE;
  hpssi.Init.DataEnablePolarity = HAL_PSSI_DEPOL_ACTIVE_LOW;
  hpssi.Init.ReadyPolarity = HAL_PSSI_RDYPOL_ACTIVE_LOW;
  if (HAL_PSSI_Init(&hpssi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN PSSI_Init 2 */

  /* USER CODE END PSSI_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 159;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void RIVIEN_SUMMAUS(uint16_t kuva_buffer[RIVI][SARAKE], uint32_t sum[10][RIVI]) {
    for (int i = 0; i < RIVI; i++) {
        uint32_t row_sum = 0;

        row_sum += kuva_buffer[i][0]   + kuva_buffer[i][1]   + kuva_buffer[i][2]   + kuva_buffer[i][3]   +
                   kuva_buffer[i][4]   + kuva_buffer[i][5]   + kuva_buffer[i][6]   + kuva_buffer[i][7]   +
                   kuva_buffer[i][8]   + kuva_buffer[i][9]   + kuva_buffer[i][10]  + kuva_buffer[i][11]  +
                   kuva_buffer[i][12]  + kuva_buffer[i][13]  + kuva_buffer[i][14]  + kuva_buffer[i][15]  +
                   kuva_buffer[i][16]  + kuva_buffer[i][17]  + kuva_buffer[i][18]  + kuva_buffer[i][19]  +
                   kuva_buffer[i][20]  + kuva_buffer[i][21]  + kuva_buffer[i][22]  + kuva_buffer[i][23]  +
                   kuva_buffer[i][24]  + kuva_buffer[i][25]  + kuva_buffer[i][26]  + kuva_buffer[i][27]  +
                   kuva_buffer[i][28]  + kuva_buffer[i][29]  + kuva_buffer[i][30]  + kuva_buffer[i][31]  +
                   kuva_buffer[i][32]  + kuva_buffer[i][33]  + kuva_buffer[i][34]  + kuva_buffer[i][35]  +
                   kuva_buffer[i][36]  + kuva_buffer[i][37]  + kuva_buffer[i][38]  + kuva_buffer[i][39]  +
                   kuva_buffer[i][40]  + kuva_buffer[i][41]  + kuva_buffer[i][42]  + kuva_buffer[i][43]  +
                   kuva_buffer[i][44]  + kuva_buffer[i][45]  + kuva_buffer[i][46]  + kuva_buffer[i][47]  +
                   kuva_buffer[i][48]  + kuva_buffer[i][49]  + kuva_buffer[i][50]  + kuva_buffer[i][51]  +
                   kuva_buffer[i][52]  + kuva_buffer[i][53]  + kuva_buffer[i][54]  + kuva_buffer[i][55]  +
                   kuva_buffer[i][56]  + kuva_buffer[i][57]  + kuva_buffer[i][58]  + kuva_buffer[i][59]  +
                   kuva_buffer[i][60]  + kuva_buffer[i][61]  + kuva_buffer[i][62]  + kuva_buffer[i][63]  +
                   kuva_buffer[i][64]  + kuva_buffer[i][65]  + kuva_buffer[i][66]  + kuva_buffer[i][67]  +
                   kuva_buffer[i][68]  + kuva_buffer[i][69]  + kuva_buffer[i][70]  + kuva_buffer[i][71]  +
                   kuva_buffer[i][72]  + kuva_buffer[i][73]  + kuva_buffer[i][74]  + kuva_buffer[i][75]  +
                   kuva_buffer[i][76]  + kuva_buffer[i][77]  + kuva_buffer[i][78]  + kuva_buffer[i][79]  +
                   kuva_buffer[i][80]  + kuva_buffer[i][81]  + kuva_buffer[i][82]  + kuva_buffer[i][83]  +
                   kuva_buffer[i][84]  + kuva_buffer[i][85]  + kuva_buffer[i][86]  + kuva_buffer[i][87]  +
                   kuva_buffer[i][88]  + kuva_buffer[i][89]  + kuva_buffer[i][90]  + kuva_buffer[i][91]  +
                   kuva_buffer[i][92]  + kuva_buffer[i][93]  + kuva_buffer[i][94]  + kuva_buffer[i][95]  +
                   kuva_buffer[i][96]  + kuva_buffer[i][97]  + kuva_buffer[i][98]  + kuva_buffer[i][99]  +
                   kuva_buffer[i][100] + kuva_buffer[i][101] + kuva_buffer[i][102] + kuva_buffer[i][103] +
                   kuva_buffer[i][104] + kuva_buffer[i][105] + kuva_buffer[i][106] + kuva_buffer[i][107] +
                   kuva_buffer[i][108] + kuva_buffer[i][109] + kuva_buffer[i][110] + kuva_buffer[i][111] +
                   kuva_buffer[i][112] + kuva_buffer[i][113] + kuva_buffer[i][114] + kuva_buffer[i][115] +
                   kuva_buffer[i][116] + kuva_buffer[i][117] + kuva_buffer[i][118] + kuva_buffer[i][119] +
                   kuva_buffer[i][120] + kuva_buffer[i][121] + kuva_buffer[i][122] + kuva_buffer[i][123] +
                   kuva_buffer[i][124] + kuva_buffer[i][125] + kuva_buffer[i][126] + kuva_buffer[i][127];

        sum[rivisummat][i] = row_sum;
    }
    rivisummat++;
}

//keskeytyksen käsittelijä aktivoituu kun rxbufferi on täyttynyt
void HAL_PSSI_RxCpltCallback(PSSI_HandleTypeDef *hpssi)
{

	if (rivisummat < 1) {
	end_time_siirto = __HAL_TIM_GET_COUNTER(&htim16) - start_time;
	}

	start_time_summaus = __HAL_TIM_GET_COUNTER(&htim16);

	RIVIEN_SUMMAUS(kuva_buffer, kuva_summa); //summataan rivit

	end_time_summaus = __HAL_TIM_GET_COUNTER(&htim16) - start_time_summaus;

	Transfer_Ready++; //lippu ylös


}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
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
