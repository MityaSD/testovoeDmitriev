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
#include "type.h"
#include "stdio.h"
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t dataProcessingSlave(void);
uint16_t dataProcessingMaster(void);
void SendMessageSlave(unionReceipt_t *message);
void SendMessageMaster(unionUART_t *message);
void creatingData(unionUART_t* unionUART, uint8_t length);
uint16_t readingLength(uint8_t *length, CircBuf_t *circBuf);
void updateHead(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	int counter = 0;
	unionUART_t unionUART;
	unionUART_t unionUARTtx;
	unionReceipt_t unionReceipt;
	
	CircBuf_t txMaster;
	CircBuf_t rxMaster;
	
	CircBuf_t txSlave = {0};
	CircBuf_t rxSlave = {0};
	

	uint32_t sysTime = 0;
	uint32_t current_time = 0;
	
	uint8_t flagTransmit =0;
	

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t operationMode = MASTER;	
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_DMA(&huart1,rxSlave.buffer, BUFFER_SIZE); 
	HAL_UART_Receive_DMA(&huart2,rxMaster.buffer, BUFFER_SIZE);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_Delay(1000);
		
		
		if(flagTransmit)
		{
			creatingData(&unionUARTtx, SIZE_DATA);
			SendMessageMaster(&unionUARTtx);
			flagTransmit = 0;
			operationMode = SLAVE;
		}
		
				
		
		if(operationMode == SLAVE)
		{
			dataProcessingSlave();
			operationMode = MASTER;
		}
		
		if(operationMode == MASTER)
		{		
			current_time = TIM1->CNT;
			HAL_Delay(250);
			//if(sysTime -  current_time >= (TIM1->ARR / 2))
			//{
				dataProcessingMaster();	
				operationMode = SLAVE;
			//}
		}
	
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 249;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
		{
			flagTransmit = 1;
			sysTime = TIM1->CNT;	
		}
}

void SendMessageMaster(unionUART_t *message)//MASTER->SLAVE
{
	for(int i=0; i< message->package.length + 1;i++)//Sizeof(length) = 1
	{
		txMaster.buffer[txMaster.head] = message->masUART[i];
		txMaster.head= (txMaster.head + 1) % BUFFER_SIZE;
		
	}
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE))
	{
		HAL_UART_Transmit_DMA(&huart2, txMaster.buffer, sizeof(package_t));
		txMaster.head = 0;
	}
	updateHead();
	return;
}

void SendMessageSlave(unionReceipt_t *message)//SLAVE->MASTER
{
	for(int i=0; i< sizeof(receipt_t);i++)
	{
		txSlave.buffer[txSlave.head] = message->masReceipt[i];
		txSlave.head = (txSlave.head + 1) % BUFFER_SIZE;
	}

	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
	{
		HAL_UART_Transmit_DMA(&huart1, txSlave.buffer, txSlave.head);
		txSlave.head =0;
	}
	
	updateHead();
	
	return;
	}

uint16_t Crc16(uint8_t * pcBlock, unsigned short len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
        crc = (crc << 8) ^ Crc16Table[(crc >> 8) ^ *pcBlock++];

    return crc;
}

void updateHead(void)
{	
	rxSlave.head = BUFFER_SIZE - hdma_usart1_rx.Instance->NDTR;
	rxMaster.head = BUFFER_SIZE - hdma_usart2_rx.Instance->NDTR;
}

uint16_t dataProcessingMaster(void)
{
	static uint8_t state = STATE_WAITING_LENGTH;
	uint16_t headPackage = 0;
	uint16_t crc = 0;
	
	updateHead();
	
	if(rxMaster.head == rxMaster.tail) //The slave did not answer
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(250);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		return 0;
	}

	while(rxMaster.head != rxMaster.tail)
	{
			if(state == STATE_WAITING_LENGTH)
			{
				headPackage = readingLength(&unionReceipt.receipt.length, &rxMaster);
				state = STATE_RECEIVING_DATA;
			}
			else if(state == STATE_RECEIVING_DATA)
			{
				while(rxMaster.tail != headPackage)
				{
					unionReceipt.masReceipt[(rxMaster.tail % sizeof(receipt_t))] = rxMaster.buffer[rxMaster.tail];
					rxMaster.tail = (rxMaster.tail + 1) % BUFFER_SIZE;
				}
				state = STATE_PROCESSING_DATA;
			}
		}
		if(state == STATE_PROCESSING_DATA)
		{
			crc = Crc16(&unionReceipt.receipt.state,1);
			if((crc!= unionReceipt.receipt.crc) || (unionReceipt.receipt.state == ERROR_RECEIPT))
			{
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				HAL_Delay(250);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				HAL_Delay(250);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				HAL_Delay(250);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			}
			else if((crc == unionReceipt.receipt.crc) && (unionReceipt.receipt.state == OK))
			{
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				HAL_Delay(250);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			}	
				state = STATE_WAITING_LENGTH;
	}	
		return 1;
}

uint16_t dataProcessingSlave(void)
{
	static uint8_t state = STATE_WAITING_LENGTH;
	uint16_t headPackage = 0;
	uint16_t crc = 0;
		
		updateHead();
	
		while(rxSlave.tail != rxSlave.head)
		{
			if(state == STATE_WAITING_LENGTH)
			{
				/*unionUART.package.length = rxSlave.buffer[rxSlave.tail];
			
				rxSlave.tail = (rxSlave.tail + 1) % BUFFER_SIZE;	
				
				headPackage = (rxSlave.tail + unionUART.package.length)%BUFFER_SIZE;
				*/
				headPackage = readingLength(&unionUART.package.length, &rxSlave);
				state = STATE_RECEIVING_DATA;
			}
			else if(state == STATE_RECEIVING_DATA)
			{
				while(rxSlave.tail != headPackage)
				{
					unionUART.masUART[(rxSlave.tail % sizeof(package_t))] = rxSlave.buffer[rxSlave.tail];
					rxSlave.tail = (rxSlave.tail + 1) % BUFFER_SIZE;				
				}
				state = STATE_PROCESSING_DATA;
			}
		}
		if(state == STATE_PROCESSING_DATA)
		{
			crc = Crc16(unionUART.package.data,  SIZE_DATA );
			if(crc != unionUART.package.crc)
			{
				unionReceipt.receipt.state = ERROR_RECEIPT;
			}
			else
			{
				unionReceipt.receipt.state = OK;
			}
			unionReceipt.receipt.length = 3;
			unionReceipt.receipt.crc = Crc16(&unionReceipt.receipt.state, 1);
			SendMessageSlave(&unionReceipt);
			state =STATE_WAITING_LENGTH; 
		}
		return 1;
}

void creatingData (unionUART_t* unionUART, uint8_t length)
{
	static uint8_t count = 0;
	
	unionUART->package.length = length + SIZE_CRC;
	
	for(int i=0; i<length;i++)
	{
		unionUART->package.data[i] +=count;
	}
	
	count++;
	
	unionUART->package.crc =Crc16(unionUART->package.data, length);
	
	return;
}

uint16_t readingLength(uint8_t *length, CircBuf_t *circBuf)
{
	length = &circBuf->buffer[circBuf->tail];
	
	circBuf->tail = (circBuf->tail + 1) % BUFFER_SIZE;
		
	return (*length + circBuf->tail) % BUFFER_SIZE ;
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
