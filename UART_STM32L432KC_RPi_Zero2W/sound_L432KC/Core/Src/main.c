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
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <haifa_comms.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMISSION_ON		// Enables transmission functionality

#define RECEPTION_ON		// Enables reception functionality

#define ARTIFICAL_DEPTH 1	// Enables generation of artificial depth values (for testing purposes)

#define TIME_PERIOD (10 * SystemCoreClock/1000)	// For 10 seconds

#define MAX_ID_LEN (2)
#define MAX_PAYLOAD_LEN (15)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

__IO ITStatus UartReady = RESET; // Flag indicating if UART transmission is complete
HAL_StatusTypeDef TransmitState; // Status of the last UART transmission

/* Reception variables */
static uint8_t msg_list[HAIFA_MESSAGE_LIST_SIZE][HAIFA_MESSAGE_SIZE];
uint8_t buffer[HAIFA_DMA_BUFFER_SIZE];
size_t dma_head = 0, dma_tail = 0;
size_t cur_msg_sz = 0;
size_t cur_msg = 0;
uint8_t last_read_msg = HAIFA_MESSAGE_LIST_SIZE - 1;
uint8_t found = 0;

/* Parsing variables */
uint8_t id[MAX_ID_LEN + 1];
uint8_t payload[MAX_PAYLOAD_LEN + 1];
double depth2reach; // Depth parsed from the received data

uint32_t time_count = 0;

/* Buffer used for transmission */
uint8_t aTxBuffer[TXBUFFERSIZE];

/* Depth transmission variables */
float depth = -1; // Current depth value
float depthTarget = -1; // Target depth value

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void Generate_Depth_Variations(float* currDepth, float* depthTarge, float maxDepth);
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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while(1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

#ifdef RECEPTION_ON

		// always be receiving from modem
		HAL_UART_Receive_DMA(&HAIFA_UART, buffer, sizeof(buffer));


		// read and bin messages split into
		do
		{
			__disable_irq();
			dma_tail = HAIFA_DMA_BUFFER_SIZE - HAIFA_UART.hdmarx->Instance->CNDTR;
			__enable_irq();

			if(dma_tail!=dma_head)
			{
				if(dma_head < dma_tail)
				{
					for(register size_t i=dma_head; i<dma_tail; i++)
					{
						msg_list[cur_msg][cur_msg_sz++]= buffer[i];
						found = (found == 0 && msg_list[cur_msg][cur_msg_sz-1] == '\r') ? 1
								: (found == 1 && msg_list[cur_msg][cur_msg_sz-1] == '\n') ? 2
										: 0;

						if(found==2)
						{
							msg_list[cur_msg][cur_msg_sz-2] = 0;
							msg_list[cur_msg][cur_msg_sz-1] = 0;
							cur_msg = cur_msg == HAIFA_MESSAGE_LIST_SIZE-1 ? 0 : cur_msg + 1;
							memset(msg_list[cur_msg],0, HAIFA_MESSAGE_SIZE);
							cur_msg_sz=0;
						}
					}
				}
				else
				{
					for(register size_t i=dma_head; i<HAIFA_DMA_BUFFER_SIZE; i++)
					{
						msg_list[cur_msg][cur_msg_sz++]= buffer[i];
						found = (found == 0 && msg_list[cur_msg][cur_msg_sz-1] == '\r') ? 1
								: (found == 1 && msg_list[cur_msg][cur_msg_sz-1] == '\n') ? 2
										: 0;

						if(found==2)
						{
							msg_list[cur_msg][cur_msg_sz-2] = 0;
							msg_list[cur_msg][cur_msg_sz-1] = 0;
							cur_msg = cur_msg == HAIFA_MESSAGE_LIST_SIZE-1 ? 0 : cur_msg + 1;
							memset(msg_list[cur_msg], 0, HAIFA_MESSAGE_SIZE);
							cur_msg_sz=0;
						}
					}
					for(register size_t i=0; i<dma_tail; i++)
					{
						msg_list[cur_msg][cur_msg_sz++]= buffer[i];
						found = (found == 0 && msg_list[cur_msg][cur_msg_sz-1] == '\r') ? 1
								: (found == 1 && msg_list[cur_msg][cur_msg_sz-1] == '\n') ? 2
										: 0;

						if(found==2)
						{
							msg_list[cur_msg][cur_msg_sz-2] = 0;
							msg_list[cur_msg][cur_msg_sz-1] = 0;

							cur_msg = cur_msg == HAIFA_MESSAGE_LIST_SIZE-1 ? 0 : cur_msg + 1;
							memset(msg_list[cur_msg],0,HAIFA_MESSAGE_SIZE);
							cur_msg_sz=0;
						}
					}
				}
				dma_head=dma_tail;
			}
		} while(dma_head!=(HAIFA_DMA_BUFFER_SIZE- HAIFA_UART.hdmarx->Instance->CNDTR));

		// try parsing all stored messages
		while ((last_read_msg + 1u) % HAIFA_MESSAGE_LIST_SIZE != cur_msg)
		{
			last_read_msg = (last_read_msg + 1) % HAIFA_MESSAGE_LIST_SIZE;

			// Parse incoming message
			if(msg_list[last_read_msg][0] == '$')
			{
				// Extract ID
				int i_id = 1;
				while(msg_list[last_read_msg][i_id] != ';' && i_id - 1 < MAX_ID_LEN)
				{

					id[i_id-1] = msg_list[last_read_msg][i_id];
					i_id++;
				}
				id[i_id-1] = '\0';

				// Pass the semicolon
				i_id++;

				// Extract payload
				int i_pl = 0;
				while(msg_list[last_read_msg][i_id + i_pl] != '\r' && i_pl < MAX_PAYLOAD_LEN)
				{
					payload[i_pl] = msg_list[last_read_msg][i_id + i_pl];
					i_pl++;
				}
				payload[i_pl] = '\0';

				// Check the ID
				if(strcmp((char*)id, "DS") == 0)
				{
					//Convert payload into float
					depth2reach = strtod((char*)payload, NULL);
					printf("Target Depth = %f\r\n", depth2reach);
				}
			}
		}

#endif

#ifdef TRANSMISSION_ON
		/* Enable transmission functionality */

		// Check if ARTIFICAL_DEPTH macro is defined (true for 1)
		if(ARTIFICAL_DEPTH)
		{
			// Generate variations for depth and target depth
			Generate_Depth_Variations(&depth, &depthTarget, 200.0);
		}

		time_count++;
		if(time_count > TIME_PERIOD)
		{
			time_count = 0;
			// Prepare the transmission buffer
			snprintf((char*)aTxBuffer, sizeof(aTxBuffer),"$DC;%06.2f\r\n",depth);

			// Format the transmission message:
			// "$" (message start marker)
			// "Depth=" (literal string)
			// "%06.2f" (format specifier: 6 digits total, 2 decimal place for depth value)
			// "\r\n" (message terminator)

			// Print the formatted message for debugging (optional)
			printf("TX : %s\r\n", aTxBuffer);

			/* Start the transmission process */
			// Check if the UART peripheral is ready for transmission
			if(huart1.gState == HAL_UART_STATE_READY)
			{
				// Initiate DMA transmission of data in aTxBuffer via USART1
				TransmitState = HAL_UART_Transmit_DMA(&huart1, aTxBuffer, TXBUFFERSIZE);

				// Handle transmission errors
				if(TransmitState != HAL_OK)
				{
					printf("Error in transmission\r\n");
					switch(TransmitState)
					{
					case HAL_ERROR :
						printf("HAL_ERROR\r\n");
						break;
					case HAL_BUSY :
						printf("HAL_BUSY\r\n");
						break;
					case HAL_TIMEOUT :
						printf("HAL_TIMEOUT\r\n");
						break;
					default :
						break;
					}
					Error_Handler();
				}

				/* Wait for the end of the transfer */
				while (UartReady != SET)
				{
				}

				/* Reset transmission flag */
				UartReady = RESET;
			}
		}



#endif /* TRANSMISSION_ON */

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

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
			|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Generates variations in the current depth value.
 * @param  currDepth: Pointer to a float variable storing the current depth.
 * @param  depthTarget: Pointer to a float variable storing the target depth.
 * @param  maxDepth: The maximum allowed depth value.
 * @note   This function initializes the current depth if not set, generates a new target depth
 *         if necessary, and adjusts the current depth randomly towards the target depth
 *         within a specific range.
 * @retval None
 */
void Generate_Depth_Variations(float* currDepth, float* depthTarget, float maxDepth)
{
	// Check if currDepth is not initialized
	if (*currDepth == -1)
	{
		// Seed the random number generator
		srand(RTC->SSR);
		// Generate a random initial depth between 0 and maxDepth
		*currDepth = ((float)rand() / (float)RAND_MAX) * maxDepth + 0.0;
	}

	// Check if the target depth has been reached
	// or if depthTarget is not initialized
	if (abs(*currDepth - *depthTarget) <= 0.6 || *depthTarget == -1.0)
	{
		// Seed the random number generator
		srand(RTC->SSR);
		// Generate a new random target depth between 0.6 and maxDepth-0.6
		*depthTarget = ((float)rand() / (float)RAND_MAX) * (maxDepth-0.6) + 0.6;
	}

	// Generate a random adjustment between 0.1 and 0.5 with two decimal places
	srand(RTC->SSR);
	float randomAdjustment = (rand()%50 + 1)/100.0;

	// Adjust currDepth based on comparison with depthTarget
	if (*currDepth > *depthTarget)
	{
		*currDepth -= randomAdjustment;
		// Ensure currDepth doesn't go below 0
		if(*currDepth < 0) *currDepth = 0;
	}
	else if (*currDepth < *depthTarget)
	{
		*currDepth += randomAdjustment;
	}
}


/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of DMA Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Set transmission flag: transfer complete */
	UartReady = SET;
}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Set transmission flag: transfer complete */
	UartReady = SET;
}

/**
 * @brief  UART error callbacks
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	printf("ERROR : Callback\r\n");
	// Get the specific error code using HAL_UART_GetError()
	uint32_t error_code = HAL_UART_GetError(UartHandle);

	// Print informative message based on the error code
	switch (error_code) {
	case HAL_UART_ERROR_FE:
		printf("Error: Frame Error\r\n");
		break;
	case HAL_UART_ERROR_NE:
		printf("Error: Noise Error\r\n");

		break;
	case HAL_UART_ERROR_PE:
		printf("Error: Parity Error\r\n");
		break;
		//	case HAL_UART_ERROR_BE:
		//		USART1_printf("Error: Break Error\r\n");
		//		break;
	case HAL_UART_ERROR_ORE:
		printf("Error: Overrun Error\r\n");
		HAL_UART_AbortReceive(&huart1);
		HAL_UART_AbortTransmit(&huart1);

		break;
	case HAL_UART_ERROR_DMA:
		printf("Error: DMA Transfer Error\r\n");
		break;
		//	case HAL_UART_ERROR_TIMEOUT:
		//		USART1_printf("Error: Timeout Error\r\n");
		//		break;
	default:
		printf("Error: Unknown Error (%lu)\r\n", error_code);
		// Additionally, check for specific error bits in the register (if applicable)
		if (READ_BIT(UartHandle->Instance->ISR, USART_ISR_RXNE) == RESET) {
			printf("Possible cause: Receive Noise Error\r\n");
		}
		if (READ_BIT(UartHandle->Instance->ISR, USART_ISR_TC) == RESET) {
			printf("Possible cause: Transmission Timeout\r\n");
		}
	}
	//Error_Handler();
}

/**
 * @brief Transmits a single character through UART2.
 * @param ch: The character to be transmitted.
 * @return The character that was transmitted (always the same as the input).
 * @note This function likely uses the STM32 HAL to transmit a single byte
 *        through UART2. It's a low-level function for character output.
 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/**
 * @brief Receives a single character through UART2.
 * @return The received character as an integer.
 * @note This function likely uses the STM32 HAL to receive a single byte
 *        through UART2. It's a low-level function for character input.
 *        Waits until a character is received before returning.
 */
int __io_getchar(void)
{
	int ch = 0;
	while(!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE));
	HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
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
