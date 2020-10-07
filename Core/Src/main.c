/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware for CDR2020's barillet pcb, handle CANbus and 6 pumps and 6 e-valves.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
CAN_HandleTypeDef hcan;

CAN_RxHeaderTypeDef Rxheader;
CAN_TxHeaderTypeDef Txheader;
uint8_t Rxmsg[8];
uint32_t RxMailbox;
uint32_t Txmailbox;
/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

void TogglePump(uint8_t pin,uint16_t state);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TogglePump(uint8_t pin, uint16_t state){

	switch (pin)
		{
		case 1:
			pin=P1_Pin;
			HAL_GPIO_WritePin(P1_GPIO_Port, pin, state);
			break;
		case 2:
			pin=P2_Pin;
			HAL_GPIO_WritePin(P2_GPIO_Port, pin, state);
			break;
		case 3:
			pin=P3_Pin;
			HAL_GPIO_WritePin(P3_GPIO_Port, pin, state);
			break;
		case 4:
			pin=P4_Pin;
			HAL_GPIO_WritePin(P4_GPIO_Port, pin, state);
			break;
		case 5:
			pin=P5_Pin;
			HAL_GPIO_WritePin(P5_GPIO_Port, pin, state);
			break;
		case 6:
			pin=P6_Pin;
			HAL_GPIO_WritePin(P6_GPIO_Port, pin, state);
			break;

		}
}

void ToggleValve(uint16_t pin, uint16_t state){

	switch (pin)
		{
		case 1:
			pin=V1_Pin;
			HAL_GPIO_WritePin(V1_GPIO_Port, pin, state);
			break;
		case 2:
			pin=V2_Pin;
			HAL_GPIO_WritePin(V2_GPIO_Port, pin, state);
			break;
		case 3:
			pin=V3_Pin;
			HAL_GPIO_WritePin(V3_GPIO_Port, pin, state);
			break;
		case 4:
			pin=V4_Pin;
			HAL_GPIO_WritePin(V4_GPIO_Port, pin, state);
			break;
		case 5:
			pin=V5_Pin;
			HAL_GPIO_WritePin(V5_GPIO_Port, pin, state);
			break;
		case 6:
			pin=V6_Pin;
			HAL_GPIO_WritePin(V6_GPIO_Port, pin, state);
			break;

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_MspInit(&hcan);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)>0)
	  {
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &Rxheader, Rxmsg);
	  uint8_t pin = Rxmsg[1];
	  uint8_t state = Rxmsg[2];

	  if(Rxmsg[0]=='p')
		 TogglePump(pin, state);
	  if(Rxmsg[0]=='v')
		  ToggleValve(pin, state);
	  }

	  if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO1)>0)
	  {
		 HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &Rxheader, Rxmsg);

		 uint8_t pin = Rxmsg[1];
		 uint8_t state = Rxmsg[2];

		 if(Rxmsg[0]=='p')
			 TogglePump(pin, state);
		 if(Rxmsg[0]=='v')
			 ToggleValve(pin, state);

	  }



	  //HAL_Delay(500);

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{
	CAN_FilterTypeDef filter;
  /* USER CODE BEGIN CAN_Init 0 */
	__HAL_RCC_CAN1_CLK_ENABLE();


  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
	/*GPIO_InitTypeDef GPIO_InitCAN;

	__HAL_RCC_CAN1_CLK_ENABLE();

	GPIO_InitCAN.Pin= GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitCAN.Mode= GPIO_MODE_AF_PP;
	GPIO_InitCAN.Pull= GPIO_NOPULL;
	GPIO_InitCAN.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitCAN.Alternate=GPIO_AF4_CAN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitCAN);*/
  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  filter.FilterBank = 0;
  filter.FilterIdHigh=0x130;
  filter.FilterIdLow=0;
  filter.FilterMaskIdHigh=0;
  filter.FilterMaskIdLow=0;
  filter.FilterMode=CAN_FILTERMODE_IDMASK;
  filter.FilterScale=CAN_FILTERSCALE_32BIT;
  filter.FilterBank=CAN_FILTER_FIFO0;
  filter.FilterActivation=CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank=14;

  Txheader.StdId = 0x320;      // Détermine l'adresse du périphérique au quel la trame est destiné.
  	                                 // Si plusieurs périphériques sur le bus comprennent cette adresse dans leur filtre, ils recevront tous la trame.
  Txheader.ExtId = 0x0;       // Adresse étendue, non utilisée dans note cas
  Txheader.RTR = CAN_RTR_DATA; // Précise que la trame contient des données
  Txheader.IDE = CAN_ID_STD;   // Précise que la trame est de type Standard
  Txheader.DLC = 4;            // Précise le nombre d'octets de données que la trame transporte ( De 0 à 8 )
  Txheader.TransmitGlobalTime = DISABLE;

  HAL_CAN_ConfigFilter(&hcan, &filter); //configure CAN filter
  HAL_CAN_Start(&hcan);
    //HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, P1_Pin|V1_Pin|P2_Pin|V2_Pin
                          |P3_Pin|V6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, V3_Pin|P6_Pin|V5_Pin|P5_Pin
                          |V4_Pin|P4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : P1_Pin V1_Pin P2_Pin V2_Pin
                           P3_Pin V6_Pin */
  GPIO_InitStruct.Pin = P1_Pin|V1_Pin|P2_Pin|V2_Pin
                          |P3_Pin|V6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : V3_Pin P6_Pin V5_Pin P5_Pin
                           V4_Pin P4_Pin */
  GPIO_InitStruct.Pin = V3_Pin|P6_Pin|V5_Pin|P5_Pin
                          |V4_Pin|P4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
 HAL_GPIO_TogglePin(GPIOA, P1_Pin);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
