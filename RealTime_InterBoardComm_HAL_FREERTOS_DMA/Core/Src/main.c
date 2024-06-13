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
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stdio.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
  int16_t a_x;
  int16_t a_y;
  int16_t a_z;

  int16_t g_x;
  int16_t g_y;
  int16_t g_z;

  int16_t temp;
  int16_t padding;
  int longtitude;
  int latitude;
}readData;

typedef struct{
	uint8_t TxCmplt;
	uint8_t RxCmplt;
	uint8_t Handshake;
}flags_ESP;

typedef struct
{
	uint8_t ESP1_Size;
	uint8_t ESP2_Size;
}dataLength_ESP;

typedef struct{
	readData Data;
	flags_ESP Flags;
	uint8_t Size;
	uint16_t ADDR;
}ESP1_Ctrl;
typedef struct{
	readData Data;
	flags_ESP Flags;
	uint8_t Size;
	uint16_t ADDR;
}ESP2_Ctrl;


typedef struct{
	uint8_t TxCmplt;
	uint8_t RxCmplt;
	uint8_t Handshake;
}flags_UART;

typedef struct{
	readData* Data;
	flags_UART Flags;
	char *USART_Handshake_msg;
	char Rx_buff[128];
}UART_Ctrl;

typedef struct
{
	TaskHandle_t ESP1_DELIVERY;
	TaskHandle_t ESP2_DELIVERY;
}RTOS_task_struct;

typedef struct
{
	RTOS_task_struct RTOS_Tasks;
	SemaphoreHandle_t xMutex;
	SemaphoreHandle_t I2CMutex;
	SemaphoreHandle_t UARTMutex;
}GLOBAL_struct;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DELAY 100U //in terms of miliseconds
#define COMM_REQLENGTH 0x17U
#define COMM_REQDATA 0x18U

#define ESP1_ADDR 0x51U
#define ESP2_ADDR 0x52U
#define UART_HANDSHAKE_MSG "handshake"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

#define UART_Text "DATA TO BE SENT OVER UART"
GLOBAL_struct GLOBAL;
ESP1_Ctrl ESP1;
ESP2_Ctrl ESP2;
UART_Ctrl UART2_Ctrl;
const uint8_t Req_Length=COMM_REQLENGTH;
const uint8_t Req_Data=COMM_REQDATA;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_Handshake(void);
void UART_ESP1_DELIVER(void *);
void UART_ESP2_DELIVER(void *);
void ESP1_Handshake(void);
void ESP2_Handshake(void);
void READI2C_ESP1(void);
void READI2C_ESP2(void);
void Init_ESPS(void);
void Init_UART(void);
void LED_Green(void *);
void LED_Orange(void *);
void LED_Red(void *);
void LED_Blue(void *);
void read_Button(void);
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
  BaseType_t status;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Init_ESPS();
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);		//WAIT UNTIL GREEN LIGHT TO CONNECT THE TX/RX LINE
  printf("PLEASE CONNECT TXRX LINE TO ARDUINO\n");
  while( ! HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));	//Press the button to continue
  HAL_Delay(500);			//Blocking delay for 500 ms
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); //TURN OFF THE GREEN LED
  Init_UART();
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12| GPIO_PIN_13|		//If UART Connection is established, all LEDS will light up until
		  GPIO_PIN_14| GPIO_PIN_15);					//config process of RTOS tasks are over


  GLOBAL.I2CMutex=xSemaphoreCreateMutex();
  GLOBAL.UARTMutex=xSemaphoreCreateMutex();
  configASSERT(GLOBAL.I2CMutex!=NULL);
  configASSERT(GLOBAL.UARTMutex!=NULL);
  status=xTaskCreate(UART_ESP1_DELIVER, "Task-1", 50, 0, 2, &GLOBAL.RTOS_Tasks.ESP1_DELIVERY);
  configASSERT(status);
  status=xTaskCreate(UART_ESP2_DELIVER, "Task-2", 50, 0, 2, &GLOBAL.RTOS_Tasks.ESP1_DELIVERY);
  configASSERT(status);
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12| GPIO_PIN_13|
		  GPIO_PIN_14| GPIO_PIN_15);
  vTaskStartScheduler();

  printf("Everything is good to go, press the button to continue\n");
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Push_Button_Pin */
  GPIO_InitStruct.Pin = Push_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Push_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void I2C1_IRQ_CallBack(DMA_HandleTypeDef *hdma)
{

}

void Init_ESPS(void)
{
	  memset((void*)&ESP1,0,sizeof(ESP1));
	  memset((void*)&ESP2,0,sizeof(ESP2));

	  ESP1.ADDR=ESP1_ADDR;
	  ESP2.ADDR=ESP2_ADDR;

	  ESP1_Handshake();
	  ESP2_Handshake();
}

void Init_UART(void)
{
	memset((void*)&UART2_Ctrl,0,sizeof(UART2_Ctrl));
	UART2_Ctrl.USART_Handshake_msg=UART_HANDSHAKE_MSG;
	UART_Handshake();
}

void UART_ESP1_DELIVER(void *param)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		xSemaphoreTake( GLOBAL.I2CMutex, portMAX_DELAY);
		READI2C_ESP1();
		xSemaphoreTake( GLOBAL.UARTMutex, portMAX_DELAY);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ESP1.Data, ESP1.Size);
		taskYIELD();
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(30));
	}

}

void UART_ESP2_DELIVER(void *param)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		xSemaphoreTake( GLOBAL.I2CMutex, portMAX_DELAY);
		READI2C_ESP2();
		xSemaphoreTake( GLOBAL.UARTMutex, portMAX_DELAY);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ESP2.Data, ESP2.Size);
		taskYIELD();
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(30));
	}
}

void READI2C_ESP1(void)
{
			while(HAL_I2C_Master_Transmit_DMA(&hi2c1, ESP1.ADDR,(uint8_t *)&Req_Data,1)!=HAL_OK);
			while(HAL_I2C_Master_Receive_DMA(&hi2c1, ESP1.ADDR, (uint8_t *)&ESP1.Data, ESP1.Size)!=HAL_OK);
			if(ESP1.Data.longtitude<0 || ESP1.Data.latitude<0 || ESP1.Data.temp<0)
			{
				configASSERT(0);		//Code error
			}
}

void READI2C_ESP2(void)
{

			while(HAL_I2C_Master_Transmit_DMA(&hi2c1, ESP2.ADDR,(uint8_t *)&Req_Data,1)!=HAL_OK);
			while(HAL_I2C_Master_Receive_DMA(&hi2c1, ESP2.ADDR, (uint8_t *)&ESP2.Data, ESP2.Size)!=HAL_OK);
			if(ESP2.Data.longtitude<0 || ESP2.Data.latitude<0 || ESP2.Data.temp<0)
			{
				configASSERT(0);		//Code error
			}
}

void ESP1_Handshake(void)
{
		if(ESP1.Flags.Handshake==RESET)
		{
			while(ESP1.Flags.Handshake==RESET)
			{
				while(HAL_I2C_Master_Transmit_IT(&hi2c1, ESP1.ADDR, (uint8_t *)&Req_Length, 1)==HAL_BUSY);
				while(ESP1.Flags.TxCmplt!=SET);

				if(ESP1.Flags.TxCmplt==SET)
				{
					ESP1.Flags.Handshake=SET;
					printf("Handshake_ESP_1 achieved with ESP32 requesting length\n");
					ESP1.Flags.RxCmplt=RESET;
					while(HAL_I2C_Master_Receive_IT(&hi2c1, ESP1.ADDR, &ESP1.Size, 1)==HAL_BUSY);
					while(ESP1.Flags.RxCmplt!=SET);
					printf("Length set %d \n", ESP1.Size);
				}
				ESP1.Flags.TxCmplt=RESET;
			}
		}
		while(HAL_I2C_Master_Transmit_IT(&hi2c1, ESP1.ADDR, (uint8_t *)&Req_Data, 1)==HAL_BUSY);
		while(ESP1.Flags.TxCmplt!=SET);
		ESP1.Flags.TxCmplt=RESET;
}

void ESP2_Handshake(void)
{
		if(ESP2.Flags.Handshake==RESET)
		{
			while(ESP2.Flags.Handshake==RESET)
			{
				while(HAL_I2C_Master_Transmit_IT(&hi2c1, (uint16_t) ESP2.ADDR, (uint8_t *)&Req_Length, 1)==HAL_BUSY);
				while(ESP2.Flags.TxCmplt!=SET);

				if(ESP2.Flags.TxCmplt==SET)
				{
					ESP2.Flags.Handshake=SET;
					printf("Handshake_ESP_2 achieved with ESP32 requesting length\n");
					ESP2.Flags.RxCmplt=RESET;
					while(HAL_I2C_Master_Receive_IT(&hi2c1,(uint16_t) ESP2.ADDR,&ESP2.Size,1)==HAL_BUSY);
					while(ESP2.Flags.RxCmplt!=SET);
					printf("Length set %d \n", ESP2.Size);
				}
				ESP2.Flags.TxCmplt=RESET;
			}
		}
		while(HAL_I2C_Master_Transmit_IT(&hi2c1, (uint16_t) ESP2.ADDR, (uint8_t *)&Req_Data, 1)==HAL_BUSY);
		while(ESP2.Flags.TxCmplt!=SET);
		ESP2.Flags.TxCmplt=RESET;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Devaddress==0x51 && ESP1.Flags.Handshake==RESET)
	{
		ESP1.Flags.Handshake = SET;
		ESP1.Flags.TxCmplt=SET;
	}else if (hi2c->Devaddress==0x52 && ESP2.Flags.Handshake==RESET)
	{
		ESP2.Flags.Handshake = SET;
		ESP2.Flags.TxCmplt=SET;
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Devaddress==0x51 && ESP1.Flags.Handshake==RESET)
	{
		ESP1.Flags.Handshake = SET;
		ESP1.Flags.RxCmplt=SET;
	}else if (hi2c->Devaddress==0x52 && ESP2.Flags.Handshake==RESET)
	{
		ESP2.Flags.Handshake = SET;
		ESP2.Flags.RxCmplt=SET;
	}else if(ESP2.Flags.Handshake==SET && ESP1.Flags.Handshake==SET)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(GLOBAL.I2CMutex,&xHigherPriorityTaskWoken);
	}
}

void UART_Handshake(void)
{
	while(UART2_Ctrl.Flags.Handshake!=SET)
	{
		while(HAL_UART_Receive_IT(&huart2,(uint8_t *)UART2_Ctrl.Rx_buff,strlen(UART2_Ctrl.USART_Handshake_msg))==HAL_BUSY);

		while(HAL_UART_Transmit_IT(&huart2,(uint8_t*)UART2_Ctrl.USART_Handshake_msg,strlen(UART2_Ctrl.USART_Handshake_msg))==HAL_BUSY);
		//Send the msg indexed by cnt in blocking mode

		while(UART2_Ctrl.Flags.TxCmplt != SET);
		while(UART2_Ctrl.Flags.RxCmplt  != SET);
		if(!strcmp(UART2_Ctrl.Rx_buff,UART2_Ctrl.USART_Handshake_msg))
		{
			printf("USART_HANDSHAKE SUCCESS\n");
			UART2_Ctrl.Flags.Handshake=SET;

		}else
		{
			printf("USART_HANDSHAKE FAILED\n");
		}
		UART2_Ctrl.Flags.TxCmplt=RESET;
		UART2_Ctrl.Flags.RxCmplt=RESET;
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  if(UART2_Ctrl.Flags.Handshake!=SET)
  {
	  UART2_Ctrl.Flags.TxCmplt=SET;
  }else{
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xSemaphoreGiveFromISR(GLOBAL.UARTMutex ,&xHigherPriorityTaskWoken);
  }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  if(UART2_Ctrl.Flags.Handshake!=SET)
  {
	  UART2_Ctrl.Flags.RxCmplt=SET;
  }

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
