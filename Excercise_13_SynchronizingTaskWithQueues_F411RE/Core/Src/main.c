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
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
typedef uint32_t TaskProfiler_t ;

TaskProfiler_t 	a=0;
TaskProfiler_t	b=0;
TaskProfiler_t	c=0;
TaskProfiler_t	idleTaskProfiler=0;

//pointers of the led
//const uint16_t * Blue_led = (uint16_t *)led_1_Pin;
//const uint16_t * Red_led = (uint16_t *)led_2_Pin;
//const uint16_t * Green_led = (uint16_t *)led_3_Pin;
//const uint16_t * Orange_led =(uint16_t *) led_4_Pin;

const TickType_t _250ms = pdMS_TO_TICKS(250);
const TickType_t _100ms = pdMS_TO_TICKS(100);
const TickType_t _1000ms = pdMS_TO_TICKS(1000);
const TickType_t _50ms = pdMS_TO_TICKS(50);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void SerialUartSendString(char *ptr);
void SenderTask1(void *pvParameters);
void SenderTask2(void *pvParameters);
void ReceiverTask(void *pvParameters);
void LedsController(void *pvParameters);
void SerialUartSendString(char *ptr);
TaskHandle_t sender_handle,receiver_handle,ledsHandler;
QueueHandle_t yearQueue;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();

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
  yearQueue=xQueueCreate(5,sizeof(uint16_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //create two sender task of the same priority
  xTaskCreate(SenderTask1, "Sender Task 1", 800, NULL, 2, &sender_handle);
  xTaskCreate(SenderTask2, "Sender Task 2", 800, NULL, 2, &sender_handle);
  //create a receiver task of a higher priority
  xTaskCreate(ReceiverTask, "Receive task", 800, NULL, 2, &receiver_handle);
  xTaskCreate(LedsController, "LedsControllerName", 200, NULL, 2, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  vTaskStartScheduler();//star the scheduler

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, led_azul_Pin|led_rojo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led_amarillo_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led_azul_Pin led_rojo_Pin */
  GPIO_InitStruct.Pin = led_azul_Pin|led_rojo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : led_amarillo_Pin LD2_Pin */
  GPIO_InitStruct.Pin = led_amarillo_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : led_verde_Pin */
  GPIO_InitStruct.Pin = led_verde_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_verde_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void SenderTask1(void *pvParameters)
{
	uint16_t valuleToSend =0;
	BaseType_t qStatus;
	while(1)
	{
		//the first vaule is the queue, the second is the data and the thrid is the timeout that queue wait in blocking state
		valuleToSend++;
		qStatus=xQueueSend(yearQueue,&valuleToSend,0);
		if(qStatus!=pdPASS)
		{
			//printf("Error : Data could no be send... \r\n");

		}
		//for(uint32_t i=0; i<100000; i++);
		vTaskDelay(_250ms);

	}

}



void SenderTask2(void *pvParameters)
{
	uint16_t valuleToSend =1234;
	BaseType_t qStatus;
	while(1)
	{
		//the first vaule is the queue, the second is the data and the thrid is the timeout that queue wait in blocking state
		valuleToSend++;
		qStatus=xQueueSend(yearQueue,&valuleToSend,0);
		if(qStatus!=pdPASS)
		{
			//printf("Error : Data could no be send... \r\n");

		}
		//for(uint32_t i=0; i<100000; i++);
		vTaskDelay(_100ms);

	}

}
void ReceiverTask(void *pvParameters)
{
	uint16_t value_received;
	BaseType_t qstatus;
	const TickType_t wait_time = pdMS_TO_TICKS(100);
	char buffer[100];
	while(1)
	{
		qstatus=xQueueReceive(yearQueue, &value_received, _250ms);
		if(qstatus == pdPASS)
		{

			//printf(buffer);
			SerialUartSendString("Queue received ");
			sprintf(buffer,"%i",value_received);
			SerialUartSendString(buffer);
			SerialUartSendString("\r\n");
			//printf("The value received\n");
			//if
			//HAL_GPIO_TogglePin(led_1_GPIO_Port, led_1_Pin);
			//vTaskDelay(_100ms);
		}
		else
		{

			SerialUartSendString("Error! could no receive...\r\n");
		}
		//HAL_GPIO_TogglePin(led_1_GPIO_Port, led_1_Pin);
			//vTaskDelay(_100ms);
	}


}

void LedsController(void *pvParameters)
{
		while(1)
	{
			HAL_GPIO_TogglePin(led_azul_GPIO_Port,led_azul_Pin);
			vTaskDelay(_50ms);

			HAL_GPIO_TogglePin(led_rojo_GPIO_Port,led_rojo_Pin);
			vTaskDelay(_50ms);
			HAL_GPIO_TogglePin(led_verde_GPIO_Port,led_verde_Pin);
			vTaskDelay(_50ms);
			HAL_GPIO_TogglePin(led_amarillo_GPIO_Port,led_amarillo_Pin);
			vTaskDelay(_50ms);
			HAL_GPIO_TogglePin(led_amarillo_GPIO_Port,led_amarillo_Pin);
			vTaskDelay(_50ms);
			HAL_GPIO_TogglePin(led_verde_GPIO_Port,led_verde_Pin);
			vTaskDelay(_50ms);
			HAL_GPIO_TogglePin(led_rojo_GPIO_Port,led_rojo_Pin);
			vTaskDelay(_50ms);
			HAL_GPIO_TogglePin(led_azul_GPIO_Port,led_azul_Pin);
			vTaskDelay(_50ms);



	}

}

//create callback from
void vApplicationIdleHook(void)
{

idleTaskProfiler++;
}


void SerialUartSendString(char *ptr)
{
uint16_t DataLen = strlen(ptr);
HAL_UART_Transmit(&huart2, (const uint8_t *)ptr, DataLen, HAL_MAX_DELAY);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
