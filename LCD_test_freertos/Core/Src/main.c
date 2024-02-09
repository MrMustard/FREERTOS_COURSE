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
#include "lcd.h"
#include "SevenSegmentDisplay.h"
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
const TickType_t _10ms = pdMS_TO_TICKS(10);
const TickType_t _2ms = pdMS_TO_TICKS(2);

typedef enum
{
	humidity_sensor,
	pressure_sensor
}DataSource_t;

typedef struct
{
	uint8_t ucValue;
	DataSource_t sDataSource;//source of the data
}Data_t;

static const Data_t xStrcutrToSend [2]=
{
		{77,humidity_sensor},//used by humidity sensor
		{63,pressure_sensor} // used by pressure sensor
};

QueueHandle_t sensorQueue,dispQueue_1,dispQueue_2;

/*LCD Variables*/

lcd_t lcd_1;
sevenSegment_t sevenSegmentDisplay;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void SerialUartSendString(char *ptr);
void HumidityTask(void *pvParameters);
void PressureTask(void *pvParameters);
void ReceiverTask(void *pvParameters);
void LedsController(void *pvParameters);
void SevenSegmentDisplay(void *pvParameters);
void sevenSegmentCounter(void *pvParameters);

/*lcd task*/
void MenuTask(void *pvParameters);

void SerialUartSendString(char *ptr);
TaskHandle_t hum_task_handle,press_task_handle,receiver_handle,ledsHandler;

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
  sensorQueue =xQueueCreate(3,sizeof(Data_t));
  dispQueue_1 = xQueueCreate(3,sizeof(uint8_t));
  dispQueue_2 = xQueueCreate(3,sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //xTaskCreate(ReceiverTask, "Receive task", 800, NULL, 2, NULL);
  //xTaskCreate(HumidityTask, "Humidity Task", 800, (void *)&(xStrcutrToSend[0]), 2, NULL);
  //xTaskCreate(PressureTask, "Pressure task", 800, (void *)&(xStrcutrToSend[1]), 2, NULL);
  //xTaskCreate(LedsController1, "LedsController1", 200, NULL, 2, NULL);
 //xTaskCreate(LedsController2, "LedsController2", 200, NULL, 2, NULL);
  xTaskCreate(SevenSegmentDisplay, "SevenSegmentDisplayTask", 200, NULL, 2, NULL);
  xTaskCreate(LedsController, "LedsControllerTask", 200, NULL, 2, NULL);
  xTaskCreate(MenuTask, "Menu Task", 800, NULL, 2, NULL);
  xTaskCreate(sevenSegmentCounter, "sevenSegmentCounterTask", 200, NULL, 2, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
 // osKernelStart();

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
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV16;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOC, led_azul_Pin|led_rojo_Pin|DISP_E_Pin|DISP_RW_Pin
                          |SSD_E_Pin|DISP_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led_amarillo_Pin|SSD_A_Pin|SSD_B_Pin|SSD_C_Pin
                          |SSD_G_Pin|SSD_F_Pin|STATUS_LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led_verde_Pin|DISP_D4_Pin|SSD_DP_Pin|DISP_D7_Pin
                          |DISP_D6_Pin|DISP_D5_Pin|STATUS_LED_1_Pin|SSD_D1_Pin
                          |SSD_D2_Pin|SSD_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led_azul_Pin led_rojo_Pin DISP_E_Pin DISP_RW_Pin
                           SSD_E_Pin DISP_RS_Pin */
  GPIO_InitStruct.Pin = led_azul_Pin|led_rojo_Pin|DISP_E_Pin|DISP_RW_Pin
                          |SSD_E_Pin|DISP_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : led_amarillo_Pin SSD_A_Pin SSD_B_Pin SSD_C_Pin
                           SSD_G_Pin SSD_F_Pin STATUS_LED_2_Pin */
  GPIO_InitStruct.Pin = led_amarillo_Pin|SSD_A_Pin|SSD_B_Pin|SSD_C_Pin
                          |SSD_G_Pin|SSD_F_Pin|STATUS_LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led_verde_Pin DISP_D4_Pin SSD_DP_Pin DISP_D7_Pin
                           DISP_D6_Pin DISP_D5_Pin STATUS_LED_1_Pin SSD_D1_Pin
                           SSD_D2_Pin SSD_D_Pin */
  GPIO_InitStruct.Pin = led_verde_Pin|DISP_D4_Pin|SSD_DP_Pin|DISP_D7_Pin
                          |DISP_D6_Pin|DISP_D5_Pin|STATUS_LED_1_Pin|SSD_D1_Pin
                          |SSD_D2_Pin|SSD_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void MenuTask(void *pvParameters)
{
	  lcd_1= lcd_create_4_bit(DISP_RS_GPIO_Port, DISP_RW_GPIO_Port, DISP_E_GPIO_Port,
	  								DISP_RS_Pin, DISP_RW_Pin, DISP_E_Pin,
	  								DISP_D7_GPIO_Port, DISP_D6_GPIO_Port, DISP_D5_GPIO_Port, DISP_D4_GPIO_Port,
	  								DISP_D7_Pin, DISP_D6_Pin, DISP_D5_Pin, DISP_D4_Pin, lcd_chr_16x2_mode);
	  	uint16_t contador=0;
	  	char buffer[20];
	  	lcd_init(&lcd_1);
	  	lcd_clear(&lcd_1);
	  	lcd_set_cursor(&lcd_1, 0, 0);
	  	lcd_print_string(&lcd_1, "Iniciando");
	  	vTaskDelay(_1000ms);
	  	lcd_clear(&lcd_1);

	  	lcd_set_cursor(&lcd_1, 0, 0);
	  	lcd_print_string(&lcd_1, "Ejemplo de");
	  	lcd_set_cursor(&lcd_1, 1, 0);
	  	lcd_print_string(&lcd_1, "FREERTOS");
	  	vTaskDelay(_1000ms);
	  	lcd_clear(&lcd_1);

	while(1)
	{
		lcd_set_cursor(&lcd_1, 0, 0);
		lcd_print_string(&lcd_1, "RTOS");
		//HAL_GPIO_TogglePin(led_azul_GPIO_Port,led_azul_Pin);
		contador++;
		sprintf(buffer,"Count:%05i",contador);
		lcd_set_cursor(&lcd_1, 1, 0);
		lcd_print_string(&lcd_1, buffer);
		vTaskDelay(_10ms);

	}

}

void HumidityTask(void *pvParameters)
{
	BaseType_t qstatus;

	while(1)
	{
		qstatus=xQueueSend(sensorQueue,pvParameters,_250ms);

		if(qstatus !=pdPASS)
		{
			//do something
		}

		vTaskDelay(_100ms);
	}
}
void PressureTask(void *pvParameters)
{
	BaseType_t qstatus;

	while(1)
	{
		qstatus=xQueueSend(sensorQueue,pvParameters,_250ms);

		if(qstatus !=pdPASS)
		{
			//do something
		}

		vTaskDelay(_50ms);
	}

}
void ReceiverTask(void *pvParameters) {

	BaseType_t qStatus;
	Data_t xReceiveDataBuffer;
	char buffer[100];
	while (1) {
		qStatus = xQueueReceive(sensorQueue, &xReceiveDataBuffer, _50ms);

		if (qStatus == pdPASS) {


			if(xReceiveDataBuffer.sDataSource==humidity_sensor)
			{
				SerialUartSendString("************************************\n");
				SerialUartSendString("Humidity Sensor:\n");
				SerialUartSendString("Queue received:\n");
							SerialUartSendString("uValue:");
							sprintf(buffer, "%i", xReceiveDataBuffer.ucValue);
							SerialUartSendString(buffer);
							SerialUartSendString("\n");
							SerialUartSendString("Source of Data:");
							sprintf(buffer, "%i", xReceiveDataBuffer.sDataSource);
							SerialUartSendString(buffer);
							SerialUartSendString("\r\n");
							SerialUartSendString("************************************\n");

			}

			if(xReceiveDataBuffer.sDataSource==pressure_sensor)
			{
				SerialUartSendString("************************************\n");
				SerialUartSendString("Pressure sensor:\n");
								SerialUartSendString("Queue received:\n");
											SerialUartSendString("uValue:");
											sprintf(buffer, "%i", xReceiveDataBuffer.ucValue);
											SerialUartSendString(buffer);
											SerialUartSendString("\n");
											SerialUartSendString("Source of Data:");
											sprintf(buffer, "%i", xReceiveDataBuffer.sDataSource);
											SerialUartSendString(buffer);
											SerialUartSendString("\r\n");
											SerialUartSendString("************************************\n");

			}




		}

		else {

			SerialUartSendString("Error\r\n");

		}

	}

}




void LedsController(void *pvParameters)
{


		while(1)
	{

			HAL_GPIO_WritePin(led_azul_GPIO_Port, led_azul_Pin, GPIO_PIN_SET);
			vTaskDelay(_100ms);
			HAL_GPIO_WritePin(led_amarillo_GPIO_Port, led_amarillo_Pin, GPIO_PIN_SET);
			vTaskDelay(_100ms);
			HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_SET);
			vTaskDelay(_100ms);
			HAL_GPIO_WritePin(led_rojo_GPIO_Port, led_rojo_Pin, GPIO_PIN_SET);
			vTaskDelay(_100ms);
			HAL_GPIO_WritePin(led_azul_GPIO_Port, led_azul_Pin, GPIO_PIN_RESET);
			vTaskDelay(_100ms);
			HAL_GPIO_WritePin(led_amarillo_GPIO_Port, led_amarillo_Pin, GPIO_PIN_RESET);
			vTaskDelay(_100ms);
			HAL_GPIO_WritePin(led_verde_GPIO_Port, led_verde_Pin, GPIO_PIN_RESET);
			vTaskDelay(_100ms);
			HAL_GPIO_WritePin(led_rojo_GPIO_Port, led_rojo_Pin, GPIO_PIN_RESET);
			vTaskDelay(_100ms);

	}

}
void sevenSegmentCounter(void *pvParameters)
{
	uint8_t counter_1=0;
	uint8_t counter_2=7;
	BaseType_t qstatus;
	while(1)
	{
		qstatus=xQueueSend(dispQueue_1,&counter_1,_10ms);
		qstatus=xQueueSend(dispQueue_2,&counter_2,_10ms);


		vTaskDelay(_50ms);
		counter_1++;
		counter_2++;
				if(counter_1>=9 )
				{

					counter_1=0;
				}

				if(counter_2>=9 )
				{

					counter_2=0;
				}

				vTaskDelay(_50ms);
				HAL_GPIO_TogglePin(STATUS_LED_1_GPIO_Port, STATUS_LED_1_Pin);




	}



}

void SevenSegmentDisplay(void *pvParameters)
{

	uint8_t value_received_1=0;
	uint8_t value_received_2=0;
	BaseType_t qstatus;
	sevenSegmentDisplay = SevenSegment_ctor(SSD_A_GPIO_Port, SSD_B_GPIO_Port, SSD_C_GPIO_Port,
			SSD_D_GPIO_Port, SSD_E_GPIO_Port, SSD_F_GPIO_Port,SSD_G_GPIO_Port, SSD_DP_GPIO_Port,
			SSD_D1_GPIO_Port, SSD_D2_GPIO_Port, SSD_A_Pin, SSD_B_Pin, SSD_C_Pin, SSD_D_Pin, SSD_E_Pin,
			SSD_F_Pin, SSD_G_Pin, SSD_DP_Pin, SSD_D1_Pin, SSD_D2_Pin, sevenSegmentCommonCathode, 10);

	while(1)
	{

		qstatus=xQueueReceive(dispQueue_1, &value_received_1, 0);

				if(qstatus == pdPASS)
				{


				}

				qstatus=xQueueReceive(dispQueue_2, &value_received_2, 0);
						if(qstatus == pdPASS)
						{


						}

		SevenSegmentDisplay_s1(&sevenSegmentDisplay, value_received_1);
		vTaskDelay(_2ms);
		SevenSegmentDisplay_off(&sevenSegmentDisplay);
		SevenSegmentDisplay_s2(&sevenSegmentDisplay, value_received_2);
		vTaskDelay(_2ms);
		SevenSegmentDisplay_off(&sevenSegmentDisplay);


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
