/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
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

/* USER CODE BEGIN PV */
typedef uint32_t TaskProfiler_t ;

TaskProfiler_t 	a=0;
TaskProfiler_t	b=0;
TaskProfiler_t	c=0;
TaskProfiler_t	idleTaskProfiler=0;

//pointers of the led
const uint16_t * Blue_led = (uint16_t *)led_1_Pin;
const uint16_t * Red_led = (uint16_t *)led_2_Pin;
const uint16_t * Green_led = (uint16_t *)led_3_Pin;
const uint16_t * Orange_led =(uint16_t *) led_4_Pin;

const TickType_t _250ms = pdMS_TO_TICKS(250);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void vLedControllerTask(void *pvParameters);//tarea de controlador de task
void vBlueLedControllerTask(void *pvParameters);//prototipo de funcion de tarea manejadora de leds
void vRedLedControllerTask(void *pvParameters);//prototipo de funcion de tarea manejadora de leds
void vGreenLedControllerTask(void *pvParameters);//prototipo de funcion de tarea manejadora de leds

void SerialUartSendString(char *ptr);
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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //este es otro metodo para controlar varios leds con solo una tareas
 /* xTaskCreate(vLedControllerTask,
  		  	  "Led main controller 1",100,(void *)Blue_led,1,NULL);
  xTaskCreate(vLedControllerTask,
  		  	  "Led main controller 2",100,(void *)Red_led,1,NULL);
  xTaskCreate(vLedControllerTask,
  		  	  "Led main controller 3",100,(void *)Green_led,1,NULL);
  xTaskCreate(vLedControllerTask,
  		  	  "Led main controller 4",100,(void *)Orange_led,1,NULL);*/

  xTaskCreate(vBlueLedControllerTask,
		  	  "Blue Led Controller",100,NULL,2,NULL); //creamos la tarea 1
  xTaskCreate(vRedLedControllerTask,
 		  	  "Red Led Controller",100,NULL,2,NULL); //creamos la tarea 2
  xTaskCreate(vGreenLedControllerTask,
 		  	  "Green Led Controller",100,NULL,2,NULL); //creamos la tarea 3


  vTaskStartScheduler();//star the scheduler
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */


  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_UART_Transmit(&huart2, (uint8_t *)&msg, 1, HAL_MAX_DELAY);
	  //HAL_Delay(1000);
	 //SerialUartSendString("Hola Mundo\n");
	printf("Hola mundo \n\r");
	  //HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, led_4_Pin|led_3_Pin|led_2_Pin|led_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_4_Pin led_3_Pin led_2_Pin led_1_Pin */
  GPIO_InitStruct.Pin = led_4_Pin|led_3_Pin|led_2_Pin|led_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*printf definition*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

//tarea controladora de leds
void vLedControllerTask (void *pvParameters)
{
	//int i;
	while (1)
	{
		//printf("vBlueLedControllerTask running... \n\r");
		a++;
		HAL_GPIO_TogglePin(led_1_GPIO_Port,(uint16_t) pvParameters);



	}

}

/*tarea controladora de led*/
void vBlueLedControllerTask(void *pvParameters)
{

	while (1)
	{
		//printf("vBlueLedControllerTask running... \n\r");
		a++;
		HAL_GPIO_TogglePin(led_1_GPIO_Port, led_1_Pin);
		vTaskDelay(_250ms);
		vTaskDelay(_250ms);

	}

}

//segunda tarea
void vRedLedControllerTask(void *pvParameters)
{
	while (1)
	{
		//printf("RedLedControllerTask running... \n\r");
		//HAL_GPIO_TogglePin(led_1_GPIO_Port, led_1_Pin);
		//HAL_GPIO_WritePin(led_1_GPIO_Port, led_1_Pin, GPIO_PIN_SET);
		b++;
		HAL_GPIO_TogglePin(led_2_GPIO_Port, led_2_Pin);
		vTaskDelay(_250ms);
		vTaskDelay(_250ms);
		vTaskDelay(_250ms);
		vTaskDelay(_250ms);
	}

}
//tercera tarea
void vGreenLedControllerTask(void *pvParameters)
{
	while (1)
	{
		//printf("GreenLedControllerTask running... \n\r");
		//HAL_GPIO_TogglePin(led_2_GPIO_Port, led_2_Pin);
		//HAL_GPIO_WritePin(led_2_GPIO_Port, led_2_Pin, GPIO_PIN_SET);
		c++;
		HAL_GPIO_TogglePin(led_3_GPIO_Port, led_3_Pin);
		vTaskDelay(_250ms);

	}

}

//create callback from
void vApplicationIdleHook(void)
{

idleTaskProfiler++;
HAL_GPIO_TogglePin(led_4_GPIO_Port,led_4_Pin);
//HAL_GPIO_TogglePin(led_2_GPIO_Port, led_2_Pin);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


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
