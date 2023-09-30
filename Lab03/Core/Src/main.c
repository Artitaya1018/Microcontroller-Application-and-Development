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
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
char str[] = "Hello, World!!\r\n";
char str2[] = "\t\tInput => ";
char str3[] = "Display Blinking LED PRESS (1, 2)\r\nDisplay Group Members PRESS m\r\nQuit PRESS q \n\r";
char str4[] = "Unknown Command\n\r";
char mem[] = "64011018\n\rARTITAYA   PIMSUPAPORN\r\n64010022\n\rKRIT    TANGPINYOPUTTIKHUN\r\n";
char ch1;
char ch2 = 'A';
uint8_t num,i;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	=============== 1. PRINT "HELLO, WORD!!" ==============================
	  // �?�?อ�?ส�?�?หรือรั�?�?�?อมูล ต�?อ�?ตรว�?สอ�?สถา�?ะ�?อ�? register(FLAG)
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET ){}
//	HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),1000);
//	HAL_Delay(500);
//
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE)==RESET){}
//	HAL_UART_Receive(&huart3, (uint8_t*) &ch1, 1, 1000);
//  =======================================================================

//	=================== 2. PRINT CHARTER "A" ==============================
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
//	HAL_UART_Transmit(&huart3, (uint8_t*) &ch2, 1,1000);
//	HAL_Delay(500);
//
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE)==RESET){}
//	HAL_UART_Receive(&huart3, (uint8_t*) &ch2, 1,1000);
	//  หมายเหตุ : ถ�?าตัว�?�?รรั�?เ�?�?�?ตัว�?�?รเดียว�?ั�?�?�?ที�?�?ี�? �?ือ &ch2 ทำ�?ห�?�?ดอะ�?ร�?เ�?อย�?า�?�?ั�?�? เ�?�?�? �?ด c �?�?print c
	//			�?ต�?ต�?อ�?อยู�?�?�?ASCII ถ�?าอย�?า�? ENTER �?ะ�?ม�?printอะ�?รออ�?มา
//  =======================================================================

//	===============  3. RECEIVE CHARACTER AND if PRESS 'q' THEN END OF WORK ======================
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET ){}
//	HAL_UART_Transmit(&huart3, (uint8_t*) str2, strlen(str2),1000);
//
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE)==RESET){}
//	HAL_UART_Receive(&huart3, (uint8_t*) &ch1, 1, 1000);
//
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET ){}
//	HAL_UART_Transmit(&huart3, (uint8_t*) &ch1, 1,1000);
//	HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r", 2, 1000);
//
//	if(ch1=='q' || ch1 == 'Q'){
//		while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
//		HAL_UART_Transmit(&huart3, (uint8_t*) "QUIT", 4, 1000);
//		break;
//	}
	//  หมายเหตุ : Transmit เ�?รีย�?เสมือ�? printf , Receive เ�?รีย�?เสมือ�? scanf
//  ==========================================================================================

//  ===============  4. PRESS(1,2): DISPLAY LED1,2 | PRESS m : DISPLAY GROUP MAMBER | PRESS q : END OF WORK | OTHER PRESS : PRINT "Unknown Command" ======================
	  if(num == 0){
		  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
		  HAL_UART_Transmit(&huart3, (uint8_t*) str3, strlen(str3), 1000);
		  num = 1 ;
	  }
	  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
	  HAL_UART_Transmit(&huart3, (uint8_t*) str2, strlen(str2), 1000);

	  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE)==RESET){}
	  HAL_UART_Receive(&huart3, (uint8_t*) &ch1, 1,1000);

	  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
	  HAL_UART_Transmit(&huart3, (uint8_t*) &ch1, 1, 1000);
	  HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r", 2, 1000);

	  if(ch1 =='1' ){
		  for(i=0; i<3;i++){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
			  HAL_Delay(300);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
			  HAL_Delay(300);
		  }
	  }else if(ch1 == '2'){
		  for(i=0; i<3;i++){
		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,1);
		  			  HAL_Delay(300);
		  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
		  			  HAL_Delay(300);
		  		  }
	  }else if(ch1 == 'm' || ch1 == 'M'){
		  HAL_UART_Transmit(&huart3, (uint8_t*) mem, strlen(mem), 1000);
	  }else if(ch1 == 'q' || ch1 == 'Q'){
		  HAL_UART_Transmit(&huart3, (uint8_t*) "QUIT", 4, 1000);
		  break;
	  }else{
		  HAL_UART_Transmit(&huart3, (uint8_t*) str4, strlen(str4), 1000);
	  }

//  ======================================================================================================================================================================
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
