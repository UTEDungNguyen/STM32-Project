/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stdlib.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t Tx_Flag =0 ;
uint8_t _rxIndex; 
uint8_t RxBuff[20] ; 
int dem =0 ; 
int k =0 ; 
uint8_t data ;
int ary[9];
char str[2]  ;
char c ;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void USART2_sendchar(uint8_t c);
void UART2_SendString (char *string);
uint8_t UART2_GetChar(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	void UART_Config(void){
	// 1. Enable the USART CLOCK and GPIO CLOCK
	RCC->APB1ENR |= (1<<17); // Enable UART2 CLOCK 
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA CLOCK 
	// 2. Configure the USART PINs for Alternate Function 
	GPIOA->MODER |= (2<<4); // Bits (5:4)= 1:0 --> Alternate Function for Pin PA2
	GPIOA->MODER |= (2<<6); // Bits (7:6)= 1:0 --> Alternate Function for Pin PA3
//	GPIOA->MODER |= (1<<(2*7)); // Bits (7:6)= 1:0 --> Alternate Function for Pin PA3
	GPIOA->OSPEEDR |= (3<<4)|(3<<6); // Bits (5:4) = 1:1 and Bits (7:6) =1:1 => High speed for PIN PA2 and PIN PA3
	
	GPIOA->AFR[0] |= (7<<8); // Bytes (11:10:9:8) = 0:1:1:1 --> AF7 Alternate function for UART2 at Pin PA2
	GPIOA->AFR[0] |= (7<<12); // Bytes (15:14:13:12) = 0:1:1:1 --> AF7 Al ternate function for UART2 at Pin PA3
	// 3. Enable the USART by writing the UE bit in USART_CR1 register to 1 
	USART2->CR1 = 0x00 ; // clear all 
	USART2->CR1 |= (1<<13) ; // UE =1 ... Enable USART
  // 4. Program the M bit in USART_CR1 to define the word length 
	USART2->CR1 &= ~(1<<12); // M=0;8 bit word length 
	//5. Select the desired baudrate using USART_BRR register
	USART2->BRR = (3<<0)|(104<<4); // Baud rate 9600 , PCLK1 at 16MHz 
	// 6. Enable the Transmitter/Receiver by Setting the TE and RE bits in USART_CR1 Register 
	USART2->CR1 |= (1<<2); // RE=1 Enable Receiver 
	USART2->CR1 |= (1<<3); // TE=1 Enable Transmitter  
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//UART_Config();
		RCC->AHB1ENR |= (1<<0);  //Toan tu gan va or la truy suat vao 1 bit bat ky tren thanh ghi. Dang ky Cap xung cho output
		GPIOA->MODER |= (1<<(2*7));  //Khai bao Output cho thanh ghi. 2 bit dieu khien 1 chan pin
		GPIOA->OTYPER = 0x0000;     //Khai bao mode output
		GPIOA->OSPEEDR |= (1<<(2*7));
		
		RCC->AHB1ENR |= (1<<1);  //Toan tu gan va or la truy suat vao 1 bit bat ky tren thanh ghi. Dang ky Cap xung cho output
		GPIOB->MODER = (1<<(2*0))|(1<<(2*2));  //Khai bao Output cho thanh ghi. 2 bit dieu khien 1 chan pin
		GPIOB->OTYPER = 0x0000;     //Khai bao mode output
		GPIOB->OSPEEDR = (1<<(2*0))|(1<<(2*2));	
		
		RCC->AHB1ENR |= (1<<4);  //Toan tu gan va or la truy suat vao 1 bit bat ky tren thanh ghi. Dang ky Cap xung cho output
		GPIOE->MODER = (1<<(2*7))|(1<<(2*8))| (1<<(2*9))| (1<<(2*10))| (1<<(2*11));  //Khai bao Output cho thanh ghi. 2 bit dieu khien 1 chan pin
		GPIOE->OTYPER = 0x0000;     //Khai bao mode output
		GPIOE->OSPEEDR = (1<<(2*7))|(1<<(2*8))| (1<<(2*9))| (1<<(2*10))| (1<<(2*11));
	
	//UART in STM32
	// 1. Enable the USART CLOCK and GPIO CLOCK
	RCC->APB1ENR |= (1<<17); // Enable UART2 CLOCK 
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA CLOCK 
	// 2. Configure the USART PINs for Alternate Function 
	GPIOA->MODER |= (2<<4); // Bits (5:4)= 1:0 --> Alternate Function for Pin PA2
	GPIOA->MODER |= (2<<6); // Bits (7:6)= 1:0 --> Alternate Function for Pin PA3
	
	GPIOA->OSPEEDR |= (3<<4)|(3<<6); // Bits (5:4) = 1:1 and Bits (7:6) =1:1 => High speed for PIN PA2 and PIN PA3
	
	GPIOA->AFR[0] |= (7<<8); // Bytes (11:10:9:8) = 0:1:1:1 --> AF7 Alternate function for UART2 at Pin PA2
	GPIOA->AFR[0] |= (7<<12); // Bytes (15:14:13:12) = 0:1:1:1 --> AF7 Alternate function for UART2 at Pin PA3
	// 3. Enable the USART by writing the UE bit in USART_CR1 register to 1 
	USART2->CR1 = 0x00 ; // clear all 
	USART2->CR1 |= (1<<13) ; // UE =1 ... Enable USART
  // 4. Program the M bit in USART_CR1 to define the word length 
	USART2->CR1 &= ~(1<<12); // M=0;8 bit word length 
	//5. Select the desired baudrate using USART_BRR register
	USART2->BRR = (3<<0)|(104<<4); // Baud rate 9600 , PCLK1 at 16MHz 
	// 6. Enable the Transmitter/Receiver by Setting the TE and RE bits in USART_CR1 Register 
	USART2->CR1 |= (1<<2); // RE=1 Enable Receiver 
	USART2->CR1 |= (1<<3); // TE=1 Enable Transmitter 
	
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
  /* USER CODE BEGIN 2 */ 
	
//	RCC->AHB1ENR  |= (1<<4);
//	GPIOE->MODER =  (1<<(2*7))| (1<<(2*8))| (1<<(2*9))| (1<<(2*10))| (1<<(2*11));
//	GPIOE->OTYPER = 0x0000;
//	GPIOE->OSPEEDR = (1<<(2*0))|(1<<(2*1))| (1<<(2*2))| (1<<(2*3))| (1<<(2*4))| (1<<(2*5))| (1<<(2*6))| (1<<(2*7))| (1<<(2*8))| (1<<(2*9))| (1<<(2*10))| (1<<(2*11))| (1<<(2*12))| (1<<(2*13))| (1<<(2*14))| (1<<(2*15));
////  RCC->AHB1ENR  |= (1<<1);
////	GPIOB->MODER = (1<<(2*0))|(1<<(2*2));
////	GPIOB->OTYPER = 0x0000;
////	GPIOB->OSPEEDR = (1<<(2*0))|(1<<(2*1))| (1<<(2*2))| (1<<(2*3))| (1<<(2*4))| (1<<(2*5))| (1<<(2*6))| (1<<(2*7))| (1<<(2*8))| (1<<(2*9))| (1<<(2*10))| (1<<(2*11))| (1<<(2*12))| (1<<(2*13))| (1<<(2*14))| (1<<(2*15));
//	RCC->AHB1ENR  |= (1<<0);
//	GPIOA->OSPEEDR = (1<<(2*7));
////		GPIOE->BSRR = (1<<7);
////		GPIOE->BSRR = (1<<8);
////		GPIOE->BSRR = (1<<9);
////		GPIOE->BSRR = (1<<10);
////		GPIOE->BSRR = (1<<11);
////		GPIOB->BSRR = (1<<0);
////		GPIOB->BSRR = (1<<2);
////		GPIOA->BSRR = (1<<7);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		data = UART2_GetChar();
		if (data == '1'){
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		}
		if (data == '0'){
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		}
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
//		HAL_Delay(100);
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
	void USART2_sendchar(uint8_t c){
	USART2->DR = c; // load data into DR register 
	while (!(USART2->SR&(1<<6)));// Wait for TC to SET .. This indicates that the data has been transmitted 
}
void UART2_SendString (char *string)
{
	while(*string) USART2_sendchar(*string++);
}
uint8_t UART2_GetChar(void){
	uint8_t temp;
	while(!(USART2->SR&(1<<5))); // wait for RXNE bit to set 
	temp = USART2->DR; // Read the data . This clears the RXNE also 
	return temp;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
