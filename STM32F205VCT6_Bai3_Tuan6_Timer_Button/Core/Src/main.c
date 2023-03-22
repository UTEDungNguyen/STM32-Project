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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
int count ;
int state = 0;
int yellow_state = 0;
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
  /* USER CODE BEGIN 2 */

	// Create Timer 2 with 200ms
	RCC->APB1ENR |= (1<<0);
	
	TIM2->PSC = 159;
	TIM2->ARR = 20000;  
	
	TIM2->CR1 |= (1<<0);
	
	TIM2->DIER |= (1<<0);
	NVIC->ISER[0] |= (1<<28);   
	
	//--> PB0
	RCC->AHB1ENR |= (1<<1);  //Toan tu gan va or la truy suat vao 1 bit bat ky tren thanh ghi. Dang ky Cap xung cho output
	GPIOB->MODER |= (1<<(2*0)) | (1<<(2*2));  //Khai bao Output cho thanh ghi. 2 bit dieu khien 1 chan pin
	GPIOB->OTYPER = 0x0000;     //Khai bao mode output
	GPIOB->OSPEEDR |= (1<<(2*0)) | (1<<(2*2));
	
	//--> PA7
	RCC->AHB1ENR |= (1<<0);  //Toan tu gan va or la truy suat vao 1 bit bat ky tren thanh ghi. Dang ky Cap xung cho output
	GPIOA->MODER |= (1<<(2*7));  //Khai bao Output cho thanh ghi. 2 bit dieu khien 1 chan pin
	GPIOA->OTYPER = 0x0000;     //Khai bao mode output
	GPIOA->OSPEEDR |= (1<<(2*7));
	
	// EXTI11
	RCC->APB2ENR |= (1<<14); // Clock for SYSCFGEN (EXTI)	
	RCC->AHB1ENR |= (1<<3); //Clock for GPIOD
	GPIOD->MODER &= ~(3<<2*11); //input PD11 dung cho 1 chan duy nhat
 	GPIOD->PUPDR |= (1<<2*11); // Dien tro keo len
	GPIOD->OSPEEDR |= (1<<2*11);
	
	SYSCFG->EXTICR[2] |= (3<<4*3); // EXTI11 - PD11
	EXTI->IMR |= (1<<11);
	
	EXTI->FTSR |= (1<<11);	
	EXTI->RTSR &= ~(1<<11); 
	
	NVIC->ISER[1] |= (1<<8);
	
	// EXTI6
	RCC->APB2ENR |= (1<<14); // Clock for SYSCFGEN (EXTI)	
	RCC->AHB1ENR |= (1<<2); //Clock for GPIOC
	GPIOC->MODER &= ~(3<<2*6); //input PC6 dung cho 1 chan duy nhat
 	GPIOC->PUPDR |= (1<<2*6); // Dien tro keo len
	GPIOC->OSPEEDR |= (1<<2*6);
	
	SYSCFG->EXTICR[1] |= (2<<4*2); // EXTI6 - PC6
	EXTI->IMR |= (1<<6);
	
	EXTI->FTSR |= (1<<6);	
	EXTI->RTSR &= ~(1<<6); 
	
	NVIC->ISER[0] |= (1<<23);
	
	// EXTI7
	RCC->APB2ENR |= (1<<14); // Clock for SYSCFGEN (EXTI)	
	RCC->AHB1ENR |= (1<<2); //Clock for GPIOC
	GPIOC->MODER &= ~(3<<2*7); //input PC7 dung cho 1 chan duy nhat
 	GPIOC->PUPDR |= (1<<2*7); // Dien tro keo len
	GPIOC->OSPEEDR |= (1<<2*7);
	
	SYSCFG->EXTICR[1] |= (2<<4*3); // EXTI7 - PC7
	EXTI->IMR |= (1<<7);
	
	EXTI->FTSR |= (1<<7);	
	EXTI->RTSR &= ~(1<<7); 
	
	NVIC->ISER[0] |= (1<<23);
	
	// EXTI14
	RCC->APB2ENR |= (1<<14); // Clock for SYSCFGEN (EXTI)	
	RCC->AHB1ENR |= (1<<4); //Clock for GPIOE
	GPIOE->MODER &= ~(3<<2*14); //input PD11 dung cho 1 chan duy nhat
 	GPIOE->PUPDR |= (1<<2*14); // Dien tro keo len
	GPIOE->OSPEEDR |= (1<<2*14);
	
	SYSCFG->EXTICR[3] |= (4<<4*2); // EXTI14 - PE14
	EXTI->IMR |= (1<<14);
	
	EXTI->FTSR |= (1<<14);	
	EXTI->RTSR &= ~(1<<14); 
	
	NVIC->ISER[1] |= (1<<8);
	
	// Set for motor and two LED wait
	GPIOB->BSRR |= (1<<0);
	GPIOB->BSRR |= (1<<2);
	GPIOA->BSRR |= (1<<7);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

	void TIM2_IRQHandler(void)
	{
			if (state == 1){
				count++;
				if ((count%2) != 0){
					GPIOB->BSRR |= (1<<2);
					GPIOA->BSRR |= (1<<23);
				}
				
				if ((count%2) == 0){
					GPIOB->BSRR |= (1<<18);
					GPIOA->BSRR |= (1<<7);
				}
				
				if (yellow_state == 1){
					GPIOB->BSRR |= (1<<0);
					GPIOB->BSRR |= (1<<2);
					GPIOA->BSRR |= (1<<7);
					count = 0;
					state = 0;
				}
				
				if (count == 25){
					if (yellow_state == 0){
						GPIOB->BSRR |= (1<<16);
					}
					GPIOB->BSRR |= (1<<2);
					GPIOA->BSRR |= (1<<7);
					count = 0;
					state = 0;
				}
				
				TIM2->SR &= ~(1<<0); 
			}
	}

void EXTI15_10_IRQHandler (void)
{
		if (EXTI->PR & (1<<11)){
				GPIOB->BSRR |= (1<<16);
		}
		else if (EXTI->PR & (1<<14)){
			GPIOB->BSRR |= (1<<0);
			GPIOB->BSRR |= (1<<2);
			GPIOA->BSRR |= (1<<7);
			state = 0;
			yellow_state = 0;
			count=0;
		}
		
		EXTI->PR |= (1<<14);
		EXTI->PR |= (1<<11);
}

void EXTI9_5_IRQHandler (void)
{

		if (EXTI->PR & (1<<6)){
			GPIOB->BSRR |= (1<<0);
			state = 1;
		}
		else if (EXTI->PR & (1<<7)){
			yellow_state = 1;
		}
		
		EXTI->PR |= (1<<6);
		EXTI->PR |= (1<<7);
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
