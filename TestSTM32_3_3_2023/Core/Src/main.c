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
	unsigned char maled[10]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};
	unsigned int dem= 0;
	int tram=0, chuc=0, donvi=0;
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
/* USER CODE BEGIN PFP */
void quet_led(int tram, int chuc, int donvi);
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
  /* USER CODE BEGIN 2 */
	
	RCC->AHB1ENR  |= (1<<0);  // Port A 
	RCC->AHB1ENR  |= (1<<3);  // Port D 
	RCC->AHB1ENR  |= (1<<4);  // Port E
	//GPIOD->MODER = 0b01010101010101010101010101010101;  // Khai bao output
	RCC->AHB1ENR  |= (1<<3);  // Port D 
	//GPIOD->MODER = 0b01010101010101010101010101010101;  // Khai bao output
//	GPIOD->MODER = (1<<(2*0))|(1<<(2*1))| (1<<(2*2))| (1<<(2*3))| (1<<(2*4))| (1<<(2*5))| (1<<(2*6))| (1<<(2*7))| (1<<(2*8))| (1<<(2*9))| (1<<(2*10))| (1<<(2*11))| (1<<(2*12))| (1<<(2*13))| (1<<(2*14))| (1<<(2*15));
//	GPIOD->OTYPER = 0x0000;  // Che do Push Pull (0,1)
//	GPIOD->OSPEEDR = (1<<(2*0))|(1<<(2*1))| (1<<(2*2))| (1<<(2*3))| (1<<(2*4))| (1<<(2*5))| (1<<(2*6))| (1<<(2*7))| (1<<(2*8))| (1<<(2*9))| (1<<(2*10))| (1<<(2*11))| (1<<(2*12))| (1<<(2*13))| (1<<(2*14))| (1<<(2*15));
	GPIOD->MODER = 0x55555555; // output 
	GPIOD->OTYPER = 0x0000;  // Che do Push Pull (0,1)
	GPIOD->OSPEEDR = 0x55555555; 
	
	GPIOA->MODER = 0x55555555; // output 
	GPIOA->OTYPER = 0x0000;  // Che do Push Pull (0,1)
	GPIOA->OSPEEDR = 0x55555555; 
	
	GPIOE->MODER =  (1<<(2*0))|(1<<(2*1))| (1<<(2*2)); // output 
	GPIOE->OTYPER = 0x0000;  // Che do Push Pull (0,1)
	GPIOE->OSPEEDR = (1<<(2*0))|(1<<(2*1))| (1<<(2*2)); 
	
	// EXTI2
	RCC->APB2ENR |= (1<<14); // Clock for SYSCFGEN (EXTI)
	RCC->AHB1ENR |= (1<<1); //Clock for GPIOB
	GPIOB->MODER &= ~(3<<2*2); //input PB2 dung cho 1 chan duy nhat
 	GPIOB->PUPDR |= (1<<2*2); // Dien tro keo len
	GPIOB->OSPEEDR |= (1<<2*2);
	
	SYSCFG->EXTICR[0] |= (1<<4*2); // EXTI2 - PB2
	EXTI->IMR |= (1<<2);
	
	EXTI->FTSR |= (1<<2);	
	EXTI->RTSR &= ~(1<<2); 
	
	NVIC->ISER[0] |= (1<<8);
	
		// EXTI3
	RCC->APB2ENR |= (1<<14); // Clock for SYSCFGEN (EXTI)
	RCC->AHB1ENR |= (1<<1); //Clock for GPIOB
	GPIOB->MODER &= ~(3<<3*2); //input PB2 dung cho 1 chan duy nhat
 	GPIOB->PUPDR |= (1<<3*2); // Dien tro keo len
	GPIOB->OSPEEDR |= (1<<3*2);
	
	SYSCFG->EXTICR[0] |= (1<<4*3); // EXTI3 - PB3	
	EXTI->IMR |= (1<<3);
	
	EXTI->FTSR |= (1<<3);	
	EXTI->RTSR &= ~(1<<3); 
	
	NVIC->ISER[0] |= (1<<9);
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//		HAL_Delay(500);
	//		//GPIOD->ODR = maled[dem]|(maled[dem]<<8);
	//		dem++;
	//		GPIOE->ODR |= (1<<5);
	//		for(int i=0 ;i <10;i++){
	//			for(int j=0 ;j <10;j++){
	//				GPIOD->ODR = maled[i]|(maled[j]<<8);
	//				HAL_Delay(200);
	//			}
	//		}
	//		dem++;
	//		HAL_Delay(100);
		
		tram = (dem/100);
		chuc = (dem/10)%10;
		donvi = (dem%100)%10;
		quet_led(tram,chuc,donvi);
		
		
		
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

/* USER CODE BEGIN 4 */
void quet_led(int tram, int chuc, int donvi)
	{
	
		GPIOE->BSRR = (1<<0)|(1<<17)|(1<<18);
		GPIOA->ODR = maled[tram];
		HAL_Delay(10);
		
		GPIOE->BSRR = (1<<16)|(1<<1)|(1<<18);
		GPIOA->ODR = maled[chuc];
		HAL_Delay(10);
		
		GPIOE->BSRR = (1<<16)|(1<<17)|(1<<2);
		GPIOA->ODR = maled[donvi];
		HAL_Delay(10);
}
	
void EXTI2_IRQHandler (void)
{
    // place your code
	dem++;	
}

void EXTI3_IRQHandler (void)
{
    // place your code
	dem--;	
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
