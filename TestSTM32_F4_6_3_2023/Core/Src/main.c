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
#include "stdlib.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
int dem =0 ; 
uint8_t data ;
uint8_t start_bit ;
int start_bit_int ;
int array[13];	
int total=0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
void UART2_SendChar (uint8_t c);
void UART2_SendString (char *string);
uint8_t UART2_GetChar ();
	
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  /* USER CODE BEGIN 2 */
	
	//Truy xuat chan D
		RCC->AHB1ENR |= (1<<3);  //Toan tu gan va or la truy suat vao 1 bit bat ky tren thanh ghi. Dang ky Cap xung cho output
		GPIOD->MODER = 0x55555555;  //Khai bao Output cho thanh ghi. 2 bit dieu khien 1 chan pin
		GPIOD->OTYPER = 0x0000;     //Khai bao mode output
		GPIOD->OSPEEDR = 0x55555555;
	
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
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		start_bit = UART2_GetChar();
		start_bit_int = (int)(start_bit);
		
		if(start_bit_int == 126){
			while(dem != 8){
				data = UART2_GetChar();
				array[dem++] = data ;  
				UART2_SendChar(data);
			}
			for(int x=8; x<=11;x++){
				array[x] = UART2_GetChar();
				total += (int)array[x];
			}
			if ((int)total == 295){
				for(int i=0; i<8; i++){
					int data_receive = (int)array[i]-48;
					if(data_receive ==1){
						GPIOD->BSRR = (1<<1)|(1<<16)|(1<<18);
						//HAL_Delay(5);
						GPIOD->BSRR = (1<<1)|(1<<0)|(1<<18);
						//HAL_Delay(5);
						GPIOD->BSRR = (1<<1)|(1<<16)|(1<<2);
						//HAL_Delay(5);
					}
					if(data_receive == 0){
						GPIOD->BSRR = (1<<17)|(1<<16)|(1<<18);
						//HAL_Delay(5);
						GPIOD->BSRR = (1<<17)|(1<<0)|(1<<18);
						//HAL_Delay(5);
						GPIOD->BSRR = (1<<17)|(1<<16)|(1<<2);
						//HAL_Delay(5);
					}
				}
//				UART2_SendChar(0x0A);
//				UART2_SendChar(0x0D);
				dem= 0 ;
				total =0;
			}
		}
		
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
//Ham UART truyen di
void UART2_SendChar (uint8_t c)
{
	USART2->DR = c; //Gui vao data vao thanh DR
	while(!(USART2->SR & (1<<6))); //wait TC duoc set . tho khi do da duoc truyen xong
}

void UART2_SendString (char *string)
{
	while(*string) UART2_SendChar(*string++);
}

//Ham UART nhan ve
uint8_t UART2_GetChar ()
{
	char value;
	while(!(USART2->SR & (1<<5))); //cho khi RXNE duoc set
	value = USART2->DR;    //dOC DATA
	return value;
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
