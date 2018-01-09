/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
void toggle_red()
{
	GPIOC->ODR &= ~((1<<7)|(1<<8)|(1<<9));
	GPIOC->ODR ^= (1<<6);
}

void toggle_orange()
{
	GPIOC->ODR &= ~((1<<6)|(1<<7)|(1<<9));
	GPIOC->ODR ^= (1<<8);
}

void toggle_green()
{
	GPIOC->ODR &= ~((1<<6)|(1<<7)|(1<<8));
	GPIOC->ODR ^= (1<<9);
}

void toggle_blue()
{
	GPIOC->ODR &= ~((1<<6)|(1<<8)|(1<<9));
	GPIOC->ODR ^= (1<<7);
}

void tx_string(char str[])
{
	//while(!(USART3->ISR & USART_ISR_TXE))	{}	/* tx reg bit not set; exits when tx bit set */
	for(int i = 0; str[i] != '\0'; ++i)
	{
		while(!(USART3->ISR & USART_ISR_TXE))	{}	/* tx reg bit not set; exits when tx bit set */
		USART3->TDR = str[i];
	}
}

void compare_char(char ch)
{
	if (ch == 'r')
		toggle_red();
	else if (ch == 'o')
		toggle_orange();
	else if (ch == 'g')
		toggle_green();
	else if (ch == 'b')
		toggle_blue();
	else
	{
		char str[] = "ERROR";
		tx_string(str);
	}
	return;
}

void tx_char(char ch)
{
	while(!(USART3->ISR & USART_ISR_TXE))	{}	/* tx reg bit not set; exits when tx bit set */
	USART3->TDR = ch;
}

void rx_char()
{
	while (!(USART3->RDR & USART_ISR_RXNE))
	{
	
	}
	char ch = USART3->RDR;
	compare_char(ch);
	USART3->RQR |= 1<<3;	// Manually reset RXNE
}
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
	  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	
  /* USER CODE BEGIN 1 */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;		// GPIOC peripheral clock enable
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;	// Enable USART3 clock
	GPIOC->MODER |= (1<<9)|(1<<11);			// Set pins PC4 and PC5 to alternate mode
	GPIOC->AFR[0] |= 0x00110000;	// Select AF1 on PC4 and PC5 in AFRL
  /* USER CODE END 1 */


  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
	int clock = HAL_RCC_GetHCLKFreq();		// Get clock frequency
	int baud = 115200;										// Desired baud rate
	int usartdiv1 = clock/baud;						// Calculate baud rate register value: ~69
	USART3->BRR = usartdiv1;							// Store value into baud rate register
	USART3->CR1 |= USART_CR1_TE;					/* Enable tx on USART3 */
	USART3->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;	/* Enable rx on USART3 */
	USART3->CR1 |= USART_CR1_UE;					/* Enable USART3 */

	/* Enable LEDs (PC6-PC9)
			RED		->	PC6
			ORANGE->	PC8
			GREEN ->	PC9
			BLUE	->	PC7	*/
	GPIOC->MODER |= (1<<12)|(1<<14)|(1<<16)|(1<<18);	// Set pins PC6 to PC9 to output mode
	GPIOC->OTYPER &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));	// Clears bits 6 & 7 to enable push-pull
	GPIOC->OSPEEDR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));	// Clears pins PC6 and PC7 to low speed
	GPIOC->PUPDR &= ~(0x000FF000);				// Clears bits of PC6 and PC7 to no pull-up, pull-down
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
		tx_char('a');
		//char str[] = "string";
		/*for(int i = 0; str[i] != '\0'; ++i)
			tx_char(str[i]);
		*/
		/*char str[] = "strings are for wimps!";
		tx_string(str);
		*/
		//rx_char();
		//USART3->RQR |= 1<<3;	// Manually reset RXNE
		HAL_Delay(200);
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
