/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
	* Devin Renshaw			 : Analog Lab
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

/* Private variables ---------------------------------------------------------*/
//static int OFFSET = 0;
//static int count = 0;
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
					232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/
void pin_off(int OFFSET)
{
	GPIOC->ODR &= ~(1<<OFFSET);
}

void pin_on(int OFFSET)
{
	GPIOC->ODR |= (1<<OFFSET);
}
	
int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all peripheral clocks */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;		// GPIOC peripheral clock enable
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;		// GPIOB peripheral clock enable
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;		// GPIOA peripheral clock enable
	
	/* Enable LEDs */
	GPIOC->MODER |= (1<<12)|(1<<14)|(1<<16)|(1<<18);		// Output mode
	GPIOC->OSPEEDR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));		// Low speed
	GPIOC->OTYPER &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));		// Push-pull
	GPIOC->PUPDR |= 0x000FF000;													// No pull-up/pull-down
	/* Setup ADC pin (PC0 -> ADC_IN10) */
	GPIOC->MODER |= 0x3;		// Analog mode
	GPIOC->PUPDR |= 0x3;		// No pullup/pulldown
	/* Setup DAC pin (PA4 -> DAC_OUT1) */
	GPIOA->MODER |= (1<<8)|(1<<9);		// Analog mode
	GPIOA->PUPDR |= (1<<8)|(1<<9);		// No pullup/pulldown
	/* ADC1 and DAC clock enable */
	//RCC->APB2ENR  |= RCC_APB2ENR_ADCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		// ADC peripheral clock enable
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;		// DAC peripheral clock enable
	
	/* ADC configuration */
	ADC1->CFGR1 |= (1<<4);		// Set bit 4 -> CFGR1[4:3] = 10 for 8 bits resolution
	ADC1->CFGR1 &= ~(1<<3);		// Clear bit 3; set data resolution to 8 bits
	ADC1->CFGR1 |= ADC_CFGR1_CONT/*(1<<13)*/;		// Continuous conversion mode
	ADC1->CFGR1 &= ~((1<<10)|(1<<11));	// Disable hardware triggers (SW only)
	ADC1->CHSELR = ADC_CHSELR_CHSEL10;		// Select channel 10 (pin PC0)
	
	/* ADC calibration */
	ADC1->CR |= ADC_CR_ADCAL;	// Sets calibration bit
	while (ADC1->CR & ADC_CR_ADCAL) {}	// While calibration in progress, loop
	//OFFSET = ADC1->DR & 0x7F;	// Saves offset error
	ADC1->CR |= ADC_CR_ADEN;	// Enable ADC command (ADEN = 1)
	
	/* DAC configuration */		
	DAC->CR |= DAC_CR_TSEL1;	// Software trigger
	DAC->CR |= DAC_CR_EN1;		// DAC enable channel 1
	
  while (1)
  {
		while (!(ADC1->ISR & ADC_ISR_ADRDY))
		{
		}
		ADC1->CR |= ADC_CR_ADSTART;
		int data = ADC1->DR;
		if (data > 0x33)
			pin_on(6);
		else
			pin_off(6);
		if (data > 0x66)
			pin_on(8);
		else
			pin_off(8);
		if (data > 0x99)
			pin_on(7);
		else
			pin_off(7);
		if (data > 0xCC)
			pin_on(9);
		else
			pin_off(9);
		HAL_Delay(100);
		/*for (int i = 0; i < 32; i++)
		{
			DAC->DHR8R1 = sine_table[i];
			HAL_Delay(1);	// 1ms delay
		}*/
  }
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
