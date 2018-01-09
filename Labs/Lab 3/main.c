/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  * Devin Renshaw		: Timers
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
/* Private variables ---------------------------------------------------------*/
extern int DIR;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;		// GPIOC peripheral clock enable
	//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;		// GPIOA peripheral clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		// TIM2 enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;		// TIM3 enable

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
	NVIC_EnableIRQ(TIM2_IRQn);						// Enable IRQ for TIM2
	//NVIC_EnableIRQ(TIM3_IRQn);						// Enable IRQ for TIM3
	TIM2->DIER |= 1<<0;										// Enable update interrupt for TIM2; UEV in manual
	//TIM3->DIER |= 1<<0;										// Enable update interrupt for TIM3; UEV in manual
	/* 8 Mhz/800 = 10000 => ARR * (PSC+1) = 10000 */
	TIM2->ARR = 250;											// Set prescaler register for TIM3
	//TIM3->ARR = 250;
	TIM3->ARR = 100;											// Set prescaler register for TIM3
	TIM2->PSC = 7999;											// Set auto-reload register for TIM2
	//TIM3->PSC = 7999;
	TIM3->PSC = 99;												// Set auto-reload register for TIM3
	/* TIM2 set to 4 Hz; TIM3 set to 800 Hz*/
	//GPIOC->MODER |= (1<<12)|(1<<14);			// Set pins PC6 and PC7 to output mode
	GPIOC->OTYPER &= ~((1<<6)|(1<<7));		// Clears bits 6 & 7 to enable push-pull
	GPIOC->OSPEEDR &= ~((1<<6)|(1<<7));		// Clears pins PC6 and PC7 to low speed
	GPIOC->PUPDR &= ~(0x0000F000);				// Clears bits of PC6 and PC7 to no pull-up, pull-down
	GPIOC->MODER |= (1<<13)|(1<<15);
	//GPIOC->MODER |= (1<<17)|(1<<19);			// Set pins PC8 and PC9 to alternate function mode: TIM3_CH3/4
	//GPIOC->OTYPER &= ~((1<<8)|(1<<9));		// Clears bits 8 & 9 to enable push-pull
	//GPIOC->OSPEEDR &= ~((1<<8)|(1<<9));		// Clears pins PC8 and PC9 to low speed
	//GPIOC->PUPDR &= ~(0x000F0000);				// Clears bits of PC8 and PC9 to no pull-up, pull-down
	//GPIOC->ODR |= (1<<6);									// Sets PC6 to be first LED on
	//GPIOC->ODR |= (1<<8);									// Sets PC8 to be first LED on
	
	//TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;	// PWM mode 1 (110)
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;				// Enables preload register
	//TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;	// PWM mode 1 (110)
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;				// Enables preload register
	//TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;	// PWM mode 2 (111)
	//TIM3->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;	// PWM mode 2 (111)
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;	// PWM mode 1 (110)
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;	// PWM mode 2 (111)
	//TIM3->CR1 |= TIM_CR1_ARPE;						// Sets ARPE bit in auto-reload reg in TIM3_CR1
	//TIM3->EGR |= TIM_EGR_UG;							// Initialize all regs in EGR
	TIM3->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;	// OC1/OC2 output enabled
	/* Set duty cycle CCR/ARR = duty cycle */
	TIM3->CCR1 = TIM3->ARR;											// Sets 100% initial duty cycle in mode 1
	TIM3->CCR2 = TIM3->ARR;											// Sets 0% initial duty cycle in mode 2
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
	
  while (1)
  {
	//TIM3->CCR1 += DIR*10;
	//TIM3->CCR2 += DIR*10;
	__WFI();
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
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while(1) 
  {
  } 
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
}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
