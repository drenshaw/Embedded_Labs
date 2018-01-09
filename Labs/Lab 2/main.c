/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* Private variables ---------------------------------------------------------*/
volatile int counter = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;		// GPIOC peripheral clock enable
	GPIOC->MODER |= (1<<12)|(1<<14);			// Set pins PC6 and PC7 to output mode
	GPIOC->OTYPER &= ~((1<<6)|(1<<7));		// Clears bits 6 & 7 to enable push-pull
	GPIOC->OSPEEDR &= ~((1<<6)|(1<<7));		// Clears pins PC6 and PC7 to low speed
	GPIOC->PUPDR &= ~(0x0000F000);				// Clears bits of PC8 and PC9 to no pull-up, pull-down
	GPIOC->MODER |= (1<<16)|(1<<18);			// Set pins PC8 and PC9 to output mode
	GPIOC->OTYPER &= ~((1<<8)|(1<<9));			// Clears bits 8 & 9 to enable push-pull
	GPIOC->OSPEEDR &= ~((1<<8)|(1<<9));			// Clears pins PC8 and PC9 to low speed
	GPIOC->PUPDR &= ~(0x000F0000);					// Clears bits of PC8 and PC9 to no pull-up, pull-down
	GPIOC->ODR |= (1<<6)|(1<<8);
	
	//HAL_RCC_GetHCLKFreq();
	SysTick_Config(HAL_RCC_GetHCLKFreq()/1000);	// Set interrupt freq to 1kHz @ 8MHz
	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn,0);
	GPIOA->MODER &= ~(1|(1<<1));					// Sets pin PA0 to input mode
	GPIOA->OSPEEDR &= ~(1|(1<<1));				// Clears pins for PA0 to low speed
	GPIOA->PUPDR |= 0x11;									// Sets bits for PA0 to 11
	GPIOA->PUPDR &= ~(1);									// Clears lowest bit for PA0; sets pull-down for PA0
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;	// Enable SysCFG
	SYSCFG->CFGR1 &= SYSCFG_EXTICR1_EXTI0_PA;	// Sets PA0 button
	EXTI->IMR = 0x0001;										// Unmask EXTI0
	EXTI->RTSR = 0x0001;									// Set up rising edge trigger
	NVIC_EnableIRQ(EXTI0_1_IRQn);					// Enable lines 0 and 1
	NVIC_SetPriority(EXTI0_1_IRQn,1);			// Set priority of interrupt to desired value

  while (1)
  {
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
