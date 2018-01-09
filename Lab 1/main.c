/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
	* Devin Renshaw				: GPIO
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f072xb.h"
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

int main(void) {
HAL_Init(); // Reset of all peripherals, init the Flash and Systick
SystemClock_Config(); //Configure the system clock
/* This example uses HAL library calls to control
the GPIOC peripheral. You'll be redoing this code
with hardware register access. */
//__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	// Use header file to determine register that enables the GPIOC peripheral clock
		// -> #define RCC_AHBENR_GPIOCEN                       RCC_AHBENR_GPIOCEN_Msk        /*!< GPIOC clock enable */
	// Set appropriate bit using bitwise operations and either a bitmask or defined bit name
		// -> #define RCC_AHBENR_GPIOCEN_Msk                   (0x1U << RCC_AHBENR_GPIOCEN_Pos) /*!< 0x00080000 */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;		// GPIOC peripheral clock enable
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;		// GPIOA peripheral clock enable
// Set up a configuration struct to pass to the initialization function
/*GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
GPIO_MODE_OUTPUT_PP,
GPIO_SPEED_FREQ_LOW,
GPIO_NOPULL};
	
HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
*/
	// pin pc8 -> pins 16-17 - green LED
	// pin pc9 -> pins 18-19 - orange LED
	// general purpose output mode: 01
	// set pc8 and pc9 to general purpose output mode in MODER register
	/*GPIOC->MODER |= (1<<16)|(1<<18);			// Set pins PC8 and PC9 to output mode
	GPIOC->OTYPER &= ~((1<<8)|(1<<9));			// Clears bits 8 & 9 to enable push-pull
	GPIOC->OSPEEDR &= ~((1<<8)|(1<<9));			// Clears pins PC8 and PC9 to low speed
	GPIOC->PUPDR &= ~(0x000F0000);					// Clears bits of PC8 and PC9 to no pull-up, pull-down
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high
	GPIOC->ODR |= 1<<8;*/
	
	GPIOC->MODER |= (1<<12)|(1<<14);				// Set pins PC8 and PC9 to output mode
	GPIOC->OTYPER &= ~((1<<6)|(1<<7));			// Clears bits 8 & 9 to enable push-pull
	GPIOC->OSPEEDR &= ~((1<<6)|(1<<7));			// Clears pins PC8 and PC9 to low speed
	GPIOC->PUPDR &= ~(0x0000F000);					// Clears bits of PC8 and PC9 to no pull-up, pull-down
	GPIOC->ODR |= 1<<6;

	GPIOA->MODER &= ~(1|(1<<1));						// Sets pin PA0 to input mode
	GPIOA->OSPEEDR &= ~(1|(1<<1));					// Clears pins for PA0 to low speed
	GPIOA->PUPDR |= 0x11;										// Sets bits for PA0 to 11
	GPIOA->PUPDR &= ~(1);										// Clears lowest bit for PA0; sets pull-down for PA0
	
	uint32_t debouncer = 0;
while (1) {
//HAL_Delay(200); // Delay 200ms
	debouncer = (debouncer << 1); // Always shift every loop iteration
	if ((GPIOA->IDR)&(0x1)) { // If input signal is set/high
		debouncer |= 0x01; // Set lowest bit of bit-vector
	}
	if (debouncer == 0xFFFFFFFF) {
		//debouncer = 0;
	}
	if (debouncer == 0x00000000) {
		//HAL_Delay(50);
	}
	if (debouncer == 0x7FFFFFFF) {
	// This code triggers only once when transitioning to steady high!
		GPIOC->ODR ^= (1<<6)|(1<<7);
	}
	// Toggle the output state of both PC8 and PC9
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
		//GPIOC->ODR ^= (1<<8)|(1<<9);
		//GPIOC->ODR ^= (1<<6)|(1<<7);
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
