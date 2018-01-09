/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
	* Devin Renshaw			 : I2C Interface
  ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

//static int counter = 0;
static int DATA1;
static int DATA2;
static int x;
static int y;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/
void pin_off(int OFFSET);
void pin_on(int OFFSET);
void pin_toggle(int OFFSET);
void all_on(void);
void all_off(void);

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
	
	
  /* Initialize all configured peripherals */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;		// GPIOB peripheral clock enable
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;		// GPIOC peripheral clock enable
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;		// Enable I2C2 peripheral clock
	
	/* PB11 to alternate function mode; open-drain output; I2C2_SDA mode */
	GPIOB->MODER |= (1<<23);		
	GPIOB->MODER &= ~(1<<22);			// PB11 to alternate mode
	GPIOB->OTYPER |= (1<<11);			// PB11 to open drain
	GPIOB->AFR[1] |= (1<<12);			// PB11 to alternate function 1: I2C2_SDA
	
	/* PB13 to alternate function mode; open-drain output; I2C2_SCL mode */
	GPIOB->MODER |= (1<<27);			// Sets bit to one
	GPIOB->MODER &= ~(1<<26);			// Clears bit; PB13 to alternate mode
	GPIOB->OTYPER |= (1<<13);			// PB13 to open drain
	GPIOB->AFR[1] |= (1<<20)|(1<<22);			// PB13 to alternate function 5: I2C2_SCL
	
	/* PB14 to output mode (01); push-pull output type; set pin HI */
	GPIOB->MODER |= (1<<28);			// Sets bit to one
	GPIOB->MODER &= ~(1<<29);			// Clears bit; PB14 to output mode
	GPIOB->OTYPER &= ~(1<<14);		// PB14 to push-pull output type
	GPIOB->ODR |= (1<<14);				// Set PB14 output HI
	
	/* Enable LEDs */
	GPIOC->MODER |= (1<<12)|(1<<14)|(1<<16)|(1<<18);
	GPIOC->OSPEEDR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));
	GPIOC->OTYPER &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));
	GPIOC->PUPDR |= 0x000FF000;
	
	/* PC0 to output mode (01); push-pull output type; set pin HI */
	GPIOC->MODER |= (1<<0);				// Sets bit to one
	GPIOC->MODER &= ~(1<<1);			// Clears bit; PC0 to output mode
	GPIOC->OTYPER &= ~(1<<0);			// PB14 to push-pull output type
	GPIOC->ODR |= (1<<0);					// Set PB14 output HI
	
	/*HAL_Delay(600);
	all_on();
	HAL_Delay(1000);
	all_off();*/
	
	/* I2C2 Enable and Setup */
	/* Set Prescaler */
	I2C2->TIMINGR |= (1<<28);			// Set prescaler bit to 1
	I2C2->TIMINGR &= ~(0xE);			// Clears other bits of prescaler; set I2C2 timing register to standard 100kHz
	
	I2C2->TIMINGR |= (0x13);			// Set SCLL to 0x13
	I2C2->TIMINGR |= (0x0F00);		// Set SCLH to 0xF	
	/* Set SDADEL to 0x2 */
	I2C2->TIMINGR |= (0x2<<16);			// set SDADEL to 0x2
	/* Set SCLDEL to 0x4 */
	I2C2->TIMINGR |= (1<<22);			// set SCLDEL to 0x4
	
	/* Enable I2C peripheral using PE bit in CR1 */
	I2C2->CR1 |= I2C_CR1_PE/*(1<<0)*/;					// Peripheral enabled
	
	/* Set L3GD20 slave address (SADD) 0b110101 */
	/* Get ready to write */
	I2C2->CR2 = 0;	// Reset bits for NBYTES and SADD
	I2C2->CR2 |= (0x6B<<1);
	/* Set number of bytes to tx to 1 and transfer direction to write */
	I2C2->CR2 |= (1<<16);					// NBYTES[7:0] (CR2[23:16] = 1)
	I2C2->CR2 &= ~(1<<10);				// Set to tx mode; RD_WRN = 0 for write
	
	/* Set START bit to start generation */
	I2C2->CR2 |= I2C_CR2_START/*(1<<13)*/;
	while (!((I2C2->ISR & I2C_ISR_TXIS)|(I2C2->ISR & I2C_ISR_NACKF)))
	{
	}
	// Continue if TXIS set
	while (I2C2->ISR & I2C_ISR_NACKF)
	{
		pin_on(6);
	}
	I2C2->TXDR = 0x0F;	// Send WHO_AM_I register address
	while (!(I2C2->ISR & I2C_ISR_TC))	// While transfer not complete, loop
	{
	}
	/* Get ready to receive */
	I2C2->CR2 = 0;							// Reset bits for NBYTES and SADD
	I2C2->CR2 |= (1<<10);				// Set to rx mode; RD_WRN = 1 for read
	I2C2->CR2 |= (0x6B<<1);
	I2C2->CR2 |= (1<<16);				// NBYTES[7:0] (CR2[23:16]
	I2C2->CR2 |= I2C_CR2_START;	// Reset START bit
	while(!((I2C2->ISR & I2C_ISR_RXNE)|(I2C2->ISR & I2C_ISR_NACKF)))
	{
	}
	// Continue if RXNE set
	while (I2C2->ISR & I2C_ISR_NACKF)
	{
		pin_on(8);
	}
	DATA1 = I2C2->RXDR;	// Saves rx register for match
	// Check for match of 0xD4 (WHO_AM_I register)
	while (DATA1 != 0xD4)		// Check for data match of 0xD4
	{
		pin_on(7);
	}
	while (!(I2C2->ISR & I2C_ISR_TC))
	{
	}
	
	I2C2->CR2 |= I2C_CR2_STOP;	// Set stop bit and release bus

	/**** Set up CTRL_REG1 ****/
	/* Set L3GD20 slave address (SADD) 0b110101 */
	I2C2->CR2 = 0;	// Reset bits for NBYTES and SADD
	I2C2->CR2 |= (0x6B<<1);			// Set address of gyro
	/* Set number of bytes to tx to 2 and transfer direction to write */
	I2C2->CR2 |= (2<<16);				// NBYTES[7:0] (CR2[23:16] = 2)
	I2C2->CR2 &= ~(1<<10);			// Set to tx mode; RD_WRN = 0 for write
	/* Set START bit to start generation */
	I2C2->CR2 |= I2C_CR2_START/*(1<<13)*/;
	while (!((I2C2->ISR & I2C_ISR_TXIS)|(I2C2->ISR & I2C_ISR_NACKF)))
	{
	}
	// Continue if TXIS set
	while (I2C2->ISR & I2C_ISR_NACKF)
	{
		pin_on(9);
	}
	I2C2->TXDR = 0x20;	// Send CTRL_REG1 register
	
	I2C2->CR2 |= I2C_CR2_START;	// Reset START bit
	while (!((I2C2->ISR & I2C_ISR_TXIS)|(I2C2->ISR & I2C_ISR_NACKF)))
	{
	}
	// Continue if TXIS set
	while (I2C2->ISR & I2C_ISR_NACKF)
	{
		pin_on(9);
	}
	I2C2->TXDR = 0xB;	// CTRL_REG1 with x, y axes enabled, normal mode -> 0b01011
	pin_on(7);
	while (!((I2C2->ISR & (I2C_ISR_TC|I2C_ISR_NACKF))))	// While transfer not complete, loop
	{	
	}
	while (I2C2->ISR & I2C_ISR_NACKF)
	{
		pin_on(6);
	}
	//HAL_Delay(1);
	
	I2C2->CR2 |= I2C_CR2_STOP;	// Set stop bit and release bus
	pin_on(7);
	/* Get ready to receive */
	/*I2C2->CR2 = 0;							// Reset bits for NBYTES and SADD
	I2C2->CR2 |= (1<<10);				// Set to rx mode; RD_WRN = 1 for read
	I2C2->CR2 |= (0x6B<<1);
	I2C2->CR2 |= (1<<16);				// NBYTES[7:0] (CR2[23:16]
	I2C2->CR2 |= I2C_CR2_START;	// Reset START bit
	while(!((I2C2->ISR & I2C_ISR_RXNE)|(I2C2->ISR & I2C_ISR_NACKF)))
	{
		pin_on(6);
	}
	// Continue if RXNE set
	while (I2C2->ISR & I2C_ISR_NACKF)
	{
		pin_on(8);
	}
	DATA2 = I2C2->RXDR;	// Saves rx register for match
	y = DATA2;//REMOVE!!!!!!
	while (!(I2C2->ISR & I2C_ISR_TC))	// While transfer not complete, loop
	{
	}
	I2C2->CR2 |= I2C_CR2_STOP;	// Set stop bit and release bus
	*/
  while (1)
  {
		x = 0;
		y = 0;
		/* Get gyroscope data */
		I2C2->CR2 = 0;	// Reset bits for NBYTES and SADD
		I2C2->CR2 |= (0x6B<<1);			// Set address of gyro
		/* Set number of bytes to tx to 1 and transfer direction to write */
		I2C2->CR2 |= (1<<16);				// NBYTES[7:0] (CR2[23:16] = 1)
		I2C2->CR2 &= ~(1<<10);			// Set to tx mode; RD_WRN = 0 for write
		/* Set START bit to start generation */
		I2C2->CR2 |= I2C_CR2_START/*(1<<13)*/;
		while (!((I2C2->ISR & I2C_ISR_TXIS)|(I2C2->ISR & I2C_ISR_NACKF)))
		{
		}
		// Continue if TXIS set
		while (I2C2->ISR & I2C_ISR_NACKF)
		{
			pin_on(9);
		}
		I2C2->TXDR = 0xA8;	// Send 0xA8 to read both high and low of OUT_X
		while (!(I2C2->ISR & I2C_ISR_TC))	// While transfer not complete, loop
		{
		}
		I2C2->CR2 |= I2C_CR2_STOP;	// Set stop bit and release bus
		/* Receive OUT_X_L */
		I2C2->CR2 = 0;	// Reset bits for NBYTES and SADD
		I2C2->CR2 |= (0x6B<<1);			// Set address of gyro
		/* Set number of bytes to tx to 4 and transfer direction to write */
		I2C2->CR2 |= (2<<16);				// NBYTES[7:0] (CR2[23:16] = 4)
		I2C2->CR2 |= (1<<10);				// Set to rx mode; RD_WRN = 1 for read
		/* Set START bit to start generation */
		I2C2->CR2 |= I2C_CR2_START/*(1<<13)*/;
		while (!((I2C2->ISR & I2C_ISR_RXNE)|(I2C2->ISR & I2C_ISR_NACKF)))
		{
		}
		while (I2C2->ISR & I2C_ISR_NACKF)
		{
			pin_on(8);
		}
		x = I2C2->RXDR;	// Save lower bits of OUT_X
		I2C2->CR2 |= I2C_CR2_START/*(1<<13)*/;
		while (!((I2C2->ISR & I2C_ISR_RXNE)|(I2C2->ISR & I2C_ISR_NACKF)))
		{
		}
		while (I2C2->ISR & I2C_ISR_NACKF)
		{
			pin_on(8);
		}
		x |= I2C2->RXDR << 8;;	// Save upper bits of OUT_X
		while (!(I2C2->ISR & I2C_ISR_TC))	// While transfer not complete, loop
		{
		}
		//I2C2->CR2 |= I2C_CR2_STOP;	// Set stop bit and release bus
		
		HAL_Delay(11);		// @95 Hz, period is 0.0153 s
	}
}
/********* End of While **********/
void pin_off(int OFFSET)
{
	GPIOC->ODR &= ~(1<<OFFSET);
}

void pin_on(int OFFSET)
{
	GPIOC->ODR |= (1<<OFFSET);
}

void pin_toggle(int OFFSET)
{
	GPIOC->ODR ^= (1<<OFFSET);
}

void all_on(void)
{
	GPIOC->ODR |= (1<<6)|(1<<7)|(1<<8)|(1<<9);
}

void all_off(void)
{
	GPIOC->ODR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));
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
