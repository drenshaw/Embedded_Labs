/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/*static char c;
static int data = -1;
static int new_data = 0;
static int command = 0;*/
volatile uint8_t buffer[20];
volatile uint8_t count = 0;
const char RED[3] = "red";
const char BLUE[4] = "blue";
const char ORANGE[6] = "orange";
const char GREEN[5] = "green";
const char ON[2] = "on";
const char OFF[3] = "off";
const char TOGGLE[6] = "toggle";

static int PIN = 0;

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

void toggle_red(void);
void toggle_orange(void);
void toggle_green(void);
void toggle_blue(void);
void tx_string(char str[]);
void print_success(char color, char option);
void pin_off(int OFFSET);
void pin_on(int OFFSET);
void pin_toggle(int OFFSET);
void set_pins(char ch, int n);
void string_check(void);
void USART3_4_IRQHandler();
void tx_char(char ch);
void rx_char();

/****************************MAIN*********************************/
int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
	NVIC_EnableIRQ(USART3_4_IRQn);
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;		// GPIOC peripheral clock enable
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;	// Enable USART3 clock
	GPIOC->MODER |= (1<<9)|(1<<11);			// Set pins PC4 and PC5 to alternate mode
	GPIOC->AFR[0] |= 0x00110000;	// Select AF1 on PC4 and PC5 in AFRL
	int clock = HAL_RCC_GetHCLKFreq();		// Get clock frequency
	int baud = 115200;										// Desired baud rate
	int usartdiv1 = clock/baud;						// Calculate baud rate register value: ~69
	USART3->BRR = usartdiv1;							// Store value into baud rate register
	USART3->ISR |= 
	/* Enable tx, rx, RXNE interrupt on USART3 */
	USART3->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;
	//USART3->CR1 |= USART_CR1_UE;					/* Enable USART3 */
	//USART3->CR1 |= USART_CR1_TE | USART_CR1_UE;
	/* Enable LEDs */
	GPIOC->MODER |= (1<<12)|(1<<14)|(1<<16)|(1<<18);
	GPIOC->OSPEEDR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));
	GPIOC->OTYPER &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));
	GPIOC->PUPDR |= 0x000FF000;
	//GPIOC->ODR |= 1<<6;
  while (1)
  {
		//tx_char('a');
		/*char str[] = "string";
		for(int i = 0; str[i] != '\0'; ++i)
		{
			tx_char(str[i]);
			HAL_Delay(200);
		}
		HAL_Delay(1000);*/
		//rx_char();
		//get_data();
		
		/*if (new_data == 2)
		{
			set_pins(c,data);
			new_data = 0;
		}*/
		/* POSTLAB */
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

void toggle_red()
{
	GPIOC->ODR ^= 1<<6;
}
void toggle_orange()
{
	GPIOC->ODR ^= (1<<8);
}
void toggle_green()
{
	GPIOC->ODR ^= (1<<9);
}
void toggle_blue()
{
	GPIOC->ODR ^= (1<<7);
}
void tx_string(char str[])
{
	for (int i=0; str[i] != '\0'; ++i)
	{
		while (!(USART3->ISR & USART_ISR_TXE)) {}
		USART3->TDR = str[i];
	}
}

void print_success(char color, char option)
{
	if (color == 0x72)
		tx_string("RED");
	else if (color == 0x6F)
		tx_string("ORANGE");
	else if (color == 0x67)
		tx_string("GREEN");
	else 	//(color == 0x62)
		tx_string("BLUE");
	if (option == 0x30)
		tx_string(" OFF");
	else if (option == 0x31)
		tx_string(" ON");
	else
		tx_string(" TOGGLE");
	tx_string("\n");
}

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

void set_pins(char ch, int n)
{
	int OFFSET;
	if (ch == 0x72)				// RED
		OFFSET = 6;
	else if (ch == 0x6F)	// ORANGE
		OFFSET = 8;
	else if (ch == 0x67)	// GREEN
		OFFSET = 9;
	else		//ch == 0x62	// BLUE
		OFFSET = 7;
	if (n == 0x30)	// OFF
		pin_off(OFFSET);
	else if (n == 0x31)
		pin_on(OFFSET);
	else if (n == 0x32)
		pin_toggle(OFFSET);
	else
	{
		tx_string("ERROR");
		return;
	}
	print_success(ch,n);

}

void clear_buffer()
{
	for (int i = 0; i < 20; i++)
		buffer[i] = NULL;
	count = 0;
}

void string_check()
{
	count = 0;
	while (buffer[count] != NULL)
	{
		if ((buffer[count] != RED[count])		&&	(buffer[count] != BLUE[count])	&&
				(buffer[count] != ORANGE[count])&&	(buffer[count] != GREEN[count])	&&
				(buffer[count] != ON[count])		&&	(buffer[count] != OFF[count])	&&	(buffer[count] != TOGGLE[count]))
		{
			tx_string("ERROR");
			clear_buffer();
			return;
		}
		else
			count++;
	}
	/* This part chooses a unique character from each of the words in the allowed set */
	if (buffer[0] == 'r')
		PIN = 6;
	else if (buffer[2] == 'a')
		PIN = 8;
	else if (buffer[0] == 'g')
		PIN = 9;
	else if (buffer[0] == 'b')
		PIN = 7;
	else if (buffer[1] == 'f')
		pin_off(PIN);
	else if (buffer[1] == 'n')
		pin_on(PIN);
	else if (buffer[0] == 't')
		pin_toggle(PIN);
	clear_buffer();
}

void USART3_4_IRQHandler()
{
	/*if (new_data == 0)
	{
		c = USART3->RDR;
		if ((c == 0x72)|(c == 0x6F)|(c == 0x67)|(c == 0x62))
			new_data = 1;
		else
			tx_string("ERROR");
	}
	else if (new_data == 1)
	{
		data = USART3->RDR;
		if ((data == 0x30)|(data == 0x31)|(data == 0x32))
			new_data = 2;
		else
		{
			new_data = 0;
			tx_string("ERROR");
		}
	}*/
	/*if (new_data == 0 && command == 0)
		c = USART3->RDR;
	*/
	/* POSTLAB */
	if (count > 20)// You've gone too far without a newline character
	{
		buffer[0] = NULL;
		count = 0;
		return;
	}
	char ch = USART3->RDR;
	if (ch != 0x0D)	// Not newline character
	{
		if (ch == 0x7F)// Backspace character
		{
			if (count > 0)
			{
				count--;
				buffer[count] = NULL;
			}
			else
				return;
		}
		else
		{
			buffer[count] = ch;
			count++;
		}
	}
	else if (ch == 0x0D)
	{
		buffer[count] = NULL;
		count = 0;
		string_check();
	}
		
}

void tx_char(char ch)
{
	while(!(USART3->ISR & USART_ISR_TXE))	{}	/* tx reg bit not set; exits when tx bit set */
	USART3->TDR = ch;
}

void rx_char()
{
	while(!(USART3->ISR & USART_ISR_RXNE))	{}	/* loops on rx register (not not) empty*/
	{
	}
	char ch = USART3->RDR;
	if (ch == 0x72)
		toggle_red();
	else if (ch == 0x6F)
		toggle_orange();
	else if (ch == 0x67)
		toggle_green();
	else if (ch == 0x62)
		toggle_blue();

	else
	{
		char str[] = "ERROR";
		tx_string(str);
	}
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
