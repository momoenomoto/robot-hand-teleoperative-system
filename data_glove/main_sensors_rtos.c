#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "stm32f7xx.h"                  // Device header
#include "Open746i_lcd.h"
#include "Driver_USART.h"

#define UART_ITEM_COUNT 10
#define BUFFER_SIZE 5

void SystemClock_Config(void);
extern ARM_DRIVER_USART Driver_USART1;
int map(double x, double in_min, double in_max, double out_min, double out_max);
void TIM5_IRQHandler(void);

static uint16_t buffer[BUFFER_SIZE] = {0};
static double vout;

osEventFlagsId_t evt_id;
osThreadId_t thrd_id1, thrd_id2, thrd_id3;

// const osThreadAttr_t thrd1_attr = {
//   .priority  = osPriorityHigh,
// };
// const osThreadAttr_t thrd2_attr = {
//   .priority  = osPriorityLow,
// };

void TIM5_IRQHandler(void){ 
	TIM5->SR = ~(1 << 0);
	ADC1->CR2 |= (1 << 30);
	while(!(ADC1->SR & (1<<1))); // wait while conversion not complete
	vout = (ADC1->DR * 5.0) / 4096;
	if (vout > 4095)
		vout = 4095;
	//buffer[0] = vout; 
	osEventFlagsSet(evt_id, 3);
}

__NO_RETURN void ADC_thread(void *argument)
{
	RCC->AHB1ENR |= (1 << 0); // Activate GPIOA
	RCC->APB1ENR |= (1 << 3); //Activate timer 5
	RCC->APB2ENR |= (1 << 8); // Activate ADC1
	GPIOA->MODER |= (3 << 2);  // PA1 for analog mode

	// ADC1_IN1
	ADC1->SQR3 |= (1 << 0);
	ADC1->CR2 |= (1 << 0); // activates output channel 1 

	// 180Hz timer
	TIM5->PSC = 9;
	TIM5->ARR = 59999;
	// 108M/(9*60000) = 180Hz
	TIM5->CR1 |= (1 << 7) | (1 << 0); // enable counter and arpe bit
	TIM5->DIER |= (1 << 0); // enable update interrupt

	NVIC_EnableIRQ(TIM5_IRQn);

	while(1)
	{
		osThreadYield();
	};
}

__NO_RETURN void UART_thread(void *argument)
{
	// initialize USART
	Driver_USART1.Initialize(UART_handler);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS | 
												ARM_USART_DATA_BITS_8 | 
												ARM_USART_PARITY_NONE |
												ARM_USART_STOP_BITS_1 |
												ARM_USART_FLOW_CONTROL_NONE, 9600);
	Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);
	
	while(1)
	{
		//osEventFlagsWait(evt_id, 1, osFlagsWaitAny, osWaitForever);
		//while (Driver_USART1.GetStatus().tx_busy == 1);
		//Driver_USART1.Send(buffer, UART_ITEM_COUNT);
		osThreadYield();
	}
}

__NO_RETURN void LCD_thread(void *argument)
{
	char string[50];

	while(1)
	{
		osEventFlagsWait(evt_id, 2, osFlagsWaitAny, osWaitForever);
		sprintf(string, "%.3fV", vout);
		BSP_LCD_DisplayStringAt(0, 300, string, CENTER_MODE);
		//sprintf(string, "0x%04x%04x%04x%04x%04x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		//BSP_LCD_DisplayStringAt(500, 300, string, CENTER_MODE);
	}
}

int map(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void)
{
	SystemClock_Config();

	BSP_LCD_Init();
	BSP_LCD_Clear(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(0, 0, (unsigned char *) "Test LCD", CENTER_MODE);
	
	osKernelInitialize();
	evt_id = osEventFlagsNew(NULL);
	//thrd_id1 = osThreadNew(UART_thread, NULL, NULL);
	thrd_id2 = osThreadNew(LCD_thread, NULL, NULL);
	thrd_id3 = osThreadNew(ADC_thread, NULL, NULL);
	
	osKernelStart();

	while(1); // reached only if there's an error

	return 0;
}

// void SysTick_Handler(void)
// {
// 	HAL_IncTick();
// }

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  HAL_PWREx_EnableOverDrive();
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
}

