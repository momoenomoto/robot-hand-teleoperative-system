//#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "stm32f7xx.h"                  // Device header
#include "Open746i_lcd.h"
#include "Driver_USART.h"
#include <stdio.h>

#define UART_ITEM_COUNT 10
#define BUFFER_SIZE 5

void SysTick_Handler(void);
void SystemClock_Config(void);
extern ARM_DRIVER_USART Driver_USART1;
void TIM5_IRQHandler(void);

static unsigned short buffer[BUFFER_SIZE] = {0};
static double vout;
static char string[50];

void TIM5_IRQHandler(void){ 
	TIM5->SR = ~(1 << 0);
	ADC1->CR2 |= (1 << 30);
	while(!(ADC1->SR & (1<<1))); // wait while conversion not complete
	vout = (ADC1->DR * 5.0) / 4096;
	if (vout > 4095)
		vout = 4095;
	sprintf(string, "%.3fV", vout);
	BSP_LCD_DisplayStringAt(0, 300, string, CENTER_MODE);
	//buffer[0] = vout; 
	//sprintf(string, "%04u", buffer[0]);
	//BSP_LCD_DisplayStringAt(0, 300, string, CENTER_MODE);
	//while (Driver_USART1.GetStatus().tx_busy == 1);
	//Driver_USART1.Send(buffer, UART_ITEM_COUNT);
}


int main(void)
{
	SystemClock_Config();

	RCC->AHB1ENR |= (1 << 0); // Activate GPIOA
	RCC->APB1ENR |= (1 << 3); //Activate timer 5
	RCC->APB2ENR |= (1 << 8); // Activate ADC1
	GPIOA->MODER |= (3 << 2);  // PA1 for analog mode

	// ADC1_IN1
	ADC1->SQR3 |= (1 << 0);
	ADC1->CR2 |= (1 << 0); // activates output channel 1 

	// 180Hz timer
	TIM5->PSC=179;
	TIM5->ARR = 59999; // 1 Hz
	//TIM5->PSC = 9;
	//TIM5->ARR = 59999;
	// 108M/(9*60000) = 180Hz
	TIM5->CR1 |= (1 << 7) | (1 << 0); // enable counter and arpe bit
	TIM5->DIER |= (1 << 0); // enable update interrupt

	NVIC_EnableIRQ(TIM5_IRQn);
	
	BSP_LCD_Init();
	BSP_LCD_Clear(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(0, 0, (unsigned char *) "Test LCD", CENTER_MODE);

	// initialize USART
	/*
	Driver_USART1.Initialize(NULL);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS | 
												ARM_USART_DATA_BITS_8 | 
												ARM_USART_PARITY_NONE |
												ARM_USART_STOP_BITS_1 |
												ARM_USART_FLOW_CONTROL_NONE, 9600);
	Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);*/
	
	// osKernelInitialize();
	// evt_id = osEventFlagsNew(NULL);
	// q_id = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(MSGQUEUE_OBJ_t), NULL);
	// thrd_id1 = osThreadNew(UART_thread, NULL, &thrd1_attr);
	// thrd_id2 = osThreadNew(LCD_thread, NULL, &thrd2_attr);
	// thrd_id3 = osThreadNew(SERVO_thread, NULL, NULL);
	
	// osKernelStart();


	while(1);

	return 0;
}


void SysTick_Handler(void)
{
	HAL_IncTick();
}

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

