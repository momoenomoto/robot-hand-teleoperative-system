#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "stm32f7xx.h"                  // Device header
#include "Open746i_lcd.h"
#include "Driver_USART.h"
#include <stdio.h>
#include <stdlib.h>

#define NUM_SERVO 5
#define IN_MIN 1000
#define IN_MAX 4095
#define OUT_MIN 500
#define OUT_MAX 2500
#define THRESHOLD 100

void SystemClock_Config(void);
extern ARM_DRIVER_USART Driver_USART1;
uint16_t map(double x, double in_min, double in_max, double out_min, double out_max);
void TIM5_IRQHandler(void);
void LCD_thread(void *argument) __NO_RETURN;
void UART_thread(void *argument) __NO_RETURN;
void TIM_thread(void *argument) __NO_RETURN;
void ADCDMA_thread(void *argument) __NO_RETURN;
void UART_handler(uint32_t event);
void DMA2_Stream4_IRQHandler(void);

static volatile uint16_t adc_vals[NUM_SERVO] = {0};
static unsigned short uart_buffer[NUM_SERVO] = {0};
static unsigned short old_uart_buffer[NUM_SERVO] = {0};
static double voltages[NUM_SERVO] = {0};

uint16_t num;

osEventFlagsId_t evt_id;
osThreadId_t thrd_id1, thrd_id2, thrd_id3, thrd_id4;

const osThreadAttr_t thrd1_attr = {
  .priority  = osPriorityHigh,
};
const osThreadAttr_t thrd2_attr = {
	.priority  = osPriorityLow,
};

void DMA2_Stream4_IRQHandler(void)
{
	if (DMA2->HISR & (1<<5)) // TCIF0
	{
		DMA2->HIFCR |= (1<<5); // set CTCIF
		osEventFlagsSet(evt_id, 8); 
	}
	if (DMA2->HISR & (1<<3))
	{
		DMA2->HIFCR |= (1<<3);
	}
}

__NO_RETURN void ADCDMA_thread(void *argument)
{
	int i;
	double vout;
	uint16_t servo_val_mapped_rounded;
	
	RCC->AHB1ENR |= (1 << 22); // Activate DMA2
	RCC->AHB1ENR |= (1 << 0); // Activate GPIOA
	RCC->APB2ENR |= (1 << 8); // Activate ADC1
	GPIOA->MODER |= ((3 << 2) | (3 << 4) | (3 << 8) | (3 << 10) | (3 << 12));  // PA1 , PA2, PA4, PA5, PA6 for analog mode

	ADC1->SQR1 |= (4 << 20); // 5 channels to convert
	// ADC1_IN1, ADC1_IN2, ADC1_IN4, ADC1_IN5, ADC1_IN6
	ADC1->SQR3 |= ((1 << 0)| (2 << 5) | (4<<10) | (5<<15) | (6<<20)); // channel numbers for sequence
	ADC1->CR1 |= (1<<8); // enable scan mode
	ADC1->CR2 &= ~(1<<1); // cont mode off

	// no need to generate ADC interrupts anymore
	//ADC1->CR1 |= (1 << 5); // EOCIE bit set, generate interrupt
	
	//ADC1->CR1 &= ~(1<<24); // 12 bit adc
	//ADC1->CR2 |= (1<<10); // EOC after each conversion
	//ADC1->CR2 &= ~(1<<11); // uart_buffer alignment right
	//ADC1->CR2 |= (1<<9); // continuous DMA
	
	//ADC1->SMPR2 |= (3<<0) | (3<<3);
	//NVIC_SetPriority(ADC_IRQn, 0);
	//NVIC_EnableIRQ(ADC_IRQn);
	//ADC1->CR2 |= (1 << 0); // enable ADC
	
	DMA2_Stream4->CR &= ~(4<<25); // channel 0 selected for stream
	
	DMA2_Stream4->CR &= ~(3<<6); // uart_buffer direction peripheral to memory
	DMA2_Stream4->CR &= ~(3<<9); // fixed peripheral address pointer
	DMA2_Stream4->CR |= ((1<<10) | (1<<11) |(1<<13)| (2<<16)); // memory address increment, 16 bit mem uart_buffer, 16 bit peripheral uart_buffer, high priority 
	DMA2_Stream4->CR &= ~(1<<9); // no peripheral address inc
	DMA2_Stream4->CR |= (1<<8); // circ mode enabled

	DMA2_Stream4->NDTR = 5; // 5 channels for dma transfer size
	DMA2_Stream4->PAR = (uint32_t) &(ADC1->DR); // source address
	DMA2_Stream4->M0AR = (uint32_t) &adc_vals[0]; // destination address
	
	//NVIC_SetPriority(DMA2_Stream4_IRQn, 1); 
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	
	DMA2_Stream4->CR |= ((1<<4) | (1<<2));  // enable transfer complete interrupt and transfer error interrupt 
	DMA2_Stream4->CR |= (1<<0); // enable DMA stream

	ADC1->CR2 |= (1 << 0); // enable ADC

	while (1)
	{
		osEventFlagsWait(evt_id, 8, osFlagsWaitAll, osWaitForever);
		for (i = 0; i < NUM_SERVO; i++)
		{
			vout = (adc_vals[i] * 3.3) / 4096;
			if (vout > 4095) vout = 4095;
			voltages[i] = vout;
			servo_val_mapped_rounded = map(adc_vals[i], IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
			if (abs(servo_val_mapped_rounded - old_uart_buffer[i]) > THRESHOLD)
			{
				uart_buffer[i] = servo_val_mapped_rounded;
				old_uart_buffer[i] = uart_buffer[i];
			} else {
				uart_buffer[i] = 0;
			}
		}
		osEventFlagsSet(evt_id, 3);
	}
	
}


void TIM5_IRQHandler(void){ 
	TIM5->SR = ~(1 << 0);
	// reset DMA every time or else doesn't work
	ADC1->CR2 &= ~(1<<8); // reset DMA
	ADC1->CR2 |= (1 << 8); //enables DMA 
	ADC1->CR2 |= (1 << 30); // start conversion
}

__NO_RETURN void TIM_thread(void *argument)
{
	RCC->AHB1ENR |= (1 << 0); // Activate GPIOA
	RCC->APB1ENR |= (1 << 3); //Activate timer 5

	// 108MHz clk
	TIM5->PSC = 59; // 15Hz
	TIM5->ARR = 59999; 
	//TIM5->PSC = 9;
	//TIM5->ARR = 59999;
	// 108M/(9*60000) = 180Hz
	TIM5->CR1 |= (1 << 7) | (1 << 0); // enable counter and arpe bit
	TIM5->DIER |= (1 << 0); // enable update interrupt

	NVIC_EnableIRQ(TIM5_IRQn);

	while(1)
	{
		osThreadExit();
	}
}

void UART_handler(uint32_t event)
{
	if (event & ARM_USART_EVENT_SEND_COMPLETE)
	{
		osEventFlagsSet(evt_id, 4);
	}
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
		osEventFlagsWait(evt_id, 1, osFlagsWaitAll, osWaitForever);
		Driver_USART1.Send(uart_buffer, NUM_SERVO * 2);
		osEventFlagsWait(evt_id, 4, osFlagsWaitAll, osWaitForever);
		
	}
}

__NO_RETURN void LCD_thread(void *argument)
{
	char string[50], string2[50], string3[50], string4[50];
	
	BSP_LCD_Init();
	BSP_LCD_Clear(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(0, 0, (unsigned char *) "Test LCD", CENTER_MODE);

	while(1)
	{
		osEventFlagsWait(evt_id, 2, osFlagsWaitAny, osWaitForever);
		sprintf(string, "%.3fV %.3fV %.3fV %.3fV %.3fV", voltages[0], voltages[1], voltages[2], voltages[3], voltages[4]);
		BSP_LCD_DisplayStringAt(0, 100, (uint8_t *) string, CENTER_MODE);
		sprintf(string2, "0x%04x%04x%04x%04x%04x", adc_vals[0], adc_vals[1], adc_vals[2], adc_vals[3], adc_vals[4]);
		BSP_LCD_DisplayStringAt(0, 300, (uint8_t *) string2, CENTER_MODE);
		sprintf(string3, "sent: %04d %04d %04d %04d %04d", uart_buffer[0], uart_buffer[1], uart_buffer[2], uart_buffer[3], uart_buffer[4]);
		BSP_LCD_DisplayStringAt(0, 500, (uint8_t *) string3, CENTER_MODE);
	}
}

uint16_t map(double x, double in_min, double in_max, double out_min, double out_max)
{
	num = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	if (num < 500) num = 500;
	if (num > 2500) num = 2500;
	num /= 100; // take out last digit
	return  num * 100;
}


int main(void)
{
	SystemClock_Config();
	
	osKernelInitialize();
	evt_id = osEventFlagsNew(NULL);
	thrd_id1 = osThreadNew(UART_thread, NULL, NULL);
	thrd_id2 = osThreadNew(LCD_thread, NULL, &thrd2_attr);
	thrd_id4 = osThreadNew(TIM_thread, NULL, NULL);
	thrd_id3 = osThreadNew(ADCDMA_thread, NULL, NULL);
	
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


