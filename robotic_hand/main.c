#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "stm32f7xx.h"                  // Device header
#include "Open746i_lcd.h"
#include "Driver_USART.h"
#include <stdio.h>

#define MSGQUEUE_OBJECTS 1                     // number of Message Queue Objects
#define NUM_SERVO 5

void SystemClock_Config(void);
extern ARM_DRIVER_USART Driver_USART1;
void servo(int id, int val);
void UART_handler(uint32_t event);
void LCD_thread(void *argument) __NO_RETURN;
void UART_thread(void *argument) __NO_RETURN;
void SERVO_thread(void *argument) __NO_RETURN;

unsigned short buffer[NUM_SERVO] = {0};
unsigned short msg_send[NUM_SERVO] = {0};
unsigned short msg_receive[NUM_SERVO] = {0};

osMessageQueueId_t q_id; 
osEventFlagsId_t evt_id;
osThreadId_t thrd_id1, thrd_id2, thrd_id3;

const osThreadAttr_t thrd1_attr = {
  .priority  = osPriorityHigh,
};
const osThreadAttr_t thrd2_attr = {
  .priority  = osPriorityLow,
};

void UART_handler(uint32_t event)
{
	int i;
	if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{
		for (i = 0; i < NUM_SERVO; i++)
		{
			msg_send[i] = buffer[i];
		}
		osMessageQueuePut(q_id, &msg_send, 0U, 0U);
		osEventFlagsSet(evt_id, 3);
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
		// continue receiving from uart
		Driver_USART1.Receive(buffer, NUM_SERVO * 2);
		osEventFlagsWait(evt_id, 1, osFlagsWaitAny, osWaitForever);
	}
}

__NO_RETURN void LCD_thread(void *argument)
{
	
	char string[50];
	
	while(1)
	{
		osEventFlagsWait(evt_id, 2, osFlagsWaitAny, osWaitForever);
		sprintf(string, "0x%04x%04x%04x%04x%04x", msg_send[0], msg_send[1], msg_send[2], msg_send[3],msg_send[4]);
		BSP_LCD_DisplayStringAt(0, 300, (uint8_t *) string, CENTER_MODE);
	}
}

__NO_RETURN void SERVO_thread(void *argument)
{
	int j;

	//initialize all servo pins
	RCC->APB1ENR |= ((1 << 1) | (1 << 3)); // activate timer 3 and timer 5
	RCC->AHB1ENR |= (1 << 0); // activate gpio A

	GPIOA->MODER |= ((2 << 0) | (2 << 2) | (2 << 4) | (2 << 12) | (2 << 14)); // PA0 in AF mode
	
	GPIOA->AFR[0] |= ((2 << 0) | (2 << 4) | (2 << 8) | (2 << 24) | (2 << 28)); 
	// PA0 in AF2 mode (TIM5_CH1), PA1 TIM5_CH2, PA2 TIM5_CH3, PA6 TIM3_CH1, PA7 TIM3_CH2
	
	// TIM 5 timer clk is 108MHz
	
	TIM5->PSC = 107;
	TIM5->ARR = 19999;
	TIM5->CCR1 = 1500;
	TIM5->CCR2 = 1500;
	TIM5->CCR3 = 1500;
	TIM5->CCMR1 |= (6 << 4); // TIM5_CH1
	TIM5->CCMR1 |= (6 << 12); // TIM5_CH2
	TIM5->CCMR2 |= (6 << 4); // TIM5_CH3
	TIM5->CCER |= ((1 << 0) | (1 << 4) | (1 << 8));
	
	// TIM 3 timer clk is 108MHz
	TIM3->PSC = 107;
	TIM3->ARR = 19999;
	TIM3->CCR1 = 1500;
	TIM3->CCR2 = 1500;
	TIM3->CCMR1 |= (6 << 4); // TIM3_CH1
	TIM3->CCMR1 |= (6 << 12); // TIM3_CH2
	TIM3->CCER |= ((1 << 0) | (1 << 4));


	// start timers
	TIM5->CR1 |= (1 << 0);
	TIM3->CR1 |= (1 << 0);
	
	char string2[50];

	while(1)
	{
		osMessageQueueGet(q_id, &msg_receive, NULL, osWaitForever);   // wait for message
		//sprintf(string2, "0x%04x%04x%04x%04x%04x", msg_receive[0], msg_receive[1], msg_receive[2], msg_receive[3], msg_receive[4]);
		//BSP_LCD_DisplayStringAt(0, 400, string2, CENTER_MODE);
		
		for (j = 0; j < NUM_SERVO; j++)
		{
			if (msg_receive[j] == 0) continue;
			else servo(j, msg_receive[j]);
		}
	}
}

void servo(int id, int val)
{
	//int ccr;
	//ccr = deg_convert(deg);
	switch(id){
		case 0:
		{
			TIM5->CCR1 = val;
			break;
		}
		case 1:
		{
			TIM5->CCR2 = val;
			break;
		}
		case 2:
		{
			TIM5->CCR3 = val;
			break;
		}
		case 3:
		{
			TIM3->CCR1 = val;
			break;
		}
		case 4:
		{
			TIM3->CCR2 = val;
			break;
		}
	}
}


int main(void)
{
	SystemClock_Config();
	
	BSP_LCD_Init();
	BSP_LCD_Clear(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(0, 0, (unsigned char *) "Test LCD", CENTER_MODE);

	
	osKernelInitialize();
	evt_id = osEventFlagsNew(NULL);
	q_id = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(msg_send), NULL);
	thrd_id1 = osThreadNew(UART_thread, NULL, NULL); // high priority
	thrd_id2 = osThreadNew(LCD_thread, NULL, &thrd2_attr); // low priority
	thrd_id3 = osThreadNew(SERVO_thread, NULL, NULL); 
	
	osKernelStart();

	while(1); // reached only if there's an error
}

//void SysTick_Handler(void)
//{
//	HAL_IncTick();
//}

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

