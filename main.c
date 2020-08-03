#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "queue.h"                      // ARM.FreeRTOS::RTOS:Core

#define POLL_DELAY					75
#define LED_DELAY						100
#define MAX_ANSWER_DELAY		0x3F000

// Global variables
static uint16_t isDataCaught = 0;
static xQueueHandle xQueue;

// Interrupt handlers
void TIM2_IRQHandler(void)
{
	TIM2->SR &= ~(uint32_t)(1 << 0);			// Clear update interrupt flag
	TIM2->CR1 &= ~(uint32_t)(1 << 0);			// Disable TIM2
	GPIOA->ODR = (0 << 0);	
	
	//Set up TIM2
	TIM2->ARR = 0xFFFFFFFF;
	TIM2->CR1 &= ~(uint32_t)(1 << 7);			// Auto-preload disable
	TIM2->CNT = 0;

	EXTI->RTSR |= EXTI_IMR_IM1;						// Rising trigger for PA1 pin
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_DisableIRQ(TIM2_IRQn);
};

void EXTI1_IRQHandler(void)
{
	EXTI->PR |= (1 << 1); // Clear interrupt flag
		
	if(EXTI->RTSR & EXTI_IMR_IM1) {
		TIM2->CR1 |= (1 << 0);							// Enable TIM2
		EXTI->RTSR &= ~(uint32_t)EXTI_IMR_IM1;
		EXTI->FTSR |= EXTI_IMR_IM1;
	}
	else {
		TIM2->CR1 &= ~(uint32_t)(1 << 0);			// Disable TIM2
		EXTI->FTSR &= ~(uint32_t)EXTI_IMR_IM1;
		NVIC_DisableIRQ(EXTI1_IRQn);
		isDataCaught = 1;
	};
};

void delay(const int constr)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(constr);
	xLastWakeTime = xTaskGetTickCount();
	
	vTaskDelayUntil(&xLastWakeTime, xPeriod); 
};

void InitHardware(void)
{
	// Enable clock for GPIO port A, port D, TIM2, SYSCFG
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN);
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	GPIOD->MODER |= (1 << 30);	// PD15 output mode
	GPIOA->MODER |= (1 << 0);		// PA0 output mode, PA1 input mode
															// External interrupt on PA1 pin
	GPIOA->ODR = 0;	
	EXTI->IMR |= EXTI_IMR_IM1;	// Enable 1 interrupt line for PA1 pin
	
	__enable_irq();
};

void vTaskVisualize(void *pvParameters)
{
	uint32_t value = 1, peek_value = 0;
	while(1) {
		peek_value = 0;
		xQueueReceive(xQueue, &peek_value, 10);
		
		if(peek_value) 
			if(peek_value > MAX_ANSWER_DELAY) 
				value = 1;
			else 
				value = MAX_ANSWER_DELAY / peek_value;
		
		delay(LED_DELAY*value);
		GPIOD->ODR |= (1 << 15);
		delay(LED_DELAY*value);
		GPIOD->ODR &= ~(uint32_t)(1 << 15);
	};
};

void vTaskTakeSensorData(void *pvParameters)
{
	uint32_t value;
	while(1) {
		vTaskDelay(POLL_DELAY);
		
		isDataCaught = 0;
		
		//Set up TIM2
		TIM2->ARR = 0x70;						// 0x70 ticks for 10 us
		TIM2->DIER |= 1;						// Update interrupt enable
		TIM2->CR1 |= (1 << 7);			// Auto-preload enable
		TIM2->CNT = 0;
		NVIC_EnableIRQ(TIM2_IRQn);
		
		GPIOA->ODR |= (1 << 0);			// Rising front of start sygnal on PA0
		TIM2->CR1 |= (1 << 0);			// Enable TIM2
		
		while(!isDataCaught);				// Wait for sensor data
		value = TIM2->CNT;
		xQueueSendToBack(xQueue, &value, 500);
	};
};

int main(void)
{
	InitHardware();
	
	xQueue = xQueueCreate(1, sizeof(uint32_t));
	if(xQueue == NULL) return 1;
	
	xTaskCreate(vTaskTakeSensorData, "TakeSensorData",
		configMINIMAL_STACK_SIZE, NULL, 0, NULL);
	xTaskCreate(vTaskVisualize, "Visualize",
		configMINIMAL_STACK_SIZE, NULL, 0, NULL);
	
	vTaskStartScheduler();
	
	while(1);
};
