/*
 * uart_rx_it_test.c
 *
 *  Created on: Feb 13, 2026
 *      Author: Ibrahim
 */

#include <stdint.h>
#include <string.h>

#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_gpio_driver.h"

void delay()
{
	for(uint32_t i=0;i<80000;i++);
}

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

USART_Handle_t huart2;
GPIO_Handle_t uart_gpio;

uint8_t rxbfr[256];
uint8_t volatile user_bfr[256];
uint8_t volatile frnt, rear, lnth;

//uart_callback	rxcallback;

void rx_callback(USART_Event_t event)
{
	if(event == USART_EVENT_RXNE)
	{
		if(lnth < 256)
		{
			user_bfr[rear] = ReadUSART_RxQueue(&huart2);
			rear = (rear + 1) % 256;
			lnth++;
		}
	}
}

void print()
{
	char tmpbfr[256];
	uint8_t n = lnth;
	lnth = 0;
	int i=0;

	for (i=0;i<n;i++)
	{
		tmpbfr[i] = user_bfr[frnt];
		frnt = (frnt + 1) % 256;
	}
	tmpbfr[i] = '\0';

	if (n)
	{
		USART_TransmitData(&huart2, (uint8_t*)tmpbfr, strlen(tmpbfr));
	}
}

int main(void)
{
	uart_gpio.pGPIOx = GPIOA;
	uart_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	uart_gpio.GPIO_PinConfig.GPIO_PinMode =	GPIO_MODE_ALTFN;
	uart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	uart_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	uart_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	uart_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&uart_gpio);

	uart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&uart_gpio);

	huart2.pUSARTx = USART2;
	huart2.usart_config.BaudRate = 115200;
	huart2.usart_config.NoOfStopBits = USART_STOPBITS_1;
	huart2.usart_config.OverSampling = USART_OVERSAMPLING_8;
	huart2.usart_config.ParityControl = USART_PARITY_DISABLE;
	huart2.usart_config.mode = USART_MODE_TXRX;
	huart2.usart_config.WordLen = USART_WORDLEN_8BITS;

	USART_Init(&huart2);

//	rxcallback = rx_callback;

	USART_RX_IT_Init(&huart2, rxbfr, 256, rx_callback);

	char data[] = "Testing USART driver\r\n";

	USART_TransmitData(&huart2, (uint8_t*)data, strlen(data));

//	while(!GetUSART_FlagStatus(USART2, USART_FLAG_RXNE));

    /* Loop forever */
	for(;;)
	{
		delay();
		delay();
		delay();
//		USART_TransmitData(&huart2, (uint8_t*)data, strlen(data));
		print();
//		USART_TransmitData(&huart2, (uint8_t*)data, strlen(data));
	}
}

void USART2_IRQHandler(void)
{
	USART_IRQ_Handling(&huart2);
}
