/*
 * common.h
 *
 *  Created on: Feb 11, 2026
 *      Author: Ibrahim
 */

#ifndef INCLUDES_COMMON_H_
#define INCLUDES_COMMON_H_

#include "stm32f407xx.h"

#define USART_RXNE_CALLBACK_ENABLED					1
#define USART_RX_SW_OVERFLOW_CALLBACK_ENABLED		1

void Error_Handler();

void IRQConfig(IRQ_Number_t IRQNumber, uint8_t En_or_Di);

void IRQPriorityConfig(IRQ_Number_t IRQNumber, uint8_t IRQPriority);

#define ASSERT(x)	do { if(!x) Error_Handler(); }while(0)

#define SET_BIT(REG,BIT)	(REG |= (1 << BIT))
#define CLR_BIT(REG,BIT)	(REG &= ~(1 << BIT))

//Function return codes
#define RET_OK		0
#define RET_ERROR	1

#endif /* INCLUDES_COMMON_H_ */
