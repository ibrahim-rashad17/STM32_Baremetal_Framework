/*
 * common.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Ibrahim
 */

#include "common.h"

void Error_Handler()
{
	while(1);
}

void IRQConfig(IRQ_Number_t IRQNumber, uint8_t En_or_Di)
{
	uint8_t temp;

	if(En_or_Di == ENABLE)
	{
		if(IRQNumber < 32)
		{
			temp = IRQNumber % 32;
			*(NVIC_ISER0) = (1 << temp);
		}
		else if(IRQNumber < 64)
		{
			temp = IRQNumber % 32;
			*(NVIC_ISER1) = (1 << temp);
		}
		else if(IRQNumber < 96)
		{
			temp = IRQNumber % 32;
			*(NVIC_ISER2) = (1 << temp);
		}
	}
	else
	{
		if(IRQNumber < 32)
		{
			temp = IRQNumber % 32;
			*(NVIC_ICER0) = (1 << temp);
		}
		else if(IRQNumber < 64)
		{
			temp = IRQNumber % 32;
			*(NVIC_ICER1) = (1 << temp);
		}
		else if(IRQNumber < 96)
		{
			temp = IRQNumber % 32;
			*(NVIC_ICER2) = (1 << temp);
		}
	}
}

void IRQPriorityConfig(IRQ_Number_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t *pPrioRegAddr = (uint8_t*)NVIC_PR_BASE_ADDR + (IRQNumber & 0xFF);

	IRQPriority  = IRQPriority << NO_PR_BITS_IMPLEMENTED;	//4 priority bits implemented

	*pPrioRegAddr = IRQPriority;
}
