/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Feb 11, 2026
 *      Author: Ibrahim
 */

#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

void USART_Init(USART_Handle_t *pUSART_Handle)
{
	uint32_t tempreg=0;

	USART_PeriClockControl(pUSART_Handle->pUSARTx, ENABLE);

	/* Check if parameters by user is valid */
	ASSERT((pUSART_Handle->usart_config.NoOfStopBits <= USART_STOPBITS_2));
	ASSERT((pUSART_Handle->usart_config.OverSampling <= USART_OVERSAMPLING_8));
	ASSERT((pUSART_Handle->usart_config.WordLen <= USART_WORDLEN_9BITS));
	ASSERT((pUSART_Handle->usart_config.ParityControl <= USART_PARITY_EN_ODD));

	/* Configure CR1 */
	tempreg |= (1 << USART_CR1_UE);	//Enable USART

	tempreg |= (pUSART_Handle->usart_config.WordLen << USART_CR1_M);

	tempreg |= (pUSART_Handle->usart_config.OverSampling << USART_CR1_OVER8);

	if(pUSART_Handle->usart_config.ParityControl == USART_PARITY_EN_EVEN)
	{
		tempreg |= (1 << USART_CR1_PCE);
	}
	else if(pUSART_Handle->usart_config.ParityControl == USART_PARITY_EN_ODD)
	{
		tempreg |= (1 << USART_CR1_PCE);
		tempreg |= (1 << USART_CR1_PS);		//To configure for odd parity
	}

	if(pUSART_Handle->usart_config.mode == USART_MODE_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}
	else if(pUSART_Handle->usart_config.mode == USART_MODE_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}
	else
	{
		tempreg |= (1 << USART_CR1_TE);
		tempreg |= (1 << USART_CR1_RE);
	}

	pUSART_Handle->pUSARTx->CR1 = tempreg;
	tempreg = 0;

	/* Configure CR2 */
	pUSART_Handle->pUSARTx->CR2 |= ( pUSART_Handle->usart_config.NoOfStopBits << USART_CR2_STOP);

	/* Configure the Baud Rate */
	USART_SetBaudRate(pUSART_Handle, pUSART_Handle->usart_config.BaudRate);

}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -	Transmits data from the buffer given by user
 *
 * @param[in]         -	ptr to USART Handle
 * @param[in]         -	Baud Rate to set
 *
 * @return            -	none
 *
 * @Note              -

 */
void USART_SetBaudRate(USART_Handle_t *pUSART_Handle, uint32_t baudrate)
{
	uint16_t mantissa, fraction;
	uint32_t usartdiv, tempreg = 0;

	if(pUSART_Handle->usart_config.OverSampling == USART_OVERSAMPLING_8)
	{
		usartdiv = (RCC_GetPCLK1Value() * 25) / (2 * pUSART_Handle->usart_config.BaudRate);
	}
	else
	{
		usartdiv = (RCC_GetPCLK1Value() * 25) / (4 * pUSART_Handle->usart_config.BaudRate);
	}

	mantissa = usartdiv / 100;
	fraction = usartdiv - (mantissa * 100);

	if(pUSART_Handle->usart_config.OverSampling == USART_OVERSAMPLING_8)
	{
		fraction = ((fraction * 8) + 50) / 100;
		fraction &= 0x07;	//Check RM (Bit 3 not used for oversampling by 8)
	}
	else
	{
		fraction = ((fraction * 16) + 50) / 100;
		fraction &= 0x0F;
	}

	mantissa &= 0x0FFF;

	tempreg = (mantissa << 4) | fraction;

	pUSART_Handle->pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_TransmitData
 *
 * @brief             -	Transmits data from the buffer given by user
 *
 * @param[in]         -	ptr to USART Handle
 * @param[in]         -	Ptr to the tx buffer
 * @param[in]         - Length of data to be transmitted
 *
 * @return            -	none
 *
 * @Note              - This is a blocking call

 */
void USART_TransmitData(USART_Handle_t *pUSART_Handle, uint8_t *pTxBfr, uint32_t len)
{
	for(uint32_t i=0;i<len;i++)
	{
		while(!GetUSART_FlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TXE));

		if(pUSART_Handle->usart_config.WordLen == USART_WORDLEN_8BITS)
		{
			pUSART_Handle->pUSARTx->DR = *pTxBfr++;
		}
		else
		{
			uint16_t *pData = (uint16_t*)pTxBfr;
			pUSART_Handle->pUSARTx->DR = (*pData & 0x1FF);

			if(pUSART_Handle->usart_config.ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBfr += 2;
				i++;	//Loop should run one less time
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBfr++;
			}
		}
	}

	while(!GetUSART_FlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TXE));
	while(!GetUSART_FlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd)
{
	if(Cmd == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}

}



/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCCK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_EN();
		}
	}
	else
	{
		//TODO
	}

}

uint8_t GetUSART_FlagStatus(USART_RegDef_t *pUSARTx, uint32_t Flag)
{
	if(pUSARTx->SR & Flag)
	{
		return SET;
	}
	return RESET;
}
