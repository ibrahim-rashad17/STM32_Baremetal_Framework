/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Feb 11, 2026
 *      Author: Ibrahim
 */

#ifndef INCLUDES_STM32F407XX_USART_DRIVER_H_
#define INCLUDES_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_TX 		0
#define USART_MODE_RX 		1
#define USART_MODE_TXRX  	2

/*
 *@USART_OverSampling
 *Possible options for USART_Mode
 */
#define USART_OVERSAMPLING_8 	1
#define USART_OVERSAMPLING_16 	0

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define USART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3


/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


/*
 * USART flags
 */

#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)


typedef struct
{
	uint32_t	BaudRate;		/* Refer @USART_Baud */
	uint8_t 	NoOfStopBits;	/* Refer @USART_NoOfStopBits */
	uint8_t		WordLen;		/* Refer @USART_WordLength */
	uint8_t 	OverSampling;	/* Refer @USART_OverSampling */
	uint8_t 	ParityControl;	/* Refer @USART_ParityControl */
	uint8_t		mode;			/* Refer @USART_Mode */
}usart_config_t;

typedef struct
{
	usart_config_t	usart_config;	/* Pointer to config structure to be filled by user app */
	USART_RegDef_t	*pUSARTx;		/* Pointer to the Base Address of USART */
}USART_Handle_t;

void USART_Init(USART_Handle_t *pUSART_Handle);
void USART_TransmitData(USART_Handle_t *pUSART_Handle, uint8_t *pTxBfr, uint32_t len);
void USART_SetBaudRate(USART_Handle_t *pUSART_Handle, uint32_t baudrate);

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd);

uint8_t GetUSART_FlagStatus(USART_RegDef_t *pUSARTx, uint32_t Flag);

#endif /* INCLUDES_STM32F407XX_USART_DRIVER_H_ */
