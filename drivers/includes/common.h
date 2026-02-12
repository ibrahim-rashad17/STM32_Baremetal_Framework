/*
 * common.h
 *
 *  Created on: Feb 11, 2026
 *      Author: Ibrahim
 */

#ifndef INCLUDES_COMMON_H_
#define INCLUDES_COMMON_H_

void Error_Handler();

#define ASSERT(x)	do { if(!x) Error_Handler(); }while(0)

#define SET_BIT(REG,BIT)	(REG |= (1 << BIT))
#define CLR_BIT(REG,BIT)	(REG &= ~(1 << BIT))

#endif /* INCLUDES_COMMON_H_ */
