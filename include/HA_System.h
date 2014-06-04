#ifndef __HA_SYSTEM_H
#define __HA_SYSTEM_H

#include "driverCC1101.h"

#define GDO0_EXTI_line	EXTI_Line0
#define GDO2_EXTI_line	EXTI_Line1

/*
 * @Global vars.
 */
extern driverCC1101 RF1;
extern EXTI_InitTypeDef EXTI_InitStructure;
extern ISRMgr* HA_ISR_ptr;

void HA_SystemInit(void);
void HA_SPI_Init(void);
void HA_CC1101_Init(void);
void EXTI0_1_Config(void);

#endif /*__HA_SYSTEM_H */
