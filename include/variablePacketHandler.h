#ifndef __VP_HANDLER_H
#define __VP_HANDLER_H

#include "HA_System.h"

extern uint8_t txBuf[201];
extern uint8_t rxBuf[203];

namespace HANDLER_ns {
	typedef enum {
		send,
		receive
	} rfModes;

	typedef struct {
		uint8_t bytesLeft;
		uint8_t *bufferPtr;
		uint8_t iterations;

		bool writeRemainingDataFlag;
	} txData;

	typedef struct {
		uint8_t bytesLeft;
		uint8_t *bufferPtr;
		uint8_t lengthByte;

//		bool crcOK;
	} rxData;
}

void variablePacketSend(uint8_t *txBuf);
void variablePacketReceive(uint8_t *rxBuf);

void subISR_rising_GDO0(void);
void subISR_falling_GDO0(void);

void subISR_rising_GDO2(void);
void subISR_falling_GDO2(void);

void setTriggerLine(EXTITrigger_TypeDef EXTITrigger, uint32_t EXTI_line);

#endif /*__VP_HANDER_H */
