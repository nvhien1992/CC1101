#include "variablePacketHandler.h"
#include "stdio.h"

uint8_t txBuf[201] = {0};
uint8_t rxBuf[203] = {0};

ISRMgr_ns::ISR_t subISR_EXTI0 = ISRMgr_ns::ISR_t::ISRMgr_EXTI0;
ISRMgr_ns::ISR_t subISR_EXTI1 = ISRMgr_ns::ISR_t::ISRMgr_EXTI1;

HANDLER_ns::rfModes mode;

HANDLER_ns::txData wrData;
HANDLER_ns::rxData rdData;

bool flag;

/*
 * @ 			_____________________________________________
 * @ GDO0______|   ____________	  __________   ________      |__________
 * @ GDO2_________|			   |_|			|_|		   |________________
 */
void variablePacketSend(uint8_t *txBuf) {
	uint8_t pktLength; //length of data
	flag = true;
	wrData.bufferPtr = NULL;
	wrData.bytesLeft = 0;
	wrData.iterations = 0;
	wrData.writeRemainingDataFlag = false;

	pktLength = txBuf[0];
	RF1.halWriteSingleReg(CC110x_IOCFG2, 0x02);
	RF1.mDelay(200);
	RF1.halSendStrobe(CC110x_SIDLE);
	RF1.halSendStrobe(CC110x_SFTX);

	/*
	 * @ Initialize Trigger mode (Falling edge interrupt) for GDO2 pin.
	 */
	setTriggerLine(EXTI_Trigger_Falling, GDO2_EXTI_line);
	/*
	 * @ Initialize Trigger mode (Rising edge interrupt) for GDO0 pin.
	 */
	setTriggerLine(EXTI_Trigger_Rising, GDO0_EXTI_line);

	mode = HANDLER_ns::send;
	wrData.bufferPtr = txBuf;	//update pointer to point to txBuf;

	if (pktLength < FIFO_SIZE) {
		HA_ISR_ptr->subISR_assign(subISR_EXTI0, subISR_falling_GDO0);

        RF1.halWriteBurstReg(CC110x_TXFIFO, txBuf, (pktLength + 1));
        RF1.halSendStrobe(CC110x_STX);
    } else { // The TX FIFO needs to be re-filled several times
    	HA_ISR_ptr->subISR_assign(subISR_EXTI0, subISR_rising_GDO0);

    	RF1.halWriteBurstReg(CC110x_TXFIFO, txBuf, FIFO_SIZE); // Fill up the TX FIFO

    	/*
    	 * @ Update info of buffer.
    	 */
        wrData.bytesLeft = pktLength + 1 - FIFO_SIZE;
        wrData.bufferPtr = txBuf + FIFO_SIZE;
        wrData.iterations = (wrData.bytesLeft / FIFO_THRSH);
        if (wrData.iterations == 0)
            wrData.writeRemainingDataFlag = true;

    	RF1.halSendStrobe(CC110x_STX);
    	while(flag);
    }
}

/*
 * @ 			_________________________________________
 * @ GDO0______|          _			  _			   _	 |______
 * @ GDO2________________| |_________| |__________| |___________
 */
void variablePacketReceive(uint8_t *rxBuf) {
	rdData.bufferPtr = NULL;
	rdData.bytesLeft = 0;
	rdData.lengthByte = 0;
	flag = true;

	RF1.halWriteSingleReg(CC110x_IOCFG2, 0x00);
	RF1.mDelay(100);
	RF1.halSendStrobe(CC110x_SIDLE);
	RF1.halSendStrobe(CC110x_SFRX);

	/*
	 * @ Initialize Trigger mode (Rising edge interrupt) for GDO2 pin.
	 */
	setTriggerLine(EXTI_Trigger_Rising, GDO2_EXTI_line);
	/*
	 * @ Initialize Trigger mode (Rising edge interrupt) for GDO0 pin.
	 */
	setTriggerLine(EXTI_Trigger_Rising, GDO0_EXTI_line);

	mode = HANDLER_ns::receive;
	rdData.bufferPtr = rxBuf;	//update pointer to point to rxBuf;

	HA_ISR_ptr->subISR_assign(subISR_EXTI0, subISR_rising_GDO0);
	RF1.halSendStrobe(CC110x_SRX);

	while(flag);

	return;
}

/*
 * @ Used to handle the rising edge interrupt at GDO pin. (Syncwords is sent)
 */
void subISR_rising_GDO0() {
	if(EXTI_GetITStatus(GDO0_EXTI_line) != RESET) {
		/*
		 * @ Change the Trigger mode (to Falling edge interrupt) for GDO0 pin.
		 */
		setTriggerLine(EXTI_Trigger_Falling, GDO0_EXTI_line);
		HA_ISR_ptr->subISR_remove(subISR_EXTI0, subISR_rising_GDO0);

		if(mode == HANDLER_ns::receive) {
			HA_ISR_ptr->subISR_assign(subISR_EXTI0, subISR_falling_GDO0);

			while((RF1.halReadSatusReg(CC110x_RXBYTES)&BYTES_IN_RXFIFO) < 2);

			rdData.lengthByte = RF1.halReadSingleReg(CC110x_RXFIFO);
			rdData.bytesLeft = rdData.lengthByte + 2;
			if(rdData.bytesLeft > 60)
				HA_ISR_ptr->subISR_assign(subISR_EXTI1, subISR_rising_GDO2);
			rdData.bufferPtr[0] = rdData.lengthByte;
			rdData.bufferPtr++;
		}else HA_ISR_ptr->subISR_assign(subISR_EXTI1, subISR_falling_GDO2);
	}

	return;
}

/*
 * @ Used to handle the falling edge interrupt at GDO pin. (Packet is sent completely)
 */
void subISR_falling_GDO0() {
//	uint8_t i;
	if(EXTI_GetITStatus(GDO0_EXTI_line) != RESET) {
		if(mode == HANDLER_ns::receive) {
			RF1.halReadBurstReg(CC110x_RXFIFO, rdData.bufferPtr, rdData.bytesLeft);
		}
		/*
		 * @ Disable all external interrupts.
		 */
		HA_ISR_ptr->subISR_remove(subISR_EXTI0, subISR_falling_GDO0);
		HA_ISR_ptr->subISR_remove(subISR_EXTI0, subISR_rising_GDO0);
		HA_ISR_ptr->subISR_remove(subISR_EXTI1, subISR_falling_GDO2);
		HA_ISR_ptr->subISR_remove(subISR_EXTI1, subISR_rising_GDO2);
	}
	MB1_RGB_green.on();
	flag = false;

	return;
}

/*
 * @ Used to handle the external interrupt at GDO2 pin in RX mode.
 */
void subISR_rising_GDO2() {	//get data from RX_FIFO
	if(EXTI_GetITStatus(GDO2_EXTI_line) != RESET) {
		if(mode == HANDLER_ns::receive) {
			// Do not empty the FIFO (See the CC1100 or 2500 Errata Note)
			RF1.halReadBurstReg(CC110x_RXFIFO, rdData.bufferPtr, (FIFO_THRSH - 1));
			rdData.bytesLeft -= (FIFO_THRSH - 1);
			rdData.bufferPtr += (FIFO_THRSH - 1);
			if(rdData.bytesLeft < FIFO_THRSH)
				HA_ISR_ptr->subISR_remove(subISR_EXTI1, subISR_rising_GDO2);
		}
	}

	return;
}

/*
 * @ Used to handle the external interrupt at GDO2 pin in TX mode.
 */
void subISR_falling_GDO2() {	//refill the TX_FIFO
	if(EXTI_GetITStatus(GDO2_EXTI_line) != RESET) {
		if(mode == HANDLER_ns::send) {
		   if (wrData.writeRemainingDataFlag) { // Less than 60 bytes to write to the TX FIFO
			   HA_ISR_ptr->subISR_remove(subISR_EXTI1, subISR_falling_GDO2);
			   HA_ISR_ptr->subISR_assign(subISR_EXTI0, subISR_falling_GDO0);
			   RF1.halWriteBurstReg(CC110x_TXFIFO, wrData.bufferPtr, wrData.bytesLeft);
		   } else {
				RF1.halWriteBurstReg(CC110x_TXFIFO, wrData.bufferPtr, FIFO_THRSH);
				wrData.bufferPtr += FIFO_THRSH;
				wrData.bytesLeft -= FIFO_THRSH;
				wrData.iterations--;
				if (wrData.iterations == 0)
					wrData.writeRemainingDataFlag = true;
			}
		}
	}

	return;
}

void setTriggerLine(EXTITrigger_TypeDef EXTITrigger, uint32_t EXTI_line) {
	EXTI_InitStructure.EXTI_Line = EXTI_line;
	EXTI_InitStructure.EXTI_Trigger = EXTITrigger;
	EXTI_Init(&EXTI_InitStructure);

	return;
}
