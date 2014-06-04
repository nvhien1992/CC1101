#include "main.h"

SPI_ns::SPI_params_t MB1_SPI_params_s;
//state_s STATE = send;
state_s STATE = receive;
uint8_t i = 0;
int8_t RSSI_dBm;

int main() {
	MB1_SPI_params_s.baudRatePrescaler = SPI_BaudRatePrescaler_4;
	MB1_SPI_params_s.CPOL = SPI_CPOL_Low;
	MB1_SPI_params_s.CPHA = SPI_CPHA_1Edge;
	MB1_SPI_params_s.dataSize = SPI_DataSize_8b;
	MB1_SPI_params_s.firstBit = SPI_FirstBit_MSB;
	MB1_SPI_params_s.mode = SPI_Mode_Master;
	MB1_SPI_params_s.direction = SPI_Direction_2Lines_FullDuplex;
	MB1_SPI_params_s.nss = SPI_NSS_Soft;
	MB1_SPI_params_s.crcPoly = 7;

	/*
	 *	@ Length of a packet.
	 */
//	uint8_t pktLength;

	/*
	 * @Send vars.
	 */
//	uint8_t txBuf[21] = {21, 'n', 'v', 'h', 'i', 'e', 'n', '1', '9', '9', '2', '@', 'g', 'm', 'a', 'i', 'l', '.', 'c', 'o', 'm'};
//	uint8_t txBuf[201] = {0};

	txBuf[0] = 200;
	for (i = 1; i < 201; i++)
		txBuf[i] = i;

	/*
	 * @Receive vars.
	 */
//	uint8_t rxBuf[203] = {0};

	/*
	 * @Initialize MB1(MBoard1) system.
	 */
	MB1_system_init();
	MB1_SPI1.init(&MB1_SPI_params_s);

	/*
	 * @Initialize HA(Home Automation) system.
	 */
	HA_SystemInit();

//	pktLength = sizeof(txBuf) - 1;

	while(1) {
//		switch(STATE) {
////		case idle:
////			break;
//		case send:
////			RF1.halSendRfPacket(txBuf, pktLength);
//			RF1.halSendVariablePacket(txBuf, pktLength);
//			MB1_RGB_green.on();
//			break;
//		case receive:
////			if(RF1.halReceiveRfPacket(rxBuf, pktLength)) {
//			if(RF1.halReceiveVariablePacket(rxBuf)) {
//				MB1_RGB_blue.on();
////				for (i = 0; i < pktLength; i++) {
////					if(i != pktLength - 1)
////						printf("%d ", rxBuf[i]);
////					else printf("%d\r\n", rxBuf[i]);
////				}
//			}else MB1_RGB_blue.off();
//			for (i = 0; i < pktLength; i++) {
//				if(i != pktLength - 1)
//					printf("%d ", rxBuf[i]);
//				else printf("%d\r\n", rxBuf[i]);
//			}
////			printf("final: %d\r\n", rxBuf[100]);
////			for (i = 0; i < pktLength; i++) {
////				rxBuf[i] = 0;
////			}
//			while(MB1_usrBtn0.pressedKey_get() == Btn_ns::retval_t::noNewKey);
//			break;
//		default:
//			break;
//		}

		switch(STATE) {
		case send:
			variablePacketSend(txBuf);
			MB1_RGB_blue.on();
			break;
		case receive:
			variablePacketReceive(rxBuf);
			if((rxBuf[202]&CRC_OK)) {
				MB1_RGB_blue.on();
				printf("_CRC: OK!\r\n");
			}else MB1_RGB_blue.off();
			for(i = 0; i < 203; i++) {
				if(i == 0)
					printf("_LengthData: %d.\r\n", rxBuf[i]);
				else if(i == 200)
					printf("%d.\r\n", rxBuf[i]);
				else if(i == 201) {
					RSSI_dBm = rxBuf[201];
					if(RSSI_dBm >= 128)
						RSSI_dBm -= 256;
					RSSI_dBm = RSSI_dBm/2 - RSSI_offset;
					printf("_RSSI: %d.\r\n", RSSI_dBm);
				}else if(i == 202)
					printf("_LQI: %d.\r\n", rxBuf[i]);
				else printf("%d ", rxBuf[i]);
			}
			for(i = 0; i < 203; i++) {
				rxBuf[i] = 0;
			}
			while(MB1_usrBtn0.pressedKey_get() == Btn_ns::retval_t::noNewKey);
			break;
		default:
			break;
		}
	}
}
