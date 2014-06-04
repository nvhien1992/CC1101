#ifndef __DRIVER_CC1101_H
#define __DRIVER_CC1101_H

#include "rfSettings.h"
#include "MB1_System.h"

/*
 *
 */
#define 	GDO0		GPIO_Pin_0	//Port B
#define 	GDO2		GPIO_Pin_1	//Port B
#define 	GDO0_Port	GPIOB
#define 	GDO2_Port	GPIOB

 /*
  *	@define SPI header byte
  */
#define 	WRITE_BURST     	0x40
#define 	READ_SINGLE     	0x80
#define 	READ_BURST      	0xC0
#define 	BYTES_IN_RXFIFO   	0x7F
#define 	CRC_OK           	0x80

#define 	RSSI_offset			79 //(250 kBaud, 433MHz)

/*
 * @ TX_FIFO and RX_FIFO threshold.
 */
#define 	FIFO_THRSH	60

#define 	FIFO_SIZE	64

/*
 * @CC110x STROBE, CONTROL AND STATUS REGSITER
 */
#define CC110x_IOCFG2       0x00        // GDO2 output pin configuration
#define CC110x_IOCFG1       0x01        // GDO1 output pin configuration
#define CC110x_IOCFG0       0x02        // GDO0 output pin configuration
#define CC110x_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC110x_SYNC1        0x04        // Sync word, high uint8_t
#define CC110x_SYNC0        0x05        // Sync word, low uint8_t
#define CC110x_PKTLEN       0x06        // Packet length
#define CC110x_PKTCTRL1     0x07        // Packet automation control
#define CC110x_PKTCTRL0     0x08        // Packet automation control
#define CC110x_ADDR         0x09        // Device address
#define CC110x_CHANNR       0x0A        // Channel number
#define CC110x_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC110x_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC110x_FREQ2        0x0D        // Frequency control word, high uint8_t
#define CC110x_FREQ1        0x0E        // Frequency control word, middle uint8_t
#define CC110x_FREQ0        0x0F        // Frequency control word, low uint8_t
#define CC110x_MDMCFG4      0x10        // Modem configuration
#define CC110x_MDMCFG3      0x11        // Modem configuration
#define CC110x_MDMCFG2      0x12        // Modem configuration
#define CC110x_MDMCFG1      0x13        // Modem configuration
#define CC110x_MDMCFG0      0x14        // Modem configuration
#define CC110x_DEVIATN      0x15        // Modem deviation setting
#define CC110x_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CC110x_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CC110x_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CC110x_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CC110x_BSCFG        0x1A        // Bit Synchronization configuration
#define CC110x_AGCCTRL2     0x1B        // AGC control
#define CC110x_AGCCTRL1     0x1C        // AGC control
#define CC110x_AGCCTRL0     0x1D        // AGC control
#define CC110x_WOREVT1      0x1E        // High uint8_t Event 0 timeout
#define CC110x_WOREVT0      0x1F        // Low uint8_t Event 0 timeout
#define CC110x_WORCTRL      0x20        // Wake On Radio control
#define CC110x_FREND1       0x21        // Front end RX configuration
#define CC110x_FREND0       0x22        // Front end TX configuration
#define CC110x_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC110x_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC110x_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC110x_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC110x_RCCTRL1      0x27        // RC oscillator configuration
#define CC110x_RCCTRL0      0x28        // RC oscillator configuration
#define CC110x_FSTEST       0x29        // Frequency synthesizer calibration control
#define CC110x_PTEST        0x2A        // Production test
#define CC110x_AGCTEST      0x2B        // AGC test
#define CC110x_TEST2        0x2C        // Various test settings
#define CC110x_TEST1        0x2D        // Various test settings
#define CC110x_TEST0        0x2E        // Various test settings

// Strobe commands
#define CC110x_SRES         0x30        // Reset chip.
#define CC110x_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CC110x_SXOFF        0x32        // Turn off crystal oscillator.
#define CC110x_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CC110x_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CC110x_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CC110x_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CC110x_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC110x_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC110x_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CC110x_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC110x_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC110x_SWORRST      0x3C        // Reset real time clock.
#define CC110x_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // uint8_ts for simpler software.
// status reg
#define CC110x_PARTNUM      0x30
#define CC110x_VERSION      0x31
#define CC110x_FREQEST      0x32
#define CC110x_LQI          0x33
#define CC110x_RSSI         0x34
#define CC110x_MARCSTATE    0x35
#define CC110x_WORTIME1     0x36
#define CC110x_WORTIME0     0x37
#define CC110x_PKTSTATUS    0x38
#define CC110x_VCO_VC_DAC   0x39
#define CC110x_TXBYTES      0x3A
#define CC110x_RXBYTES      0x3B

#define CC110x_PATABLE      0x3E
#define CC110x_TXFIFO       0x3F
#define CC110x_RXFIFO       0x3F

namespace CC_ns {
	typedef struct {
		SPI_ns::SM_device_t deviceType;
		SPI* CC_SPI_ptr;
	} CC_SPI_params_s;
}

/*
 * @ RF object
 */
class driverCC1101 {
public:
	driverCC1101(void);

	void initCC1101(CC_ns::CC_SPI_params_s* CC_conf);

	void halSendStrobe(uint8_t cmdStrobe);
	uint8_t halReadSingleReg(uint8_t addr);
	uint8_t halReadBurstReg(uint8_t addr, uint8_t *rxBuf, uint8_t length);
	void halWriteSingleReg(uint8_t addr, uint8_t value);
	void halWriteBurstReg(uint8_t addr, uint8_t* txBuf, uint8_t length);
	uint8_t halReadSatusReg(uint8_t addr);

//	void halSendRfPacket(uint8_t *txBuf, uint8_t length);
//	bool halReceiveRfPacket(uint8_t *rxBuf, uint8_t length);

	void mDelay(uint16_t time);
private:
	SPI* SPI_ptr;
	SPI_ns::SM_device_t deviceType;

	void powerUpOnResetCC1101(void);
	void resetCC1101(void);

	void halRfWriteSettings(void);

	void reinitSPI(void);
	void waitMisoLow(void);
	void halWait(uint16_t timeout);

};

#endif /*__DRIVER_CC1101_H */
