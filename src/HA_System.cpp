#include "HA_System.h"

/*
 * @Global vars.
 */

driverCC1101 RF1;
SPI* HA_SPI_ptr = &MB1_SPI1;
ISRMgr* HA_ISR_ptr = &MB1_ISRs;

/*
 * @ define slave select lines
 */
const uint8_t HA_SPI_numOfSSLines = 1;
GPIO_TypeDef* HA_SPI_Cs0GPIO_port = GPIOA;
const uint16_t HA_SPI_Cs0GPIO_pin = GPIO_Pin_4;
const uint32_t HA_SPI_Cs0GPIO_clk = RCC_APB2Periph_GPIOA;

/*
 * @ define all device in usings
 */
SPI_ns::SM_device_t HA_SPI_decodeTable [0x01 << HA_SPI_numOfSSLines] = {
	SPI_ns::cc1101_1,   //CS = 0
    SPI_ns::allFree,    //CS = 1
};

/*
 * @ define SPI pointer for cc1101.
 */
SPI* HA_CC_SPI_ptr = HA_SPI_ptr;
const SPI_ns::SM_device_t HA_CC_deviceType = SPI_ns::cc1101_1;

EXTI_InitTypeDef EXTI_InitStructure;

/*
 *
 */
void HA_SystemInit() {
	HA_SPI_Init();
	HA_CC1101_Init();

	return;
}

/*
 *	@init all chip select line
 */
void HA_SPI_Init() {
	HA_SPI_ptr->SM_numOfSSLines_set(HA_SPI_numOfSSLines);

	SPI_ns::SM_GPIOParams_s HA_SPI_GPIO_Cs_struct;
	HA_SPI_GPIO_Cs_struct.GPIO_port = GPIOA;
	HA_SPI_GPIO_Cs_struct.GPIO_pin = GPIO_Pin_4;
	HA_SPI_GPIO_Cs_struct.GPIO_clk = RCC_APB2Periph_GPIOA;
	HA_SPI_GPIO_Cs_struct.ssLine = 0;

	HA_SPI_ptr->SM_GPIO_set(&HA_SPI_GPIO_Cs_struct);

	uint8_t i;
	for ( i = 0; i < (0x01 << HA_SPI_numOfSSLines); i++ ){
		HA_SPI_ptr->SM_deviceToDecoder_set (HA_SPI_decodeTable[i], i);
	}

	return;
}


/*
 * @ initialize rf chip.
 */
void HA_CC1101_Init() {
	CC_ns::CC_SPI_params_s CC_SPI;
	CC_SPI.CC_SPI_ptr = HA_CC_SPI_ptr;
	CC_SPI.deviceType = HA_CC_deviceType;

	RF1.initCC1101(&CC_SPI);
	EXTI0_1_Config();

	return;
}

/*
 * @ external interrupt configure
 */
void EXTI0_1_Config(void) {

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Configure PB0, 1 pins as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect EXTI0 Line to PB0 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

	/* Connect EXTI1 Line to PB1 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = GDO0_EXTI_line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI1 line */
	EXTI_InitStructure.EXTI_Line = GDO2_EXTI_line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0&1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}


