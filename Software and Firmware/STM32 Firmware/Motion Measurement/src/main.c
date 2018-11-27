// ----------------------------------------------------------------------------
// Description
//
// Runs the control and comms algorithms for the Two-Axis Telescope Stabilization System.
//
//-------------------------------------------------------------------------------------------------------------------------------------------
// Declarations

#include <stdint.h>
#include "stm32f0xx.h"

// I2C SLAVE ADDRESSES ----------------------------------------------------------------------------------------------------------------------

#define MPU9150ADD0		0xD0

// MPU9150 ADDRESSES ------------------------------------------------------------------------------------------------------------------------

#define CONFIG 			0x1A
#define WHO_AM_I 		0x75
#define SMPRT_DIV		0x19
#define GYRO_CONFIG 	0x1B
#define FIFO_EN			0x23
#define USER_CTRL		0x6A
#define PWR_MGMT_1		0x6B
#define GYRO_XOUTH		0x43
#define GYRO_XOUTL		0x44
#define GYRO_YOUTH		0x45
#define GYRO_YOUTL		0x46
#define GYRO_ZOUTH		0x47
#define GYRO_ZOUTL		0x48
#define FIFO_COUNT_H	0x72
#define FIFO_COUNT_L	0x73
#define FIFO_R_W		0x74
#define INT_ENABLE		0x38
#define INT_STATUS		0x3A

// MPU9150 NIBBLE DEFINES -------------------------------------------------------------------------------------------------------------------

#define FS_SEL_0 		0x00			// 250 deg/s
#define FS_SEL_1 		0x08			// 500 deg/s
#define FS_SEL_2 		0x10			// 1000 deg/s
#define FS_SEL_3 		0x18			// 2000 deg/s
#define SMPLRT_200_NF	0x27			// 200 Hz sample rate, no DLPF
#define SMPLRT_500_NF	0x0F			// 500 Hz sample rate, no DLPF
#define SMPLRT_1000_NF	0x07			// 1000 Hz sample rate, no DLPF
#define SMPLRT_1000_F	0x00			// 1000 Hz sample rate, DLPF
#define SMPLRT_500_F	0x01			// 500 Hz sample rate, DLPF
#define SMPLRT_200_F	0x04			// 200 Hz sample rate, DLPF
#define SMPLRT_100_F	0x09			// 100 Hz sample rate, DLPF
#define SMPLRT_20_F		0x31			// 20 Hz sample rate, DLPF
#define SMPLRT_10_F		0x63			// 10 Hz sample rate, DLPF
#define DLPF_CFG_0		0x00			// No filter
#define DLPF_CFG_1		0x01			// 184a, 188g Hz DLPF
#define DLPF_CFG_2		0x02			// 94a, 98g Hz DLPF
#define DLPF_CFG_3		0x03			// 44a, 42g Hz DLPF
#define DLPF_CFG_4		0x04			// 21a, 20g Hz DLPF
#define ZG_FIFO_EN 		0x10			// gyro z-axis to FIFO buffer
#define FIFO_ENBL		0x40			// Enable FIFO buffer
#define FIFO_RESET		0x04			// Reset FIFO buffer
#define TEMP_DIS		0x08 			// Disable temperature sensor
#define DEVICE_RESET	0x40			// Device reset
#define CLKSEL0			0x00			// 8 MHz internal oscillator
#define CLKSEL1			0x01			// PLL with gyro x-axis
#define CLKSEL2			0x02			// PLL with gyro y-axis
#define CLKSEL3			0x03			// PLL with gyro z-axis
#define DATA_RDY_EN		0x01			// Data ready interrupt enable
#define DATA_RDY_INT	0x01			// Data ready interrupt

// CLARIFICATION DEFINES --------------------------------------------------------------------------------------------------------------------

#define I2C_ISR_TC_TXIS 0x42
#define I2C_ISR_TC_RXNE 0x44
#define ARLO_ERROR		0x200
#define BERR_ERROR		0x100
#define NACKF_ERROR		0x10
#define CLEAR_CR2		0x00

// USART DEFINES ----------------------------------------------------------------------------------------------------------------------------

#define BAUD_9600		0x1388
#define BAUD_19200		0x9C4
#define BAUD_38400		0x4E2
#define BAUD_57600		0x341
#define BAUD_115200		0x1A1
#define BAUD_230400     0xD0

// FUNCTION DECLARATIONS --------------------------------------------------------------------------------------------------------------------

void init_i2c(void);
void init_Pins(void);
void init_USART1(uint32_t wordLength);
void init_EXTI(void);
void init_Timer1(uint16_t delayMilliseconds);
void init_MPU9150(void);
uint32_t handleInterrupt(void);
void i2cWriteSetup(I2C_TypeDef* I2Cx, uint8_t devAdd, uint16_t nbytesWrite);
void i2cReadSetup(I2C_TypeDef* I2Cx, uint8_t devAdd, uint16_t nbytesRead);
void i2cWrite(I2C_TypeDef* I2Cx, uint8_t devAdd, uint8_t regAdd, uint16_t nbytesWrite, uint8_t* data);
void resetGyroOffset(void);
void determineOffset(int16_t gyroDataXT, int16_t gyroDataYT, int16_t gyroDataZT);
void monitorData(void);
void I2C2_IRQHandler(void);
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void EXTI4_15_IRQHandler(void);

// STATE VARIABLES --------------------------------------------------------------------------------------------------------------------------

enum {READ_INT_STATUS = 0, READ_GYRO_XOUTH = 1, READ_GYRO_XOUTL = 2, READ_GYRO_YOUTH = 3, READ_GYRO_YOUTL = 4, READ_GYRO_ZOUTH = 5, READ_GYRO_ZOUTL = 6};
enum {NO_ERROR = 0, ERROR_OCCURRED = 1 /*BERR = 1, ARLO = 2, NACKF = 3*/};
enum {I2C_BUSY = 0, READY = 1};
enum {OFFSET_UNKNOWN = 0, OFFSET_KNOWN = 1};
enum {GYRO_NOT_READY = 0, GYRO_READY = 1};
enum {IN_DATA_BUSY = 0, IN_DATA_READY = 1};
enum {T_DATA_OLD = 0, T_DATA_NEW = 1};
enum {YAW_SHIFT_0 = 0, YAW_SHIFT_90 = 1, YAW_SHIFT_ADD = 2, YAW_SHIFT_SUB = 3};
enum {SETUP = 0, MANUAL = 1, AUTOMATIC = 2, LIMIT_ERROR = 3, STOP = 4};
enum {YAW = 0, PITCH = 2};

uint8_t progState = SETUP;
uint8_t i2cState = READ_INT_STATUS;
uint8_t i2cError = NO_ERROR;
uint8_t programState = READY;
uint8_t offsetState = OFFSET_UNKNOWN;
uint8_t gyroReadyState = GYRO_NOT_READY;
uint8_t inertialData = IN_DATA_BUSY;
uint8_t targetPos = T_DATA_OLD;
uint8_t yawShift = YAW_SHIFT_0;
uint8_t stabChannel = YAW;

// GYRO DATA MEASUREMENT VARIABLES ---------------------------------------------------------------------------------------------------------------

uint32_t datasetCounter = 0;
uint8_t intStatus = 0;
uint8_t readCount = 0;
uint8_t gyroXoutH[] = {0, 0, 0};
uint8_t gyroXoutL[] = {0, 0, 0};
uint8_t gyroYoutH[] = {0, 0, 0};
uint8_t gyroYoutL[] = {0, 0, 0};
uint8_t gyroZoutH[] = {0, 0, 0};
uint8_t gyroZoutL[] = {0, 0, 0};


// DATA HANDLING VARIABLES ------------------------------------------------------------------------------------------------------------------

uint16_t gyroDataSetCounter = 0;
int32_t initialDataSumNew[] = {0, 0, 0};
int32_t initialDataSumOld[] = {0, 0, 0};
int16_t gyroOffset[] = {0, 0, 0};
int16_t gyroDataXT = 0;
int16_t gyroDataYT = 0;
int16_t gyroDataZT = 0;

float w[4];


// Main -------------------------------------------------------------------------------------------------------------------------------------

void main(void)

{
	init_Pins();										//hal

	init_i2c();

	init_USART1(USART_WordLength_9b);					//hal
	init_MPU9150();
	init_Timer1(50);									//hal

	// Wait for gyro to start up before progressing
	while (gyroReadyState == GYRO_NOT_READY);
	for (uint32_t i = 0; i < 6000000; i++)
	{
		__asm("nop");
	}

	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	init_EXTI();										//hal

	NVIC_EnableIRQ(EXTI2_3_IRQn);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	//NVIC_EnableIRQ(USART1_IRQn);

	while(1)
	{
		if (inertialData == IN_DATA_READY)
		{
			monitorData();
		}
	}
}

// FUNCTIONS --------------------------------------------------------------------------------------------------------------------------------

void init_i2c(void)
{
	// Enable GPIOF clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

	// Open drain, Output on PF6_7
	GPIO_InitTypeDef I2C2_PF6_InitStruct;
	I2C2_PF6_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	I2C2_PF6_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	I2C2_PF6_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	I2C2_PF6_InitStruct.GPIO_OType = GPIO_OType_OD;
	I2C2_PF6_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &I2C2_PF6_InitStruct);

	// Setup PF6 and 7 for alternate function, open drain
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_0);

	/* I2C initialisation */

	// Enable I2C2 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	// Disable I2C2
	I2C_Cmd(I2C2, DISABLE);
	/* Configure timing in PRESC, SCLDEL, SDADEL in TIMINGR, using STM32 I2C timing configuration tool.
	 * I2C configured for 400 kHz, analogue filter on, digital filter off, rise time = 100 ns, fall time = 100 ns.
	 * PRESC = 0 SCLDEL = 9 SDADEL = 0 SCLH = 71 SCLL = 71 */

	I2C_InitTypeDef I2C2_InitStruct;
	I2C2_InitStruct.I2C_Timing = 0x00901347;
	I2C2_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Disable;
	I2C2_InitStruct.I2C_DigitalFilter = 5;
	I2C2_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C2_InitStruct.I2C_OwnAddress1 = 0;
	I2C2_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C2_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C2, &I2C2_InitStruct);

	// ER, TC, NACK, RX, TX interrupts
	I2C_ITConfig(I2C2, I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_NACKI | I2C_IT_RXI | I2C_IT_TXI, ENABLE);
	// Enable I2C2
	I2C_Cmd(I2C2, ENABLE);
}

void init_Pins(void)
{
	// Enable GPIOA&B clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	// Setup GPIOB pins 0-7 as outputs
	GPIO_InitTypeDef GPIOB_InitStruct;
	GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOB_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOB_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIOB_InitStruct);

	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);

	GPIO_InitTypeDef GPIOA_InitStruct;
	GPIOA_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOA_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOA_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIOA_InitStruct);
}

// USART 1 used for micro-PC comms: TX-PA9, RX-PA10
void init_USART1(uint32_t wordLength)
{
	// Clock to USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// PA9 and PA10 to AF
	GPIO_InitTypeDef USART1_PA_InitStruct;
	USART1_PA_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	USART1_PA_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	USART1_PA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	USART1_PA_InitStruct.GPIO_OType = GPIO_OType_PP;
	USART1_PA_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &USART1_PA_InitStruct);
	// Remap to correct AF: PA9 to AF1, PA10 to AF1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	SystemCoreClockUpdate();

	// Setup USART
	USART_InitTypeDef USART1_InitStruct;
	USART1_InitStruct.USART_BaudRate = 115200;
	USART1_InitStruct.USART_WordLength = wordLength;
	USART1_InitStruct.USART_StopBits = USART_StopBits_1;
	USART1_InitStruct.USART_Parity = USART_Parity_No;
	USART1_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART1_InitStruct);

	// Set baud rate to 115200 //19200 //9600 //
	USART1->BRR = 0x1A1;//0x9C4;//0x1388;/

	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

	//NVIC_EnableIRQ(USART1_IRQn);
}

void init_MPU9150(void)
{
	uint8_t devConfig[] = {DLPF_CFG_1};
	uint8_t sampleRate[] = {SMPLRT_500_F};
	uint8_t gyroConfig[] = {FS_SEL_0};
	uint8_t powerManage1[] = {TEMP_DIS | CLKSEL3};
	uint8_t powerManageRST[] = {DEVICE_RESET};
	uint8_t interruptEn[] = {DATA_RDY_EN};

	/* i2cWrite() does not use interrupts therefore only enable ints on NVIC when I2C reads are expected
	 * Disable I2C ints on NVIC before attempting to use i2cWrite() again */

	// Setup power management 1 register to wake up device
	i2cWrite(I2C2, MPU9150ADD0, PWR_MGMT_1, 2, &powerManage1[0]);
	// Reset all device registers
	i2cWrite(I2C2, MPU9150ADD0, PWR_MGMT_1, 2, &powerManageRST[0]);
	// Setup power management 1 register to turn off the temperature sensor and use the gyro z axis as clock
	i2cWrite(I2C2, MPU9150ADD0, PWR_MGMT_1, 2, &powerManage1[0]);
	// Setup config register for no filter
	i2cWrite(I2C2, MPU9150ADD0, CONFIG, 2, &devConfig[0]);
	// Setup sample rate divider register for 100 Hz sampling
	i2cWrite(I2C2, MPU9150ADD0, SMPRT_DIV, 2, &sampleRate[0]);
	// Setup gyro config register for 250 deg/s
	i2cWrite(I2C2, MPU9150ADD0, GYRO_CONFIG, 2, &gyroConfig[0]);
	// Setup interrupt enable register for data ready interrupts
	i2cWrite(I2C2, MPU9150ADD0, INT_ENABLE, 2, &interruptEn[0]);

	//NVIC_EnableIRQ(I2C2_IRQn);
}

void init_Timer1(uint16_t delayMilliseconds)
{
	// Enable Timer 1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef TIM1_InitStruct;
	TIM1_InitStruct.TIM_Prescaler = 47999;
	TIM1_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM1_InitStruct.TIM_Period = delayMilliseconds;
	TIM1_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1 ;
	TIM1_InitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM1_InitStruct);

	// Allow overflow interrupts
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	// Enable counter for Timer 3, and auto-reload preload enable
	TIM_Cmd(TIM1, ENABLE);
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

void init_EXTI(void)
{
	// Enable System Configuration clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	// Setup EXTI line 0 to be Port A
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	// Setup EXTI line 4 to be Port A
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
	// Setup EXTI line 5 to be Port A
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);

	// Setup Rising edge trigger interrupt
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line4 | EXTI_Line2;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
}

uint32_t handleInterrupt()
{
	uint16_t ARLO = 0;
	uint16_t BERR = 0;
	uint16_t NACKF = 0;
	uint16_t RXNE = 0;
	uint16_t TXIS = 0;
	uint16_t TC = 0;

	// ERRORS---------------------------------------------------------------------------------------------
	ARLO = I2C_GetITStatus(I2C2, I2C_IT_ARLO);
	BERR = I2C_GetITStatus(I2C2, I2C_IT_BERR);
	NACKF = I2C_GetITStatus(I2C2, I2C_IT_NACKF);
	RXNE = I2C_GetITStatus(I2C2, I2C_IT_RXNE);
	TXIS = I2C_GetITStatus(I2C2, I2C_IT_TXIS);
	TC = I2C_GetITStatus(I2C2, I2C_IT_TC);

	if (ARLO == 1) {
		return ARLO_ERROR;
		}
	else if (BERR == 1) {
		return BERR_ERROR;
		}
	else if (NACKF == 1) {
		return NACKF_ERROR;
		}
	// DATA HANDLING--------------------------------------------------------------------------------------
	else if (RXNE == 1)
	{
		if (TC == 1)
		{
			//=> Write S or P
			return I2C_ISR_TC_RXNE;
		}
		//=> Read I2C_RXDR
		else return I2C_FLAG_RXNE;
	}
	else if (TXIS == 1)
	{
		if (TC == 1)
		{
			//=> Write S or P
			return I2C_ISR_TC_TXIS;
		}
		//=> Write I2C_TXIS
		else return I2C_FLAG_TXIS;
	}
	else return I2C_FLAG_TC;
	// CLEAR FLAGS----------------------------------------------------------------------------------------
	I2C_ClearFlag(I2C2, I2C_FLAG_ARLO | I2C_FLAG_BERR | I2C_FLAG_NACKF);
}

void i2cWriteSetup(I2C_TypeDef* I2Cx, uint8_t devAdd, uint16_t nbytesWrite)
{
	programState = I2C_BUSY;
	I2C_MasterRequestConfig(I2Cx, I2C_Direction_Transmitter);
	I2C_SlaveAddressConfig(I2Cx, devAdd);
	I2C_NumberOfBytesConfig(I2Cx, nbytesWrite);
	I2C_AutoEndCmd(I2Cx, DISABLE);
	I2C_GenerateSTART(I2Cx, ENABLE);
}

void i2cReadSetup(I2C_TypeDef* I2Cx, uint8_t devAdd, uint16_t nbytesRead)
{
	I2C_MasterRequestConfig(I2Cx, I2C_Direction_Receiver);
	I2C_SlaveAddressConfig(I2Cx, devAdd);
	I2C_NumberOfBytesConfig(I2Cx, nbytesRead);
	I2C_AutoEndCmd(I2Cx, ENABLE);
	I2C_GenerateSTART(I2Cx, ENABLE);
}

void i2cWrite(I2C_TypeDef* I2Cx, uint8_t devAdd, uint8_t regAdd, uint16_t nbytesWrite, uint8_t* data)
{
	// Wait for I2C bus to be free
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == 1);
	// Set device address, write, 7-bit addressing mode, write nbytes.
	I2C_MasterRequestConfig(I2Cx, I2C_Direction_Transmitter);
	I2C_SlaveAddressConfig(I2Cx, devAdd);
	I2C_NumberOfBytesConfig(I2Cx, nbytesWrite);
	I2C_AutoEndCmd(I2Cx, ENABLE);
	I2C_GenerateSTART(I2Cx, ENABLE);

	if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_NACKF) != I2C_FLAG_NACKF)
	{
		if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) == I2C_FLAG_TXE) I2C_SendData(I2Cx, regAdd);
	}
	else
	{
		// To observe NACK error
		GPIOB->ODR = 0xFF;
	}
	for (uint8_t i = 2; i <= nbytesWrite; i++)
	{

		while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == !I2C_FLAG_TXIS);
		if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_NACKF) == !I2C_FLAG_NACKF)
		{
			// Write data
			I2C_SendData(I2Cx, *data++);
		}
	}
}

void resetGyroOffset(void)
{
	for (int j = 0; j < 3; j++)
	{
		initialDataSumNew[j] = 0;
		initialDataSumOld[j] = 0;
		gyroOffset[j] = 0;
	}
	gyroDataSetCounter = 0;
	offsetState = OFFSET_UNKNOWN;
}

void determineOffset(int16_t gyroDataXT, int16_t gyroDataYT, int16_t gyroDataZT)
{
	// Sum of first 1000 readings to find ave. offset on gyro reading
	initialDataSumOld[0] = initialDataSumNew[0];
	initialDataSumOld[1] = initialDataSumNew[1];
	initialDataSumOld[2] = initialDataSumNew[2];

	initialDataSumNew[0] = gyroDataXT + initialDataSumOld[0];
	initialDataSumNew[1] = gyroDataYT + initialDataSumOld[1];
	initialDataSumNew[2] = gyroDataZT + initialDataSumOld[2];

	if (gyroDataSetCounter == 1000)
	{
		gyroOffset[0] = initialDataSumNew[0]/1000;
		gyroOffset[1] = initialDataSumNew[1]/1000;
		gyroOffset[2] = initialDataSumNew[2]/1000;
		offsetState = OFFSET_KNOWN;
	}

	gyroDataSetCounter++;
}

void monitorData(void)
{
	if (offsetState == OFFSET_KNOWN)
	{
		int16_t w_xg = w[1];
		int16_t w_yg = w[2];
		int16_t w_zg = w[3];
		uint8_t w_xgH = 0;
		uint8_t w_xgL = 0;
		uint8_t w_ygH = 0;
		uint8_t w_ygL = 0;
		uint8_t w_zgH = 0;
		uint8_t w_zgL = 0;
		uint32_t dSCC = datasetCounter;
		uint8_t dSLL = 0;
		uint8_t dSLH = 0;
		uint8_t dSHL = 0;
		uint8_t dSHH = 0;

		w_xgL = w_xg;
		w_xgH = (w_xg >> 8);
		w_ygL = w_yg;
		w_ygH = (w_yg >> 8);
		w_zgL = w_zg;
		w_zgH = (w_zg >> 8);

		dSLL = dSCC;
		dSLH = (dSCC >> 8);
		dSHL = (dSCC >> 16);
		dSHH = (dSCC >> 24);

		if (w_xgH == 73) w_xgH = 74;
		if (w_xgL == 73) w_xgL = 74;
		if (w_ygH == 73) w_ygH = 74;
		if (w_ygL == 73) w_ygL = 74;
		if (w_zgH == 73) w_zgH = 74;
		if (w_zgL == 73) w_zgL = 74;
		if (dSLL == 73) dSLL = 74;
		if (dSLH == 73) dSLH = 74;
		if (dSHL == 73) dSHL = 74;
		if (dSHH == 73) dSHH = 74;

		if (progState == STOP)
		{
			USART_SendData(USART1, 73);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			for (uint8_t i = 0; i < 10; i++)
			{
				if (i == 9) while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0);
				else while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
				USART_SendData(USART1, 0);
			}
		}
		else
		{
			USART_SendData(USART1, 73);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_xgH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_xgL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_ygH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_ygL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_zgH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_zgL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, dSHH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, dSHL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, dSLH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, dSLL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0);
		}
		targetPos = T_DATA_OLD;
		inertialData = IN_DATA_BUSY;
	}
}

// INTERRUPT SERVICE ROUTINES ---------------------------------------------------------------------------------------------------------------

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	// Acknowledge interrupt
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	if (progState == SETUP)
	{
		// Allow reading of the gyro and disable Timer
		gyroReadyState = GYRO_READY;
		NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);
		progState = MANUAL;
	}
}

void EXTI0_1_IRQHandler(void)
{
	// Clear interrupt flag
	EXTI_ClearFlag(EXTI_Line0);
	datasetCounter++;

	NVIC_DisableIRQ(EXTI4_15_IRQn);
	if (i2cError == ERROR_OCCURRED)
	{
		init_i2c();
		i2cError = NO_ERROR;
	}
	NVIC_EnableIRQ(I2C2_IRQn);
	i2cWriteSetup(I2C2, MPU9150ADD0, 1);
}

void I2C2_IRQHandler(void)
{
	uint32_t interruptCause = handleInterrupt();

	if 	((interruptCause == I2C_ISR_TC_RXNE) || (interruptCause == I2C_FLAG_RXNE) || (interruptCause == I2C_ISR_TC_TXIS) || (interruptCause == I2C_FLAG_TXIS) || (interruptCause == I2C_ISR_TC))
	{
		switch (i2cState)
		{
		// Read interrupt status register
		case READ_INT_STATUS:
			switch (interruptCause)
			{
			case I2C_ISR_TXIS:
				// Transmit address of INT_STATUS register
				I2C_SendData(I2C2, INT_STATUS);
				break;
			case I2C_ISR_TC:
				// Setup to read INT_STATUS register
				i2cReadSetup(I2C2, MPU9150ADD0, 1);
				break;
			case I2C_ISR_RXNE:
				// Receive data
				intStatus = I2C_ReceiveData(I2C2);
				if ((intStatus & DATA_RDY_INT) == DATA_RDY_INT)
				{
					i2cState = READ_GYRO_XOUTH;
					i2cWriteSetup(I2C2, MPU9150ADD0, 1);
				}
				break;
			}
			break;
		// Read gyro x-axis high register
		case READ_GYRO_XOUTH:
			switch (interruptCause)
			{
			case I2C_ISR_TXIS:
				// Transmit address of GYRO_XOUTH register
				I2C_SendData(I2C2, GYRO_XOUTH);
				break;
			case I2C_ISR_TC:
				// Setup to read GYRO_XOUTH register
				i2cReadSetup(I2C2, MPU9150ADD0, 1);
				break;
			case I2C_ISR_RXNE:
				// Receive data
				gyroXoutH[readCount] = I2C_ReceiveData(I2C2);
				i2cState = READ_GYRO_XOUTL;
				i2cWriteSetup(I2C2, MPU9150ADD0, 1);
				break;
			}
			break;
		// Read gyro x-axis low register
		case READ_GYRO_XOUTL:
			switch (interruptCause)
			{
			case I2C_ISR_TXIS:
				// Transmit address of GYRO_XOUTL register
				I2C_SendData(I2C2, GYRO_XOUTL);
				break;
			case I2C_ISR_TC:
				// Setup to read GYRO_XOUTL register
				i2cReadSetup(I2C2, MPU9150ADD0, 1);
				break;
			case I2C_ISR_RXNE:
				// Receive data
				gyroXoutL[readCount] = I2C_ReceiveData(I2C2);
				i2cState = READ_GYRO_YOUTH;
				i2cWriteSetup(I2C2, MPU9150ADD0, 1);
				break;
			}
			break;
		// Read gyro y-axis high register
		case READ_GYRO_YOUTH:
			switch (interruptCause)
			{
			case I2C_ISR_TXIS:
				// Transmit address of GYRO_YOUTH register
				I2C_SendData(I2C2, GYRO_YOUTH);
				break;
			case I2C_ISR_TC:
				// Setup to read GYRO_YOUTH register
				i2cReadSetup(I2C2, MPU9150ADD0, 1);
				break;
			case I2C_ISR_RXNE:
				// Receive data
				gyroYoutH[readCount] = I2C_ReceiveData(I2C2);
				i2cState = READ_GYRO_YOUTL;
				i2cWriteSetup(I2C2, MPU9150ADD0, 1);
				break;
			}
			break;
		// Read gyro y-axis low register
		case READ_GYRO_YOUTL:
			switch (interruptCause)
			{
			case I2C_ISR_TXIS:
				// Transmit address of GYRO_YOUTL register
				I2C_SendData(I2C2, GYRO_YOUTL);
				break;
			case I2C_ISR_TC:
				// Setup to read GYRO_YOUTL register
				i2cReadSetup(I2C2, MPU9150ADD0, 1);
				break;
			case I2C_ISR_RXNE:
				// Receive data
				gyroYoutL[readCount] = I2C_ReceiveData(I2C2);
				i2cState = READ_GYRO_ZOUTH;
				i2cWriteSetup(I2C2, MPU9150ADD0, 1);
				break;
			}
			break;
		// Read gyro z-axis high register
		case READ_GYRO_ZOUTH:
			switch (interruptCause)
			{
			case I2C_ISR_TXIS:
				// Transmit address of GYRO_ZOUTH register
				I2C_SendData(I2C2, GYRO_ZOUTH);
				break;
			case I2C_ISR_TC:
				// Setup to read GYRO_ZOUTH register
				i2cReadSetup(I2C2, MPU9150ADD0, 1);
				break;
			case I2C_ISR_RXNE:
				// Receive data
				gyroZoutH[readCount] = I2C_ReceiveData(I2C2);
				i2cState = READ_GYRO_ZOUTL;
				i2cWriteSetup(I2C2, MPU9150ADD0, 1);
				break;
			}
			break;
		// Read gyro z-axis low register
		case READ_GYRO_ZOUTL:
			switch (interruptCause)
			{
			case I2C_ISR_TXIS:
				// Transmit address of GYRO_ZOUTL register
				I2C_SendData(I2C2, GYRO_ZOUTL);
				break;
			case I2C_ISR_TC:
				// Setup to read GYRO_ZOUTL register
				i2cReadSetup(I2C2, MPU9150ADD0, 1);
				break;
			case I2C_ISR_RXNE:
				// Receive data
				gyroZoutL[readCount] = I2C_ReceiveData(I2C2);

				gyroDataXT = gyroXoutH[0];
				gyroDataXT = (gyroDataXT << 8) + gyroXoutL[0];
				gyroDataYT = gyroYoutH[0];
				gyroDataYT = (gyroDataYT << 8) + gyroYoutL[0];
				gyroDataZT = gyroZoutH[0];
				gyroDataZT = (gyroDataZT << 8) + gyroZoutL[0];

				if (offsetState == OFFSET_UNKNOWN) determineOffset(gyroDataXT, gyroDataYT, gyroDataZT);
				else
				{
					gyroDataXT = gyroDataXT - gyroOffset[0];
					gyroDataYT = gyroDataYT - gyroOffset[1];
					gyroDataZT = gyroDataZT - gyroOffset[2];

					/*if ((gyroDataXT > -5) && (gyroDataXT < 5)) gyroDataXT = 0;
					if ((gyroDataYT > -5) && (gyroDataYT < 5)) gyroDataYT = 0;
					if ((gyroDataZT > -5) && (gyroDataZT < 5)) gyroDataZT = 0;*/

					w[1] = gyroDataXT;
					w[2] = gyroDataYT;
					w[3] = gyroDataZT;


				}

				inertialData = IN_DATA_READY;
				i2cState = READ_INT_STATUS;
				// Enable interrupt for EXT4_15 to allow start of Pi Comms
				NVIC_EnableIRQ(EXTI4_15_IRQn);
				break;
			}
			break;
		}
	}
	else
	{
		// Disable GPIOF clock
		RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOF, DISABLE);
		// Disable I2C2 clock
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
		// Disable I2C2
		I2C_Cmd(I2C2, DISABLE);
		NVIC_DisableIRQ(I2C2_IRQn);
		i2cState = READ_INT_STATUS;
		i2cError = ERROR_OCCURRED;
	}
}

void EXTI2_3_IRQHandler(void)
{
	EXTI_ClearFlag(EXTI_Line2);
	//progState = STOP;
	NVIC_DisableIRQ(EXTI0_1_IRQn);
	offsetState = OFFSET_UNKNOWN;
	//NVIC_DisableIRQ(I2C2_IRQn);
}

void EXTI4_15_IRQHandler(void)
{
	// Clear interrupt flag
	EXTI_ClearFlag(EXTI_Line4);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

