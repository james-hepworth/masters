// ----------------------------------------------------------------------------
// Description
//
// Runs the control and comms algorithms for the Two-Axis Telescope Stabilization System.
//
//-------------------------------------------------------------------------------------------------------------------------------------------
// Declarations

#include "stm32f0xx.h"
#include "lcd_stm32f0.h"

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
void init_USART2(void);
void init_EXTI(void);
void init_ADC(void);
void init_Timer2_3(void);
void init_Timer1(uint16_t delayMilliseconds);
void init_MPU9150(void);
uint32_t handleInterrupt(void);
void i2cWriteSetup(I2C_TypeDef* I2Cx, uint8_t devAdd, uint16_t nbytesWrite);
void i2cReadSetup(I2C_TypeDef* I2Cx, uint8_t devAdd, uint16_t nbytesRead);
void i2cWrite(I2C_TypeDef* I2Cx, uint8_t devAdd, uint8_t regAdd, uint16_t nbytesWrite, uint8_t* data);
void resetGyroOffset(void);
void determineOffset(int16_t gyroDataXT, int16_t gyroDataYT, int16_t gyroDataZT);
void monitorData(void);
void controller(void);
void I2C2_IRQHandler(void);
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void EXTI0_1_IRQHandler(void);
void EXTI4_15_IRQHandler(void);
void USART2_IRQHandler(void);
void ADC_Read(void);
void limitErrorHandler(void);
void stopTargetTracker(void);
void writeStartUpLCD(void);
uint16_t trackStabController(uint8_t channel);
void yawServoLoop(float target);//, float target_p);
void pitchServoLoop(float target);//, float target_p);
float sin1(float x);
float cos1(float x);
int factorial(int x);

// STATE VARIABLES --------------------------------------------------------------------------------------------------------------------------

enum {READ_INT_STATUS = 0, READ_GYRO_XOUTH = 1, READ_GYRO_XOUTL = 2, READ_GYRO_YOUTH = 3, READ_GYRO_YOUTL = 4, READ_GYRO_ZOUTH = 5, READ_GYRO_ZOUTL = 6};
enum {NO_ERROR = 0, ERROR_OCCURRED = 1 /*BERR = 1, ARLO = 2, NACKF = 3*/};
enum {I2C_BUSY = 0, READY = 1};
enum {OFFSET_UNKNOWN = 0, OFFSET_KNOWN = 1};
enum {GYRO_NOT_READY = 0, GYRO_READY = 1};
enum {DATA_BUSY = 0, DATA_READY = 1};
enum {YAW_SHIFT_0 = 0, YAW_SHIFT_90 = 1, YAW_SHIFT_ADD = 2, YAW_SHIFT_SUB = 3};
enum {SETUP = 0, MANUAL = 1, AUTOMATIC = 2, LIMIT_ERROR = 3};
enum {YAW = 0, PITCH = 2};

uint8_t progState = SETUP;
uint8_t i2cState = READ_INT_STATUS;
uint8_t i2cError = NO_ERROR;
uint8_t programState = READY;
uint8_t offsetState = OFFSET_UNKNOWN;
uint8_t gyroReadyState = GYRO_NOT_READY;
uint8_t dataReady = DATA_BUSY;
uint8_t yawShift = YAW_SHIFT_0;
uint8_t stabChannel = YAW;

// GYRO DATA MEASUREMENT VARIABLES ---------------------------------------------------------------------------------------------------------------

volatile float c,s,The;

uint8_t intStatus = 0;
uint8_t readCount = 0;
uint8_t gyroXoutH[] = {0, 0};
uint8_t gyroXoutL[] = {0, 0};
uint8_t gyroYoutH[] = {0, 0};
uint8_t gyroYoutL[] = {0, 0};
uint8_t gyroZoutH[] = {0, 0};
uint8_t gyroZoutL[] = {0, 0};
uint16_t relYaw[]= {0, 0, 0, 0, 0};
uint16_t relPitch[1];
uint16_t relYawAve = 0;
uint16_t relPitchAve = 0;
uint16_t MA_P = 0;
uint16_t MAp_P[] = {0, 0};
uint16_t MA_Y = 0;
uint16_t MAp_Y[] = {0, 0};
uint8_t ADC_intCounter = 0;
uint8_t posCC = 0;
uint8_t negCC = 0;

uint8_t cnt = 0;

// TARGET POSITION VARIABLES ---------------------------------------------------------------------------------------------------------------

uint8_t targetData[4];
int16_t T[] = {0, 0};

uint8_t dataCounter = 0;

// DATA HANDLING VARIABLES ------------------------------------------------------------------------------------------------------------------

uint32_t datasetCounter = 0;
uint16_t gyroDataSetCounter = 0;
int32_t initialDataSumNew[] = {0, 0, 0};
int32_t initialDataSumOld[] = {0, 0, 0};
int16_t gyroOffset[] = {0, 0, 0};
int16_t gyroDataXT = 0;
int16_t gyroDataYT = 0;
int16_t gyroDataZT = 0;

float w[4];

uint8_t posCommand[2];
uint8_t comCounter = 0;
int16_t theta = 0;
int8_t psi = 0;
float thetaMapped_p = 0;
float psiMapped_p = 0;
float thetaMapped = 0;
float psiMapped = 0;
int16_t psi_i = 0;
int16_t theta_i = 0;

// CONTROL VARIABLES ------------------------------------------------------------------------------------------------------------------------
// TRACK ---------------------------------------------------------------------------------------------------------------------------------
float Kt = 0.0031;

float E_t[] = {0, 0, 0, 0};
float K_t[] = {0, 0, 0, 0};
float C_t[] = {0, 0, 0, 0};

const float U = 0.002;
const float V = 0.002;
const float W = 0.0348;
const float X = -0.0288;

// STAB ---------------------------------------------------------------------------------------------------------------------------------
float PI_r_LIM[] = {0.0103, 0.0147};
float Kr[] = {252450,178200};
float E_r[] = {0, 0, 0, 0};
float PI_r[] = {0, 0, 0, 0};
float K_r[] = {0, 0, 0, 0};
float C_r[] = {0, 0, 0, 0};

const float E[] = {0.1081, 0.1081};
const float F[] = {-0.1041, -0.1041};
const float G = 2;
const float H = -2;

const float I = 0.002;
const float J = 0.002;
const float K = 0.0041;
const float L = -0.0001;
uint16_t speed_yr = 0;
uint16_t speed_pr = 0;

// MANUAL POS -------------------------------------------------------------------------------------------------------------------------------
const float A = 0.6376;
const float B = -0.6356;
const float C = 2;
const float D = -2;

const float M = 1.5925;
const float N = -1.5906;
const float O = 0.0806;
const float P = -0.0786;

float V3[] = {0, 0};
float V4[] = {0, 0};
float V5[] = {0, 0};
float V6[] = {0, 0};
uint16_t speed = 0;

float V3y[] = {0, 0};
float V4y[] = {0, 0};
float V5y[] = {0, 0};
float V6y[] = {0, 0};
uint16_t speed_y = 0;
// Main -------------------------------------------------------------------------------------------------------------------------------------

void main(void)
{
	init_Pins();										//hal
	lcd_init();
	lcd_command(LCD_CLEAR_DISPLAY);
	lcd_string("INITIALSING");

	init_i2c();

	init_ADC();											//hal
	init_USART1(USART_WordLength_8b);					//hal
	init_USART2();										//hal
	init_MPU9150();
	init_Timer2_3();									//hal
	init_Timer1(50);									//hal

	// Wait for gyro to start up before progressing
	while (gyroReadyState == GYRO_NOT_READY);

	writeStartUpLCD();
	GPIOB->ODR &= ~(GPIO_Pin_0 & GPIO_Pin_1);
	GPIOB->ODR |= GPIO_Pin_2;
	GPIOA->ODR &= ~(GPIO_Pin_8 & GPIO_Pin_7);


	init_EXTI();										//hal

	while(1)
	{
		if (dataReady == DATA_READY)
		{
			monitorData();
		}
		//else if (progState == LIMIT_ERROR) monitorData();
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
	GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOB_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOB_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIOB_InitStruct);

	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);
}

void init_ADC(void)
{
	// Enable GPIOA clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	// Enable ADC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// Set PA5 and 6 to analogue mode
	GPIO_InitTypeDef ADC_PA_InitStruct;
	ADC_PA_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	ADC_PA_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	ADC_PA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	ADC_PA_InitStruct.GPIO_OType = GPIO_OType_PP;
	ADC_PA_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &ADC_PA_InitStruct);

	// Set ADC resolution before enabling the ADC, 10 bit resolution
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_10b;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStruct);
	// Select ADC channel 5 and 6
	ADC_ChannelConfig(ADC1, ADC_Channel_5 | ADC_Channel_6, 0);
	// Setup in wait mode, upward scan, 10 bit
	ADC_WaitModeCmd(ADC1, ENABLE);
	 // Calibrate ADC before enabling
	ADC_GetCalibrationFactor(ADC1);
	// Enable ADC
	ADC_Cmd(ADC1, ENABLE);
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
	USART1_InitStruct.USART_Parity = USART_Parity_Even;
	USART1_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART1_InitStruct);

	// Set baud rate to 115200 //19200 //9600 //
	USART1->BRR = 0x1A1;//0x9C4;//0x1388;/

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

	NVIC_EnableIRQ(USART1_IRQn);

}

// USART 2 used for micro-RPi comms: TX-PA2, RX-PA3
void init_USART2()
{
	// Clock to USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// PA9 and PA10 to AF
	GPIO_InitTypeDef USART2_PA_InitStruct;
	USART2_PA_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	USART2_PA_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	USART2_PA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	USART2_PA_InitStruct.GPIO_OType = GPIO_OType_PP;
	USART2_PA_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &USART2_PA_InitStruct);
	// Remap to correct AF: PA9 to AF1, PA10 to AF1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	SystemCoreClockUpdate();

	// Setup USART
	USART_InitTypeDef USART2_InitStruct;
	USART2_InitStruct.USART_BaudRate = 230400;
	USART2_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART2_InitStruct.USART_StopBits = USART_StopBits_1;
	USART2_InitStruct.USART_Parity = USART_Parity_Even;
	USART2_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART2_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART2_InitStruct);

	// Set baud rate to 19200 //9600 //115200
	USART2->BRR = 0xD0;//0x1A1;//0x9C4;//0x1388;//

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
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

void init_Timer2_3()
{
	// Enable Timer 2&3 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Set PB3 to AF2 for TIM2 CH2, PB4 to AF1 for TIM3 CH1
	GPIO_InitTypeDef TIM2_3_PB_InitStruct;
	TIM2_3_PB_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	TIM2_3_PB_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	TIM2_3_PB_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	TIM2_3_PB_InitStruct.GPIO_OType = GPIO_OType_PP;
	TIM2_3_PB_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &TIM2_3_PB_InitStruct);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);

	// Set TIM2_3 basic settings
	TIM_TimeBaseInitTypeDef TIM2_3_InitStruct;
	TIM2_3_InitStruct.TIM_Prescaler = 0;
	TIM2_3_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_3_InitStruct.TIM_Period = 2400;
	TIM2_3_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1 ;
	TIM2_3_InitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM2_3_InitStruct);
	TIM_TimeBaseInit(TIM3, &TIM2_3_InitStruct);

	// Set TIM2 OC settings
	TIM_OCInitTypeDef TIM2_OC_InitStruct;
	TIM2_OC_InitStruct.TIM_OCMode = TIM_OCMode_PWM1;					//TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
	TIM2_OC_InitStruct.TIM_OutputState = TIM_OutputState_Enable;		//TIM2->CCER |= TIM_CCER_CC2E;
	TIM2_OC_InitStruct.TIM_Pulse = 0;
	TIM2_OC_InitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM2, &TIM2_OC_InitStruct);

	// Set TIM3 OC settings
	TIM_OCInitTypeDef TIM3_OC_InitStruct;
	TIM3_OC_InitStruct.TIM_OCMode = TIM_OCMode_PWM1;					//TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
	TIM3_OC_InitStruct.TIM_OutputState = TIM_OutputState_Enable;		//TIM3->CCER |= TIM_CCER_CC1E;
	TIM3_OC_InitStruct.TIM_Pulse = 0;
	TIM3_OC_InitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM3_OC_InitStruct);

	// Set Update generation bit
	TIM_GenerateEvent(TIM2, TIM_FLAG_Update);  							//TIM2->EGR |= TIM_EGR_UG;
	TIM_GenerateEvent(TIM3, TIM_FLAG_Update);							//TIM3->EGR |= TIM_EGR_UG;

	// Enable auto-reload preload enable
	TIM_ARRPreloadConfig(TIM2, ENABLE);									//TIM2->CR1 |= TIM_CR1_ARPE;
	TIM_ARRPreloadConfig(TIM3, ENABLE);									//TIM3->CR1 |= TIM_CR1_ARPE;

	// Enable TIM2_3
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	TIM_SetCompare2(TIM2, 0);
	TIM_SetCompare1(TIM3, 0);
}

void init_EXTI(void)
{
	// Enable System Configuration clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	// Setup EXTI line 0 to be Port A
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	// Setup EXTI line 4 to be Port A
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);

	// Setup Rising edge trigger interrupt
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line4;
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
	/*else
	{
		// To observe NACK error
		GPIOB->ODR = 0xFF;
	}*/
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

void writeStartUpLCD(void)
{
	lcd_command(LCD_CLEAR_DISPLAY);
	lcd_string("SEND TARGET POS");
	lcd_command(LCD_GOTO_LINE_2);
	lcd_string("or AUTO MODE");
}

void ADC_Read(void)
{
	//relYaw[4] = relYaw[3];
	//relYaw[3] = relYaw[2];
	//relYaw[2] = relYaw[1];
	relYaw[1] = relYaw[0];
	ADC_StartOfConversion(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);
	relYaw[0] = ADC_GetConversionValue(ADC1);

	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);
	relPitch[0] = ADC_GetConversionValue(ADC1);

	MAp_Y[1] = MAp_Y[0];
	MAp_Y[0] = MAp_Y[1] + relYaw[0] - (MAp_Y[1]>>4);
	MA_Y = MAp_Y[0]>>4;

	MAp_P[1] = MAp_P[0];
	MAp_P[0] = MAp_P[1] + relPitch[0] - (MAp_P[1]>>4);
	MA_P = MAp_P[0]>>4;

	relPitchAve = MA_P;
	relYawAve = MA_Y;

	int16_t relYaw_1 = relYaw[1];
	int16_t relYaw_0 = relYaw[0];
	int16_t relYawDiff = relYaw_0 - relYaw_1;

	if (progState == AUTOMATIC)
	{
		if (relYawDiff > 200)//Approached from right
		{
			negCC++;
			if (posCC > 0) posCC--;
		}
		else if (relYawDiff < -200)
		{
			posCC++;
			if (negCC > 0) negCC--;
		}

		if ((posCC > 1) || (negCC > 1))
		{
			limitErrorHandler();
		}
		else if ((relPitch[0] < 140) || (relPitch[0] > 884))
		{
			limitErrorHandler();
		}
	}
}

void limitErrorHandler(void)
{
	NVIC_DisableIRQ(EXTI0_1_IRQn);
	NVIC_DisableIRQ(EXTI4_15_IRQn);
	TIM_SetCompare1(TIM3, 0);
	TIM_SetCompare2(TIM2, 0);
	progState = LIMIT_ERROR;
	lcd_command(LCD_CLEAR_DISPLAY);
	lcd_command(LCD_CURSOR_HOME);
	lcd_string("LIMIT ERROR");
	lcd_command(LCD_GOTO_LINE_2);
	lcd_string("Please reset");
	for (uint8_t i = 0; i < 10; i++)
	{
		monitorData();
	}
}

void yawServoLoop(float target)
{
	V3y[1] = V3y[0];
	V3y[0] = target - relYawAve;

	V4y[1] = V4y[0];
	V4y[0] = (A*V3y[0] + B*V3y[1] - D*V4y[1])/C;

	if (V4y[0] > 116.17) V4y[0] = 116.17;
	else if (V4y[0] < -116.17) V4y[0] = -116.17;

	V5y[1] = V5y[0];
	V5y[0] = V4y[0]*30.68;

	V6y[1] = V6y[0];
	V6y[0] = (M*V5y[0] + N*V5y[1] - P*V6y[1])/O;

	if (V6y[0] > 2376) V6y[0] = 2376;
	else if (V6y[0] < -2376) V6y[0] = -2376;

	if (V6y[0] >= 0)
	{
		speed_y = V6y[0];
		GPIO_SetBits(GPIOB, GPIO_Pin_5);
	}
	else if (V6y[0] < 0)
	{
		speed_y = -V6y[0];
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	}

	TIM_SetCompare2(TIM2, speed_y);

	if (yawShift == YAW_SHIFT_ADD)
	{
		ADC_Read();

		if ((relYawAve > 192) && (relYawAve < 200))
		{
			TIM_SetCompare2(TIM2, 0);
			yawShift = YAW_SHIFT_90;
		}
	}

	else if (yawShift == YAW_SHIFT_SUB)
	{
		ADC_Read();

		if ((relYawAve > 448) && (relYawAve < 456))
		{
			TIM_SetCompare2(TIM2, 0);
			yawShift = YAW_SHIFT_0;
		}
	}
}

void pitchServoLoop(float target)
{
	V3[1] = V3[0];
	V3[0] = target - relPitchAve;

	V4[1] = V4[0];
	V4[0] = (A*V3[0] + B*V3[1] - D*V4[1])/C;

	if (V4[0] > 193.61) V4[0] = 193.61;
	else if (V4[0] < -193.61) V4[0] = -193.61;

	V5[1] = V5[0];
	V5[0] = V4[0]*10;//8.41;

	V6[1] = V6[0];
	V6[0] = (M*V5[0] + N*V5[1] - P*V6[1])/O;

	if (V6[0] > 2376) V6[0] = 2376;
	else if (V6[0] < -2376) V6[0] = -2376;

	if (V6[0] >= 0)
	{
		speed = V6[0];
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	}
	else if (V6[0] < 0)
	{
		speed = -V6[0];
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
	}

	TIM_SetCompare1(TIM3, speed);
}

uint16_t trackStabController(uint8_t channel)
{
	uint16_t speed = 0;

	// TRACK ----------------------------------------------------------------------------------------------------------------------------
	E_t[1+channel] = E_t[0+channel];

	if (T[channel>>1] == 0)
	{
		E_t[0+channel] = 0;
		//if ((relPitchAve < 512) && (channel == YAW)) E_t[0] = -E_t[0];
	}
	else
	{
		E_t[0+channel] = (T[channel>>1] + (40*channel) - 320);
		if ((relPitchAve < 512) && (channel == YAW)) E_t[0+channel] = -E_t[0+channel];
	}

	K_t[1+channel] = K_t[0+channel];
	K_t[0+channel] = E_t[0+channel]*Kt;

	C_t[1+channel] = C_t[0+channel];
	C_t[0+channel] = (U*K_t[0+channel] + V*K_t[1+channel] - X*C_t[1+channel])/W;

	// STAB ----------------------------------------------------------------------------------------------------------------------------
	E_r[1+channel] = E_r[0+channel];
	E_r[0+channel] = C_t[0+channel] - (w[channel]/7509.64);//3754.82);
	if (channel == 0)
	{
		if ((The>1.396) && (The<1.745)) {
			E_r[0] = (C_t[0]+(w[1]/7509.64));
		}
		else {
			E_r[0] = E_r[0]/c;
		}
	}

	PI_r[1+channel] = PI_r[0+channel];
	PI_r[0+channel] = (E[channel>>1]*E_r[0+channel] + F[channel>>1]*E_r[1+channel] - H*PI_r[1+channel])/G;

	//LIMIT INTEGRATOR
	if (PI_r[0+channel] > PI_r_LIM[channel>>1]) PI_r[0+channel] = PI_r_LIM[channel>>1];
	else if (PI_r[0+channel] < -PI_r_LIM[channel>>1]) PI_r[0+channel] = -PI_r_LIM[channel>>1];

	K_r[1+channel] = K_r[0+channel];
	K_r[0+channel] = PI_r[0+channel]*Kr[channel>>1];


	C_r[1+channel] = C_r[0+channel];
	C_r[0+channel] = (I*K_r[0+channel] + J*K_r[1+channel] - L*C_r[1+channel])/K;

	//LIMIT COMPENSATOR FOR PWM
	if (C_r[0+channel] > 2376) C_r[0+channel] = 2376;
	else if (C_r[0+channel] < -2376) C_r[0+channel] = -2376;

	if (C_r[0+channel] >= 0)
	{
		speed = C_r[0+channel];
		GPIO_ResetBits(GPIOB, 1<<(5 + (channel>>1)));
	}
	else if (C_r[0+channel] < 0)
	{
		speed = -C_r[0+channel];
		GPIO_SetBits(GPIOB, 1<<(5 + (channel>>1)));
	}

	return speed;
}

float cos1(float x)
{
	int i,fact=1;
	float sum,term;
	sum=1.0;
	term=1.0;
	for(i=1;i<5;i++)
	{
		term=term*x*x;
		term=term*(-1);
		fact=factorial(2*i);
		sum=sum+(term/fact);
	}
	return(sum);
}

float sin1(float x)
{
	int i,fact=1;
	float sum,term;
	sum=x;
	term=x;
	for(i=1;i<5;i++)
	{
		term=(term/x)*x*x*x;
		term=term*(-1);
		fact=factorial(2*i+1);
		sum=sum+(term/fact);
	}
	return(sum);
}

int factorial(int x)
    {
    int fact;
    if(x==1) return(1);
    else fact=x*factorial(x-1);
    return(fact);
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

void stopTargetTracker(void)
{
	for (uint32_t i = 0; i < 200000; i++)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_1);
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	}
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
		uint16_t cxC = T[0];
		uint16_t cyC = T[1];
		int16_t w_xp = w[1];
		int16_t w_yp = w[2];
		int16_t w_zp = w[3];
		uint8_t w_xpH = 0;
		uint8_t w_xpL = 0;
		uint8_t w_ypH = 0;
		uint8_t w_ypL = 0;
		uint8_t w_zpH = 0;
		uint8_t w_zpL = 0;
		uint8_t relYawL = 0;
		uint16_t relYawCC = relYawAve;
		uint8_t relPitchL = 0;
		uint16_t relPitchCC = relPitchAve;
		uint8_t relYawPitchMix = 0;
		uint8_t cxL = (cxC & 0xFF);
		uint8_t cxH = (cxC >> 8);
		uint8_t cyL = (cyC & 0xFF);
		uint8_t cyH = (cyC >> 8);
		uint32_t dSCC = datasetCounter;
		uint8_t dSLL = 0;
		uint8_t dSLH = 0;
		uint8_t dSHL = 0;
		uint8_t dSHH = 0;

		w_xpL = w_xp;
		w_xpH = (w_xp >> 8);
		w_ypL = w_yp;
		w_ypH = (w_yp >> 8);
		w_zpL = w_zp;
		w_zpH = (w_zp >> 8);

		relYawPitchMix = (((relYawCC >> 8) << 4) + (relPitchCC >> 8));
		relYawL = relYawAve;
		relPitchL = relPitchAve;

		dSLL = dSCC;
		dSLH = (dSCC >> 8);
		dSHL = (dSCC >> 16);
		dSHH = (dSCC >> 24);

		if (w_xpH == 73) w_xpH = 74;
		if (w_xpL == 73) w_xpL = 74;
		if (w_ypH == 73) w_ypH = 74;
		if (w_ypL == 73) w_ypL = 74;
		if (w_zpH == 73) w_zpH = 74;
		if (w_zpL == 73) w_zpL = 74;
		if (relYawL == 73) relYawL = 74;
		if (relPitchL == 73) relPitchL = 74;
		if (relYawPitchMix == 73) relYawPitchMix = 74;
		if (cxL == 73) cxL = 74;
		if (cxH == 73) cxH = 74;
		if (cyL == 73) cyL = 74;
		if (cyH == 73) cyH = 74;
		if (dSLL == 73) dSLL = 74;
		if (dSLH == 73) dSLH = 74;
		if (dSHL == 73) dSHL = 74;
		if (dSHH == 73) dSHH = 74;

		if (progState == LIMIT_ERROR)
		{
			USART_SendData(USART1, 73);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			for (uint8_t i = 0; i < 17; i++)
			{
				if (i == 16) while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0);
				else while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
				USART_SendData(USART1, 0);
			}
		}
		else
		{
			USART_SendData(USART1, 73);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_xpH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_xpL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_ypH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_ypL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_zpH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, w_zpL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, relYawL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, relPitchL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, relYawPitchMix);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, cxH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, cxL);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, cyH);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);
			USART_SendData(USART1, cyL);
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
		dataReady = DATA_BUSY;
	}
}

void controller(void)
{
//	volatile float c,s,The;
	The = 0.0061*(relPitchAve-4)-1.571;
	c = cos1(The);
	s = sin1(The);

	w[0] = w[3];//w[3]*c - w[1]*s;

	if (((((The > 1.553) && (The < 1.588)) && ((T[0] > 325) || (T[0]  < 315))) && (yawShift == YAW_SHIFT_0)) && ((T[1]  > 238) && (T[1] < 242)))
	{
		yawShift = YAW_SHIFT_ADD;
	}
	if (((((The > 1.553) && (The < 1.588)) && ((T[0] > 325) || (T[0]  < 315))) && (yawShift == YAW_SHIFT_90)) && ((T[1] > 238) && (T[1] < 242)))
	{
		yawShift = YAW_SHIFT_SUB;
	}
	if ((yawShift == YAW_SHIFT_90) && (relYawAve > 452))
	{
		yawShift = YAW_SHIFT_0;
	}
	if ((progState == AUTOMATIC) && ((yawShift == YAW_SHIFT_0) || (yawShift == YAW_SHIFT_90)))
	{
		speed_yr = trackStabController(YAW);
		speed_pr = trackStabController(PITCH);

		TIM_SetCompare2(TIM2, speed_yr);
		TIM_SetCompare1(TIM3, speed_pr);
	}
	else if ((progState == AUTOMATIC) && (yawShift == YAW_SHIFT_ADD))
	{
		yawServoLoop(196);
	}
	else if ((progState == AUTOMATIC) && (yawShift == YAW_SHIFT_SUB))
	{
		yawServoLoop(452);
	}
	else
	{
		TIM_SetCompare2(TIM2, 0);
		TIM_SetCompare1(TIM3, 0);
	}
}

// INTERRUPT SERVICE ROUTINES ---------------------------------------------------------------------------------------------------------------

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	// Acknowledge interrupt
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

	ADC_Read();

	if (progState == SETUP)
	{
		// Allow reading of the gyro and disable Timer
		gyroReadyState = GYRO_READY;
		NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);
		progState = MANUAL;
	}

	else if (progState == MANUAL)
	{
		pitchServoLoop(thetaMapped);//, thetaMapped_p);
		yawServoLoop(psiMapped);//, psiMapped_p);
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
				else i2cError = ERROR_OCCURRED;
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

				ADC_Read();

				if (offsetState == OFFSET_UNKNOWN) determineOffset(gyroDataXT, gyroDataYT, gyroDataZT);
				else
				{
					gyroDataXT = gyroDataXT - gyroOffset[0];
					gyroDataYT = gyroDataYT - gyroOffset[1];
					gyroDataZT = gyroDataZT - gyroOffset[2];

					/*if ((gyroDataXT > -7) && (gyroDataXT < 7)) gyroDataXT = 0;
					if ((gyroDataYT > -7) && (gyroDataYT < 7)) gyroDataYT = 0;
					if ((gyroDataZT > -7) && (gyroDataZT < 7)) gyroDataZT = 0;*/

					w[1] = -gyroDataZT;
					w[2] = gyroDataYT;
					w[3] = gyroDataXT;

					controller();
				}

				dataReady = DATA_READY;
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

void EXTI4_15_IRQHandler(void)
{
	// Clear interrupt flag
	EXTI_ClearFlag(EXTI_Line4);
	NVIC_DisableIRQ(EXTI0_1_IRQn);
	GPIO_ResetBits(GPIOB, GPIO_Pin_2);//Pi pin_37 - lowered to trigger Cx, Cy send
}

void USART2_IRQHandler(void)
{
	targetData[dataCounter] = USART_ReceiveData(USART2);

	if (dataCounter < 3) dataCounter++;
	else
	{
		T[0] = (targetData[0] << 8) + targetData[1];
		T[1] = (targetData[2] << 8) + targetData[3];
		dataCounter = 0;

		GPIO_SetBits(GPIOB, GPIO_Pin_2);

		NVIC_EnableIRQ(EXTI0_1_IRQn);
	}
}

void USART1_IRQHandler(void)
{
	posCommand[comCounter] = USART_ReceiveData(USART1);

	if (comCounter == 0) comCounter = 1;
	else
	{
		TIM_SetCompare2(TIM2, 0);
		TIM_SetCompare1(TIM3, 0);

		if ((posCommand[0] == 150) && (posCommand[1] = 150))
		{
			progState = AUTOMATIC;
			resetGyroOffset();

			NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
			RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);

			lcd_command(LCD_CLEAR_DISPLAY);
			lcd_string("TRACK & STAB");

			USART_Cmd(USART1, DISABLE);
			init_USART1(USART_WordLength_9b);					// For some reason wordlength must be 9 bits when sending data to LabView UI

			GPIO_SetBits(GPIOB, (GPIO_Pin_0 | GPIO_Pin_7));

			// Activate relevant interrupts for Automatic State
			NVIC_EnableIRQ(USART2_IRQn);
			NVIC_EnableIRQ(EXTI0_1_IRQn);
			comCounter = 0;
			datasetCounter = 0;
		}
		else if ((posCommand[0] == 120) && (posCommand[1] = 136/*-120*/))
		{
			NVIC_DisableIRQ(I2C2_IRQn);
			progState = MANUAL;
			resetGyroOffset();

			NVIC_DisableIRQ(EXTI0_1_IRQn);
			GPIO_SetBits(GPIOB, GPIO_Pin_8);
			NVIC_DisableIRQ(EXTI4_15_IRQn);

			USART_Cmd(USART1, DISABLE);
			init_USART1(USART_WordLength_8b);					// For some reason wordlength must be 8 bits when receiving data from LabView UI

			for (uint16_t i = 0; i < 5000; i++)
			{
				GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_7);
			}

			writeStartUpLCD();
			comCounter = 0;
		}

		// STOP pressed
		else if ((posCommand[0] == 136/*-120*/) && (posCommand[1] == 136/*-120*/))
		{
			NVIC_DisableIRQ(I2C2_IRQn);
			NVIC_DisableIRQ(EXTI4_15_IRQn);
			USART_Cmd(USART1, DISABLE);
			progState = SETUP;
			GPIO_SetBits(GPIOB, GPIO_Pin_8);
			stopTargetTracker();
			lcd_command(LCD_CLEAR_DISPLAY);
			lcd_string("SYSTEM STOP");
		}
		else
		{
			uint8_t tHundreds = 0;
			uint8_t tTens = 0;
			uint8_t tUnits = 0;
			uint8_t pHundreds = 0;
			uint8_t pTens = 0;
			uint8_t pUnits = 0;
			int8_t posCommand_1 = 0;

			theta = posCommand[0];
			posCommand_1 = posCommand[1];
			psi = posCommand_1;

			int16_t thetaC = theta;
			int16_t psiC = psi;

			thetaMapped_p = thetaMapped;
			thetaMapped = 2.844*(theta-3) + 256;
			psiMapped_p = psiMapped;
			psiMapped = -2.844*(psi) + 512;

			init_Timer1(1);

			lcd_command(LCD_CLEAR_DISPLAY);

			if (thetaC < 0)
			{
				thetaC = thetaC*(-1);
				tHundreds = (thetaC/100)+48;
				thetaC = thetaC -100*(tHundreds-48);
				tTens = (thetaC/10)+48;
				tUnits =  thetaC - 10*(tTens-48) + 48;

				lcd_command(LCD_CURSOR_HOME);
				lcd_string("Theta: -");
				lcd_num(tHundreds);
				lcd_num(tTens);
				lcd_num(tUnits);
			}
			else
			{
				tHundreds = (thetaC/100)+48;
				thetaC = thetaC -100*(tHundreds-48);
				tTens = (thetaC/10)+48;
				tUnits =  thetaC - 10*(tTens-48) + 48;

				lcd_command(LCD_CURSOR_HOME);
				lcd_string("Theta: +");
				lcd_num(tHundreds);
				lcd_num(tTens);
				lcd_num(tUnits);
			}

			if (psiC <0)
			{
				psiC = psiC*(-1);
				pHundreds = (psiC/100)+48;
				psiC = psiC -100*(pHundreds-48);
				pTens = (psiC/10)+48;
				pUnits =  psiC - 10*(pTens-48) + 48;

				lcd_command(LCD_GOTO_LINE_2);
				lcd_string("Psi: -");
				lcd_num(pHundreds);
				lcd_num(pTens);
				lcd_num(pUnits);
			}
			else
			{
				pHundreds = (psiC/100)+48;
				psiC = psiC -100*(pHundreds-48);
				pTens = (psiC/10)+48;
				pUnits =  psiC - 10*(pTens-48) + 48;

				lcd_command(LCD_GOTO_LINE_2);
				lcd_string("Psi: +");
				lcd_num(pHundreds);
				lcd_num(pTens);
				lcd_num(pUnits);
			}

			comCounter = 0;
		}
	}
}

