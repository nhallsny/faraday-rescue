/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "BQ769x2Header.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

/* Program Options */
#define DEBUG 0 //Debug mode disables RS485 communications to the bike and instead enables debug messages over RS485, including printf.
#define LEDS 1 //Set to 1 to enable LEDs, 0 to disable. LEDs consume surprisingly large amounts of power because of the linear regulator from 48V to 3.3V
#define WATCHDOG 1 //set to 1 to enable watchdog. Good for production, bad for debugging

/*BQ Parameters */
#define DEV_ADDR  0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_MODE 1  // 0 for disabled, 1 for enabled
#define ACTIVE_CELLS 0xAAFF //bitfield which indicates which cells to have protections on. Specific to this Faraday 12S pack

/*Pin Definitions
 * --------------
 * Note - I2C Pin definitions and RS485 TX/RX definitions are handled in HAL init functions*/

//LED that indicates CHG state
#define LED_CHG_PORT GPIOB
#define LED_CHG_PIN GPIO_PIN_0

//LED that indicates DSG state
#define LED_DSG_PORT GPIOB
#define LED_DSG_PIN GPIO_PIN_1

//Active-high to veto charge FET
#define CFETOFF_PORT GPIOB
#define CFETOFF_PIN GPIO_PIN_3

//Active-high to veto discharge FET
#define DFETOFF_PORT GPIOB
#define DFETOFF_PIN GPIO_PIN_4

//Active-low on-button from faraday bike. Used in this file and stm32l0xx_it.c
#define BUTTON_PORT GPIOA
#define BUTTON_PIN GPIO_PIN_0

//RST_SHUT_CTRL used by STM to change BQ sleep states
#define RST_SHUT_CTRL_PORT GPIOA
#define RST_SHUT_CTRL_PIN GPIO_PIN_1

//REG18 analog value from BQ chip. Read to check if BQ is in SHUTDOWN
#define REG18_AIN_PORT GPIOA
#define REG18_AIN_PIN GPIO_PIN_5

//LED2 General Purpose
#define LED_BF_PORT GPIOA
#define LED_BF_PIN GPIO_PIN_8

//LED3 General Purpose
#define LED_SF_PORT GPIOA
#define LED_SF_PIN GPIO_PIN_9

//Alert configurable output from BQ chip. Not used in this file, set in stm32l0xx_it.c
#define ALERT_PORT GPIOB
#define ALERT_PIN GPIO_PIN_5

//UART_TX_EN - enable transmit on RS485 transceiver, active high
#define UART_TX_EN_PORT GPIOA
#define UART_TX_EN_PIN GPIO_PIN_6

//UART_RX_EN - enable receive on RS485 transceiver, active low
#define UART_RX_EN_PORT GPIOA
#define UART_RX_EN_PIN GPIO_PIN_7

/* User-Facing Timing Parameters */
#define RETRY_LIMIT 100 //number of times the STM will retry communications with BQ before calling for a hard reset
#define INACTIVITY_LOOPS_MAX 15000 //number of loops without significant battery current before bike goes into sleep. Normally around 5 minutes for 10,000 loops
#define BUTTON_LONG_PRESS_LOOPS 15 //number of loops that count as a long press
#define UART_TIMEOUT_S 3 //default timeout for RS485 in seconds
#define PACK_CURRENT_INACTIVITY_LOWER_LIMIT_MA -250
#define PACK_CURRENT_INACTIVITY_UPPER_LIMIT_MA 75

/* Fake Enums */
#define R 0 // Read; Used in BQ769x2_DirectCommand and Subcommands functions
#define W 1 // Write; Used in BQ769x2_DirectCommand and Subcommands functions
#define W2 2 // Write data with two bytes; Used in Subcommands function

/*Uart Variables flag*/
__IO ITStatus UartBusy = RESET;

/* USER CODE BEGIN PV */
#define MAX_BUFFER_SIZE 32
uint8_t RX_data[2] = { 0x00, 0x00 }; // used in several functions to store data read from BQ769x2
uint8_t RX_32Byte[32] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
int16_t CellVoltage[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
int16_t CellMinV = 0;
int16_t CellMaxV = 0;
float Temperature[4] = { 0, 0, 0, 0 };
int8_t Offset[4] = { 0, 0, 0, 0 };
uint16_t Stack_Voltage = 0x00;
uint16_t Pack_Voltage = 0x00;
uint16_t LD_Voltage = 0x00;
int16_t Pack_Current = 0x00;
uint16_t test_var = 0x00;
uint16_t AlarmBits = 0x00;
uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_PFStatusA;   // Permanent Fail Status Register A
uint8_t value_PFStatusB;   // Permanent Fail Status Register B
uint8_t value_PFStatusC;   // Permanent Fail Status Register C
uint16_t value_BatteryStatus; //battery status a
uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs
uint16_t CB_ActiveCells;  // Cell Balancing Active Cells
uint8_t UV_Fault = 0;   // under-voltage fault state
uint8_t OV_Fault = 0;   // over-voltage fault state
uint8_t SCD_Fault = 0;  // short-circuit fault state
uint8_t OCD_Fault = 0;  // over-current fault state
uint8_t ProtectionsTriggered = 0; // Set to 1 if any protection triggers
uint8_t LD_ON = 0;	// Load Detect status bit
uint8_t Dsg = 0;   // discharge FET state
uint8_t Chg = 0;   // charge FET state
uint8_t PChg = 0;  // pre-charge FET state
uint8_t PDsg = 0;  // pre-discharge FET state

uint32_t AccumulatedCharge_Int; // in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Frac; // in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Time; // in BQ769x2_READPASSQ func

//Used for state machine
uint8_t ButtonCount = 0;
uint16_t CRC_Fail = 0;
uint16_t InactivityCount = 0;
uint16_t RetryCount = 0;
uint8_t ResetByWatchdog = 0;
uint8_t ButtonPressedDuringBoot = 0;

//Used by UART
uint8_t UART_RxData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t UART_TxData[32] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void EXTI0_1_IRQHandler_Config(void);
static void EXTI2_3_IRQHandler_Config(void);
static void THVD2410_Sleep();
static void THVD2410_Transmit();
static void THVD2410_Receive();
static uint8_t UART_PrepCellVoltageMessage();
static uint8_t UART_PrepCellBalancingMessage();
static uint8_t UART_PrepBatteryStatusMessage1();
static uint8_t UART_PrepBatteryStatusMessage2();
uint8_t STM32_Wake_Button_Pressed();
uint16_t BQ769x2_ReadControlStatus();
void Sleep();

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */

/* USER CODE BEGIN PFP */

/* Helper Functions -----------------------------------------------*/

void delayUS(uint32_t us) {   // Sets the delay in microseconds.
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;
}

void delayMS(uint32_t ms) {
	for (int i = 0; i < ms; i++) {
		delayUS(1000);
	}
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count) {
	uint8_t copyIndex = 0;
	for (copyIndex = 0; copyIndex < count; copyIndex++) {
		dest[copyIndex] = source[copyIndex];
	}
}

float ReverseFloat(const float inFloat) {
	float retVal;
	char *floatToConvert = (char*) &inFloat;
	char *returnFloat = (char*) &retVal;

	// swap the bytes into a temporary buffer
	returnFloat[0] = floatToConvert[3];
	returnFloat[1] = floatToConvert[2];
	returnFloat[2] = floatToConvert[1];
	returnFloat[3] = floatToConvert[0];

	return retVal;
}

uint32_t FloatToUInt(float n) {
	return (uint32_t) (*(uint32_t*) &n);
}

float UIntToFloat(uint32_t n) {
	return (float) (*(float*) &n);
}

float unpackFloat(const void *buf) {
	const unsigned char *b = (const unsigned char*) buf;
	uint32_t temp = 0;
	temp = ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
	return *((float*) &temp);
}

unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
{
	unsigned char i;
	unsigned char checksum = 0;

	for (i = 0; i < len; i++)
		checksum += ptr[i];

	checksum = 0xff & ~checksum;

	return (checksum);
}

unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions
{
	unsigned char i;
	unsigned char crc = 0;
	while (len-- != 0) {
		for (i = 0x80; i != 0; i /= 2) {
			if ((crc & 0x80) != 0) {
				crc *= 2;
				crc ^= 0x107;
			} else
				crc *= 2;

			if ((*ptr & i) != 0)
				crc ^= 0x107;
		}
		ptr++;
	}
	return (crc);
}

/* BQ Specific I2C Functions -----------------------------------------------*/
void I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
	uint8_t TX_Buffer[MAX_BUFFER_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00 };
#if CRC_MODE
	{
		uint8_t crc_count = 0;
		crc_count = count * 2;
		uint8_t crc1stByteBuffer[3] = { 0x10, reg_addr, reg_data[0] };
		unsigned int j;
		unsigned int i;
		uint8_t temp_crc_buffer[3];

		TX_Buffer[0] = reg_data[0];
		TX_Buffer[1] = CRC8(crc1stByteBuffer, 3);

		j = 2;
		for (i = 1; i < count; i++) {
			TX_Buffer[j] = reg_data[i];
			j = j + 1;
			temp_crc_buffer[0] = reg_data[i];
			TX_Buffer[j] = CRC8(temp_crc_buffer, 1);
			j = j + 1;
		}

		HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, TX_Buffer, crc_count,
				1000);

	}
#else
	HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
}

/* hacked version that doesn't segfault */
int I2C_BQ769x2_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
	unsigned int RX_CRC_Fail = 0; // reset to 0. If in CRC Mode and CRC fails, this will be incremented.
	uint8_t RX_Buffer[MAX_BUFFER_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00 };

	//uint8_t RX_Buffer[MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#if CRC_MODE
	{
		uint8_t crc_count = 0;
		//uint8_t ReceiveBuffer [10] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		uint8_t ReceiveBuffer[32] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00 };
		crc_count = count * 2;
		unsigned int j;
		unsigned int i;
		unsigned char CRCc = 0;
		uint8_t temp_crc_buffer[3];

		HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, ReceiveBuffer,
				crc_count, 1000);
		uint8_t crc1stByteBuffer[4] = { 0x10, reg_addr, 0x11, ReceiveBuffer[0] };
		CRCc = CRC8(crc1stByteBuffer, 4);
		if (CRCc != ReceiveBuffer[1]) {
			RX_CRC_Fail += 1;
		}
		RX_Buffer[0] = ReceiveBuffer[0];

		j = 2;
		for (i = 1; i < count; i++) {
			RX_Buffer[i] = ReceiveBuffer[j];
			temp_crc_buffer[0] = ReceiveBuffer[j];
			j = j + 1;
			CRCc = CRC8(temp_crc_buffer, 1);
			if (CRCc != ReceiveBuffer[j])
				RX_CRC_Fail += 1;
			j = j + 1;
		}
		CopyArray(RX_Buffer, reg_data, crc_count);
		CRC_Fail += RX_CRC_Fail;
	}
#else
	HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
	return 0;
}

/* BQ Functions -----------------------------------------------*/
void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen) {
	uint8_t TX_Buffer[2] = { 0x00, 0x00 };
	uint8_t TX_RegData[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	//TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff;
	TX_RegData[1] = (reg_addr >> 8) & 0xff;
	TX_RegData[2] = reg_data & 0xff; //1st byte of data

	switch (datalen) {
	case 1: //1 byte datalength
		I2C_WriteReg(0x3E, TX_RegData, 3);
		delayUS(2000);
		TX_Buffer[0] = Checksum(TX_RegData, 3);
		TX_Buffer[1] = 0x05; //combined length of register address and data
		I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
		delayUS(2000);
		break;
	case 2: //2 byte datalength
		TX_RegData[3] = (reg_data >> 8) & 0xff;
		I2C_WriteReg(0x3E, TX_RegData, 4);
		delayUS(2000);
		TX_Buffer[0] = Checksum(TX_RegData, 4);
		TX_Buffer[1] = 0x06; //combined length of register address and data
		I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
		delayUS(2000);
		break;
	case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
		TX_RegData[3] = (reg_data >> 8) & 0xff;
		TX_RegData[4] = (reg_data >> 16) & 0xff;
		TX_RegData[5] = (reg_data >> 24) & 0xff;
		I2C_WriteReg(0x3E, TX_RegData, 6);
		delayUS(2000);
		TX_Buffer[0] = Checksum(TX_RegData, 6);
		TX_Buffer[1] = 0x08; //combined length of register address and data
		I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
		delayUS(2000);
		break;
	}
	delayMS(2);
}

void BQ769x2_CommandSubcommand(uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{ //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

	uint8_t TX_Reg[2] = { 0x00, 0x00 };

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	I2C_WriteReg(0x3E, TX_Reg, 2);
	delayMS(2);
}

void BQ769x2_ReadReg(uint16_t command, uint16_t data, uint8_t type) {
	//security keys and Manu_data writes dont work with this function (reading these commands works)
	//max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot. Seems to segfault above 16 bytes
	uint8_t TX_Reg[4] = { 0x00, 0x00, 0x00, 0x00 };

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	if (type == R) {	//read
		I2C_WriteReg(0x3E, TX_Reg, 2);
		delayUS(2000);
		I2C_BQ769x2_ReadReg(0x40, RX_32Byte, 16); //RX_32Byte is a global variable, but CRC fails if I read more than 16 bytes
	}
}

//read data from Subcommands. Max readback size is 16 bytes because of a bug that would cause CRC errors with readbacks longer than 16.
void BQ769x2_Subcommand(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
	//security keys and Manu_data writes dont work with this function (reading these commands works)
	//max readback size is 16 bytes i.e. DASTATUS, CUV/COV snapshot are not supported by this function
	uint8_t TX_Reg[4] = { 0x00, 0x00, 0x00, 0x00 };
	uint8_t TX_Buffer[2] = { 0x00, 0x00 };

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	if (type == R) {	//read
		I2C_WriteReg(0x3E, TX_Reg, 2);
		delayUS(2000);
		//I2C_BQ769x2_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
		I2C_BQ769x2_ReadReg(0x40, RX_32Byte, 16); //more then 16 would cause CRC errors for a reason I didn't dig into, so I limit this to 16. This does mean that large data reads are not supported.
	} else if (type == W) {
		//FET_Control, REG12_Control
		TX_Reg[2] = data & 0xff;
		I2C_WriteReg(0x3E, TX_Reg, 3);
		delayUS(2000);
		TX_Buffer[0] = Checksum(TX_Reg, 3);
		TX_Buffer[1] = 0x05; //combined length of registers address and data
		I2C_WriteReg(0x60, TX_Buffer, 2);
	} else if (type == W2) { //write data with 2 bytes
		//CB_Active_Cells, CB_SET_LVL
		TX_Reg[2] = data & 0xff;
		TX_Reg[3] = (data >> 8) & 0xff;
		I2C_WriteReg(0x3E, TX_Reg, 4);
		delayUS(1000);
		TX_Buffer[0] = Checksum(TX_Reg, 4);
		TX_Buffer[1] = 0x06; //combined length of registers address and data
		I2C_WriteReg(0x60, TX_Buffer, 2);
	}
	delayMS(2);
}

void BQ769x2_DirectCommand(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{	//type: R = read, W = write
	uint8_t TX_data[2] = { 0x00, 0x00 };

	//little endian format
	TX_data[0] = data & 0xff;
	TX_data[1] = (data >> 8) & 0xff;

	if (type == R) {	//Read
		I2C_BQ769x2_ReadReg(command, RX_data, 2); //RX_data is a global variable
		delayUS(2000);
	}
	if (type == W) { //write
		//Control_status, alarm_status, alarm_enable all 2 bytes long
		I2C_WriteReg(command, TX_data, 2);
		delayUS(2000);
	}
}



uint16_t BQ769x2_ReadUnsignedRegister(uint16_t reg_addr, uint8_t count) {
	// Read Unsigned Register of 1 or 2 byte length
	BQ769x2_Subcommand(reg_addr, 0x00, R);
	switch (count) {
	case 1:
		return RX_32Byte[0];
	case 2:
		return (RX_32Byte[1] * 256 + RX_32Byte[0]);
	}
	return 0;
}

int16_t BQ769x2_ReadSignedRegister(uint16_t reg_addr, uint8_t count) {
	// Read signed Register of 1 or 2 byte length
	BQ769x2_Subcommand(reg_addr, 0x00, R);
	switch (count) {
	case 1:
		return RX_32Byte[0];
	case 2:
		return (RX_32Byte[1] * 256 + RX_32Byte[0]);
	}

	return 0;
}

float BQ769x2_ReadFloatRegister(uint16_t reg_addr) {
	// Read a 4 byte float (CC_Gain and Capacity Gain Only)
	BQ769x2_ReadReg(reg_addr, 0x00, R);

	const unsigned char *b = (const unsigned char*) RX_32Byte;
	uint32_t temp = 0;
	temp = ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
	return ReverseFloat(*((float*) &temp));

	//Example Fault Flags
}

void BQ769x2_Set_Confirm_Register(uint16_t reg_addr, uint32_t reg_data,
		uint8_t datalen) {
	//set and then verify that a register has been set
	uint8_t tries = 0;
	uint32_t buf = 0;
	while (tries < 10) {
		BQ769x2_SetRegister(reg_addr, reg_data, datalen);
		buf = BQ769x2_ReadUnsignedRegister(reg_addr, datalen);
		if (reg_data == buf) {
			return;
		}
		tries++;
	}
}

void BQ769x2_ReadBatteryStatus() {
	// Read Battery Status with DirectCommand
	// This shows which primary protections have been triggered
	BQ769x2_DirectCommand(BatteryStatus, 0x00, R);
	value_BatteryStatus = (RX_data[1] * 256 + RX_data[0]);
}

uint16_t BQ769x2_ReadDeviceNumber() {
	// Read Device Number with SubCommand
	BQ769x2_Subcommand(DEVICE_NUMBER, 0x00, R);
	return (RX_32Byte[1] * 256 + RX_32Byte[0]);
}

void BQ769x2_Configure() {
	// Configures all parameters in device RAM

	// Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
	BQ769x2_CommandSubcommand(SET_CFGUPDATE);

	delayUS(2000);

	// After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
	// programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
	// An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
	// a full description of the register and the bits will pop up on the screen.

	// 'Power Config' - 0x9234 = 0x2D80
	// Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
	// Set wake speed bits to 00 for best performance

	//BQ769x2_ReadBatteryStatus();

	//BQ769x2_SetRegister(PowerConfig, 0x2D80, 2);
	/* Power Config
	 *
	 * Bit 13 - 1 for Over-temp in deepsleep
	 * Bit 12 - 0 for standard SHUTDOWN behavior
	 * BIT 11 - 1 for Enable wake from DEEPSLEEP with charger attached
	 * BIT 10 - 1 for enable LDOs in DEEPSLEEP - important for low-power STM32 to wake with button
	 * BIT 9 - 0 to disable low frequency oscillator in DEEPSLEEP
	 * BIT 8 - 0 to disable sleep mode by default (not needed for this application)
	 * BIT 7 - 1 to enable SHUTDOWN for severe chip overtemp
	 * BIT 6 - 0 for default ADC speed
	 * BIT 5/4 - 10 for quarter speed measurements during balancing, increases balancing power
	 * BIT 3-2 - 00 for standard loop ADC loop speed
	 * BIT 1-0 - 10 for default coulomb counter conversion speed
	 */
	BQ769x2_SetRegister(PowerConfig, 0b0010110010100010, 2);
	BQ769x2_SetRegister(FETOptions, 0x0D, 1); //device may not turn FETs on autonomously unless allowed to do so. Important because the STM32 is the only thing that can turn the pack off.
	BQ769x2_SetRegister(REG0Config, 0x01, 1); // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
	BQ769x2_SetRegister(REG12Config, 0x0D, 1); // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V and disable REG2 as not used

	/* MFG status - keep chip in FULLACCESS, but set FETs to autonomous mode and enable permanent fail*/
	BQ769x2_SetRegister(MfgStatusInit, 0b0000000001010000, 2); //autonomous mode and enable permanent fail

	/* Pin Function Configs */
	BQ769x2_SetRegister(DFETOFFPinConfig, 0b10000010, 1); // Set DFETOFF pin to control BOTH DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
	BQ769x2_SetRegister(CFETOFFPinConfig, 0b10000010, 1); // Set CFETOFF pin to control BOTH CHG FET - 0x92FA = 0x42 (set to 0x00 to disable). Configures as ALT function, active-low, individual control

	// Set up ALERT Pin - 0x92FC = 0x2A
	// This configures the ALERT pin to drive high (REG1 voltage) when enabled.
	// The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available

	//OPT5: 0 for active high
	//OPT4: not used
	//OPT3:1 to use REG1 to drive rising edge
	//OPT2:0 for no pull-up to reg1
	//OPT1:0
	//OPT0:1 weak pull-down enabled
	//FXN1/FXN0: 10 for alert
	BQ769x2_SetRegister(ALERTPinConfig, 0b00100110, 1);
	BQ769x2_SetRegister(TS1Config, 0x07, 1); // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
	BQ769x2_SetRegister(TS3Config, 0x07, 1); // Set TS3 to measure Cell Temperature - 0x74
	BQ769x2_SetRegister(HDQPinConfig, 0x07, 1); // Set HDQ pin to measure Cell temperature

	/* Alarm Configuration - what can pull the ALERT pin high */
	// 'Default Alarm Mask' - only use the FULLSCAN bit - this will be set to trigger the ALERT pin to wake from DEEPSLEEP
	BQ769x2_SetRegister(DefaultAlarmMask, 0x0080, 2);

	/* Current measurement */
	BQ769x2_SetRegister(CCGain, FloatToUInt((float) 0.75684), 4); // 7.5684/R_sense(mOhm) = 7.5684/10 = 7.76
	BQ769x2_SetRegister(CapacityGain, FloatToUInt((float) 225736.32), 4); // CC Gain * 298261.6178 = = 7.5684/10 * 298261.6178 = 225736.32

	/* Thermistor Calibration from TI BQ tool
	 *
	 bestA [A1 A2 A3 A4 A5] =  [-22175  31696 -16652  31696 4029]
	 bestB [B1 B2 B3 B4] =  [-23835  20738 -8470  4596]
	 Adc0 = 11703
	 */
	BQ769x2_SetRegister(T18kCoeffa1, (int16_t) -22175, 2);
	BQ769x2_SetRegister(T18kCoeffa2, (int16_t) 31696, 2);
	BQ769x2_SetRegister(T18kCoeffa3, (int16_t) -16652, 2);
	BQ769x2_SetRegister(T18kCoeffa4, (int16_t) 31696, 2);
	BQ769x2_SetRegister(T18kCoeffa5, (int16_t) 4029, 2);
	BQ769x2_SetRegister(T18kCoeffb1, (int16_t) -23835, 2);
	BQ769x2_SetRegister(T18kCoeffb2, (int16_t) 20738, 2);
	BQ769x2_SetRegister(T18kCoeffb3, (int16_t) -8470, 2);
	BQ769x2_SetRegister(T18kCoeffb4, (int16_t) 4596, 2);
	BQ769x2_SetRegister(T18kAdc0, (int16_t) 11703, 2);

	/* Protections Config */
	BQ769x2_SetRegister(VCellMode, ACTIVE_CELLS, 2); //Faraday BMS = 0xAAFF for 12 cells. See schematic

	// Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
	// Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
	// COV (over-voltage), CUV (under-voltage)
	//BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1);
	BQ769x2_SetRegister(EnabledProtectionsA, 0b10111100, 1);

	// Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
	// Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
	// OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
	BQ769x2_SetRegister(EnabledProtectionsB, 0b01110111, 1);

	BQ769x2_SetRegister(UTCThreshold, (signed char) 5, 1); // Set undertemperature in charge threshold. Assume +/- 5C because of poor thermistor coupling
	BQ769x2_SetRegister(UTDThreshold, (signed char) -10, 1); // Set undertemperature in discharge threshold. Assume +/- 5C because of poor thermistor coupling
	BQ769x2_SetRegister(OTCThreshold, (signed char) 40, 1); // Set overtemperature in charge threshold. Assume +/- 5C because of poor thermistor coupling
	BQ769x2_SetRegister(OTDThreshold, (signed char) 55, 1); // Set overtemperature in discharge threshold. Assume +/- 5C because of poor thermistor coupling

	/*Set Permanent Fail for SUV and SOV to just disable FETs to prevent charging of severely undervolted cells. STM32 goes to sleep due to inactivity afterwards*/
	BQ769x2_SetRegister(SUVThreshold,1900,2); //0x92CB - set safety undervoltage threshold to 1.9V
	BQ769x2_SetRegister(SOVThreshold,4500,2); //0x92CE - set safety overvoltage threshold to 4.5V
	BQ769x2_SetRegister(EnabledPFA,0b00000011,1); //Enable Permanent Fail for Safety Undervoltage and Safety Overvoltage Only
	BQ769x2_SetRegister(EnabledPFD,0b00000001,1); //Enable Permanent Fail for Safety Undervoltage and Safety Overvoltage Only
	BQ769x2_SetRegister(TOSSThreshold,240,2); //enable permanent fail if top of stack voltage deviates from cell voltages added up by 240*12 = 2.8V
	BQ769x2_SetRegister(ProtectionConfiguration,0x02,2); //Set Permanent Fail to turn FETs off only. Assume that idle will take care of DEEPSLEEP

	/* Balancing Configuration - leaving defaults for deltas (>40mV to start, <20mV to stop*/
	BQ769x2_SetRegister(BalancingConfiguration, 0b00000011, 1); //Set balancing to autonomously operate while in RELAX and CHARGE configurations. Sleep is disabled.
	BQ769x2_SetRegister(CellBalanceMaxCells, 3, 1); //0x933A  - set maximum number of cells that may balance at once. Contributes to thermal limit of BQ chip
	BQ769x2_SetRegister(CellBalanceMinDeltaCharge, 25, 1); //0x933D - set minimum cell balance delta at which balancing starts in CHARGE to 15mV
	BQ769x2_SetRegister(CellBalanceMinDeltaRelax, 25, 1); //0x933D - set minimum cell balance delta at which balancing starts in RELAX to 15mV
	BQ769x2_SetRegister(CellBalanceStopDeltaCharge, 15, 1); //0x933D - set minimum cell balance delta at which balancing stops in CHARGE to 15mV
	BQ769x2_SetRegister(CellBalanceStopDeltaRelax, 15, 1); //0x933D - set minimum cell balance delta at which balancing stops in RELAX to 15mV
	BQ769x2_SetRegister(CellBalanceMinCellVCharge, (int16_t) 0x0E74, 2); //0x933B -Minimum voltage at which cells start balancing. Set to 3700mV for now
	BQ769x2_SetRegister(CellBalanceMinCellVRelax, (int16_t) 0x0E74, 2); //0x933F -Minimum voltage at which cells start balancing. Set to 3700mV for now

	/*Over and Under Voltage configuration*/
	BQ769x2_SetRegister(CUVThreshold, 0x34, 1); //CUV (under-voltage) Threshold - 0x9275 = 0x34 (2631 mV) this value multiplied by 50.6mV = 2631mV
	BQ769x2_SetRegister(COVThreshold, 0x53, 1); //COV (over-voltage) Threshold - 0x9278 = 0x53 (4199 mV) this value multiplied by 50.6mV = 4199mV
	BQ769x2_SetRegister(ShutdownCellVoltage, 0x0960, 2); // Shutdown 0x923F - enter SHUTDOWN when below this cell voltage to minimize power draw . Set to 2400mV
	BQ769x2_SetRegister(ShutdownStackVoltage, 0x0AC8, 2); //ShutdownStackVoltage 0x9241 - enter SHUTDOWn when the stack is below this voltage - set to 2300mV/cell -> 2760*10mV
	BQ769x2_SetRegister(CUVRecoveryHysteresis, 0x04, 1); //CUVRecoveryHystersis 0x927B - hysteresis value after COV - set to 200mV -> 4* 50.6mV = 202.4mV

	/*Definitions of charge and discharge*/
	BQ769x2_SetRegister(DsgCurrentThreshold, 0x64, 2); //0x9310   Set definition of discharge in mA. 100mA. Balancing happens when current is above the negative of this current.
	BQ769x2_SetRegister(ChgCurrentThreshold, 0x32, 2); //0x9312  Set definition of charge in mA. 50mA Balancing happens in charge when above this current

	/*Charge current limit*/
	BQ769x2_SetRegister(OCCThreshold, 0x0F, 1); //OCC (over-current in charge) Threshold - 0x9280 = 0x05 (30mV = 3A across 10mOhm sense resistor) Units in 2mV
	BQ769x2_SetRegister(OCCDelay, 0x1E, 1); //OCC Delay (over current in charge delay) - 0x9281 = 0x0D (around 100ms)

	/*Overcurrent in Discharge Config*/
	BQ769x2_SetRegister(OCD1Threshold, 0x7D, 1); //OCD1 "fast"Threshold - 0x9282 = 0x62 (250 mV = -25A across 10mOhm sense resistor) units of 2mV
	BQ769x2_SetRegister(OCD1Delay, 0x04, 1); //OCD1 Delay- 0x9283 = 10 ms to 426 ms in units of 3.3 ms, with the actual delay being 3.3 ms × (2 + setting) = 20ms.

	BQ769x2_SetRegister(OCD2Threshold, 0x64, 1); //OCD1 Threshold - 0x9284 = (0x50 * 20 mV = 20A across 10mOhm sense resistor) units of 2mV
	BQ769x2_SetRegister(OCD2Delay, 0x5A, 1); //OCD2 Threshold - 0x9285 = 10 ms to 426 ms in units of 3.3 ms, with the actual delay being 3.3 ms × (2 + 120) = 90ms.

	BQ769x2_SetRegister(OCD3Threshold, (int16_t) -16000, 2); //OCD3 Threshold - 0x928A (-16A in units of user Amps (mA))
	BQ769x2_SetRegister(OCD3Delay, 15, 2); //OCD3 Threshold - 0x928C (15 second delay)

	/*Fast short circuit detection config */
	BQ769x2_SetRegister(SCDThreshold, 0x0D, 1); //Short circuit discharge Threshold - 0x9286 = 0x0B (400 mV = 40A across 10mOhm sense resistor)
	BQ769x2_SetRegister(SCDDelay, 0x11, 1); //SCD Delay - 0x9287 = 0x11 (240us = (17-1)*15us = 240us
	BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1); //Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01

	// Exit CONFIGUPDATE mode  - Subcommand 0x0092
	BQ769x2_CommandSubcommand(EXIT_CFGUPDATE);
	delayUS(60000); //wait for chip to be ready
}

/*Initialize BQ chip by configuring registers and then checking that a critical register was written. Return 1 if successfully configured, 0 if failed*/
uint8_t BQ769x2_Initialize() {

	BQ769x2_Configure();

	//Fail if device is still in config update mode.
	BQ769x2_ReadBatteryStatus();
	if (value_BatteryStatus & 1) {
		return 0;
	}
	//Fail if the active cells register is not 0xAAFF
	if (BQ769x2_ReadUnsignedRegister(VCellMode, 2) != ACTIVE_CELLS) {
		return 0;
	}

	//config successful and register spot check complete
	return 1;

}

//  ********************************* FET Control Commands  ***************************************

void BQ769x2_ForceDisableFETs() {
	// Disables all FETs using the DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(CFETOFF_PORT, CFETOFF_PIN, GPIO_PIN_RESET); // CFETOFF pin (BOTHOFF) set low
	HAL_GPIO_WritePin(DFETOFF_PORT, DFETOFF_PIN, GPIO_PIN_RESET); // DFETOFF pin (BOTHOFF) set low
}

void BQ769x2_AllowFETs() {
	// Resets DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(CFETOFF_PORT, CFETOFF_PIN, GPIO_PIN_SET); // CFETOFF pin set high
	HAL_GPIO_WritePin(DFETOFF_PORT, DFETOFF_PIN, GPIO_PIN_SET); // DFETOFF pin set high
}

void BQ769x2_ReadFETStatus() {
	// Read FET Status to see which FETs are enabled
	BQ769x2_DirectCommand(FETStatus, 0x00, R);
	FET_Status = (RX_data[1] * 256 + RX_data[0]);
	Dsg = ((0x4 & RX_data[0]) >> 2); // discharge FET state
	Chg = (0x1 & RX_data[0]); // charge FET state
	PChg = ((0x2 & RX_data[0]) >> 1); // pre-charge FET state
	PDsg = ((0x8 & RX_data[0]) >> 3); // pre-discharge FET state
}

// ********************************* End of FET Control Commands *********************************

// ********************************* BQ769x2 Power Commands   *****************************************

void BQ769x2_SetShutdownPin() {
	// Puts the device into SHUTDOWN mode using the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(RST_SHUT_CTRL_PORT, RST_SHUT_CTRL_PIN, GPIO_PIN_SET); // Sets RST_SHUT pin
}

void BQ769x2_ResetShutdownPin() {
	// Releases the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(RST_SHUT_CTRL_PORT, RST_SHUT_CTRL_PIN, GPIO_PIN_RESET); // Resets RST_SHUT pin
}

/*Put the BQ into DEEPSLEEP by writing the DEEPSLEEP command twice. Returns 1 if successful, otherwise 0*/
uint8_t BQ769x2_EnterDeepSleep() {
	BQ769x2_CommandSubcommand(DEEPSLEEP);
	delayUS(2000);
	BQ769x2_CommandSubcommand(DEEPSLEEP);
	delayUS(2000);
	if (BQ769x2_ReadControlStatus() & 0x04) //if bit 2 is high, then device is in DEEPSLEEP. Return 1 for success
			{
		return 1;
	}
	return 0;
}

/*Wake up the BQ from DEEPSLEEP by toggling RST_SHUT once. Return 1 if successfully woken up, otherwise 0*/
uint8_t BQ769x2_Wake() {
	HAL_GPIO_WritePin(RST_SHUT_CTRL_PORT, RST_SHUT_CTRL_PIN, GPIO_PIN_SET);
	delayUS(2000);
	HAL_GPIO_WritePin(RST_SHUT_CTRL_PORT, RST_SHUT_CTRL_PIN, GPIO_PIN_RESET);
	delayUS(2000);

	if (BQ769x2_ReadControlStatus() & 0x04) { //if bit 2 is high, then device is in DEEPSLEEP
		return 0;
	}

	return 1;

}

/*Put the BQ chip into SHUTDOWN by holding RST_SHUT high for more than 1 second.
 * Typically only used for debugging, since there is no software way to get out
 * of shutdown without connecting a charger*/
void BQ769x2_EnterShutDown() {
	HAL_GPIO_WritePin(RST_SHUT_CTRL_PORT, RST_SHUT_CTRL_PIN, GPIO_PIN_SET);
	delayMS(1500);
	HAL_GPIO_WritePin(RST_SHUT_CTRL_PORT, RST_SHUT_CTRL_PIN, GPIO_PIN_RESET);
}

/* BQ769x2_Reset - hard reset of the BQ chip, which takes ~250ms. The 3V3 rail comes up 20ms after the rest begins
 * Should only be called if the 3v3 rail needs to be reset, which can fix some issues related to sleep with the STM32 after using the debugger */
void BQ769x2_Reset() {
	uint8_t RetryCounter = 0;
	while (RetryCounter < 20) {
		HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
		BQ769x2_CommandSubcommand(BQ769x2_RESET); // Resets the BQ769x2 registers and kills the 3v3 Rail
		RetryCounter++;
	}

	Error_Handler(); //If we end up here, something is truly messed up
}
/* BQ769x2_Ready - return 1 if BQ is initialized, 0 otherwise */
uint8_t BQ769x2_Ready() {
	BQ769x2_ReadBatteryStatus();
	if (value_BatteryStatus & 0x300) { //if bits 8 and 9 of battery status are set, device has finished booting
		delayMS(60);
		return 1;
	};
	return 0;
}

/* Quickly toggle RST_SHUT to wake the BQ chip. Simple function that doesn't block */
void BQ769x2_SoftWake() {
	HAL_GPIO_WritePin(RST_SHUT_CTRL_PORT, RST_SHUT_CTRL_PIN, GPIO_PIN_SET);
	delayUS(2000);
	HAL_GPIO_WritePin(RST_SHUT_CTRL_PORT, RST_SHUT_CTRL_PIN, GPIO_PIN_RESET);
	delayUS(2000);
}

// ********************************* End of BQ769x2 Power Commands   *****************************************

// ********************************* BQ769x2 Status and Fault Commands   *****************************************

uint16_t BQ769x2_ReadAlarmStatus() {
	// Read this register to find out why the ALERT pin was asserted
	BQ769x2_DirectCommand(AlarmStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

uint16_t BQ769x2_ReadControlStatus() {
	// Read this register to get the Control Status Pins
	BQ769x2_DirectCommand(ControlStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

uint16_t BQ769x2_ReadRawAlarmStatus() {
	// Read this register to find out why the ALERT raw pin was asserted. Distinct from AlarmStatus in that these do not latch
	BQ769x2_DirectCommand(AlarmRawStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

uint8_t BQ769x2_ReadSafetyStatus() {
	// Read Safety Status A/B/C and find which bits are set
	// This shows which primary protections have been triggered
	// Return 1 at end to match structure of other functions
	BQ769x2_DirectCommand(SafetyStatusA, 0x00, R);
	value_SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);
	//Example Fault Flags
	UV_Fault = ((0x4 & RX_data[0]) >> 2);
	OV_Fault = ((0x8 & RX_data[0]) >> 3);
	SCD_Fault = ((0x8 & RX_data[1]) >> 3);
	OCD_Fault = ((0x2 & RX_data[1]) >> 1);
	BQ769x2_DirectCommand(SafetyStatusB, 0x00, R);
	value_SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);

	BQ769x2_DirectCommand(SafetyStatusC, 0x00, R);
	value_SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);

	if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 1) {
		ProtectionsTriggered = 1;
	} else {
		ProtectionsTriggered = 0;
	}

	return 1;
}

void BQ769x2_ReadPFStatus() {
	// Read Permanent Fail Status A/B/C and find which bits are set
	// This shows which permanent failures have been triggered
	BQ769x2_DirectCommand(PFStatusA, 0x00, R);
	value_PFStatusA = (RX_data[1] * 256 + RX_data[0]);
	BQ769x2_DirectCommand(PFStatusB, 0x00, R);
	value_PFStatusB = (RX_data[1] * 256 + RX_data[0]);
	BQ769x2_DirectCommand(PFStatusC, 0x00, R);
	value_PFStatusC = (RX_data[1] * 256 + RX_data[0]);
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************

// ********************************* BQ769x2 Measurement Commands   *****************************************

uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
	//RX_data is global var
	BQ769x2_DirectCommand(command, 0x00, R);
	delayUS(2000);
	if (command >= Cell1Voltage && command <= Cell16Voltage) {//Cells 1 through 16 (0x14 to 0x32)
		return (RX_data[1] * 256 + RX_data[0]); //voltage is reported in mV
	} else { //stack, Pack, LD
		return 10 * (RX_data[1] * 256 + RX_data[0]); //voltage is reported in 0.01V units
	}

}
void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
	//int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
	//for (int x = 0; x < 16; x++) { //Reads all cell voltages
	//	CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
	//	cellvoltageholder = cellvoltageholder + 2;
	//}
	for (int x = 0; x < 16; x++) { //Reads all cell voltages
			CellVoltage[x] = BQ769x2_ReadVoltage(Cell1Voltage + 2 * x);
	}

	Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
	Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
	LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}

int16_t BQ769x2_ReadCurrent()
// Reads PACK current
{
	BQ769x2_DirectCommand(CC2Current, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]); // cell current is reported as an int16 in UserAmps
}

float BQ769x2_ReadTemperature(uint8_t command) {
	BQ769x2_DirectCommand(command, 0x00, R);
	//RX_data is a global var
	return (0.1 * (float) (RX_data[1] * 256 + RX_data[0])) - 273.15; // converts from 0.1K to Celcius
}

void BQ769x2_ReadBalancingStatus() {
	BQ769x2_Subcommand(CB_ACTIVE_CELLS, 0x00, R);
	CB_ActiveCells = (RX_32Byte[1] * 256 + RX_32Byte[0]); //CB_ACTIVE_CELLS returns a 2 byte bitfield of which cells are balancing
}

void BQ769x2_ReadPassQ() { // Read Accumulated Charge and Time from DASTATUS6
	BQ769x2_Subcommand(DASTATUS6, 0x00, R);
	AccumulatedCharge_Int = ((RX_32Byte[3] << 24) + (RX_32Byte[2] << 16)
			+ (RX_32Byte[1] << 8) + RX_32Byte[0]); //Bytes 0-3
	AccumulatedCharge_Frac = ((RX_32Byte[7] << 24) + (RX_32Byte[6] << 16)
			+ (RX_32Byte[5] << 8) + RX_32Byte[4]); //Bytes 4-7
	AccumulatedCharge_Time = ((RX_32Byte[11] << 24) + (RX_32Byte[10] << 16)
			+ (RX_32Byte[9] << 8) + RX_32Byte[8]); //Bytes 8-11
}

/* update variables in STM32 with values from the BQ chip. Returns 1 if successful, 0 if failed or data not yet ready */
uint8_t BQ769x2_ReadBatteryData() {
	AlarmBits = BQ769x2_ReadAlarmStatus();
	if (AlarmBits & 0x80) { // Check if FULLSCAN is complete. If set, new measurements are available
		Pack_Current = BQ769x2_ReadCurrent(); //needed for STM32_HandleInactivity, do not remove
		BQ769x2_ReadAllVoltages(); //get most recent voltages
		BQ769x2_ReadBalancingStatus(); //needed for balancing status message
		Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
		Temperature[1] = BQ769x2_ReadTemperature(TS3Temperature);
		Temperature[2] = BQ769x2_ReadTemperature(HDQTemperature);
		Temperature[3] = BQ769x2_ReadTemperature(IntTemperature);

		BQ769x2_DirectCommand(AlarmStatus, 0x0080, W); // Clear the FULLSCAN bit
		return 1;
	} else {
		return 0;
	}
}

/* calculate min and max voltage, update globals. */
void BQ769x2_CalcMinMaxCellV() {
	// Assume the first element is the minimum
	int maxV = 50;
	int minV = 6000;

	// Loop through the array to find the minimum
	for (int i = 0; i < 16; i++) {
		if (ACTIVE_CELLS & (1 << i)) {
			if (CellVoltage[i] < minV) {
				minV = CellVoltage[i];
			}
			if (CellVoltage[i] > maxV) {
				maxV = CellVoltage[i];
			}
		}
	}
	CellMinV = minV;
	CellMaxV = maxV;
}
;

/* debug function to print lots of status bits */
void BQ769x2_PrintStatus() {
	AlarmBits = BQ769x2_ReadRawAlarmStatus();
	BQ769x2_ReadPFStatus(); //TODO remove later
	printf(
			"CRC: %d Bal: %d Dsg: %d Chg: %d ScanComplete: %d Prots: %d SSA: %d SSBC: %d PFA: %d PFB: %d PFC: %d UV: %d OV: %d SCD: %d OCD: %d SSA: %#x, SSB: %#x, SSC: %#x",
			CRC_Fail,
			//AlarmBits & 0x00 ? 1 : 0, //WAKE
			//AlarmBits & 0x02 ? 1 : 0, //ADSCAN
			AlarmBits & 0x04 ? 1 : 0, //CB
			//AlarmBits & 0x10 ? 1 : 0, //SHUTV
			AlarmBits & 0x20 ? 0 : 1, //XDSG
			AlarmBits & 0x40 ? 0 : 1, //XCHG
			AlarmBits & 0x80 ? 1 : 0, //FULLSCAN
			//AlarmBits & 0x200 ? 1 : 0, //INITCOMP
			//AlarmBits & 0x400 ? 1 : 0, //INITSTART
			ProtectionsTriggered, AlarmBits & 0x4000 ? 1 : 0, //Safety Status A
			AlarmBits & 0x8000 ? 1 : 0, //Safety Status B or C
			value_PFStatusA, value_PFStatusB, value_PFStatusC,
			UV_Fault, OV_Fault, SCD_Fault, OCD_Fault, value_SafetyStatusA,
			value_SafetyStatusB, value_SafetyStatusC);

	BQ769x2_CalcMinMaxCellV();
	printf(
			"PackV: %d StackV:%d LdV:%d I: %d MIN_V: %d MAX_V: %d InactivityCount: %d TS1: %.3f TS3: %.3f T_HDQ: %.3f T_INT: %.3f \r\n",
			Pack_Voltage,Stack_Voltage,LD_Voltage,Pack_Current, CellMinV, CellMaxV, InactivityCount, Temperature[0],
			Temperature[1], Temperature[2], Temperature[3]);

}

// ********************************* End of BQ769x2 Measurement Commands   *****************************************

/* UART Functions
 * ===================== */

//calculate faraday modbus CRC
uint16_t UART_CRC(uint8_t *buf, uint16_t size) {
	uint16_t crc = 0xFFFF;

	uint8_t n;
	uint8_t i;
	for (n = 0; n < size; n++) {
		crc = crc ^ buf[n];
		for (i = 0; i < 8; i++) {
			if (crc & 1) {
				crc = crc >> 1;
				crc = crc ^ 0xA001;
			} else {
				crc = crc >> 1;
			}
		}
	}
	return crc;
}

void THVD2410_Sleep() {
	HAL_GPIO_WritePin(UART_TX_EN_PORT, UART_TX_EN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_RX_EN_PORT, UART_RX_EN_PIN, GPIO_PIN_SET);
}
;
void THVD2410_Transmit() {
	HAL_GPIO_WritePin(UART_TX_EN_PORT, UART_TX_EN_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UART_RX_EN_PORT, UART_RX_EN_PIN, GPIO_PIN_SET);
}
;
void THVD2410_Receive() {
	HAL_GPIO_WritePin(UART_TX_EN_PORT, UART_TX_EN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_RX_EN_PORT, UART_RX_EN_PIN, GPIO_PIN_RESET);
}
;

void UART_WaitForCommand() {

	if (!UartBusy) {
		//set RX buffer to all zeros
		for (int i = 0; i < 8; i++) {
			UART_RxData[i] = 0;
		}

		//start recieve-to-idle interrupt
		THVD2410_Receive();
		huart2.RxState = HAL_UART_STATE_READY; //TODO EVALUATE THIS WITH SASHA

		if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, UART_RxData, 8) != HAL_OK) {
			Error_Handler();
		}
	}

}

//prep the cell voltage message (30 bytes). Refer to documentation for packet format.
uint8_t UART_PrepCellVoltageMessage() {

	for (int i = 0; i < 32; i++) {
		UART_TxData[i] = 0;
	}

	//device, fxn, length
	UART_TxData[0] = 0x02;
	UART_TxData[1] = 0x03;
	UART_TxData[2] = 0x18;

	// loop through voltage field and construct uints
	int k = 0;
	for (int i = 0; i < 16; i++) {
		if (ACTIVE_CELLS & (1 << i)) {
			UART_TxData[2 * k + 3] = (CellVoltage[i] * 2 / 3) >> 8;
			UART_TxData[2 * k + 4] = (CellVoltage[i] * 2 / 3) & 0xFF;

			//UART_TxData[2 * k + 3] = (3100 * 2 / 3) >> 8;
			//UART_TxData[2 * k + 4] = (3100 * 2 / 3) & 0xFF;
			k++;
		}
	}

	//add crc in reverse byte order
	uint16_t crc = UART_CRC(UART_TxData, 27);
	UART_TxData[27] = crc & 0xFF;
	UART_TxData[28] = crc >> 8;
	UART_TxData[29] = 0xFF; //add extra byte of zeros to match faraday protocol

	return 30;
}

//prep the cell balancing message (8 bytes). Refer to documentation for packet format
uint8_t UART_PrepCellBalancingMessage() {

	for (int i = 0; i < 32; i++) {
		UART_TxData[i] = 0;
	}

	UART_TxData[0] = 0x02;
	UART_TxData[1] = 0x03;
	UART_TxData[2] = 0x02;

	// CB_ActiveCells is a bitfield of which of 16 channels are active. Bike expects a bitfield with the first byte blank and the remainder a bitfield
	uint16_t CB = 0x00;
	int k = 0;
	for (int i = 0; i < 16; i++) {
		if (ACTIVE_CELLS & (1 << i)) {
			//i is the cell number active cell bitfield (0xAAFF)
			//k is the cell number in faraday index
			if (CB_ActiveCells & (1 << i)) {
				CB = CB | (1 << k); //TODO determine whether the cell number is mapped correctly
			};
			k++;
		}
	}

	UART_TxData[3] = CB >> 8;
	UART_TxData[4] = CB & 0xFF;

	//add crc in reverse byte order
	uint16_t crc = UART_CRC(UART_TxData, 5);
	UART_TxData[5] = crc & 0xFF;
	UART_TxData[6] = crc >> 8;
	UART_TxData[7] = 0xFF; //add extra byte of zeros to match faraday protocol
	return 8;
}

/*
 * Prep battery status 1 message (8 bytes). Refer to documentation for packet format
 * Respond to battery status message 1 - battery OK.
 *
 * Hard coded for now
 * Example request 0x2 0x3 0x0 0x0 0x0 0x1 0x84 0x39
 * Example response 0x2 0x3 0x2 0x0 0x0 0xfc 0x44
 */
//
uint8_t UART_PrepBatteryStatusMessage1() {

	for (int i = 0; i < 32; i++) {
		UART_TxData[i] = 0;
	}

	UART_TxData[0] = 0x02;
	UART_TxData[1] = 0x03;
	UART_TxData[2] = 0x02;
	UART_TxData[3] = 0x00;
	UART_TxData[4] = 0x00;

	//add crc in reverse byte order
	uint16_t crc = UART_CRC(UART_TxData, 5);
	UART_TxData[5] = crc & 0xFF;
	UART_TxData[6] = crc >> 8;
	UART_TxData[7] = 0xFF; //add extra byte of zeros to match faraday protocol
	return 8;
}

/* Prep battery status 2 message (7 bytes). Refer to documentation for packet format
 * Fill UART_TxData to battery status message 2 - purpose unknown. Hard coded for now
 *
 * Example request 0x2 0x3 0x0 0x1 0x0 0x1 0xd5 0xf9
 * Example response 0x2 0x3 0x2 0x0 0x19 0x3d 0x8e
 */
uint8_t UART_PrepBatteryStatusMessage2() {

	for (int i = 0; i < 32; i++) {
		UART_TxData[i] = 0;
	}

	UART_TxData[0] = 0x02;
	UART_TxData[1] = 0x03;
	UART_TxData[2] = 0x02;
	UART_TxData[3] = 0x00;
	UART_TxData[4] = 0x19;

	//add crc in reverse byte order
	uint16_t crc = UART_CRC(UART_TxData, 5);
	UART_TxData[5] = crc & 0xFF;
	UART_TxData[6] = crc >> 8;

	UART_TxData[7] = 0xFF; //add extra byte of zeros to match faraday protocol
	return 8;
}

/*Takes in an 8 byte message, parses, checks CRC, and then responds with the appropriate message */
void UART_Respond(uint8_t *buf, uint16_t size) {

	//Code currently only supports 8 byte read messages, which is all that's necessary to get the bike moving.
	if (size == 8) {

		//unpack message
		uint8_t SlaveID = buf[0];
		uint8_t FunctionCode = buf[1];
		uint16_t Address = buf[2] << 8 | buf[3];
		uint16_t NumRegs = buf[4] << 8 | buf[5];
		uint16_t CRCRecv = buf[7] << 8 | buf[6];

		//Check CRC and that we're the target audience (BMS is 0x02)
		if (CRCRecv == UART_CRC(buf, 6) && SlaveID == 0x02) {
			delayUS(450); //insert 1ms delay to match timing of original battery
			if (FunctionCode == 0x03) { //it's a read
				if (LEDS) {
					HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
					delayUS(50);
					HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_RESET);
				}

				uint8_t len;

				if (Address == 0x02) { //Battery Voltage Message
					len = UART_PrepCellVoltageMessage();
					THVD2410_Transmit();
					HAL_UART_Transmit(&huart2, UART_TxData, len,
							UART_TIMEOUT_S);

				} else if (Address == 0x17) { //Battery Balancing Message
					len = UART_PrepCellBalancingMessage();
					THVD2410_Transmit();
					HAL_UART_Transmit(&huart2, UART_TxData, len,
							UART_TIMEOUT_S);

				} else if (Address == 0x00 && NumRegs == 0x01) { //Battery Status Message 1
					len = UART_PrepBatteryStatusMessage1();
					THVD2410_Transmit();
					HAL_UART_Transmit(&huart2, UART_TxData, len,
							UART_TIMEOUT_S);

				} else if (Address == 0x01 && NumRegs == 0x01) { //Battery Status Message 1
					len = UART_PrepBatteryStatusMessage2();
					THVD2410_Transmit();
					HAL_UART_Transmit(&huart2, UART_TxData, len,
							UART_TIMEOUT_S);
				}
			}
		}

	}
	THVD2410_Receive();
	HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_RESET);
}

/* State Machine Functions
 * ===================== */

/*Pet Independent Watchdog. Must be done once a second or sooner*/
void STM32_PetWatchdog() {

	/* Refresh IWDG: reload counter */
	if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
		/* Refresh Error */
		Error_Handler();
	}
}


/* Implement button press timing*/
void STM32_HandleButton() {
	if (STM32_Wake_Button_Pressed()) {
		ButtonCount++;

	} else if (ButtonCount) {
		ButtonCount -= 1;
	};

	if (ButtonCount > BUTTON_LONG_PRESS_LOOPS) {
		if (DEBUG) {
			printf("\r\nbutton long press...time to get ready for bed\r\n");
		}
		ButtonCount = 0;
		Sleep();
	};
}

uint8_t STM32_Wake_Button_Pressed() {
	return !(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN));
}

/* If current below 250mA for more than XXX loops, activate sleep. Must have a fresh value in Pack_Current */
void STM32_HandleInactivity() {

//Increment sleep timer if current is between +20mA and -250mA
	if (Pack_Current >= PACK_CURRENT_INACTIVITY_LOWER_LIMIT_MA
			&& Pack_Current <= PACK_CURRENT_INACTIVITY_UPPER_LIMIT_MA) {
		InactivityCount++;
	} else {
		InactivityCount = InactivityCount / 2; //exponential decay if current detected
	}

	if (!DEBUG) {
		if (InactivityCount > INACTIVITY_LOOPS_MAX) {
			Sleep();
		}

		//loop is much slower with prinfs enabled, so make this more reasonable
	} else {
		if (InactivityCount > 500) {
			printf("\r\ninactivity timeout...time to get ready for bed\r\n");
			Sleep();
		}

	}

}

/* STM32_Stop - puts the STM32 into STOP mode at minimum power consumption.
 * Leaves two ways to wake - EXTI0_1 (PA0 pulldown) and EXTI2_3 (PB5 pullup)
 *
 * Note that this function will fail if IWDG is running. This must be called before IWDG is started to be successful*/
void STM32_Stop() {
	//blink to show that we're entering sleep
	if (LEDS) {
		HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
		delayMS(100);
		HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_RESET);
	}

	//turn off LEDs
	HAL_GPIO_WritePin(LED_CHG_PORT, LED_CHG_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_DSG_PORT, LED_DSG_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BF_PORT, LED_BF_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_RESET);

	//Turn off FETs
	BQ769x2_ForceDisableFETs();

	//Turn off RS485 Chip
	THVD2410_Sleep();

	//Configure wake Interrupt on ALERT pin
	EXTI2_3_IRQHandler_Config();

	//Configure wake interrupt on BUTTON pin
	EXTI0_1_IRQHandler_Config();

	/* Enable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();


	//Configure all GPIO port pins in Analog Input mode (floating input trigger OFF)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	//special config for port A since we use it for wake
	GPIO_InitStructure.Pin = GPIO_PIN_All & ~GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	//special config for port B since we use it for wake
	GPIO_InitStructure.Pin = GPIO_PIN_All & ~GPIO_PIN_5;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

	//disable GPIO clocks to save power
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();

	//disable RTC to save power
	HAL_RTC_MspDeInit(&hrtc);

	//here we go!
	//HAL_SuspendTick(); TODO investigate whether this prevents wake-on-detect-load
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	//reset on wake
	NVIC_SystemReset(); //When woken up, just reset, it's simpler that way.

}

void STM32_CheckForWatchdogReset() {
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
		ResetByWatchdog = 1;
	}

	/* Clear reset flags in any cases */
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

/* Sleep() - start the process of putting the BMS in a low power mode
 * Puts the BQ chip to DEEPSLEEP, set a persistent flag that indicates we want to enter STOP,
 * reset the STM32. This clears the IWDG and the main loop reads the flag and puts the STM32 in STOP.
 * When the STM32 resets it will call STM32_Stop().
 * This is the only good way with an STM32L0 to disable IWDG and stay in STOP mode*/
void Sleep() {

	//pet the watchdog so this delay doesn't turn it off
	STM32_PetWatchdog();

	//disable the only interrupt that should be running (RS485 RX interrupt)
	CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);

	//turn off both fets
	BQ769x2_ForceDisableFETs();
	//BQ769x2_CommandSubcommand(ALL_FETS_OFF);

	//wait 200ms for the bus voltage to decay
	delayMS(200);

	//Put the BQ to sleep
	while (!BQ769x2_EnterDeepSleep()) {
		delayUS(5000);
	}

	BQ769x2_DirectCommand(AlarmStatus, 0x0080, W); // Clear the FULLSCAN bit, otherwise STM32 will wake up immediately

	if (DEBUG) {
		printf("\r\nbq put into DEEP SLEEP, STM about to reset to disable watchdog...\r\n");
	}

	//set persistent flag that we will want to STOP immediately upon reset
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xDEAD); // Writes a data in a RTC Backup data Register 1
	HAL_PWR_DisableBkUpAccess();

	//restart into STOP mode
	NVIC_SystemReset();
}


/* LED Helper Functions
 * ===================== */

/* Update FET LEDs with most recent status */
void STM32_UpdateFETLEDs() {
	BQ769x2_ReadFETStatus();
	HAL_GPIO_WritePin(LED_CHG_PORT, LED_CHG_PIN, Chg);
	HAL_GPIO_WritePin(LED_DSG_PORT, LED_DSG_PIN, Dsg);
}

/**
 * @brief  This blinks forever, mainly useful for debugging. Period in us
 * @retval None
 */
void STM32_BlinkForever(uint16_t period_ms) {
	HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
	delayMS(period_ms);
	HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_RESET);
	delayMS(period_ms);
}

/* Check to see if the magic value was written into the persistent RTC registers on the last reboot. Return 1 if true, 0 if not. */
uint8_t STM32_ShouldStop() {
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0xDEAD) {
		HAL_PWR_EnableBkUpAccess(); // Write Back Up Register 1 Data
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x0000); // Writes a data in a RTC Backup data Register 1
		HAL_PWR_DisableBkUpAccess(); //Disable access
		return 1;
	}

	return 0;
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

	/* USER CODE BEGIN Init */
	if (!DEBUG) {
		STM32_CheckForWatchdogReset();
	}
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all the things */
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	SystemPower_Config();
	MX_RTC_Init();
	HAL_TIM_Base_Start(&htim2);

	if (DEBUG) {
			printf("stm32 init complete\r\n");
	}

	/*Check to see if the button is currently pressed*/
	if (STM32_Wake_Button_Pressed()) {
		ButtonPressedDuringBoot= 1;
	}

	/* Check to see if the STOP flag is set from prior reset. If so, initiate STOP*/


	if (STM32_ShouldStop()) {
		if (DEBUG){
			printf("stm32 reset with intent to sleep, time to sleep...zzz\r\n");
		}
		STM32_Stop();
	}


	BQ769x2_ResetShutdownPin(); // RST_SHUT pin set low just in case

	delayMS(50); //Wait for everything to stabilize

	if (WATCHDOG) {
		MX_IWDG_Init();
	}

	/* Useful functions for debugging, especially if the STM32 gets into a weird state where it won't sleep */
	//BlinkForever(1); //uncomment to test IWDG
	//BQ769x2_Reset(); //Use this for several reasons - the main reason is to kill the 3.3V rail and power cycle the STM32, which may be necessary if the programmer puts it into a state where it won't sleep properly
	UART_WaitForCommand(); //Start UART Receiving

	while (1) {

		/* USER CODE BEGIN 3 */
		BQ769x2_ForceDisableFETs(); //disable FETs until we are getting communication from the BQ chip

		BQ769x2_SoftWake(); //wiggle RST_SHUT to do a partial reset of the BQ chip. Not sure if this is necessary but it doesn't seem to hurt.

		if (DEBUG) {
			if (BQ769x2_ReadControlStatus() & 0x04){
				printf("bq woke from DEEPSLEEP\r\n");
			}
		}

		/* old code
		// Check to see if BQ is in DEEPSLEEP or not
		if (BQ769x2_ReadControlStatus() & 0x04){//if bit 2 is high, then device is in DEEPSLEEP
			if (DEBUG) {
				printf("BQ was asleep\r\n");
			}

			delayUS(1000);
			if (RetryCount > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RetryCount++;
		}

		RetryCount = 0;

		*/

		//Check for BQ state FULLACCESS, SEALED, or UNSEALED. Device must be connected and ACKing to get past this point.
		// This may take quite a few reads for the chip to wake up if it's the first time it's booting (I've seen 15 reads!)
		while (!BQ769x2_Ready()) {
			if (DEBUG) {
				printf("bq not ready\r\n");
			}
			delayUS(1000);
			if (RetryCount > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RetryCount++;
		};

		RetryCount = 0;

		//Wake up the device if it isn't already awake
		while (!BQ769x2_Wake()) {
			if (DEBUG){
				printf("bq not awake\r\n");
			}
			delayUS(1000);
			if (RetryCount > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RetryCount++;
		}

		RetryCount = 0;

		//Initialize registers by calling BQ769x2_Init and then checking that the configuration was successful
		// BQ769x2 does a spot check of a register that should have been configured if BQ769x2_Init() was successful.
		while (!BQ769x2_Initialize()) {
			if (DEBUG) {
				printf("bq not configured\r\n");
			}
			if (RetryCount > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RetryCount++;
		}

		RetryCount = 0;

		// get first ADC reading. Normal for this to take a few retries
		while (!BQ769x2_ReadBatteryData()) {
			if (RetryCount > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RetryCount++;
			delayUS(5000); //wait a bit for the ADC to finish measuring
		}

		RetryCount = 0;

		// check whether a charger >15V is connected and the button wasn't pressed during boot. If not, go back to sleep.
		if (!ButtonPressedDuringBoot && Pack_Voltage < 15000) {
			if (DEBUG){
				printf("stm32 going back to sleep because user didn't press button and charger isn't connected...zzz\r\n");
			}
			Sleep();
		}

		//why are we starting? the only reasons are button press or charger detected
		if (DEBUG){
			if (ButtonPressedDuringBoot){
				printf("\r\nbutton pressed, time to attempt to allow FETs\r\n");
			}
			if (Pack_Voltage > 15000){
				printf("\r\ncharger connected, time to allow FETs\r\n");
			}
		}

		//enable FETs. It appears to be important to call FET_ENABLE twice
		BQ769x2_CommandSubcommand(FET_ENABLE); // Enable the CHG and DSG FETs
		BQ769x2_AllowFETs();
		BQ769x2_CommandSubcommand(FET_ENABLE); //for some reason need to do this twice...TODO investigate

		//Call outside loop
		UART_WaitForCommand();

		while (1) {

			//detect button press and take action if needed
			STM32_HandleButton();

			//Update FET registers and update LEDs
			if (LEDS) {
				STM32_UpdateFETLEDs();
			}

			//Handle sleep current and put battery to sleep if not much is going on
			STM32_HandleInactivity();

			//Useful for logging
			BQ769x2_CalcMinMaxCellV();

			//Print battery status over RS485 for debug
			BQ769x2_ReadBatteryStatus();
			if (DEBUG) {
				BQ769x2_PrintStatus();
			}

			//Get the latest data from the BQ chip
			if (BQ769x2_ReadBatteryData()) {
				RetryCount = 0;
			} else {
				RetryCount++;
			}

			//Check for faults and trigger the LED if so
			if (BQ769x2_ReadSafetyStatus()) {
				RetryCount = 0;
			} else {
				RetryCount++;
			};

			//Set Fault LED
			if (LEDS && (ProtectionsTriggered & 1)) {
				HAL_GPIO_WritePin(LED_BF_PORT, LED_BF_PIN, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(LED_BF_PORT, LED_BF_PIN, GPIO_PIN_RESET);
			}

			//If there are too many failures, reset the BQ chip
			if (RetryCount > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
				break;
			}

			STM32_PetWatchdog();
			delayMS(10);  // repeat loop every 20 ms

		}
	}
	/* USER CODE END 3 */
}

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_GPIO_WritePin(UART_RX_EN_PORT, UART_RX_EN_PIN, GPIO_PIN_SET); // Receive Off
	delayUS(20);
	HAL_GPIO_WritePin(UART_TX_EN_PORT, UART_TX_EN_PIN, GPIO_PIN_SET); // Transmit On
	delayUS(20);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	HAL_GPIO_WritePin(UART_TX_EN_PORT, UART_TX_EN_PIN, GPIO_PIN_RESET); //Transmit off
	delayUS(20);
	HAL_GPIO_WritePin(UART_RX_EN_PORT, UART_RX_EN_PIN, GPIO_PIN_SET); // Receive still off
	delayUS(20);

	return ch;
}

/**
 * @brief  Configures EXTI Port A PIN 0 as an interrupt
 * @param  None
 * @retval None
 */
static void EXTI0_1_IRQHandler_Config(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOC clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configure PC.13 pin as input floating */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable and set EXTI lines 4 to 15 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/**
 * @brief  Configures EXTI Port B Pin 5 as an interrupt
 * @param  None
 * @retval None
 */
static void EXTI2_3_IRQHandler_Config(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOC clock */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure PC.13 pin as input floating */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Enable and set EXTI lines 4 to 15 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerFrequencyMode = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00300617;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}
/**
 * @brief  System Power Configuration
 *         The system Power is configured as follow :
 *            + Regulator in LP mode
 *            + VREFINT OFF, with fast wakeup enabled
 *            + HSI as SysClk after Wake Up
 *            + No IWDG
 *            + Wakeup using EXTI Line (Key Button PC.13)
 * @param  None
 * @retval None
 */
static void SystemPower_Config(void) {
//GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable Ultra low power mode */
	HAL_PWREx_EnableUltraLowPower();

	/* Enable the fast wake up from Ultra low power mode */
	HAL_PWREx_EnableFastWakeUp();

	/* Select HSI as system clock source after Wake Up from Stop mode */
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI); //probably not necessary given reset upon resume from STOP
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
	hiwdg.Init.Window = 0xFFF;
	hiwdg.Init.Reload = 2300; //around 1 second with prescaler 16
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	//htim2.Init.Prescaler = 31; for 32Mhz
	htim2.Init.Prescaler = 14; //for 16MHz
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA1 PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB3 PB4 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//receive commands, all of which should be 8 bytes long
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (!UartBusy) {
		UartBusy = SET;
		CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE); //prevent interrupt from pre-empting responding and re-initialization
		if (Size == 8) {
			UART_Respond(UART_RxData, 8);
		}

		UartBusy = RESET;
		THVD2410_Receive();
		UART_WaitForCommand();
	}
}

/* Function handle UART errors. Tries to clear the error bit. If the error isn't one of these, then we reset the UART */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE); //disable interrupt

	if (huart->ErrorCode & HAL_UART_ERROR_FE) {
		// frame error
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_FEF);
	}
	if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
		// overrun error
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
	}
	if (huart->ErrorCode & HAL_UART_ERROR_NE) {
		// overrun error
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_NEF);
	}
	if (huart->ErrorCode & HAL_UART_ERROR_PE) {
		// overrun error
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF);
	}

	// if there are any remaining error codes, reset the UART peripheral
	if (!huart->ErrorCode) {
		//fully re-initialize uart. Slower, but we might need to recover from a new error
		if (HAL_UART_DeInit(&huart2) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_UART_Init(&huart2) != HAL_OK) {
			Error_Handler();

		}
	}

	UartBusy = RESET;
	UART_WaitForCommand();
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	if (LEDS) {
		HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
	}

	__disable_irq();

	if (DEBUG) {
		STM32_BlinkForever(2000); //watchdog should be disabled in DEBUG mode, so this is OK.
	}

	//reset and hope things go better the next time around
	delayMS(1000); //wait a second
	while (1)
		; //let the watchdog reset us

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
