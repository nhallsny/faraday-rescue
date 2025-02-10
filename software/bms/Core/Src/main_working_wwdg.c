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
#define DEV_ADDR  0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode 1  // 0 for disabled, 1 for enabled
//#define MAX_BUFFER_SIZE 10
#define MAX_BUFFER_SIZE 32
#define R 0 // Read; Used in DirectCommands and Subcommands functions
#define W 1 // Write; Used in DirectCommands and Subcommands functions
#define W2 2 // Write data with two bytes; Used in Subcommands function

//TBD More User
#define RETRY_LIMIT 100
#define UART_TIMEOUT_S 3

#define DEBUG 0
#define LEDS 0
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

/* UART handler declaration */
__IO ITStatus UartReady = RESET;

/*Uart flag*/
__IO ITStatus UartBusy = RESET;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "Hello, World!\r\n";

/* Size of Transmission buffer for UART */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer for UART */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* USER CODE BEGIN PV */
uint8_t RX_1Byte[1] = { 0x00 };
uint8_t RX_data[2] = { 0x00, 0x00 }; // used in several functions to store data read from BQ769x2
uint8_t RX_4Byte[4] = { 0x00, 0x00, 0x00, 0x00 };
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
uint8_t DSG = 0;   // discharge FET state
uint8_t CHG = 0;   // charge FET state
uint8_t PCHG = 0;  // pre-charge FET state
uint8_t PDSG = 0;  // pre-discharge FET state

uint32_t AccumulatedCharge_Int; // in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Frac; // in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Time; // in BQ769x2_READPASSQ func

//Used for state machine
uint8_t BUTTON_COUNT = 0;
uint16_t CRC_FAIL = 0;
uint16_t INACTIVITY_COUNT = 0;
uint16_t RETRY_COUNT = 0;
uint8_t RESET_BY_WATCHDOG = 0;

//Used by UART
uint8_t RxData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t TxData[32] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
int sequence = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_WWDG_Init(void);
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

void delayUS(uint32_t us) {   // Sets the delay in microseconds.
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;
}

void delayMS(uint32_t ms) {
	for (int i = 0; i < ms; i++) {
		//PetWatchdog();
		delayUS(1000);
	}
}

void delayMS_WDG(uint32_t ms) {
	for (int i = 0; i < ms; i++) {
		PetWatchdog();
		delayUS(1000);
	}
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count) {
	uint8_t copyIndex = 0;
	for (copyIndex = 0; copyIndex < count; copyIndex++) {
		dest[copyIndex] = source[copyIndex];
	}
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

void I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
	uint8_t TX_Buffer[MAX_BUFFER_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00 };
#if CRC_Mode
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

/* hacked version taht doesn't segfault */
int I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
	unsigned int RX_CRC_Fail = 0; // reset to 0. If in CRC Mode and CRC fails, this will be incremented.
	uint8_t RX_Buffer[MAX_BUFFER_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00 };

	//uint8_t RX_Buffer[MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#if CRC_Mode
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
		CRC_FAIL += RX_CRC_Fail;
	}
#else
	HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
	return 0;
}

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

void CommandSubcommands(uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{ //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

	uint8_t TX_Reg[2] = { 0x00, 0x00 };

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	I2C_WriteReg(0x3E, TX_Reg, 2);
	delayMS(2);
}

void ReadReg(uint16_t command, uint16_t data, uint8_t type) {
	//security keys and Manu_data writes dont work with this function (reading these commands works)
	//max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
	uint8_t TX_Reg[4] = { 0x00, 0x00, 0x00, 0x00 };

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	if (type == R) {	//read
		I2C_WriteReg(0x3E, TX_Reg, 2);
		delayUS(2000);
		I2C_ReadReg(0x40, RX_32Byte, 16); //RX_32Byte is a global variable, but CRC fails if I read more than 16 bytes
	}
}

//read data from Subcommands. Max readback size is 16 bytes because of a bug that would cause CRC errors with readbacks longer than 16.
void Subcommands(uint16_t command, uint16_t data, uint8_t type)
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
		//I2C_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
		I2C_ReadReg(0x40, RX_32Byte, 16); //more then 16 would cause CRC errors for a reason I didn't dig into, so I limit this to 16. This does mean that large data reads are not supported.
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

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{	//type: R = read, W = write
	uint8_t TX_data[2] = { 0x00, 0x00 };

	//little endian format
	TX_data[0] = data & 0xff;
	TX_data[1] = (data >> 8) & 0xff;

	if (type == R) {	//Read
		I2C_ReadReg(command, RX_data, 2); //RX_data is a global variable
		delayUS(2000);
	}
	if (type == W) { //write
		//Control_status, alarm_status, alarm_enable all 2 bytes long
		I2C_WriteReg(command, TX_data, 2);
		delayUS(2000);
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

void BQ769x2_ReadBatteryStatus() { //good example functions
	// Read Battery Status to see which bits are set
	// This shows which primary protections have been triggered
	DirectCommands(BatteryStatus, 0x00, R);
	value_BatteryStatus = (RX_data[1] * 256 + RX_data[0]);
	//Example Fault Flags
}

uint16_t BQ769x2_ReadDeviceNumber() { //good example functions
	// Read Battery Status to see which bits are set
	// This shows which primary protections have been triggered
	Subcommands(DEVICE_NUMBER, 0x00, R);
	return (RX_32Byte[1] * 256 + RX_32Byte[0]);
	//Example Fault Flags
}

uint16_t BQ769x2_ReadUnsignedRegister(uint16_t reg_addr, uint8_t count) {
	// Read Battery Status to see which bits are set
	// This shows which primary protections have been triggered
	Subcommands(reg_addr, 0x00, R);
	switch (count) {
	case 1:
		return RX_32Byte[0];
	case 2:
		return (RX_32Byte[1] * 256 + RX_32Byte[0]);
	}

	//Example Fault Flags
	return 0;
}

int16_t BQ769x2_ReadSignedRegister(uint16_t reg_addr, uint8_t count) {
	// Ready a 1 or 2 byte unsigned register
	Subcommands(reg_addr, 0x00, R);
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
	ReadReg(reg_addr, 0x00, R);

	const unsigned char *b = (const unsigned char*) RX_32Byte;
	uint32_t temp = 0;
	temp = ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
	return ReverseFloat(*((float*) &temp));

	//Example Fault Flags
}

uint32_t FloatToUInt(float n) {
	return (uint32_t) (*(uint32_t*) &n);
}

float UIntToFloat(uint32_t n) {
	return (float) (*(float*) &n);
}

void BQ769x2_Set_Confirm_Register(uint16_t reg_addr, uint32_t reg_data,
		uint8_t datalen) {
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

void BQ769x2_Init() {
	// Configures all parameters in device RAM

	// Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
	// See TRM for full description of CONFIG_UPDATE mode

	CommandSubcommands(SET_CFGUPDATE);

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

	/* MFG status - keep chip in FULLACCESS, but set FETs to autonomous mode */
	BQ769x2_SetRegister(MfgStatusInit, 0b0000000000010000, 2); //autonomous mode

	//BQ769x2_SetRegister(MfgStatusInit,0b0000000000000000,2); //debug mode, not for production

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
	BQ769x2_SetRegister(VCellMode, 0b1010101011111111, 2); //Faraday BMS = 0xAAFF for 12 cells. See schematic

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

	/* Balancing Configuration - leaving defaults for deltas (>40mV to start, <20mV to stop*/
	BQ769x2_SetRegister(BalancingConfiguration, 0b00000011, 1); //Set balancing to autonomously operate while in RELAX and CHARGE configurations. Sleep is disabled.
	//BQ769x2_SetRegister(BalancingConfiguration, 0b00000000, 1); //Set balancing to autonomously operate while in RELAX and CHARGE configurations. Sleep is disabled.
	BQ769x2_SetRegister(CellBalanceMaxCells, 3, 1); //0x933A  - set maximum number of cells that may balance at once. Contributes to thermal limit
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

	/* Sleep voltage and current thresholds */
	//Sleep Current
	//Sleep Voltage time
	/*Overcurrent configuration*/
	BQ769x2_SetRegister(OCCThreshold, 0x0F, 1); //OCC (over-current in charge) Threshold - 0x9280 = 0x05 (30mV = 3A across 10mOhm sense resistor) Units in 2mV
	BQ769x2_SetRegister(OCD1Threshold, 0x62, 1); //OCD1 "fast"Threshold - 0x9282 = 0x62 (195 mV = 19.5A across 10mOhm sense resistor) units of 2mV
	BQ769x2_SetRegister(OCD1Delay, 0x04, 1); //OCD1 Delay- 0x9283 = 10 ms to 426 ms in units of 3.3 ms, with the actual delay being 3.3 ms × (2 + setting) = 20ms.
	BQ769x2_SetRegister(OCD2Threshold, 0x4B, 1); //OCD1 Threshold - 0x9284 = 0x0A (150 mV = 15A across 10mOhm sense resistor) units of 2mV
	BQ769x2_SetRegister(OCD2Delay, 0x58, 1); //OCD2 Threshold - 0x9285 = 10 ms to 426 ms in units of 3.3 ms, with the actual delay being 3.3 ms × (2 + 88) = 90ms.
	BQ769x2_SetRegister(SCDThreshold, 0x0B, 1); //Short circuit discharge Threshold - 0x9286 = 0x05 (300 mV = 25A across 10mOhm sense resistor)
	BQ769x2_SetRegister(SCDDelay, 0x03, 1); //SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (3 - 1) * 15 µs; min value of 1
	BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1); //Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01

	// Exit CONFIGUPDATE mode  - Subcommand 0x0092
	CommandSubcommands(EXIT_CFGUPDATE);
	delayUS(60000); //wait for chip to be ready
}

/*verify that device has been configured*/
uint8_t BQ769x2_Configured() {

	//Verify that device is not in config update
	BQ769x2_ReadBatteryStatus();
	if (value_BatteryStatus & 1) {
		return 0;
	}
	//Read back a register that should have been configured to double check
	if (BQ769x2_ReadUnsignedRegister(VCellMode, 2) == 0b1010101011111111) {
		return 1;
	}

	//battery spot check complete.
	return 0;

}

//  ********************************* FET Control Commands  ***************************************

void MCU_BOTHOFF() {
	// Disables all FETs using the DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // CFETOFF pin (BOTHOFF) set low
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // DFETOFF pin (BOTHOFF) set low
}

void MCU_DISABLE_BOTHOFF() {
	// Resets DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);  // DFETOFF pin set high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // DFETOFF pin set high
}

void BQ769x2_ReadFETStatus() {
	// Read FET Status to see which FETs are enabled
	DirectCommands(FETStatus, 0x00, R);
	FET_Status = (RX_data[1] * 256 + RX_data[0]);
	DSG = ((0x4 & RX_data[0]) >> 2); // discharge FET state
	CHG = (0x1 & RX_data[0]); // charge FET state
	PCHG = ((0x2 & RX_data[0]) >> 1); // pre-charge FET state
	PDSG = ((0x8 & RX_data[0]) >> 3); // pre-discharge FET state
}

// ********************************* End of FET Control Commands *********************************

// ********************************* BQ769x2 Power Commands   *****************************************

void BQ769x2_ShutdownPin() {
	// Puts the device into SHUTDOWN mode using the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // Sets RST_SHUT pin
}

void BQ769x2_ReleaseShutdownPin() {
	// Releases the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // Resets RST_SHUT pin
}

/*Put the BQ into DEEPSLEEP by writing the DEEPSLEEP command twice. Confirm that deepsleep mode has been achieved. Returns 0 if successful.*/
uint8_t BQ769x2_EnterDeepSleep() {
	CommandSubcommands(DEEPSLEEP);
	delayUS(2000);
	CommandSubcommands(DEEPSLEEP);
	delayUS(2000);
	if (BQ769x2_ReadControlStatus() & 0x04) //if bit 2 is high, then device is in DEEPSLEEP. Return 0 for success
			{
		return 0;
	}
	return 1;
}

/*Wake up the BQ from DEEPSLEEP by toggling RST_SHUT once. Return 1 if not in DEEPSLEEP, otherwise 0*/
uint8_t BQ769x2_Wake() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	delayUS(2000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	delayUS(2000);

	if (BQ769x2_ReadControlStatus() & 0x04) //if bit 2 is high, then device is in DEEPSLEEP
			{
		return 0;
	}
	return 1;

}

/*Put the BQ chip into SHUTDOWN by holding RST_SHUT high for more than 1 second.
 * Typically only used for debugging, since there is no software way to get out
 * of shutdown without connecting a charger*/
void BQ769x2_EnterShutDown() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	delayMS_WDG(1500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

//warning - this will turn off the 3V3 rail. Use with care
uint8_t BQ769x2_Reset() {
	PetWatchdog();
	uint8_t retry_counter = 0;
	RETRY_COUNT = 0; //set global retry counter to 0
	while (retry_counter < 20) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		CommandSubcommands(BQ769x2_RESET);  // Resets the BQ769x2 registers
		//wait at least 250ms for everything to start up
		for (int i = 0; i++; i < 25) {
			delayMS_WDG(10);
		}
		BQ769x2_ReadBatteryStatus();

		delayUS(1000);
		if (value_BatteryStatus & 0x300) { //if bits 8 and 9 of battery status are anything other than zero, device is initialized
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			return 0;
		};
		retry_counter++;
	}
	return retry_counter;
}

uint8_t BQ769x2_Ready() {
	BQ769x2_ReadBatteryStatus();
	if (value_BatteryStatus & 0x300) { //if bits 8 and 9 of battery status are anything other than zero, device is initialized
		delayMS_WDG(60);
		return 1;
	};
	return 0;
}

/* Quickly toggle RST_SHUT to wake the BQ chip. Simple function that doesn't block */
void BQ769x2_SoftWake() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	delayUS(2000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	delayUS(2000);
}

// ********************************* End of BQ769x2 Power Commands   *****************************************

// ********************************* BQ769x2 Status and Fault Commands   *****************************************

uint16_t BQ769x2_ReadAlarmStatus() {
	// Read this register to find out why the ALERT pin was asserted
	DirectCommands(AlarmStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

uint16_t BQ769x2_ReadControlStatus() {
	// Read this register to find out why the ALERT pin was asserted
	DirectCommands(ControlStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

uint16_t BQ769x2_ReadRawAlarmStatus() {
	// Read this register to find out why the ALERT pin was asserted
	DirectCommands(AlarmRawStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

void BQ769x2_ReadSafetyStatus() { //good example functions
	// Read Safety Status A/B/C and find which bits are set
	// This shows which primary protections have been triggered
	DirectCommands(SafetyStatusA, 0x00, R);
	value_SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);
	//Example Fault Flags
	UV_Fault = ((0x4 & RX_data[0]) >> 2);
	OV_Fault = ((0x8 & RX_data[0]) >> 3);
	SCD_Fault = ((0x8 & RX_data[1]) >> 3);
	OCD_Fault = ((0x2 & RX_data[1]) >> 1);
	DirectCommands(SafetyStatusB, 0x00, R);
	value_SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);

	DirectCommands(SafetyStatusC, 0x00, R);
	value_SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);

	if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 1) {
		ProtectionsTriggered = 1;
	} else {
		ProtectionsTriggered = 0;
	}
}

void BQ769x2_ReadPFStatus() {
	// Read Permanent Fail Status A/B/C and find which bits are set
	// This shows which permanent failures have been triggered
	DirectCommands(PFStatusA, 0x00, R);
	value_PFStatusA = (RX_data[1] * 256 + RX_data[0]);
	DirectCommands(PFStatusB, 0x00, R);
	value_PFStatusB = (RX_data[1] * 256 + RX_data[0]);
	DirectCommands(PFStatusC, 0x00, R);
	value_PFStatusC = (RX_data[1] * 256 + RX_data[0]);
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************

// ********************************* BQ769x2 Measurement Commands   *****************************************

uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
	//RX_data is global var
	DirectCommands(command, 0x00, R);
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
	int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
	for (int x = 0; x < 16; x++) { //Reads all cell voltages
		CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
		cellvoltageholder = cellvoltageholder + 2;
	}
	Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
	Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
	LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}

int16_t BQ769x2_ReadCurrent()
// Reads PACK current
{
	DirectCommands(CC2Current, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]); // cell balancing is reported as a uint_16 bitfield
}

float BQ769x2_ReadTemperature(uint8_t command) {
	DirectCommands(command, 0x00, R);
	//RX_data is a global var
	return (0.1 * (float) (RX_data[1] * 256 + RX_data[0])) - 273.15; // converts from 0.1K to Celcius
}

void BQ769x2_ReadBalancingStatus() {
	Subcommands(CB_ACTIVE_CELLS, 0x00, R);
	CB_ActiveCells = (RX_32Byte[1] * 256 + RX_32Byte[0]); //CB_ACTIVE_CELLS returns a 2 byte bitfield of which cells are balancing
}

void BQ769x2_ReadPassQ() { // Read Accumulated Charge and Time from DASTATUS6
	Subcommands(DASTATUS6, 0x00, R);
	AccumulatedCharge_Int = ((RX_32Byte[3] << 24) + (RX_32Byte[2] << 16)
			+ (RX_32Byte[1] << 8) + RX_32Byte[0]); //Bytes 0-3
	AccumulatedCharge_Frac = ((RX_32Byte[7] << 24) + (RX_32Byte[6] << 16)
			+ (RX_32Byte[5] << 8) + RX_32Byte[4]); //Bytes 4-7
	AccumulatedCharge_Time = ((RX_32Byte[11] << 24) + (RX_32Byte[10] << 16)
			+ (RX_32Byte[9] << 8) + RX_32Byte[8]); //Bytes 8-11
}

float unpackFloat(const void *buf) {
	const unsigned char *b = (const unsigned char*) buf;
	uint32_t temp = 0;
	temp = ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
	return *((float*) &temp);
}
void CalcMinMaxCellV() {
	// Assume the first element is the minimum
	int maxV = 50;
	int minV = 6000;

	// Loop through the array to find the minimum
	for (int i = 0; i < 16; i++) {
		if (0xAAFF & (1 << i)) {
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

void BQ769x2_PrintStatus() {
	AlarmBits = BQ769x2_ReadRawAlarmStatus();
	printf(
			"CRC_FAIL: %d Balancing: %d DSG: %d CHG: %d ScanComplete: %d Prots: %d SSA: %d SSBC: %d UV: %d OV: %d SCD: %d OCD: %d SSA: %#x, SSB: %#x, SSC: %#x",
			CRC_FAIL,
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
			UV_Fault, OV_Fault, SCD_Fault, OCD_Fault, value_SafetyStatusA,
			value_SafetyStatusB, value_SafetyStatusC);

	CalcMinMaxCellV();
	printf(
			" I: %d MIN_V: %d MAX_V: %d INACTIVITY_COUNT: %d TS1: %f TS3: %f T_HDQ: %f T_INT: %f \r\n",
			Pack_Current, CellMinV, CellMaxV, INACTIVITY_COUNT, Temperature[0],
			Temperature[1], Temperature[2], Temperature[3]);

}

/* UART Functions
 * ===================== */

void THVD2410_Sleep() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}
;
void THVD2410_Transmit() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}
;
void THVD2410_Receive() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}
;

void UART_WaitForCommand() {

	if (!UartBusy) {
		//set RX buffer to all zeros
		for (int i = 0; i < 8; i++) {
			RxData[i] = 0;
		}

		//start recieve-to-idle interrupt
		THVD2410_Receive();
		huart2.RxState = HAL_UART_STATE_READY; //TODO EVALUATE THIS WITH SASHA
		if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, RxData, 8) != HAL_OK) {
			Error_Handler();
		}
	}

}

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

//prep the cell voltage message (29 byte)
uint8_t UART_PrepCellVoltageMessage() {

	int i;
	for (i = 0; i < 32; i++) {
		TxData[i] = 0;
	}

	//device, fxn, length
	TxData[0] = 0x02;
	TxData[1] = 0x03;
	TxData[2] = 0x1D;

	// loop through voltage field and construct uints
	int k = 0;
	for (int i = 0; i < 16; i++) {
		if (0xAAFF & (1 << i)) {
			TxData[2 * k + 3] = (CellVoltage[i] * 2 / 3) >> 8;
			TxData[2 * k + 4] = (CellVoltage[i] * 2 / 3) & 0xFF;
			k++;
		}
	}

	//add crc in reverse byte order
	uint16_t crc = UART_CRC(TxData, 27);
	TxData[27] = crc & 0xFF;
	TxData[28] = crc >> 8;

	return 29;
}

uint8_t UART_PrepCellBalancingMessage() {

	for (int i = 0; i < 32; i++) {
		TxData[i] = 0;
	}

	TxData[0] = 0x02;
	TxData[1] = 0x03;
	TxData[2] = 0x01;

	// CB_ActiveCells is a bitfield of which of 16 channels are active. Bike expects a bitfield with the first byte blank and the remainder a bitfield
	uint16_t CB = 0x00;
	int k = 0;
	for (int i = 0; i < 16; i++) {
		if (0xAAFF & (1 << i)) {
			//i is the cell number active cell bitfield (0xAAFF)
			//k is the cell number in faraday index
			if (CB_ActiveCells & (1 << i)) {
				CB = CB | (1 << k); //TODO determine whether the cell number is mapped correctly
			};
			k++;
		}
	}

	TxData[3] = CB >> 8;
	TxData[4] = CB & 0xFF;

	//add crc in reverse byte order
	uint16_t crc = UART_CRC(TxData, 5);
	TxData[5] = crc & 0xFF;
	TxData[6] = crc >> 8;

	return 7;
}

/* Respond to battery status message 1 - battery OK. Hard coded for now
 * Example request 0x2 0x3 0x0 0x0 0x0 0x1 0x84 0x39
 * Example response 0x2 0x3 0x2 0x0 0x0 0xfc 0x44
 */
uint8_t UART_PrepBatteryStatusMessage1() {

	for (int i = 0; i < 32; i++) {
		TxData[i] = 0;
	}

	TxData[0] = 0x02;
	TxData[1] = 0x03;
	TxData[2] = 0x02;
	TxData[3] = 0x00;
	TxData[4] = 0x00;

	//add crc in reverse byte order
	uint16_t crc = UART_CRC(TxData, 5);
	TxData[5] = crc & 0xFF;
	TxData[6] = crc >> 8;

	return 7;
}

/* Fill TxData to battery status message 2 - purpose unknown. Hard coded for now
 * Example request 0x2 0x3 0x0 0x1 0x0 0x1 0xd5 0xf9
 * Example response 0x2 0x3 0x2 0x0 0x19 0x3d 0x8e
 */
uint8_t UART_PrepBatteryStatusMessage2() {

	for (int i = 0; i < 32; i++) {
		TxData[i] = 0;
	}

	TxData[0] = 0x02;
	TxData[1] = 0x03;
	TxData[2] = 0x02;
	TxData[3] = 0x00;
	TxData[4] = 0x19;

	//add crc in reverse byte order
	uint16_t crc = UART_CRC(TxData, 5);
	TxData[5] = crc & 0xFF;
	TxData[6] = crc >> 8;

	return 7;
}

/*Takes in an 8 byte message, parses, checks CRC, and then responds with the appropriate message */
void UART_Respond(uint8_t *buf, uint16_t size) {

	//Check to make sure that the UART message is an 8 byte read
	if (size == 8) {

		//unpack message
		uint8_t SlaveID = buf[0];
		uint8_t FunctionCode = buf[1];
		uint16_t Address = buf[2] << 8 | buf[3];
		uint16_t NumRegs = buf[4] << 8 | buf[5];
		uint16_t CRCRecv = buf[7] << 8 | buf[6];

		//Check CRC and that we're the target audience (BMS is 0x02)
		if (CRCRecv == UART_CRC(buf, 6) && SlaveID == 0x02) {
			if (FunctionCode == 0x03) { //it's a read
#ifdef LEDS
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				delayUS(50);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
#endif
				uint8_t len;

				if (Address == 0x02) { //Battery Voltage Message
					len = UART_PrepCellVoltageMessage();
					THVD2410_Transmit();
					HAL_UART_Transmit(&huart2, TxData, len, UART_TIMEOUT_S);

				} else if (Address == 0x17) { //Battery Balancing Message
					len = UART_PrepCellBalancingMessage();
					THVD2410_Transmit();
					HAL_UART_Transmit(&huart2, TxData, len, UART_TIMEOUT_S);

				} else if (Address == 0x00 && NumRegs == 0x01) { //Battery Status Message 1
					len = UART_PrepBatteryStatusMessage1();
					THVD2410_Transmit();
					HAL_UART_Transmit(&huart2, TxData, len, UART_TIMEOUT_S);

				} else if (Address == 0x01 && NumRegs == 0x01) { //Battery Status Message 1
					len = UART_PrepBatteryStatusMessage2();
					THVD2410_Transmit();
					HAL_UART_Transmit(&huart2, TxData, len, UART_TIMEOUT_S);
				}
			}
		}

	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

/* State Machine Functions
 * ===================== */

/* Implement button press timing*/
void HandleButton() {
	if (STM32_Wake_Button_Pressed()) {
		BUTTON_COUNT++;

	} else if (BUTTON_COUNT) {
		BUTTON_COUNT -= 1;
	};

	if (BUTTON_COUNT > 15) {
		if (DEBUG) {
			printf("\r\ntime to sleep\r\n");
		}
		BUTTON_COUNT = 0;
		Sleep();
	};
}

/* If current below 250mA for more than XXX loops, activate sleep. Must have a fresh value in Pack_Current */
void HandleInactivity() {

//Increment sleep timer if current is between +20mA and -250mA
	if (Pack_Current >= -250 && Pack_Current < 50) {
		INACTIVITY_COUNT++;
	} else if (Pack_Current < -250) {
		INACTIVITY_COUNT = INACTIVITY_COUNT / 2; //exponential decay if current detected
	}

	if (!DEBUG) {
		if (INACTIVITY_COUNT > 10000) {
			Sleep();
		}

		//loop is much slower with prinfs enabled, so make this more reasonable
	} else {
		if (INACTIVITY_COUNT > 500) {
			printf("\r\ninactivity timeout\r\n");
			Sleep();
		}

	}

}
void STM32_Stop() {
	//blink to show that we're entering sleep
	if (LEDS) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		delayMS(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	}

	//put RS485 transciever into sleep
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

	//turn off LEDs
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

	//Turn off FETs
	MCU_DISABLE_BOTHOFF();

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

	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
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

	/* Disable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();

	/* Enter Stop Mode */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	/* resume execution */
	NVIC_SystemReset(); //When woken up, just reset, it's simpler that way.

}

void CheckForWatchdogReset() {
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) {
		RESET_BY_WATCHDOG = 1;
	}

	/* Clear reset flags in any cases */
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

uint8_t STM32_Wake_Button_Pressed() {
	return !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));
}

void PetWatchdog() {
	BlinkLED(50);
	if (HAL_WWDG_Refresh(&hiwdg) != HAL_OK) {
		/* Refresh Error */
		Error_Handler();
	}
}

/* Put the BQ chip to sleep and then put the STM32 to STOP. STM32 resets on wake */
void Sleep() {

	//clear the FULLSCAN bit before going into sleep
	while (BQ769x2_EnterDeepSleep()) {
		delayUS(5000);
	}

	DirectCommands(AlarmStatus, 0x0080, W); // Clear the FULLSCAN bit

	if (DEBUG) {
		printf("\r\nBQ in DEEP SLEEP, STM getting ready for bed...\r\n");
	}
	STM32_Stop();
}

/* LED Helper Functions
 * ===================== */

/* Update FET LEDs with most recent status */
void UpdateFETLEDs() {
	BQ769x2_ReadFETStatus();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, DSG);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, CHG);
}

/* Helper function used to calibrate delayUS*/
void FlashLED(uint16_t period) {
	while (1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		delayUS(period);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		delayUS(period);
	}
}

/* Helper function used to calibrate delayUS*/
void BlinkLED(uint16_t period) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	delayUS(period);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	delayUS(period);
}

// ********************************* End of BQ769x2 Measurement Commands   *****************************************

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buff[10];
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
		CheckForWatchdogReset();
	}
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();

	MX_USART2_UART_Init();

	/* Configure the system Power */
	SystemPower_Config();

	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // RST_SHUT pin set low
	delayMS(50); //Wait for everything to stabilize

	if (DEBUG) {
		printf("STM32 awake\r\n");
	}

	//if (!DEBUG){
	//	MX_WWDG_Init();
	//}

	UART_WaitForCommand(); //Start UART Recv interrupt

	//BQ769x2_Reset(); //TODO remove, this appears to only be necssary to update balancing parameters

	while (1) {

		/* USER CODE BEGIN 3 */
		MCU_BOTHOFF(); //disable FETs until we are getting communication from the BQ chip

		BQ769x2_SoftWake(); //wiggle RST_SHUT to do a partial reset of the BQ chip. Not sure if this is necessary but it doesn't seem to hurt.
		PetWatchdog();

		if (BQ769x2_ReadControlStatus() & 0x04) //if bit 2 is high, then device is in DEEPSLEEP
				{
			if (DEBUG) {
				printf("BQ was asleep\r\n");
			}

			delayUS(1000);
			if (RETRY_COUNT > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RETRY_COUNT++;
			PetWatchdog();
		}

		RETRY_COUNT = 0;
		PetWatchdog();

		//Check for BQ state FULLACCESS, SEALED, or UNSEALED. Device must be connected and ACKing to get past this point
		while (!BQ769x2_Ready()) {
			if (DEBUG) {
				printf("not ready\r\n");
			}
			delayUS(1000);
			if (RETRY_COUNT > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RETRY_COUNT++;
			PetWatchdog();
		};

		RETRY_COUNT = 0;

		//Wake up the device if it isn't already awake
		while (!BQ769x2_Wake()) {
			printf("not awake\r\n");
			delayUS(1000);
			if (RETRY_COUNT > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RETRY_COUNT++;
			PetWatchdog();
		}

		RETRY_COUNT = 0;
		PetWatchdog();

		//Initialize registers. Must complete in order to be successful
		while (!BQ769x2_Configured()) {
			BQ769x2_Init();
			if (DEBUG) {
				printf("not configured\r\n");
			}
			if (RETRY_COUNT > RETRY_LIMIT) {
				BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			RETRY_COUNT++;
			PetWatchdog();
		}

		RETRY_COUNT = 0;
		PetWatchdog();

		//int16_t min_bal_v = BQ769x2_ReadSignedRegister(CellBalanceMinCellVCharge,2);

		//Some register cannot be written without a hard reset apparently?
		//while (BQ769x2_ReadSignedRegister(CellBalanceMinCellVCharge,2) != 3700) {
		//	BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
		//}

		//enable FETs. It appears to be important to call FET_ENABLE twice
		CommandSubcommands(FET_ENABLE); // Enable the CHG and DSG FETs
		MCU_DISABLE_BOTHOFF();
		CommandSubcommands(FET_ENABLE); //for some reason need to do this twice...

		while (1) {

			PetWatchdog();

			UART_WaitForCommand();

			//detect button press and take action if needed
			HandleButton();

			//Update FET registers and update LEDs
			//#ifdef LEDS
			UpdateFETLEDs();
			//#endif

			//Handle sleep current and put battery to sleep if not much is going on
			HandleInactivity();

			//Print battery status over RS485 for debug
			if (DEBUG) {
				BQ769x2_ReadBatteryStatus();
				BQ769x2_PrintStatus();
			}

			//Check for measurements. These
			//char balancemax = BQ769x2_ReadUnsignedRegister(CellBalanceMaxCells, 1);
			//int16_t min_bal_v = BQ769x2_ReadSignedRegister(CellBalanceMinCellVCharge,2);
			//char balanceconfig = BQ769x2_ReadUnsignedRegister(BalancingConfiguration,1);
			PetWatchdog();
			AlarmBits = BQ769x2_ReadAlarmStatus();

			if (AlarmBits & 0x80) { // Check if FULLSCAN is complete. If set, new measurements are available

				Pack_Current = BQ769x2_ReadCurrent(); //needed for HandleInactivity, do not remove
				PetWatchdog();

				int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
				for (int x = 0; x < 16; x++) { //Reads all cell voltages
					PetWatchdog();
					CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
					cellvoltageholder = cellvoltageholder + 2;
				}

				PetWatchdog();
				Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
				Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);

				PetWatchdog();
				LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);

				PetWatchdog();
				BQ769x2_ReadBalancingStatus(); //needed for balancing status message

				PetWatchdog();
				Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
				Temperature[1] = BQ769x2_ReadTemperature(TS3Temperature);
				Temperature[2] = BQ769x2_ReadTemperature(HDQTemperature);
				Temperature[3] = BQ769x2_ReadTemperature(IntTemperature);

				PetWatchdog();
				DirectCommands(AlarmStatus, 0x0080, W); // Clear the FULLSCAN bit

				RETRY_COUNT = 0;

			} else {
				if (RETRY_COUNT > RETRY_LIMIT) {
					BQ769x2_Reset(); //gotta reset the BQ and try again. This kills the 3V3 rail
					break;
				}
				RETRY_COUNT++;
			}

			//Check for faults and trigger the LED if so
			PetWatchdog();
			BQ769x2_ReadSafetyStatus();
			if (ProtectionsTriggered & 1) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			}

			PetWatchdog();
			delayMS(10);  // repeat loop every 20 ms

		}
	}
	/* USER CODE END 3 */
}

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // Receive Off
	delayUS(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // Transmit On
	delayUS(20);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //Transmit off
	delayUS(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // Receive still off
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
	GPIO_InitStructure.Pull = GPIO_NOPULL;
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
	GPIO_InitStructure.Pull = GPIO_NOPULL;
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
	//RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16; //slow this down to get WWDG to ~20ms or so
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
	htim2.Init.Prescaler = 1; //for 16MHz with 1/16 prescaler for PCLK1
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
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of IT Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	/* Set transmission flag: transfer complete */
	UartReady = SET;

}

//receive commands, all of which should be 8 bytes long
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (!UartBusy) {
		UartBusy = SET;
		//HAL_UARTEx_ReceiveToIdle_IT(&huart2, RxData, 8);
		if (Size == 8) {
			CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE); //theoretically stop the interrupt https://stackoverflow.com/questions/55394656/how-do-i-reset-the-stm32-hal-uart-driver-hal-state
			UART_Respond(RxData, 8);
		}

		UartBusy = RESET;
		THVD2410_Receive();
		//delayUS(1000);
		UART_WaitForCommand();
	}
}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	/* Set transmission flag: transfer complete */
	UartReady = SET;

}

/* Function handle UART errors. Tries to clear the error bit. If the error isn't one of these, then we reset the UART */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->ErrorCode & HAL_UART_ERROR_FE) {
		// frame error
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_FEF);
	} else if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
		// overrun error
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
	} else if (huart->ErrorCode & HAL_UART_ERROR_NE) {
		// overrun error
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_NEF);
	} else if (huart->ErrorCode & HAL_UART_ERROR_PE) {
		// overrun error
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF);
	} else {
		//fully re-initialize uart. Slower, but we might need to recover from a new error
		if (HAL_UART_DeInit(&huart2) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_UART_Init(&huart2) != HAL_OK) {
			Error_Handler();
			;
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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	__disable_irq();

	//reset and hope things go better the next time around
	if (DEBUG) {
		while (1) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			delayMS(500);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			delayMS(2000);
		}
	} else {
		delayMS(1000); //wait a second and let's try this out again
		//NVIC_SystemReset();
		while (1)
			; //let the watchdog reset us
	}
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
