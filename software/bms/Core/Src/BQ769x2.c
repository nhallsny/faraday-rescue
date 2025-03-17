#include "utils.c"

#include "stdio.h"
#include "stdint.h"
#include "stm32l0xx_hal.h"
#include "BQ769x2.h"

/*BQ Parameters */
#define DEV_ADDR  0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_MODE 1  // 0 for disabled, 1 for enabled
#define ACTIVE_CELLS 0xAAFF //bitfield which indicates which cells to have protections on. Specific to this Faraday 12S pack
#define MAX_BUFFER_SIZE 32

/* Fake Enums */
#define R 0 // Read; Used in BQ769x2_DirectCommand and Subcommands functions
#define W 1 // Write; Used in BQ769x2_DirectCommand and Subcommands functions
#define W2 2 // Write data with two bytes; Used in Subcommands function

// Global Variables for buffers
uint8_t RX_data[32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t RX_32Byte[32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};



/*initialze BQ state struct*/
void BQ769x2_InitState(BQState* s, void* i2c_hdl,
		uint8_t RST_SHUT_PORT,
		uint8_t RST_SHUT_PIN,
		uint8_t CFETOFF_PORT,
		uint8_t CFETOFF_PIN,
		uint8_t DFETOFF_PORT,
		uint8_t DFETOFF_PIN){
	s->bq_i2c = i2c_hdl;
	s->RST_SHUT_PORT = RST_SHUT_PORT;
	s->RST_SHUT_PIN = RST_SHUT_PIN;
	s->CFETOFF_PORT = CFETOFF_PORT;
	s->CFETOFF_PIN = CFETOFF_PIN;
	s->DFETOFF_PORT = DFETOFF_PORT;
	s->DFETOFF_PIN = DFETOFF_PIN;
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
void I2C_WriteReg(BQState* s, uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
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

		HAL_I2C_Mem_Write(s->bq_i2c, DEV_ADDR, reg_addr, 1, TX_Buffer, crc_count,
				1000);

	}
#else
	HAL_I2C_Mem_Write(s->bq_i2c, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
}

/* hacked version that doesn't segfault */
int I2C_BQ769x2_ReadReg(BQState* s, uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
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

		HAL_I2C_Mem_Read(s->bq_i2c, DEV_ADDR, reg_addr, 1, ReceiveBuffer,
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
		s->CRC_Fail += RX_CRC_Fail;
	}
#else
	HAL_I2C_Mem_Read(s->bq_i2c, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
	return 0;
}

/* BQ Functions -----------------------------------------------*/
void BQ769x2_SetRegister(BQState* s, uint16_t reg_addr, uint32_t reg_data, uint8_t datalen) {
	uint8_t TX_Buffer[2] = { 0x00, 0x00 };
	uint8_t TX_RegData[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	//TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff;
	TX_RegData[1] = (reg_addr >> 8) & 0xff;
	TX_RegData[2] = reg_data & 0xff; //1st byte of data

	switch (datalen) {
	case 1: //1 byte datalength
		I2C_WriteReg(s,0x3E, TX_RegData, 3);
		delayUS(2000);
		TX_Buffer[0] = Checksum(TX_RegData, 3);
		TX_Buffer[1] = 0x05; //combined length of register address and data
		I2C_WriteReg(s,0x60, TX_Buffer, 2); // Write the checksum and length
		delayUS(2000);
		break;
	case 2: //2 byte datalength
		TX_RegData[3] = (reg_data >> 8) & 0xff;
		I2C_WriteReg(s,0x3E, TX_RegData, 4);
		delayUS(2000);
		TX_Buffer[0] = Checksum(TX_RegData, 4);
		TX_Buffer[1] = 0x06; //combined length of register address and data
		I2C_WriteReg(s,0x60, TX_Buffer, 2); // Write the checksum and length
		delayUS(2000);
		break;
	case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
		TX_RegData[3] = (reg_data >> 8) & 0xff;
		TX_RegData[4] = (reg_data >> 16) & 0xff;
		TX_RegData[5] = (reg_data >> 24) & 0xff;
		I2C_WriteReg(s,0x3E, TX_RegData, 6);
		delayUS(2000);
		TX_Buffer[0] = Checksum(TX_RegData, 6);
		TX_Buffer[1] = 0x08; //combined length of register address and data
		I2C_WriteReg(s,0x60, TX_Buffer, 2); // Write the checksum and length
		delayUS(2000);
		break;
	}
	delayMS(2);
}

void BQ769x2_CommandSubcommand(BQState* s, uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{ //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

	uint8_t TX_Reg[2] = { 0x00, 0x00 };

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	I2C_WriteReg(s,0x3E, TX_Reg, 2);
	delayMS(2);
}

void BQ769x2_ReadReg(BQState* s, uint16_t command, uint16_t data, uint8_t type) {
	//security keys and Manu_data writes dont work with this function (reading these commands works)
	//max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot. Seems to segfault above 16 bytes
	uint8_t TX_Reg[4] = { 0x00, 0x00, 0x00, 0x00 };

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	if (type == R) {	//read
		I2C_WriteReg(s,0x3E, TX_Reg, 2);
		delayUS(2000);
		I2C_BQ769x2_ReadReg(s,0x40, RX_32Byte, 16); //RX_32Byte is a global variable, but CRC fails if I read more than 16 bytes
	}
}

//read data from Subcommands. Max readback size is 16 bytes because of a bug that would cause CRC errors with readbacks longer than 16.
void BQ769x2_Subcommand(BQState* s, uint16_t command, uint16_t data, uint8_t type)
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
		I2C_WriteReg(s,0x3E, TX_Reg, 2);
		delayUS(2000);
		//I2C_BQ769x2_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
		I2C_BQ769x2_ReadReg(s,0x40, RX_32Byte, 16); //more then 16 would cause CRC errors for a reason I didn't dig into, so I limit this to 16. This does mean that large data reads are not supported.
	} else if (type == W) {
		//FET_Control, REG12_Control
		TX_Reg[2] = data & 0xff;
		I2C_WriteReg(s,0x3E, TX_Reg, 3);
		delayUS(2000);
		TX_Buffer[0] = Checksum(TX_Reg, 3);
		TX_Buffer[1] = 0x05; //combined length of registers address and data
		I2C_WriteReg(s,0x60, TX_Buffer, 2);
	} else if (type == W2) { //write data with 2 bytes
		//CB_Active_Cells, CB_SET_LVL
		TX_Reg[2] = data & 0xff;
		TX_Reg[3] = (data >> 8) & 0xff;
		I2C_WriteReg(s,0x3E, TX_Reg, 4);
		delayUS(1000);
		TX_Buffer[0] = Checksum(TX_Reg, 4);
		TX_Buffer[1] = 0x06; //combined length of registers address and data
		I2C_WriteReg(s,0x60, TX_Buffer, 2);
	}
	delayMS(2);
}

void BQ769x2_DirectCommand(BQState* s, uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{	//type: R = read, W = write
	uint8_t TX_data[2] = { 0x00, 0x00 };

	//little endian format
	TX_data[0] = data & 0xff;
	TX_data[1] = (data >> 8) & 0xff;

	if (type == R) {	//Read
		I2C_BQ769x2_ReadReg(s,command, RX_data, 2); //RX_data is a global variable
		delayUS(2000);
	}
	if (type == W) { //write
		//Control_status, alarm_status, alarm_enable all 2 bytes long
		I2C_WriteReg(s,command, TX_data, 2);
		delayUS(2000);
	}
}



uint16_t BQ769x2_ReadUnsignedRegister(BQState* s, uint16_t reg_addr, uint8_t count) {
	// Read Unsigned Register of 1 or 2 byte length
	BQ769x2_Subcommand(s,reg_addr, 0x00, R);
	switch (count) {
	case 1:
		return RX_32Byte[0];
	case 2:
		return (RX_32Byte[1] * 256 + RX_32Byte[0]);
	}
	return 0;
}

int16_t BQ769x2_ReadSignedRegister(BQState* s, uint16_t reg_addr, uint8_t count) {
	// Read signed Register of 1 or 2 byte length
	BQ769x2_Subcommand(s,reg_addr, 0x00, R);
	switch (count) {
	case 1:
		return RX_32Byte[0];
	case 2:
		return (RX_32Byte[1] * 256 + RX_32Byte[0]);
	}

	return 0;
}

float BQ769x2_ReadFloatRegister(BQState* s, uint16_t reg_addr) {
	// Read a 4 byte float (CC_Gain and Capacity Gain Only)
	BQ769x2_ReadReg(s,reg_addr, 0x00, R);

	const unsigned char *b = (const unsigned char*) RX_32Byte;
	uint32_t temp = 0;
	temp = ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
	return ReverseFloat(*((float*) &temp));

	//Example Fault Flags
}

void BQ769x2_Set_Confirm_Register(BQState* s, uint16_t reg_addr, uint32_t reg_data,
		uint8_t datalen) {
	//set and then verify that a register has been set
	uint8_t tries = 0;
	uint32_t buf = 0;
	while (tries < 10) {
		BQ769x2_SetRegister(s,reg_addr, reg_data, datalen);
		buf = BQ769x2_ReadUnsignedRegister(s,reg_addr, datalen);
		if (reg_data == buf) {
			return;
		}
		tries++;
	}
}

void BQ769x2_ReadBatteryStatus(BQState* s) {
	// Read Battery Status with DirectCommand
	// This shows which primary protections have been triggered
	BQ769x2_DirectCommand(s,BatteryStatus, 0x00, R);
	s->value_BatteryStatus = (RX_data[1] * 256 + RX_data[0]);
}

uint16_t BQ769x2_ReadDeviceNumber(BQState* s) {
	// Read Device Number with SubCommand
	BQ769x2_Subcommand(s,DEVICE_NUMBER, 0x00, R);
	return (RX_32Byte[1] * 256 + RX_32Byte[0]);
}

void BQ769x2_Configure(BQState* s) {
	// Configures all parameters in device RAM

	// Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
	BQ769x2_CommandSubcommand(s,SET_CFGUPDATE);

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
	BQ769x2_SetRegister(s,PowerConfig, 0b0010110010100010, 2);
	BQ769x2_SetRegister(s,FETOptions, 0x0D, 1); //device may not turn FETs on autonomously unless allowed to do so. Important because the STM32 is the only thing that can turn the pack off.
	BQ769x2_SetRegister(s,REG0Config, 0x01, 1); // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
	BQ769x2_SetRegister(s,REG12Config, 0x0D, 1); // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V and disable REG2 as not used

	/* MFG status - keep chip in FULLACCESS, but set FETs to autonomous mode and enable permanent fail*/
	BQ769x2_SetRegister(s,MfgStatusInit, 0b0000000001010000, 2); //autonomous mode and enable permanent fail

	/* Pin Function Configs */
	BQ769x2_SetRegister(s,DFETOFFPinConfig, 0b10000010, 1); // Set DFETOFF pin to control BOTH DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
	BQ769x2_SetRegister(s,CFETOFFPinConfig, 0b10000010, 1); // Set CFETOFF pin to control BOTH CHG FET - 0x92FA = 0x42 (set to 0x00 to disable). Configures as ALT function, active-low, individual control

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
	BQ769x2_SetRegister(s,ALERTPinConfig, 0b00100110, 1);
	BQ769x2_SetRegister(s,TS1Config, 0x07, 1); // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
	BQ769x2_SetRegister(s,TS3Config, 0x07, 1); // Set TS3 to measure Cell Temperature - 0x74
	BQ769x2_SetRegister(s,HDQPinConfig, 0x07, 1); // Set HDQ pin to measure Cell temperature

	/* Alarm Configuration - what can pull the ALERT pin high */
	// 'Default Alarm Mask' - only use the FULLSCAN bit - this will be set to trigger the ALERT pin to wake from DEEPSLEEP
	BQ769x2_SetRegister(s,DefaultAlarmMask, 0x0080, 2);

	/* Current measurement */
	BQ769x2_SetRegister(s,CCGain, FloatToUInt((float) 0.75684), 4); // 7.5684/R_sense(mOhm) = 7.5684/10 = 7.76
	BQ769x2_SetRegister(s,CapacityGain, FloatToUInt((float) 225736.32), 4); // CC Gain * 298261.6178 = = 7.5684/10 * 298261.6178 = 225736.32

	/* Thermistor Calibration from TI BQ tool
	 *
	 bestA [A1 A2 A3 A4 A5] =  [-22175  31696 -16652  31696 4029]
	 bestB [B1 B2 B3 B4] =  [-23835  20738 -8470  4596]
	 Adc0 = 11703
	 */
	BQ769x2_SetRegister(s,T18kCoeffa1, (int16_t) -22175, 2);
	BQ769x2_SetRegister(s,T18kCoeffa2, (int16_t) 31696, 2);
	BQ769x2_SetRegister(s,T18kCoeffa3, (int16_t) -16652, 2);
	BQ769x2_SetRegister(s,T18kCoeffa4, (int16_t) 31696, 2);
	BQ769x2_SetRegister(s,T18kCoeffa5, (int16_t) 4029, 2);
	BQ769x2_SetRegister(s,T18kCoeffb1, (int16_t) -23835, 2);
	BQ769x2_SetRegister(s,T18kCoeffb2, (int16_t) 20738, 2);
	BQ769x2_SetRegister(s,T18kCoeffb3, (int16_t) -8470, 2);
	BQ769x2_SetRegister(s,T18kCoeffb4, (int16_t) 4596, 2);
	BQ769x2_SetRegister(s,T18kAdc0, (int16_t) 11703, 2);

	/* Protections Config */
	BQ769x2_SetRegister(s,VCellMode, ACTIVE_CELLS, 2); //Faraday BMS = 0xAAFF for 12 cells. See schematic

	// Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
	// Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
	// COV (over-voltage), CUV (under-voltage)
	//BQ769x2_SetRegister(s,EnabledProtectionsA, 0xBC, 1);
	BQ769x2_SetRegister(s,EnabledProtectionsA, 0b10111100, 1);

	// Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
	// Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
	// OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
	BQ769x2_SetRegister(s,EnabledProtectionsB, 0b01110111, 1);

	BQ769x2_SetRegister(s,UTCThreshold, (signed char) 5, 1); // Set undertemperature in charge threshold. Assume +/- 5C because of poor thermistor coupling
	BQ769x2_SetRegister(s,UTDThreshold, (signed char) -10, 1); // Set undertemperature in discharge threshold. Assume +/- 5C because of poor thermistor coupling
	BQ769x2_SetRegister(s,OTCThreshold, (signed char) 40, 1); // Set overtemperature in charge threshold. Assume +/- 5C because of poor thermistor coupling
	BQ769x2_SetRegister(s,OTDThreshold, (signed char) 55, 1); // Set overtemperature in discharge threshold. Assume +/- 5C because of poor thermistor coupling

	/*Set Permanent Fail for SUV and SOV to just disable FETs to prevent charging of severely undervolted cells. STM32 goes to sleep due to inactivity afterwards*/
	BQ769x2_SetRegister(s,SUVThreshold,1900,2); //0x92CB - set safety undervoltage threshold to 1.9V
	BQ769x2_SetRegister(s,SOVThreshold,4500,2); //0x92CE - set safety overvoltage threshold to 4.5V
	BQ769x2_SetRegister(s,EnabledPFA,0b00000011,1); //Enable Permanent Fail for Safety Undervoltage and Safety Overvoltage Only
	BQ769x2_SetRegister(s,EnabledPFD,0b00000001,1); //Enable Permanent Fail for Safety Undervoltage and Safety Overvoltage Only
	BQ769x2_SetRegister(s,TOSSThreshold,240,2); //enable permanent fail if top of stack voltage deviates from cell voltages added up by 240*12 = 2.8V
	BQ769x2_SetRegister(s,ProtectionConfiguration,0x02,2); //Set Permanent Fail to turn FETs off only. Assume that idle will take care of DEEPSLEEP

	/* Balancing Configuration - leaving defaults for deltas (>40mV to start, <20mV to stop*/
	BQ769x2_SetRegister(s,BalancingConfiguration, 0b00000011, 1); //Set balancing to autonomously operate while in RELAX and CHARGE configurations. Sleep is disabled.
	BQ769x2_SetRegister(s,CellBalanceMaxCells, 3, 1); //0x933A  - set maximum number of cells that may balance at once. Contributes to thermal limit of BQ chip
	BQ769x2_SetRegister(s,CellBalanceMinDeltaCharge, 25, 1); //0x933D - set minimum cell balance delta at which balancing starts in CHARGE to 15mV
	BQ769x2_SetRegister(s,CellBalanceMinDeltaRelax, 25, 1); //0x933D - set minimum cell balance delta at which balancing starts in RELAX to 15mV
	BQ769x2_SetRegister(s,CellBalanceStopDeltaCharge, 15, 1); //0x933D - set minimum cell balance delta at which balancing stops in CHARGE to 15mV
	BQ769x2_SetRegister(s,CellBalanceStopDeltaRelax, 15, 1); //0x933D - set minimum cell balance delta at which balancing stops in RELAX to 15mV
	BQ769x2_SetRegister(s,CellBalanceMinCellVCharge, (int16_t) 0x0E74, 2); //0x933B -Minimum voltage at which cells start balancing. Set to 3700mV for now
	BQ769x2_SetRegister(s,CellBalanceMinCellVRelax, (int16_t) 0x0E74, 2); //0x933F -Minimum voltage at which cells start balancing. Set to 3700mV for now

	/*Over and Under Voltage configuration*/
	BQ769x2_SetRegister(s,CUVThreshold, 0x34, 1); //CUV (under-voltage) Threshold - 0x9275 = 0x34 (2631 mV) this value multiplied by 50.6mV = 2631mV
	BQ769x2_SetRegister(s,COVThreshold, 0x53, 1); //COV (over-voltage) Threshold - 0x9278 = 0x53 (4199 mV) this value multiplied by 50.6mV = 4199mV
	BQ769x2_SetRegister(s,ShutdownCellVoltage, 0x0960, 2); // Shutdown 0x923F - enter SHUTDOWN when below this cell voltage to minimize power draw . Set to 2400mV
	BQ769x2_SetRegister(s,ShutdownStackVoltage, 0x0AC8, 2); //ShutdownStackVoltage 0x9241 - enter SHUTDOWn when the stack is below this voltage - set to 2300mV/cell -> 2760*10mV
	BQ769x2_SetRegister(s,CUVRecoveryHysteresis, 0x04, 1); //CUVRecoveryHystersis 0x927B - hysteresis value after COV - set to 200mV -> 4* 50.6mV = 202.4mV

	/*Definitions of charge and discharge*/
	BQ769x2_SetRegister(s,DsgCurrentThreshold, 0x64, 2); //0x9310   Set definition of discharge in mA. 100mA. Balancing happens when current is above the negative of this current.
	BQ769x2_SetRegister(s,ChgCurrentThreshold, 0x32, 2); //0x9312  Set definition of charge in mA. 50mA Balancing happens in charge when above this current

	/*Charge current limit*/
	BQ769x2_SetRegister(s,OCCThreshold, 0x0F, 1); //OCC (over-current in charge) Threshold - 0x9280 = 0x05 (30mV = 3A across 10mOhm sense resistor) Units in 2mV
	BQ769x2_SetRegister(s,OCCDelay, 0x1E, 1); //OCC Delay (over current in charge delay) - 0x9281 = 0x0D (around 100ms)

	/*Overcurrent in Discharge Config*/
	BQ769x2_SetRegister(s,OCD1Threshold, 0x7D, 1); //OCD1 "fast"Threshold - 0x9282 = 0x62 (250 mV = -25A across 10mOhm sense resistor) units of 2mV
	BQ769x2_SetRegister(s,OCD1Delay, 0x04, 1); //OCD1 Delay- 0x9283 = 10 ms to 426 ms in units of 3.3 ms, with the actual delay being 3.3 ms × (2 + setting) = 20ms.

	BQ769x2_SetRegister(s,OCD2Threshold, 0x64, 1); //OCD1 Threshold - 0x9284 = (0x50 * 20 mV = 20A across 10mOhm sense resistor) units of 2mV
	BQ769x2_SetRegister(s,OCD2Delay, 0x5A, 1); //OCD2 Threshold - 0x9285 = 10 ms to 426 ms in units of 3.3 ms, with the actual delay being 3.3 ms × (2 + 120) = 90ms.

	BQ769x2_SetRegister(s,OCD3Threshold, (int16_t) -16000, 2); //OCD3 Threshold - 0x928A (-16A in units of user Amps (mA))
	BQ769x2_SetRegister(s,OCD3Delay, 15, 2); //OCD3 Threshold - 0x928C (15 second delay)

	/*Fast short circuit detection config */
	BQ769x2_SetRegister(s,SCDThreshold, 0x0D, 1); //Short circuit discharge Threshold - 0x9286 = 0x0B (400 mV = 40A across 10mOhm sense resistor)
	BQ769x2_SetRegister(s,SCDDelay, 0x11, 1); //SCD Delay - 0x9287 = 0x11 (240us = (17-1)*15us = 240us
	BQ769x2_SetRegister(s,SCDLLatchLimit, 0x01, 1); //Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01

	// Exit CONFIGUPDATE mode  - Subcommand 0x0092
	BQ769x2_CommandSubcommand(EXIT_CFGUPDATE);
	delayUS(60000); //wait for chip to be ready
}

/*Initialize BQ chip by configuring registers and then checking that a critical register was written. Return 1 if successfully configured, 0 if failed*/
uint8_t BQ769x2_Initialize(BQState* s) {

	BQ769x2_Configure(s);

	//Fail if device is still in config update mode.
	BQ769x2_ReadBatteryStatus(s);
	if (s->value_BatteryStatus & 1) {
		return 0;
	}
	//Fail if the active cells register is not 0xAAFF
	if (BQ769x2_ReadUnsignedRegister(s,VCellMode, 2) != ACTIVE_CELLS) {
		return 0;
	}

	//config successful and register spot check complete
	return 1;

}

//  ********************************* FET Control Commands  ***************************************

void BQ769x2_ForceDisableFETs(BQState* s) {
	// Disables all FETs using the DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(s->CFETOFF_PORT, s->CFETOFF_PIN, GPIO_PIN_RESET); // CFETOFF pin (BOTHOFF) set low
	HAL_GPIO_WritePin(s->DFETOFF_PORT, s->DFETOFF_PIN, GPIO_PIN_RESET); // DFETOFF pin (BOTHOFF) set low
}

void BQ769x2_AllowFETs(BQState* s) {
	// Resets DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(s->CFETOFF_PORT, s->CFETOFF_PIN, GPIO_PIN_SET); // CFETOFF pin set high
	HAL_GPIO_WritePin(s->DFETOFF_PORT, s->DFETOFF_PIN, GPIO_PIN_SET); // DFETOFF pin set high
}

void BQ769x2_ReadFETStatus(BQState* s) {
	// Read FET Status to see which FETs are enabled
	BQ769x2_DirectCommand(s,FETStatus, 0x00, R);
	s->FET_Status = (RX_data[1] * 256 + RX_data[0]);
	s->Dsg = ((0x4 & RX_data[0]) >> 2); // discharge FET state
	s->Chg = (0x1 & RX_data[0]); // charge FET state
	s->PChg = ((0x2 & RX_data[0]) >> 1); // pre-charge FET state
	s->PDsg = ((0x8 & RX_data[0]) >> 3); // pre-discharge FET state
}

// ********************************* End of FET Control Commands *********************************

// ********************************* BQ769x2 Power Commands   *****************************************

void BQ769x2_SetShutdownPin(BQState* s) {
	// Puts the device into SHUTDOWN mode using the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(s->RST_SHUT_PORT, s->RST_SHUT_PIN, GPIO_PIN_SET); // Sets RST_SHUT pin
}

void BQ769x2_ResetShutdownPin(BQState* s) {
	// Releases the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(s->RST_SHUT_PORT, s->RST_SHUT_PIN, GPIO_PIN_RESET); // Resets RST_SHUT pin
}

/*Put the BQ into DEEPSLEEP by writing the DEEPSLEEP command twice. Returns 1 if successful, otherwise 0*/
uint8_t BQ769x2_EnterDeepSleep(BQState* s) {
	BQ769x2_CommandSubcommand(s,DEEPSLEEP);
	delayUS(2000);
	BQ769x2_CommandSubcommand(s,DEEPSLEEP);
	delayUS(2000);
	if (BQ769x2_ReadControlStatus(s) & 0x04) //if bit 2 is high, then device is in DEEPSLEEP. Return 1 for success
			{
		return 1;
	}
	return 0;
}

/*Wake up the BQ from DEEPSLEEP by toggling RST_SHUT once. Return 1 if successfully woken up, otherwise 0*/
uint8_t BQ769x2_Wake(BQState* s, uint8_t rst_shut_port, uint8_t rst_shut_pin) { //TODO add port/pin to this
	HAL_GPIO_WritePin(s->RST_SHUT_PORT, s->RST_SHUT_PIN, GPIO_PIN_SET);
	delayUS(2000);
	HAL_GPIO_WritePin(s->RST_SHUT_PORT, s->RST_SHUT_PIN, GPIO_PIN_RESET);
	delayUS(2000);

	if (BQ769x2_ReadControlStatus(s) & 0x04) { //if bit 2 is high, then device is in DEEPSLEEP
		return 0;
	}

	return 1;

}

/*Put the BQ chip into SHUTDOWN by holding RST_SHUT high for more than 1 second.
 * Typically only used for debugging, since there is no software way to get out
 * of shutdown without connecting a charger*/
void BQ769x2_EnterShutDown(BQState* s, uint8_t rst_shut_port, uint8_t rst_shut_pin) {
	HAL_GPIO_WritePin(s->RST_SHUT_PORT, s->RST_SHUT_PIN, GPIO_PIN_SET);
	delayMS(1500);
	HAL_GPIO_WritePin(s->RST_SHUT_PORT, s->RST_SHUT_PIN, GPIO_PIN_RESET);
}

/* BQ769x2_Reset - hard reset of the BQ chip, which takes ~250ms. The 3V3 rail comes up 20ms after the reset begins. Returns 0 if the 3v3 rail is externally powered.
 * Should only be called if the 3v3 rail needs to be reset, which can fix some issues related to sleep with the STM32 after using the debugger */
void BQ769x2_Reset(BQState* s) {
	uint8_t RetryCounter = 0;
	while (RetryCounter < 20) {
		BQ769x2_CommandSubcommand(s,BQ769x2_RESET); // Resets the BQ769x2 registers and kills the 3v3 Rail
		delayUS(1000); //wait for the 3.3v rail to die
		RetryCounter++;
	}
	return 0; //should never get here unless 3v3 rail is externally powered by debugger

}
/* BQ769x2_Ready - return 1 if BQ is initialized, 0 otherwise */
uint8_t BQ769x2_Ready(BQState* s) {
	BQ769x2_ReadBatteryStatus(s);
	if (s->value_BatteryStatus & 0x300) { //if bits 8 and 9 of battery status are set, device has finished booting
		delayMS(60);
		return 1;
	};
	return 0;
}

/* Quickly toggle RST_SHUT to wake the BQ chip. Simple function that doesn't block */
void BQ769x2_SoftWake(BQState* s) {
	HAL_GPIO_WritePin(s->RST_SHUT_PORT, s->RST_SHUT_PIN, GPIO_PIN_SET);
	delayUS(2000);
	HAL_GPIO_WritePin(s->RST_SHUT_PORT, s->RST_SHUT_PIN, GPIO_PIN_RESET);
	delayUS(2000);
}

// ********************************* End of BQ769x2 Power Commands   *****************************************

// ********************************* BQ769x2 Status and Fault Commands   *****************************************

uint16_t BQ769x2_ReadAlarmStatus(BQState* s) {
	// Read this register to find out why the ALERT pin was asserted
	BQ769x2_DirectCommand(s,AlarmStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

uint16_t BQ769x2_ReadControlStatus(BQState* s) {
	// Read this register to get the Control Status Pins
	BQ769x2_DirectCommand(s,ControlStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

uint16_t BQ769x2_ReadRawAlarmStatus(BQState* s) {
	// Read this register to find out why the ALERT raw pin was asserted. Distinct from AlarmStatus in that these do not latch
	BQ769x2_DirectCommand(s,AlarmRawStatus, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]);
}

//update safety status A,B,C and UV/OV/SCD/OCD/ProtectionsTriggered flags in state struct
uint8_t BQ769x2_ReadSafetyStatus(BQState* s) {
	// Read Safety Status A/B/C and find which bits are set
	// This shows which primary protections have been triggered
	// Return 1 at end to match structure of other functions
	BQ769x2_DirectCommand(s,SafetyStatusA, 0x00, R);
	s->value_SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);
	//Example Fault Flags
	s->UV_Fault = ((0x4 & RX_data[0]) >> 2);
	s->OV_Fault = ((0x8 & RX_data[0]) >> 3);
	s->SCD_Fault = ((0x8 & RX_data[1]) >> 3);
	s->OCD_Fault = ((0x2 & RX_data[1]) >> 1);
	BQ769x2_DirectCommand(s,SafetyStatusB, 0x00, R);
	s->value_SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);

	BQ769x2_DirectCommand(s,SafetyStatusC, 0x00, R);
	s->value_SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);

	if ((s->value_SafetyStatusA + s->value_SafetyStatusB + s->value_SafetyStatusC) > 1) {
		s->ProtectionsTriggered = 1;
	} else {
		s->ProtectionsTriggered = 0;
	}

	return 1;
}

void BQ769x2_ReadPFStatus(BQState* s) {
	// Read Permanent Fail Status A/B/C and find which bits are set
	// This shows which permanent failures have been triggered
	BQ769x2_DirectCommand(s,PFStatusA, 0x00, R);
	s->value_PFStatusA = (RX_data[1] * 256 + RX_data[0]);
	BQ769x2_DirectCommand(s,PFStatusB, 0x00, R);
	s->value_PFStatusB = (RX_data[1] * 256 + RX_data[0]);
	BQ769x2_DirectCommand(s,PFStatusC, 0x00, R);
	s->value_PFStatusC = (RX_data[1] * 256 + RX_data[0]);
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************

// ********************************* BQ769x2 Measurement Commands   *****************************************

uint16_t BQ769x2_ReadVoltage(BQState* s,uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
	//RX_data is global var
	BQ769x2_DirectCommand(s,command, 0x00, R);
	delayUS(2000);
	if (command >= Cell1Voltage && command <= Cell16Voltage) {//Cells 1 through 16 (0x14 to 0x32)
		return (RX_data[1] * 256 + RX_data[0]); //voltage is reported in mV
	} else { //stack, Pack, LD
		return 10 * (RX_data[1] * 256 + RX_data[0]); //voltage is reported in 0.01V units
	}

}
void BQ769x2_ReadAllVoltages(BQState* s)
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
	//int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
	//for (int x = 0; x < 16; x++) { //Reads all cell voltages
	//	CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
	//	cellvoltageholder = cellvoltageholder + 2;
	//}
	for (int x = 0; x < 16; x++) { //Reads all cell voltages
			s->CellVoltage[x] = BQ769x2_ReadVoltage(s,Cell1Voltage+2*x); //TODO check this line
	}

	s->Stack_Voltage = BQ769x2_ReadVoltage(s,StackVoltage);
	s->Pack_Voltage = BQ769x2_ReadVoltage(s,PACKPinVoltage);
	s->LD_Voltage = BQ769x2_ReadVoltage(s,LDPinVoltage);
}

int16_t BQ769x2_ReadCurrent(BQState* s)
// Reads PACK current
{
	BQ769x2_DirectCommand(s,CC2Current, 0x00, R);
	return (RX_data[1] * 256 + RX_data[0]); // cell current is reported as an int16 in UserAmps
}

float BQ769x2_ReadTemperature(BQState* s, uint8_t command) {
	BQ769x2_DirectCommand(s,command, 0x00, R);
	//RX_data is a global var
	return (0.1 * (float) (RX_data[1] * 256 + RX_data[0])) - 273.15; // converts from 0.1K to Celcius
}

void BQ769x2_ReadBalancingStatus(BQState* s) {
	BQ769x2_Subcommand(s,CB_ACTIVE_CELLS, 0x00, R);
	s->CB_ActiveCells = (RX_32Byte[1] * 256 + RX_32Byte[0]); //CB_ACTIVE_CELLS returns a 2 byte bitfield of which cells are balancing
}

void BQ769x2_ReadPassQ(BQState* s) { // Read Accumulated Charge and Time from DASTATUS6
	BQ769x2_Subcommand(s,DASTATUS6, 0x00, R);
	s->AccumulatedCharge_Int = ((RX_32Byte[3] << 24) + (RX_32Byte[2] << 16)
			+ (RX_32Byte[1] << 8) + RX_32Byte[0]); //Bytes 0-3
	s->AccumulatedCharge_Frac = ((RX_32Byte[7] << 24) + (RX_32Byte[6] << 16)
			+ (RX_32Byte[5] << 8) + RX_32Byte[4]); //Bytes 4-7
	s->AccumulatedCharge_Time = ((RX_32Byte[11] << 24) + (RX_32Byte[10] << 16)
			+ (RX_32Byte[9] << 8) + RX_32Byte[8]); //Bytes 8-11
}

/* update variables in STM32 with values from the BQ chip. Returns 1 if successful, 0 if failed or data not yet ready */
uint8_t BQ769x2_ReadBatteryData(BQState* s) {
	s->AlarmBits = BQ769x2_ReadAlarmStatus(s);
	if (s->AlarmBits & 0x80) { // Check if FULLSCAN is complete. If set, new measurements are available
		s->Pack_Current = BQ769x2_ReadCurrent(s); //needed for STM32_HandleInactivity, do not remove
		BQ769x2_ReadAllVoltages(s); //get most recent voltages
		BQ769x2_ReadBalancingStatus(s); //needed for balancing status message
		s->Temperature[0] = BQ769x2_ReadTemperature(s,TS1Temperature);
		s->Temperature[1] = BQ769x2_ReadTemperature(s,TS3Temperature);
		s->Temperature[2] = BQ769x2_ReadTemperature(s,HDQTemperature);
		s->Temperature[3] = BQ769x2_ReadTemperature(s,IntTemperature);

		BQ769x2_DirectCommand(s,AlarmStatus, 0x0080, W); // Clear the FULLSCAN bit
		return 1;
	} else {
		return 0;
	}
}

/* calculate min and max voltage, update globals. */
void BQ769x2_CalcMinMaxCellV(BQState* s) {
	// Assume the first element is the minimum
	int maxV = 50;
	int minV = 6000;

	// Loop through the array to find the minimum
	for (int i = 0; i < 16; i++) {
		if (ACTIVE_CELLS & (1 << i)) {
			if (s->CellVoltage[i] < minV) {
				minV = s->CellVoltage[i];
			}
			if (s->CellVoltage[i] > maxV) {
				maxV = s->CellVoltage[i];
			}
		}
	}
	s->CellMinV = minV;
	s->CellMaxV = maxV;
}
;



// ********************************* End of BQ769x2 Measurement Commands   *****************************************
