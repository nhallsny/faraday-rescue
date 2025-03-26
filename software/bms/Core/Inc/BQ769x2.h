/**
 ******************************************************************************
 * @file    BQ769x2.h
 * @author  nhallsny
 * @brief   BQ769x2 battery management chip driver using STM32 HAL.
 *
 *
 ******************************************************************************
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================

 Connect the following pins to GPIOs on your MCU:

 - RST_SHUT
 - CFETOFF
 - DFETOFF
 - ALERT -> connect to wake input if using for wake, such as waking when a charger is detected.

 Connect to 400khz I2c with <1.5kOhm pullups
 - SDA
 - SCL

 Connect the following to an ADC:
 - REG18 -> only needed if the MCU needs to confirm the status of the BQ chip.

 For configuration for your application, you will need to set the following:
 - Set the i2c channel in BQ769x2_InitState (typically 0x10)
 - Set the active cells (bitfield of which of the 16 channels you're using) in BQ769x2_InitState. If using less than 16 cells, make sure to follow Ti guide for doing this.
 - Set the CRC mode (1 for active, 0 for inactive) in BQ769x2_InitState
 - Set RST_SHUT, CFETOFF, and DFETOFF ports and pins
 - Edit BQ769x2_Configure in BQ769x2.c for your application. Spend lots of quality time with the Technical Reference Manual. https://www.ti.com/lit/pdf/sluucw9

 Generally, there are a few steps to using the driver to get the BQ chip to work. These instructions are based on a simple e-bike application without pre-charge:

 1. Declare a global BQState on the stack in the main.c
 > BQState mybatt;

 2. Call BQ769x2_InitState to initialize the state variable. 0x10 is the default i2c address.
 > BQ769x2_InitState(&mybatt, i2c_hdl, 0x10, etc)

 3. Wake up the chip by wiggling RST_SHUT. May not be necessary, but it doesn't hurt
 > BQ769x2_SoftWake(&mybatt)

 4. Check repeatedly to make sure the chip is responding to i2c. Only proceed if this succeeds, it could take 20-30 tries while the chip boots.
 > BQ769x2_Ready(&mybatt)

 5. Make sure the chip is in NORMAL mode
 > BQ769x2_Wake(&mybatt)

 6. Configure the chips and verifies that it was configured. Super super important.
 > BQ769x2_Initialize(&mybatt)

 7. Read ADC values
 > BQ769x2_ReadBatteryData(&mybatt)

 8. Enable FETs
 > BQ769x2_AllowFETs(&mybatt);

 What is not supported:
 - One-time-programming functions, including chip locking
 - Pre-charge configuration
 - Coulomb counting
 - SLEEP State (only using DEEPSLEEP, NORMAL, and SHUTDOWN)

 ******************************************************************************
 */

#ifndef BQ769x2_H_
#define BQ769x2_H_

#include "BQ769x2Header.h"

typedef struct {
	uint16_t ActiveCells;
	I2C_HandleTypeDef *i2c_hdl;
	uint8_t i2c_adr;
	uint8_t i2c_crc; //1 for CRC enabled, 0 for disabled
	TIM_HandleTypeDef *tim_hdl;
	GPIO_TypeDef *RST_SHUT_PORT;
	uint8_t RST_SHUT_PIN;
	GPIO_TypeDef *CFETOFF_PORT;
	uint8_t CFETOFF_PIN;
	GPIO_TypeDef *DFETOFF_PORT;
	uint8_t DFETOFF_PIN;
	int16_t CellVoltage[16];
	int16_t CellMinV;
	int16_t CellMaxV;
	float Temperature[4];
	int8_t CellMinT;
	int8_t CellMaxT;
	uint16_t Stack_Voltage;
	uint16_t Pack_Voltage;
	uint16_t LD_Voltage;
	int16_t Pack_Current;
	uint16_t AlarmBits;
	uint8_t value_SafetyStatusA;  // Safety Status Register A
	uint8_t value_SafetyStatusB;  // Safety Status Register B
	uint8_t value_SafetyStatusC;  // Safety Status Register C
	uint8_t value_PFStatusA;   // Permanent Fail Status Register A
	uint8_t value_PFStatusB;   // Permanent Fail Status Register B
	uint8_t value_PFStatusC;   // Permanent Fail Status Register C
	uint16_t value_BatteryStatus; //battery status a
	uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs
	uint16_t CB_ActiveCells;  // Cell Balancing Active Cells
	uint8_t UV_Fault;  // under-voltage fault state
	uint8_t OV_Fault;   // over-voltage fault state
	uint8_t SCD_Fault;  // short-circuit fault state
	uint8_t OCD_Fault;  // over-current fault state
	uint8_t ProtectionsTriggered; // Set to 1 if any protection triggers
	uint8_t LD_ON;	// Load Detect status bit
	uint8_t Dsg;   // discharge FET state
	uint8_t Chg;   // charge FET state
	uint8_t PChg;  // pre-charge FET state
	uint8_t PDsg;  // pre-discharge FET state
	uint16_t AccumulatedCharge_Int;
	uint16_t AccumulatedCharge_Frac;
	uint16_t AccumulatedCharge_Time;
	uint16_t CRC_Fail;
} BQState;

void BQ769x2_Set_Confirm_Register(BQState *s, uint16_t reg_addr,
		uint32_t reg_data, uint8_t datalen);

// Function Prototypes
void BQ769x2_InitState(BQState *s, void *i2c_hdl, uint8_t i2c_adr,
		uint8_t crc_mode, void *tim_hdl, uint16_t ACTIVE_CELLS,
		GPIO_TypeDef *RST_SHUT_PORT, uint8_t RST_SHUT_PIN,
		GPIO_TypeDef *CFETOFF_PORT, uint8_t CFETOFF_PIN,
		GPIO_TypeDef *DFETOFF_PORT, uint8_t DFETOFF_PIN);

unsigned char Checksum(unsigned char *ptr, unsigned char len);
unsigned char CRC8(unsigned char *ptr, unsigned char len);

void I2C_WriteReg(BQState *s, uint8_t reg_addr, uint8_t *reg_data,
		uint8_t count);
int I2C_BQ769x2_ReadReg(BQState *s, uint8_t reg_addr, uint8_t *reg_data,
		uint8_t count);

void BQ769x2_SetRegister(BQState *s, uint16_t reg_addr, uint32_t reg_data,
		uint8_t datalen);
void BQ769x2_CommandSubcommand(BQState *s, uint16_t command);
void BQ769x2_ReadReg(BQState *s, uint16_t command, uint16_t data, uint8_t type);
void BQ769x2_Subcommand(BQState *s, uint16_t command, uint16_t data,
		uint8_t type);
void BQ769x2_DirectCommand(BQState *s, uint8_t command, uint16_t data,
		uint8_t type);

uint16_t BQ769x2_ReadUnsignedRegister(BQState *s, uint16_t reg_addr,
		uint8_t count);
int16_t BQ769x2_ReadSignedRegister(BQState *s, uint16_t reg_addr,
		uint8_t count);
float BQ769x2_ReadFloatRegister(BQState *s, uint16_t reg_addr);
void BQ769x2_Set_Confirm_Register(BQState *s, uint16_t reg_addr,
		uint32_t reg_data, uint8_t datalen);

void BQ769x2_ReadBatteryStatus(BQState *s);
uint8_t BQ769x2_ReadBatteryData(BQState *s);
uint16_t BQ769x2_ReadDeviceNumber(BQState *s);
void BQ769x2_Configure(BQState *s);
uint8_t BQ769x2_Initialize(BQState *s);

void BQ769x2_ForceDisableFETs(BQState *s);
void BQ769x2_AllowFETs(BQState *s);
void BQ769x2_ReadFETStatus(BQState *s);

void BQ769x2_SetShutdownPin(BQState *s);
void BQ769x2_ResetShutdownPin(BQState *s);
uint8_t BQ769x2_EnterDeepSleep(BQState *s);
uint8_t BQ769x2_Wake(BQState *s);
void BQ769x2_EnterShutDown(BQState *s);
uint8_t BQ769x2_Reset(BQState *s);
uint8_t BQ769x2_Ready(BQState *s);
void BQ769x2_SoftWake(BQState *s);
void BQ769x2_ClearFullScan(BQState *s);

uint16_t BQ769x2_ReadAlarmStatus(BQState *s);
uint16_t BQ769x2_ReadControlStatus(BQState *s);
uint16_t BQ769x2_ReadRawAlarmStatus(BQState *s);
uint8_t BQ769x2_ReadSafetyStatus(BQState *s);
void BQ769x2_ReadPFStatus(BQState *s);
uint16_t BQ769x2_ReadVoltage(BQState *s, uint8_t command);
void BQ769x2_ReadAllVoltages(BQState *s);
int16_t BQ769x2_ReadCurrent(BQState *s);
float BQ769x2_ReadTemperature(BQState *s, uint8_t command);

void BQ769x2_ReadBalancingStatus(BQState *s);
void BQ769x2_CalcMinMaxCellV(BQState *s);
void BQ769x2_CalcMinMaxCellT(BQState *s);

#endif
