#ifndef BQ769x2_H_
#define BQ769x2_H_

#include "BQ769x2Header.h"

typedef struct {
	uint16_t ActiveCells;
	I2C_HandleTypeDef *i2c_hdl;
	uint8_t i2c_adr;
	uint8_t i2c_crc; //1 for CRC enabled, 0 for disabled
	TIM_HandleTypeDef *tim_hdl;
	GPIO_TypeDef * RST_SHUT_PORT;
	uint8_t RST_SHUT_PIN;
	GPIO_TypeDef * CFETOFF_PORT;
	uint8_t CFETOFF_PIN;
	GPIO_TypeDef * DFETOFF_PORT;
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
void BQ769x2_InitState(BQState *s, void *i2c_hdl, uint8_t i2c_adr,uint8_t crc_mode,void *tim_hdl,
		uint16_t ACTIVE_CELLS, GPIO_TypeDef * RST_SHUT_PORT, uint8_t RST_SHUT_PIN, GPIO_TypeDef * CFETOFF_PORT,
		uint8_t CFETOFF_PIN, GPIO_TypeDef * DFETOFF_PORT, uint8_t DFETOFF_PIN);

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
