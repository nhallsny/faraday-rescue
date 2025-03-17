#include "BQ769x2Header.h"

typedef struct {
	void* bq_i2c;
	uint8_t RST_SHUT_PORT;
	uint8_t RST_SHUT_PIN;
	uint8_t CFETOFF_PORT;
	uint8_t CFETOFF_PIN;
	uint8_t DFETOFF_PORT;
	uint8_t DFETOFF_PIN;
	int16_t CellVoltage[16];
	int16_t CellMinV;
	int16_t CellMaxV;
	float Temperature[4];
	int8_t Offset[4];
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
} BQState;

extern BQState bq;


