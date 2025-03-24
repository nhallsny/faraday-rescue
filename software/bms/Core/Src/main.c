/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
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
#include "stdio.h"
#include "utils.h"
#include "BQ769x2.h"

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
#define WATCHDOG 1//set to 1 to enable watchdog. Good for production, bad for debugging
#define RESET_3V3 0 //set to 1 reset 3v3 rail. Useful if STM32 doesn't sleep after debugging with debugger.

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
#define RST_SHUT_PORT GPIOA
#define RST_SHUT_PIN GPIO_PIN_1

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

/*BQ Chip Parameters*/
#define ACTIVE_CELLS 0xAAFF //bitfield which indicates which cells to have protections on. Specific to this Faraday 12S pack
#define BQ_DEV_ADDR  0x10 //i2c address
#define BQ_CRC_MODE 1  // 0 for disabled, 1 for enabled

/* User-Facing Timing Parameters */
#define RETRY_LIMIT 100 //number of times the STM will retry communications with BQ before calling for a hard reset
#define INACTIVITY_LOOPS_MAX 11000 //number of loops without significant battery current before bike goes into sleep. Normally around 5 minutes for 10,000 loops
#define BUTTON_LONG_PRESS_LOOPS 15 //number of loops that count as a long press
#define UART_TIMEOUT_S 3 //default timeout for RS485 in seconds
#define PACK_CURRENT_INACTIVITY_LOWER_LIMIT_MA -250
#define PACK_CURRENT_INACTIVITY_UPPER_LIMIT_MA 75

/*Uart Variables flag*/
__IO ITStatus UartBusy = RESET;

/* USER CODE BEGIN PV */

//Used for state machine
typedef struct {
	uint8_t ButtonCount;
	uint16_t CRC_Fail;
	uint16_t InactivityCount;
	uint16_t RetryCount;
	uint8_t ResetByWatchdog;
	uint8_t ButtonPressedDuringBoot;
} STM32State;

//Global Variables
STM32State state;
BQState batt;
uint8_t UART_RxData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t UART_TxData[32] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
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
/* debug function to print lots of status bits */
void BQ769x2_PrintStatus(BQState *s) {
	s->AlarmBits = BQ769x2_ReadRawAlarmStatus(&batt);
	BQ769x2_ReadPFStatus(&batt); //TODO remove later
	BQ769x2_CalcMinMaxCellV(&batt);
	printf(
			"CRC: %d Bal: %d Dsg: %d Chg: %d ScanComplete: %d Prots: %d SSA: %d SSBC: %d PFA: %d PFB: %d PFC: %d UV: %d OV: %d SCD: %d OCD: %d SSA: %#x, SSB: %#x, SSC: %#x",
			batt.CRC_Fail,
			//batt.AlarmBits & 0x00 ? 1 : 0, //WAKE
			//batt.AlarmBits & 0x02 ? 1 : 0, //ADSCAN
			batt.AlarmBits & 0x04 ? 1 : 0, //CB
			//batt.AlarmBits & 0x10 ? 1 : 0, //SHUTV
			batt.AlarmBits & 0x20 ? 0 : 1, //XDSG
			batt.AlarmBits & 0x40 ? 0 : 1, //XCHG
			batt.AlarmBits & 0x80 ? 1 : 0, //FULLSCAN
			//batt.AlarmBits & 0x200 ? 1 : 0, //INITCOMP
			//batt.AlarmBits & 0x400 ? 1 : 0, //INITSTART
			batt.ProtectionsTriggered, batt.AlarmBits & 0x4000 ? 1 : 0, //Safety Status A
			batt.AlarmBits & 0x8000 ? 1 : 0, //Safety Status B or C
			batt.value_PFStatusA, batt.value_PFStatusB, batt.value_PFStatusC,
			batt.UV_Fault, batt.OV_Fault, batt.SCD_Fault, batt.OCD_Fault,
			batt.value_SafetyStatusA, batt.value_SafetyStatusB,
			batt.value_SafetyStatusC);

	printf(
			"PackV: %d StackV:%d LdV:%d I: %d MIN_V: %d MAX_V: %d InactivityCount: %d TS1: %.3f TS3: %.3f T_HDQ: %.3f T_INT: %.3f \r\n",
			batt.Pack_Voltage, batt.Stack_Voltage, batt.LD_Voltage,
			batt.Pack_Current, batt.CellMinV, batt.CellMaxV,
			state.InactivityCount, batt.Temperature[0], batt.Temperature[1],
			batt.Temperature[2], batt.Temperature[3]);

}

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

/* interrupt-safe function to clear RxData and wait for a new message */
void UART_WaitForCommand() {

	if (!UartBusy) {
		//set RX buffer to all zeros
		for (int i = 0; i < 8; i++) {
			UART_RxData[i] = 0;
		}

		//set transciever to receive
		THVD2410_Receive();

		//start recieve-to-idle interrupt
		huart2.RxState = HAL_UART_STATE_READY;
		if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, UART_RxData, 8) != HAL_OK) {
			Error_Handler();
		}
	}

}

/*
 * Prep cell balancing message (30 bytes). Refer to documentation for packet format
 */
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
		if (batt.ActiveCells & (1 << i)) {
			UART_TxData[2 * k + 3] = (batt.CellVoltage[i] * 2 / 3) >> 8;
			UART_TxData[2 * k + 4] = (batt.CellVoltage[i] * 2 / 3) & 0xFF;

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

/*
 * Prep cell balancing message (8 bytes). Refer to documentation for packet format
 */
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
		if (batt.ActiveCells & (1 << i)) {
			//i is the cell number active cell bitfield (0xAAFF)
			//k is the cell number in faraday index
			if (batt.CB_ActiveCells & (1 << i)) {
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

	//get highest cell temp

	for (int i = 0; i < 32; i++) {
		UART_TxData[i] = 0;
	}

	UART_TxData[0] = 0x02;
	UART_TxData[1] = 0x03;
	UART_TxData[2] = 0x02;
	UART_TxData[3] = 0x00;
	// UART_TxData[4] = 0x19; old hard coded value
	UART_TxData[4] = batt.CellMaxT;

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
			delayUS(&htim2, 450); //insert 1ms delay to match timing of original battery
			if (FunctionCode == 0x03) { //it's a read
				if (LEDS) {
					HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
					delayUS(&htim2, 50);
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
		state.ButtonCount++;

	} else if (state.ButtonCount) {
		state.ButtonCount -= 1;
	};

	if (state.ButtonCount > BUTTON_LONG_PRESS_LOOPS) {
		if (DEBUG) {
			printf("\r\nbutton long press...time to get ready for bed\r\n");
		}
		state.ButtonCount = 0;
		Sleep();
	};
}

uint8_t STM32_Wake_Button_Pressed() {
	return !(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN));
}

/* If current below 250mA for more than XXX loops, activate sleep. Must have a fresh value in Pack_Current */
void STM32_HandleInactivity() {

	//Increment sleep timer if current is between +20mA and -250mA. Increment much faster if we're done with charge to get the correct Faraday LED behavior.
	if (batt.Pack_Current >= PACK_CURRENT_INACTIVITY_LOWER_LIMIT_MA
			&& batt.Pack_Current <= PACK_CURRENT_INACTIVITY_UPPER_LIMIT_MA) {
		if (!batt.Chg && batt.Dsg) {
			state.InactivityCount += 300; //if CHG fet is off but DSG is still on, we're finished with charge and should sleep in around a second.
		} else {
			state.InactivityCount++;
		}
	} else {
		state.InactivityCount = state.InactivityCount / 2; //exponential decay if current detected
	}

	if (!DEBUG) {
		if (state.InactivityCount > INACTIVITY_LOOPS_MAX) {
			Sleep();
		}

		//loop is much slower with prinfs enabled, so make this more reasonable
	} else {
		if (state.InactivityCount > 500) {
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
		delayMS(&htim2, 100);
		HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_RESET);
	}

	//turn off LEDs
	HAL_GPIO_WritePin(LED_CHG_PORT, LED_CHG_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_DSG_PORT, LED_DSG_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BF_PORT, LED_BF_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_RESET);

	//Turn off FETs
	BQ769x2_ForceDisableFETs(&batt);

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
		state.ResetByWatchdog = 1;
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
	BQ769x2_ForceDisableFETs(&batt);

	//wait 200ms for the bus voltage to decay
	delayMS(&htim2, 200);

	//Put the BQ to sleep
	while (!BQ769x2_EnterDeepSleep(&batt)) {
		delayUS(&htim2, 5000);
	}

	BQ769x2_ClearFullScan(&batt);

	if (DEBUG) {
		printf(
				"\r\nbq put into DEEP SLEEP, STM about to reset to disable watchdog...\r\n");
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

	HAL_GPIO_WritePin(LED_CHG_PORT, LED_CHG_PIN, batt.Chg);
	HAL_GPIO_WritePin(LED_DSG_PORT, LED_DSG_PIN, batt.Dsg);
}

/**
 * @brief  This blinks forever, mainly useful for debugging. Period in us
 * @retval None
 */
void STM32_BlinkForever(uint16_t period_ms) {
	HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
	delayMS(&htim2, period_ms);
	HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_RESET);
	delayMS(&htim2, period_ms);
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
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	SystemPower_Config();
	MX_RTC_Init();
	HAL_TIM_Base_Start(&htim2);
	/* USER CODE END SysInit */

	if (DEBUG) {
		printf("stm32 init complete\r\n");
	}

	/*Check to see if the button is currently pressed*/
	if (STM32_Wake_Button_Pressed()) {
		state.ButtonPressedDuringBoot = 1;
	}

	/* Check to see if the STOP flag is set from prior reset. If so, initiate STOP*/
	if (STM32_ShouldStop()) {
		if (DEBUG) {
			printf("stm32 reset with intent to sleep, time to sleep...zzz\r\n");
		}
		STM32_Stop();
	}

	/* Initialize BQ State*/
	BQ769x2_ResetShutdownPin(&batt); // RST_SHUT pin set low just in case
	BQ769x2_InitState(&batt, &hi2c1, BQ_DEV_ADDR, BQ_CRC_MODE, &htim2,
	ACTIVE_CELLS, RST_SHUT_PORT, RST_SHUT_PIN, CFETOFF_PORT, CFETOFF_PIN,
	DFETOFF_PORT, DFETOFF_PIN);

	delayMS(&htim2, 50); //Wait for everything to stabilize

	/* Init watchdog */
	if (WATCHDOG) {
		MX_IWDG_Init();
	}
	//BlinkForever(1); //uncomment to test IWDG

	/* Useful functions for debugging, especially if the STM32 gets into a weird state where it won't sleep */
	if (RESET_3V3) {
		HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
		BQ769x2_Reset(&batt); //Use this for several reasons - the main reason is to kill the 3.3V rail and power cycle the STM32, which may be necessary if the programmer puts it into a state where it won't sleep properly
	}

	UART_WaitForCommand(); //Start UART Receiving

	while (1) {

		/* USER CODE BEGIN 3 */
		BQ769x2_ForceDisableFETs(&batt); //disable FETs until we are getting communication from the BQ chip
		BQ769x2_SoftWake(&batt); //wiggle RST_SHUT to do a partial reset of the BQ chip. Not sure if this is necessary but it doesn't seem to hurt.

		if (DEBUG) {
			if (BQ769x2_ReadControlStatus(&batt) & 0x04) {
				printf("bq woke from DEEPSLEEP\r\n");
			}
		}

		//Check for BQ state FULLACCESS, SEALED, or UNSEALED. Device must be connected and ACKing to get past this point.
		// This may take quite a few reads for the chip to wake up if it's the first time it's booting (I've seen 15 reads!)
		while (!BQ769x2_Ready(&batt)) {
			if (DEBUG) {
				printf("bq not ready\r\n");
			}
			delayUS(&htim2, 1000);
			if (state.RetryCount > RETRY_LIMIT) {
				HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
				BQ769x2_Reset(&batt); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			state.RetryCount++;
		};

		state.RetryCount = 0;

		//Wake up the device if it isn't already awake
		while (!BQ769x2_Wake(&batt)) {
			if (DEBUG) {
				printf("bq not awake\r\n");
			}
			delayUS(&htim2, 1000);
			if (state.RetryCount > RETRY_LIMIT) {
				HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
				BQ769x2_Reset(&batt); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			state.RetryCount++;
		}

		state.RetryCount = 0;

		//Initialize registers by calling BQ769x2_Init and then checking that the configuration was successful
		// BQ769x2 does a spot check of a register that should have been configured if BQ769x2_Init() was successful.
		while (!BQ769x2_Initialize(&batt)) {
			if (DEBUG) {
				printf("bq not configured\r\n");
			}
			if (state.RetryCount > RETRY_LIMIT) {
				HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
				BQ769x2_Reset(&batt); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			state.RetryCount++;
		}

		state.RetryCount = 0;

		// get first ADC reading. Normal for this to take a few retries
		while (!BQ769x2_ReadBatteryData(&batt)) {
			if (state.RetryCount > RETRY_LIMIT) {
				HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
				BQ769x2_Reset(&batt); //gotta reset the BQ and try again. This kills the 3V3 rail
			}
			state.RetryCount++;
			delayUS(&htim2, 5000); //wait a bit for the ADC to finish measuring
		}

		state.RetryCount = 0;

		// check whether a charger >15V is connected and the button wasn't pressed during boot. If not, go back to sleep.
		if (!state.ButtonPressedDuringBoot && batt.Pack_Voltage < 15000) {
			if (DEBUG) {
				printf(
						"stm32 going back to sleep because user didn't press button and charger isn't connected...zzz\r\n");
			}
			Sleep();
		}

		//why are we starting? the only reasons are button press or charger detected
		if (DEBUG) {
			if (state.ButtonPressedDuringBoot) {
				printf("\r\nbutton pressed, time to attempt to allow FETs\r\n");
			}
			if (batt.Pack_Voltage > 15000) {
				printf("\r\ncharger connected, time to allow FETs\r\n");
			}
		}

		//Enable FETs because the BQ chip is up and running
		BQ769x2_AllowFETs(&batt);

		//Start UART interrupt
		UART_WaitForCommand();

		while (1) {

			//detect button press and take action if needed
			STM32_HandleButton();

			//Update FET registers and update LEDs
			BQ769x2_ReadFETStatus(&batt);
			if (LEDS) {
				STM32_UpdateFETLEDs();
			}

			//Handle sleep current and put battery to sleep if not much is going on
			STM32_HandleInactivity();

			//Useful for logging
			BQ769x2_CalcMinMaxCellV(&batt);

			//Needed for battery status message 2
			BQ769x2_CalcMinMaxCellT(&batt);

			//Print battery status over RS485 for debug
			BQ769x2_ReadBatteryStatus(&batt);
			if (DEBUG) {
				BQ769x2_PrintStatus(&batt);
			}

			//Get the latest data from the BQ chip
			if (BQ769x2_ReadBatteryData(&batt)) {
				state.RetryCount = 0;
			} else {
				state.RetryCount++;
			}

			//Check for faults and trigger the LED if so
			if (BQ769x2_ReadSafetyStatus(&batt)) {
				state.RetryCount = 0;
			} else {
				state.RetryCount++;
			};

			//Set Fault LED
			if (LEDS && (batt.ProtectionsTriggered & 1)) {
				HAL_GPIO_WritePin(LED_BF_PORT, LED_BF_PIN, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(LED_BF_PORT, LED_BF_PIN, GPIO_PIN_RESET);
			}

			//If there are too many failures, reset the BQ chip
			if (state.RetryCount > RETRY_LIMIT) {
				HAL_GPIO_WritePin(LED_SF_PORT, LED_SF_PIN, GPIO_PIN_SET);
				if (!BQ769x2_Reset(&batt)) {
					Error_Handler(); //If we end up here, something is truly messed up
					//gotta reset the BQ and try again. This kills the 3V3 rail
				}
				break;
			}

			STM32_PetWatchdog();
			delayMS(&htim2, 10);  // repeat loop every 20 ms

		}
	}
	/* USER CODE END 3 */
}

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_GPIO_WritePin(UART_RX_EN_PORT, UART_RX_EN_PIN, GPIO_PIN_SET); // Receive Off
	delayUS(&htim2, 20);
	HAL_GPIO_WritePin(UART_TX_EN_PORT, UART_TX_EN_PIN, GPIO_PIN_SET); // Transmit On
	delayUS(&htim2, 20);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	HAL_GPIO_WritePin(UART_TX_EN_PORT, UART_TX_EN_PIN, GPIO_PIN_RESET); //Transmit off
	delayUS(&htim2, 20);
	HAL_GPIO_WritePin(UART_RX_EN_PORT, UART_RX_EN_PIN, GPIO_PIN_SET); // Receive still off
	delayUS(&htim2, 20);

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
	delayMS(&htim2, 1000); //wait a second
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
