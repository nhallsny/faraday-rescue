# bms software

Contains the firmware for the battery management system as a STM32CubeIDE project using the HAL.

The majority of the software reference material is located in the [design doc](https://github.com/nhallsny/faraday-rescue/blob/main/reference/design/README.md)

## Binaries

[STM32L010K8 .bin File](https://github.com/nhallsny/faraday-rescue/blob/main/software/bms/Release/faraday_stm32_bms.bin)

The .bin file _faraday_stm32_bms_ is compiled for the STM32L010K8 processor. The default build is:
- DEBUG off (not for production, turning this on enables printf and spews lots of debug info over RS485 at 115220 baud)
- LEDs off (not for production, LEDs consume relatively large amounts of power)
- Watchdog On
- Reset 3v3 (not for production, used if the STM32 needs to be hard reset to clear registers related to sleep/wake, such as can happen after debugging).

If you need to change the settings, it's best to get the project running so you can use live debugging.

## Functionality

| Function  |  Responsible | Why?  | 
|---|---|---|
| Overvoltage |  BQ76972 configured by STM32 |  Prevent cells from overcharge |
| Undervoltage  | BQ76972 configured by STM32  |  Prevent cells from undercharge |
| Charging Overtemp Lockout  | BQ76972 configured by STM32  |  prevent cells from charging beyond their rated temperature  |
| Charging Undertemp Lockout  | BQ76972 configured by STM32  | prevent cells from charging below their rated tempertaure  |
| Discharging Overtemp Lockout  | BQ76972 configured by STM32  | prevent cells from discharging above their rated temperature  |
| Discharging Undertemp Lockout  | BQ76972 configured by STM32  |  prevent cells from discharging below their rated temperature |
| Deep Discharge Lockout  | BQ76972 configured by STM32  | prevent deeply discharge and possibly damaged cells from being charged  |
| Overcurrent  | BQ76972 configured by STM32  | turn off immediately when a short circuit occurs  |
| Overcurrent Backup  | 25A, 75V Fuse  | prevent shorting batteries if software and BQ chip fail somehow  |
| Balancing in CHARGE and RELAX | BQ76972 configured by STM32 | Keep cells with a few mV of each other|
| DEEPSLEEP sleep state | BQ76972 and STM32 | Minimize power consumption while off. BQ goes into DEEPSLEEP, STM32 goes into STOP modes|
| SHUTDOWN sleep state | BQ76972 | Dramatically reduce power consumption between 0% SOC and bricking voltage to extend life|
| Wake on insertion of charge plug | BQ76972 and STM32 | Mirror original faraday behavior|
| Wake on momentary pull-down of button | BQ76972 and STM32 | Mirror original faraday behavior |
| Watchdog | STM32 | reset STM32 if code crashes|
| Battery OK message | STM32 | Responds to messages from rear controller to enable high torque mode and update screen|
| Battery voltage message | STM32 | Responds to messages from rear controller to enable high torque mode and update screen|
| Battery balancing message | STM32 | Responds to messages from PC debug tool|
| Debug Compilation Flag | STM32 | Dump lots and lots of info over serial to facilitiate debugging|
| CHG and DSG LEDs | STM32 | Indicate status of FETs|

## Debug with Faraday BMS Tester
[Download Faraday BMS Tester for Windows]([url](https://github.com/nhallsny/faraday-rescue/tree/main/reference/official/Battery%20Utility))
This simple utility uses an RS485-USB adapter hard-coded to COM3 (yes, you have to go deep in device manager and force COM3) to query battery voltages.

Checkboxes force balancing on the original design. The code does not implement this at the moment.

![image](https://github.com/user-attachments/assets/581ab021-6684-424f-a63a-b6ad42b899e2)

## Debug with STMStudio

[STMStudio](https://www.st.com/en/development-tools/stm-studio-stm32.html) is a great way to read and plot memory addresses. Connect the programming 5p cable to the battery, load these memory addresses in, turn on the battery, and graph away!

| Value  |  Address | Units  | 
|---|---|---|
|  Current |  0x200003fe | int16 mA  |
| Temp 1  | 0x200003e8  | float  |
| Temp 2  | 0x200003ec  | float  |
| Temp 3  |  0x200003f0 | float  |
| Temp 4 - BQ Chip  | 0x200003f4  | float  |
| Min Cell | 0x200003e4  | uint16 mV  |
| Max Cell | 0x200003e6 | uint16 mV  |

![image](https://github.com/user-attachments/assets/154a1445-454e-4585-8d08-c96fb81087bc)
