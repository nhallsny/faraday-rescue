# bms software

Contains the firmware for the battery management system as a STM32CubeIDE project using the HAL.

## Binaries

The .hex file _faraday_stm32_bms_ contains the STM32L0K8 processor code for a STM32L010K8 processor. The default build is:
- LEDs off
- Watchdog On

If you need to change the settings, it's best to get the project running so you can use live debuggig

## Debug with STMStudio

STM32Studio is a great way to read and plot memory addresses. 

| Value  |  Address | Units  | 
|---|---|---|
|  Current |   | int16 mA  |
| Temp 1  |   | float  |
| Temp 2  |   | float  |
| Temp 3  |   | float  |
| Temp 4 - BQ Chip  |   | float  |
| Min Cell |   | uint16 mV  |
| Max Cell |   | uint16 mV  |
