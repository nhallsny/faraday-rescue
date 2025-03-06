# bms software

Contains the firmware for the battery management system as a STM32CubeIDE project using the HAL.

The majority of the software reference material is located:
[https://github.com/nhallsny/faraday-rescue/blob/main/reference/design/README.md](url)

## Binaries

The .hex file _faraday_stm32_bms_ contains the STM32L0K8 processor code for a STM32L010K8 processor. The default build is:
- LEDs off (not for production, LEDs consume relatively large amounts of power)
- Watchdog On
- Reset STM32 Off (not for production, used if the STM32 needs to be hard reset to clear registers related to sleep/wake, such as can happen after debugging).

If you need to change the settings, it's best to get the project running so you can use live debugging.

## Debug with STMStudio

STM32Studio is a great way to read and plot memory addresses. 

| Value  |  Address | Units  | 
|---|---|---|
|  Current |  0x200003fe | int16 mA  |
| Temp 1  | 0x200003e8  | float  |
| Temp 2  | 0x200003ec  | float  |
| Temp 3  |  0x200003f0 | float  |
| Temp 4 - BQ Chip  | 0x200003f4  | float  |
| Min Cell | 0x200003e4  | uint16 mV  |
| Max Cell | 0x200003e6 | uint16 mV  |
