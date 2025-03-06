# bms software

Contains the firmware for the battery management system as a STM32CubeIDE project using the HAL.

The majority of the software reference material is located in the design doc](https://github.com/nhallsny/faraday-rescue/blob/main/reference/design/README.md)

## Binaries

[STM32L010K8 .bin File](https://github.com/nhallsny/faraday-rescue/blob/main/software/bms/Release/faraday_stm32_bms.bin)

The .bin file _faraday_stm32_bms_ is compiled for the STM32L010K8 processor. The default build is:
- DEBUG off (not for production, turning this on enables printf and spews lots of debug info over RS485 at 115220 baud)
- LEDs off (not for production, LEDs consume relatively large amounts of power)
- Watchdog On
- Reset 3v3 (not for production, used if the STM32 needs to be hard reset to clear registers related to sleep/wake, such as can happen after debugging).

If you need to change the settings, it's best to get the project running so you can use live debugging.

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
