# Realtime Data Logging

Use STMSTudio and the programming header.

List of memory locations for the batt state struct:

| Name | Type | Address |
| -----| ----| ------|
| CellVMin | int16_t mV| 0x20000432|
| CellVMax | int16_t mV| 0x20000434|
| Current | int16_t  mA| 0x20000450|
| Temp1 Cell | float  C| 0x20000438|
| Temp2 Cell | float  C| 0x2000043c|
| Temp3 Cell | float  C| 0x20000440|
| Temp4 BQ | float  C| 0x20000444|
| Safety Status A Register | uint8_t | 0x20000454 |
| Safety Status B Register | uint8_t | 0x20000455 |
| Safety Status C Register | uint8_t | 0x20000456 |
| Permanent Fail A Register | uint8_t | 0x20000457 |
| Permanent Fail B Register | uint8_t | 0x20000458 |
| Permanent Fail C Register | uint8_t | 0x20000459 |
