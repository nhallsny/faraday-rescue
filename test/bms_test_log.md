
# for branch BQ-refactor

Listing out the functions of the BMS and when/what was tested

| Function  |  What?  | When tested|
|---|---|---|
| Overvoltage |  Prevent cells from overcharge | verified 3/24/2025 |
| Undervoltage |  Prevent cells from undercharge | verified 3/25/2025 |
| Charging Overtemp Cutoff| prevent cells from charging beyond their rated temperature  | Verified during soldering of PCB 3/23/2025|
| Charging Undertemp Cutoff | prevent cells from charging below their rated temperature  | Not yet verified |
| Discharging Overtemp Cutoff| prevent cells from discharging above their rated temperature  | Verified during soldering of PCB 3/23/2025|
| Discharging Undertemp Cutoff | prevent cells from discharging below their rated temperature | Not yet verified |
| Deep Discharge Lockout |prevent deeply discharge and possibly damaged cells from being charged  | Not yet verified|
| Overcurrent | turn off immediately when a short circuit occurs  | Not yet verified |
| Overcurrent in Charge | turn off immediately when excessive charge current is applied | Verified 3/24/2025 |
| Overcurrent Backup  | 25A, 75V Fuse  | prevent shorting batteries if software and BQ chip fail somehow  | Not yet verified |
| Balancing in CHARGE and RELAX | Keep cells with a few mV of each other| Verfified via DEBUG command 3/24/2025 |
| DEEPSLEEP sleep state | Minimize power consumption while off. BQ goes into DEEPSLEEP, STM32 goes into STOP modes| Not yet verified |
| SHUTDOWN sleep state | Dramatically reduce power consumption between 0% SOC and bricking voltage to extend life| Challenging to verify|
| Wake on insertion of charge plug | Mirror original faraday behavior| Verified 3/24/2025 |
| Wake on momentary pull-down of button | Mirror original faraday behavior | Verified 3/24/2025 |
| Watchdog | reset STM32 if code crashes| Not yet verified |
| Battery OK message | Responds to messages from rear controller to enable high torque mode and update screen| Not yet verified |
| Battery voltage message | Responds to messages from rear controller to enable high torque mode and update screen| Verified 3/24/2025 |
| Battery balancing message |  Responds to messages from PC debug tool| Not yet verified |
| Debug Compilation Flag | Dump lots and lots of info over serial to facilitiate debugging| Verified 3/23/2025|
| CHG and DSG LEDs | Indicate status of FETs| Verified 3/24/2025|
