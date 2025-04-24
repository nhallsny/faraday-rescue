# faraday-rescue/build/pcb/faraday-rescue-bms

PCB attached to battery modules that measures voltages, protects battery, manages sleep/wake, communicates with Faraday bike over RS485

## Revision table
### Release 2.1
- Change RS485 TVS diodes to bidirectional

### Release 2.0	
  - Fixed incorrect diode footprints
  - Fixed C15 Silk
  - Added VSS Test Point
  - Adjust I2C Pullups to 1.5kOhms
  - Add Cell0-VSS GND tie
  - Add more bulk capacitance near STM32 and RS485 Chip
  - Increase capacitance on BUTTON for more debouncing
  - Change shunt for a smaller one
  - Add thermal reliefs for SMT components on ground planes
  - Remove CHG and DSG test points
  - Add easy test points for sleep current measurement
  - Change thermistor capacitance to <470pF
  - Change to larger 20A slow-blow fuse for higher voltage rating
  - Add picoblade connector for RS485/button connector
  - Changing programming header to picoblade

Considered but not implemented:
  - Add bluetooth module for STM32
  - Switch Linear regulator to switching regulator
### 1.0
Initial Release
		

## Using these files
These files were designed in KiCad 8.0.4. There are two PCBs to fit in JLCPCB's design rules:
- Long, thin pcb which contains the BQ chip, STM32, and connectors
- Small stub board to complete the 8th module.

### Manufacturing parameters
If sending out this PCB, I suggest the following for the main PCB:
- 1mm board thickness (to fit inside tube)
- 2oz copper thickness on all layers (to reduce resistive heating. May not be necessary at 9A max current)
- Higher Tg material to avoid degredation of insulation during operation

For the smaller stub PCB:
- 1mm board thickness
- 1oz copper
- Higher Tg material to avoid degradation of insulation during operation

Ordering parameters from JLCPCB:
![image](https://github.com/user-attachments/assets/d0957980-3816-4d8a-9568-1d07f6bb66f6)



