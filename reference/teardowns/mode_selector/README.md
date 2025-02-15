# faraday-rescue/reference/teardowns/mode_selector

## Teardown Notes

The mode selectoris a small PCB mounted on the handlebars of the bike which has two functions:
- Select whether the bike is in no boost/medium boost/high boost
- Display the battery SOC% on the e-paper screen

The mode selector has 4 pins:
- GND
- 12V
- RS485 A
- RS485 B

The ePaper display doesn't have a great lifetime and is subject to burn-in. Fortunately, it's pretty simple to replace with a bit of careful prying.

The mode selector is 'waterproofed' with VHB tape. Prying gently at it after removing the screws will expose the innards.
Replacing the VHB is hard without lasercutting the VHB into the right shape - I used DOW 738 sealant when I put it back together.

The plastic used on the selector is typically cracked by tension from the bolt. I used some acetone to solvent weld it back together.

## Key Parts
MCU: N51822-QFAA
Transciever: MAX3060E
ePaper Display: SCD722003


