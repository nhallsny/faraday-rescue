# faraday-rescue/reference/teardowns/mode_selector

## Known Issues
- Burn-in on the e-paper display
- Water ingress: stress cracking on the plastic housing allows moisture into the pc
- M5 Connector failure: connectors can go high resistance or have water ingress

## Bike Behaviors
- If the mode selector is disconnected or comms messages are not received, the bike will default to 'low' boost.

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
Replacing the VHB is hard without cutting the VHB into the right shape - I used DOW 738 sealant when I put it back together.

The plastic used on the selector is typically cracked by tension from the bolt. I used some acetone to solvent weld it back together.

## Key Parts
MCU: N51822-QFAA
Transciever: MAX3060E
ePaper Display: SCD722003


