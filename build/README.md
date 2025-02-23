# faraday-rescue/build Build Guide

## Intro

This is a build guide for assembling a faraday rescue battery.

## Required Tools
- General small hand tools - hex wrenches, x-acto knife, wire cutters/strippers
- 18650 Spot welder
- Soldering iron with chisel tip
- Solder
- Fine-pitch crimper (for Molex Picoblade)
- Large-pitch crimper (for TE 1.5mm crimps)
- Heat gun
- Isopropanol wipes
- Calipers
- Button Harness (see below)
- STLink to Green Julet Adapter Harness (see below)

## Bill of Materials
| Part Number             | Description                  | Quantity | Link                                                                  |
|-------------------------|------------------------------|----------|-----------------------------------------------------------------------|
| INR18650-35E            | Batteries                    | 24       | https://www.18650batterystore.com/products/samsung-35e                |
| weld_tab_large-v42       | Custom Nickel Tabs, 0.15mm   | 16       | DIY                                                                   |
| weld_tab_small-v42       | Custom Nickel Tabs, 0.15mm   | 16       | DIY                                                                   |
| module_end-v41           | Module end caps              | 16       | 3D Print 10% GF ASA                                                                   |
| end_bracket_wire_end-v31    | Module stack end cap, wire side | 1        | 3D Print 10% GF ASA                                                               |
| end_bracket_non_wire_end-v10| Module stack end cap, non-wire side  | 1        | 3D Print 10% GF ASA                                                         |
| power_connector_v11     | Battery to faraday power connector  | 1        | 3D Print 10% GF ASA                                                           |
| PCB, Long               | Custom PCBA                  | 1        | JLCPCB                                                                |
| PCB, Short              | Custom PCB                   | 1        | JLCPCB                                                                |
| tube_cap-v14            | Aluminum tubing end cap      | 2        | JLCPCB                                                                |
| Keyelco 5104            | Jumpers between PCBs         | 4        | Digikey                                                               |
| Wire 16ga               | Power wire, red/black        | 2        | Digikey                                                               |
|Molex / 0500798100       |26-28 Awg Picoblade Crimp     | 9        | Digikey |
|Molex / 0510210500       |ONN RCPT HSG 5POS 1.25MM      |1         | Digikey | 
|Molex / 0510210400       |CONN RCPT HSG 4POS 1.25MM     |1         | Digikey |
|TE 929990-1              | CONTACT SOCKET 13-17AWG CRIMP|2         | Digikey |
| 42mmx0.9mm thickness Ti | Tube, 568mm long             | 1        | https://www.ebay.com/itm/174293717819  |
| a18051400ux0423     | Thin Heatshrink PVC, 70mm flat|            | 1        | Amazon                                                                   |
| 1289N171                | End-cap o-ring               | 2        | https://www.mcmaster.com/1289N171/                                    |
| O-ring lube     | O-ring Lubricant, Silicone   | 1        | Local hardware store                                                  |
| 97163A152               | Insert, M4, 5mm long         | 2        | https://www.mcmaster.com/97163A152/                                   |
| Dow 738                 | RTV Sealant                  | 1        | https://www.mcmaster.com/1832A51/                                     |
| 91292A027               | M3 Bolts                     | 18       | https://www.mcmaster.com/91292A027/                                   |
| 93625A100               | M3 Locknuts                  | 18       | https://www.mcmaster.com/93625A100/                                   |
| Loctite 4861            | CA flexible adhesive         | 1        | https://www.mcmaster.com/74795A74/                                    |
| 5233T358                | Small O-rings                | 2        | https://www.mcmaster.com/5233T358/                                    |
| 90991A121               | End-cap bolts, non-wire side | 1        | https://www.mcmaster.com/90991A121/                                   |
| 90991A124               | End-cap bolts, wire side     | 1        | https://www.mcmaster.com/90991A124/                                   |
|                         |                              |          |                                                                       |

## Build Instructions
You’ll need 8x of these modules:
![image](https://github.com/user-attachments/assets/8e18ad41-89e3-40d0-87c4-394e4deba327)

Pre-assembly 16 end plates by gently placing the tabs and press-fitting the nylon locknut
![image](https://github.com/user-attachments/assets/5958920a-513e-4c46-99d9-5ee023f848cd)

Place the end-caps onto the batteries with the battery polarity as shown.
_NOTE - ALL CELLS MUST BE IN THE EXACT SAME ORIENTATION IN ALL MODULES_
![image](https://github.com/user-attachments/assets/fc23ecd9-fbf2-4f3d-b961-f0185950132b)

Weld the modules. I used 6 welds per tab. Welding prior to gluing prevents the glue from wicking between the tabs and cells
![image](https://github.com/user-attachments/assets/e6a1b575-5ebc-4aa9-aa4e-5672ce8817f3)

Applying gentle force to the top, add 1 drop of thin superglue to the space between the cells. Verify with calipers that the stack height is no greater than 67.8mm

![image](https://github.com/user-attachments/assets/481b018a-0583-440f-a34e-c601b86391a1)

Repeat 8 times.
![image](https://github.com/user-attachments/assets/64ccb126-f3ca-4a44-973b-4aefd9d00b83)

8 Modules welded, glued, and inspected
![image](https://github.com/user-attachments/assets/e27859ae-455a-46f6-9562-edbdb7056c31)

Solder the XT30 connector to the bottom of the board, making sure to match + and - to the labels on the board (VOUT)
![image](https://github.com/user-attachments/assets/47f4381a-34ea-4ebf-9f5a-b120252efda4)

On a flat surface, solder the 4 Keystone jumpers between the main and stub pcb
_Note - this is only necessary if you can't find a vendor who will make the full length board_
![image](https://github.com/user-attachments/assets/986eb874-c51b-4775-b8c5-2278a241ce30)

Wrap 1/4" Kapton tape in a loop around the jumpers. Make sure to cover the underside to avoid shorting during soldering.
![image](https://github.com/user-attachments/assets/00fb4756-9fa4-4f11-943e-89f4f3b08a72)

Add ~1mm thermal gap pad or thermal paste to thermistors on back-side of board. I conformal coated these sensors, probably unnecessarily.
![image](https://github.com/user-attachments/assets/0ef16602-1389-465b-9d1f-13577ac9d184)

Bolt together, align, and install the first two modules
![image](https://github.com/user-attachments/assets/31fc278d-5b44-481e-8b33-93733403ebcb)

Place, bolt, and solder the remaining modules. Adjust the modules prior to soldering to eliminate twist and non-concentric modules as much as possible.
![image](https://github.com/user-attachments/assets/9208fd61-4b22-4b58-af84-c6071a1bf9a0)

Heat-set the inserts into the non-wire-side end cap
![image](https://github.com/user-attachments/assets/8ebbc298-32c9-4bce-9325-bfd7bfb7dfc0)

Bolt the end-cap into place onto the side with the stub PCB (the negative-most side). The bolts should be flush or close to flush.
![image](https://github.com/user-attachments/assets/45434f1c-5989-4e1b-89e9-275e7366f911)

Test fit the end-cap to make sure it fits flush
![image](https://github.com/user-attachments/assets/589d33f1-d7c2-4181-a72a-2bf287fa1e1e)

Heat-set the M4 insert into the wire-side end cap and install 2 nuts.
_Note - having two nuts on this side technically violates the bolt/nut pattern, but trust me, it's way easier to assemble._
![image](https://github.com/user-attachments/assets/838f58b5-8dae-4d4c-bfc5-6c605501213c)


Drill 5 holes into the end cap based on the wire sizes. TODO image which shows all 5 holes
_Note - the wires must be lined up in order to avoid crosses. There isn't much space in the end_

![image](https://github.com/user-attachments/assets/5cc14871-ce2e-40fc-b46c-27e73f86d825)


Wipe the surface with isopropanol, let dry, and the adhere the the vent to the outside
![image](https://github.com/user-attachments/assets/5c3c6ef0-1f11-4706-8f06-0abf7ae63eaa)

Pre-crimp the four wires 
- Positive Power (crimp bike end with TE crimp)
- Negative Power (crimp bike end with TE crimp)
- RS-485 (Light Blue 4-pin Julet Connector, Male, with Picoblade crimps)
- SWD Programming Header (Green 5-Pin Julet Connector, Female, with picoblade crimps)

![image](https://github.com/user-attachments/assets/75d6984b-1a5e-4671-bc0b-7aac4ad66e07)
TODO - include crimped power wire and polarity diagram

Apply a bit of thick tape (I used fiberglass tape) to prevent the thin wires from getting cut by the battery tab
![image](https://github.com/user-attachments/assets/fa5544eb-1182-4457-9afe-faeebd5dc013)

Insert the power wires into the printed housing
![image](https://github.com/user-attachments/assets/bb1b3d3f-5da7-4b2e-b256-cf995cdc2658)

Pass the power wires through the holes drilled. Solder the XT30 to the backside
TODO - include image of XT30 soldered on backside

Pot the back of the housing with DOW 738. Use a toothpick to make sure that the coverage is 100%
![image](https://github.com/user-attachments/assets/5c6b1237-7f63-4169-bf57-5ccb5373413d)

Wait for the DOW 738 to dry

Flash the .hex file onto the STM32 and test your new battery! If you have flashed with LEDS = 1, then the CHG and DSG lights should light up. If not, try shorting BUTTON and GND. TODO image
_Note - the LEDs consume significant power and cause heating of the linear reg. I suggest flashing with LEDS = 0 after debugging_
_Note 2 - I made a 'faraday on/off' harness with RS485 and the on/off button broken out. See 'tools' section below_
![image](https://github.com/user-attachments/assets/9f96c159-753c-4f99-91f4-1391e0d92ff2)


When satisfied, heat-shrink the battery. TODO image

Apply a light coat of o-ring lubricant on the o-rings and install on the aluminum end caps.  TODO image

Install the wire-side cap (if you haven't already). Use the small 4mm ID o-ring to achieve sealing with the M4 bolt. TODO image

Slide the tube over the entire battery. Some gentle pressure and a light coating of grease may be required. TODO image
_Note - if too much force is used, the heatshrink may bunch and rip, possibly shorting the battery. Go gently.

Bolt on the remaining end-cap with o-ring and lubriant. Make sure that the o-ring is engaged with the tube. TODO image

Final test procedure
| Test                    |Expected Result               | Tool |
|-------------------------|------------------------------|----------|
| Charge                  | Cut-off around 4.2V                   | 2A, 51V power supply or Faraday Charger with DIY harness        | 
| Discharge               | Cut-off around 2.7V                | 1-5A discharge through power resistor or programmable load        | 
| Verify balance          | Almost no imbalance (<0.020V)         | Faraday BMS Tester        |
| Check temperatures      | 4x rational temperatures              | STMStudio Direct Memory Address Reader (see software documentation      | 

# Tools

## ST-Link to Green Julet Adapter Harness

Use this adapter to program the processor or read parameters with STMStudio

| Part Number             | Description                  | Quantity | Link                                                                  |
|-------------------------|------------------------------|----------|-----------------------------------------------------------------------|
| ST-Link V2                 | PRogrammer                   | 1        | Digikey                |
| 0901420020                 | Molex 20 Pin header adapter                 | 1        | Digikey                |
| 0016020088                | Molex C-Grid Crimps                 | 1        | Digikey                |
| Julet 5-pin Green connector, Male    | Crimps                 | 1        | Digikey                |

Note that you will need to double crimp the 3.3V wire to provide a reference voltage for the STLink like so:

![image](https://github.com/user-attachments/assets/1f5cfe38-e13c-4bf2-830e-5fe9113f55d4)


## Button Harness

Use this to turn the battery on and off and connect it to RS485
| Part Number             | Description                  | Quantity | Link                                                                  |
|-------------------------|------------------------------|----------|-----------------------------------------------------------------------|
| Waveshare 17286            | RS485 to USB Converter         | 1        | Amazon             |
| No Part Number            | Julet 4-pin blue connector, female              | 1        | Amazon                |
| Any momentary switch            | Crimps                 | 1        | Amazon               |

Note - I do not know if the cable colors are standardized. Check your pinouts! TODO add diagram of pin numbers
PCB Pin 1 - GND----------Red------GND----BUTTON
PCB Pin 2 - BUTTON ------Green----BUTTON
PCB Pin 3 - RS485 A -----Black----RS485 A
PCB Pin 4 - RS485 B -----White----RS485 B

![image](https://github.com/user-attachments/assets/63b6034c-4d9e-4d6d-854b-5a069ed0edf0)

## Sizer Ring
I suggest printing a 40mm sizer ring to pass over the battery prior to heat-shrinking. This verifies that it will go in the tube smoothly.
![image](https://github.com/user-attachments/assets/95699e0d-cdbf-4c79-bad2-37c8c50f5484)





















