
# faraday-rescue/software/bus_snooper RS485 Bus Overview

## Background

The Faraday bike implements the ASI Modbus protocol: https://support.accelerated-systems.com/wp-content/uploads/2022/06/ASI-Modbus-Protocol-Rev-1.2.pdf

## Serial Config

The Faraday uses an half duplex RS485 bus with a minimum signaling voltage of 3.3V. 

RS485 Bus Configuration
Baud: 115200 8/N/1 
Mixed endianness - data is Big, CRC is little

End of message is a RS232 BREAK, which pyserial can’t parse. Need to infer the end of the packet with timing or using a UART device which supports BREAK

## Message Format

Byte 0 is the slave ID. Byte 1 is the function code (0x03 for read, 0x10 for write). The slave responds with the same byte 0 and byte 1 within a few milliseconds of receiving the request.

### Read Register
![image](https://github.com/user-attachments/assets/da397096-a705-4498-82c2-4264efef524c)

### Write Register
![image](https://github.com/user-attachments/assets/79cb1a2b-fe6f-4624-85e3-76c3cc313b89)

### Nodes
There are two IDs on the network that respond to requests:
- ID 0x02 - BMS
- ID 0x03 - Mode Selector

Presumably 0x01 is the rear board controller. The ASI motor controller is on a separate bus and communicates directly with the rear board controller.

## Snooping the bus

I was able to parse RS485 messages by using a RS485 USB Dongle, writing all the bus traffic to a text file, and then parsing it after the fact with a script.

`python parse_raw_log.py example.log -v`

***Note: because the RS485 bus uses a BREAK character to signify an end-of-message, which is not supported by PySerial, I wasn't able to parse messages in real time. It's obviously possible with other serial libraries, that work just hasn't been completed.***

# Messages

## Battery Messages

### Cell Voltage Request
Occurs continuously when charging and periodically when biking. Basically a raw output of the LTC AFE used by the battery.

Example:
req 0x02, 0x03, 0x00, 0x02, 0x00, 0x0C, 0xE4, 0x3C

resp 0x2 0x3 0x18 0xa 0x6a 0xa 0x6a 0xa 0x6a 0xa 0x6a 0xa 0x6a 0xa 0x6a 0xa 0x6a 0xa 0x6a 0xa 0x6a 0xa 0x61 0xa 0x60 0xa 0x62 0x87 0xd6

Occurs: when charging, biking, or with battery debug tool
Request:
|SlaveID |FxnCode | Address |Data |Cksum|
|--------|--------|---------|------|-----|
|2|3|0x02|0x0C|Cksum|

Response:

|SlaveID |Register | Length  |Data |Cksum|
|--------|--------|---------|------|-----|
|2|3|24|12x UINT16 * 1.5mV|Cksum|

### Balancing Status Request

Used by debug tool to asking if any of the cells are balancing

Example:

req 0x02, 0x03, 0x00, 0x17, 0x00, 0x01, 0x34, 0x3d

resp TBD
Occurs: with battery debug tool

Request:
|SlaveID |FxnCode | Address |Data |Cksum|
|--------|--------|---------|------|-----|
|2|3|0x17|0x01|Cksum|

Response:

|SlaveID |Register | Length  |Data |Cksum|
|--------|--------|---------|------|-----|
|2|3|2|First 4 bytes are 0. Remaining 12 bits a bitfield. 1 if cell is balancing, 0 if not|Cksum|

### Battery Status 1

Likely a ‘are you there? message’ Occurs continuously while battery is off but rear controller is powered by charger. Response doesn’t seem to change based on mode.

Example:

req 0x2 0x3 0x0 0x0 0x0 0x1 0x84 0x39

resp 0x2 0x3 0x2 0x0 0x0 0xfc 0x44

Request:
|SlaveID |FxnCode | Address |Data |Cksum|
|--------|--------|---------|------|-----|
|0x02|0x03|0x00|0x01|Cksum|

Response:

|SlaveID |Register | Length  |Data |Cksum|
|--------|--------|---------|------|-----|
|0x02|0x03|1|0x00|Cksum|

### Battery Status 2

Purpose unclear. Generally comes after battery read 1. 
Maybe be a temperature or perhaps charge rate? Changes occasionally.

Example:

req 0x2 0x3 0x0 0x1 0x0 0x1 0xd5 0xf9

resp 1 0x2 0x3 0x2 0x8e 0xc4 0x98 0x77

resp 2 0x2 0x3 0x2 0x0 0x19 0x3d 0x8e

resp 3 0x2 0x3 0x2 0x0 0x1e 0x7c 0x4c

Request:
|SlaveID |FxnCode | Address |Data |Cksum|
|--------|--------|---------|------|-----|
|0x02|0x03|0x01|0x01|Cksum|

Response:

|SlaveID |Register | Length  |Data |Cksum|
|--------|--------|---------|------|-----|
|0x02|0x03|2|2 bytes|Cksum|





