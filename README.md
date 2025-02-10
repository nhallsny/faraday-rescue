# faraday-rescue
Open source battery for e-bikes made by the now-defunct Faraday Bicycles

This respository contains designs for an e-bike battery which replaces the original battery in Faraday e-bikes (utilizing exactly nothing from the original battery design!). This design fixes the major flaw in the original design, a high quiescent current which would quickly brick batteries if left un-tended.

_Under construction, check back soon!_

## Overview
![overview](https://github.com/user-attachments/assets/03d99ff3-242b-4089-a17a-9e69a970b3f1)

Like the original design, this battery is a 12S 2P 18650 based design. It utilize a PCB the length of the battery for both 'high current' (10A) bussing and for the major BMS controls. Given that the original batteries were designed in 2013, batteries made with modern (2025 as of writing this) have roughly 20% more energy, boosting the range of the bike. However, that's not why this project exists...the project exists because of the many sad Faraday bikes and their unhappy owners. 

Why are their owners sad? The original battery design has a quiescent draw of around 10mW even when 'off'. While it might not sound like much, that tiny amount of power will drain batteries from 0% state of charge to "bricked" i.e. copper dendrite formation in 2-3 weeks. Left unplugged over the winter, or even a long vacation, the batteries would be irreperably harmed.

How does this battery fix the sleep problem? Modern technology! When the bike was designed, there were no quiescent-optimized battery monitoring chips on the market that could measure 12 cells. The original designer chose an LTC6802-G...a good choice at the time, but designed for electric vehicle batteries with oodles of amp-hours. It doesn't help that the microcontroller on the BMS never sleeps. By using a low-power processor, sleep states, and a TI BQ 16-cell BMS chip, I was able to achieve:
+ Original design, off: 300uA
+ This design, off: 3uA

This means that when off, the improved battery can be left at 0% state-of-charge for roughly 5 years before it's a useless hunk of lithium, copper, and cobalt!

### Hardware
+ 12S 2P divided into 8 'modules'
+ Titanium or Fiberglass tube (you choose!)
+ TI BQ76972 fully integrated battery monitoring chip
+ STM32L0 Low-power microprocessor

+ RS485 driver

_Detailed hardware build files are in the `./Docs` folder_

### Software
+ BQ chip implements overvoltage, undervoltage, balancing, temp measurement, high-side FET control, all the super important stuff
+ STM32 manages communications (required!) with the rest of the bike over RS485 and sleep/wake states
+ Currently implemented in C using the STM32 HAL

_Software, debug tools, etc are in the `./Software`. folder_

Detailed updates or build instructions are better suited for the subfolders such as `./Docs` or `./Results`.

## Repository

```
.
|--- build
|--- referece
|--- software
| LICENSE
| README.md

```

The purpose of each subfolder is explained below:
+ build: Contains all the (binary) design files needed to make the battery and PCB.
+ reference: contains material from the original company as well as teardown photos from reverse engineering the original boards
+ software: contains firmware for the STM32 on the BMS as well as engineering tools used

## Build instructions
Build instructions -->
BOM link -->
Test instructions -->

## Goals

Beautiful, complex objects of the 21st century require modern techniques to keep them running. Car restoration has been around as long as there have cars, bicycles are no different. However, unlike classic bicycles, e-bikes tend to become e-waste at the first sign of trouble, especially if the company isn't around to provide parts or documentation. In putting this info out there, I hope:
+ Bicycle companies will consider putting documentation for designs online before shutting off part supply forever
+ Motivated enthusiasts (like myself) will consider taking the plunge to keep machines they value alive for another few years.

## Team

+ Project initiator: Nathan Hall-Snyder @nhallsny
+ Contributors:
	+ _add a list of main contributors_


## License

This project is released under CERN Open Hardware Licence Version 2 - Permissive
You can modify and reuse as you like.
