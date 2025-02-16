# faraday-rescue/reference/design design doc

## Philosophy

Much of this effort was born out of the rather nerve-wracking and labor intensive process of attempting to replace cells on a borked battery pack. The original design required a lot of freehand welding, easy to pinch wires, and opportunities for shorting. The original battery protection system also had high quiescent draw which rapidly bricked the pack if left un-tended. In re-doing the design, the goals were:
- Simplify module welding and bussing
- Achieve a quiescent draw in the low uA range to allow for multi-year storage. This was achieved with a sleep power of around 480uW normally and <50uW in undervoltage lockout.
- Learn embedded software and the modern (as of 2025, anyways) options for hobby electronics projects.

In the PCB design, the goals were:
- Move all critical functions (overvoltage, undervoltage, overcurrent, balancing) to a validated, standard IC
- Use the reference design as much as possible to avoid nasty hardware bugs
- Keep it simple - for example, using a linear regulator and ultra low power processor rather than a more efficient switching regulator

In the software design, the goals were:
- No critical decision making, delgate all important functions to the Battery Management IC
- Reset at every use (avoid bugs from long-term operation).
- Faraday power button must cause a reset
- When in doubt, reset

## String and module design

The original Faraday design featured two module groups - 8x 2P side-by-side groups and 4x 2P end-to-end modules. This arragement results in an intricate series of specialized nickel strips to build the voltage.

This diagram shows the original (top) vs this design (bottom) cell layout.

![image](https://github.com/user-attachments/assets/e916ecc8-f00d-4344-9355-900dcf13f602)

All of the series/parallel wiring takes place inside the PCB. The routing is rather complex but fortunately, complexity is free on a PCB. The PCB has wide traces (generally at least 10mm and 2 oz copper for a maximum T_rise of around 10C.
![image](https://github.com/user-attachments/assets/f5ff47eb-c867-453f-9188-1380af13215c)

Each individual module is identical with the voltage building as shown.
![image](https://github.com/user-attachments/assets/9509ed13-7c89-43e8-a142-7d9aa86311a0)


## Cell Selection

I chose a Samsung 35E 18650 for energy density and capability to meet a 10A continuous discharge (in the 2P config).

|Parameter|Limit|
|---------|-----|
|Max V| 4.2V|
|Min V| 2.XV|
|Max Continuous Discharge Current| 8A |
|Min Charge Temperature| 5C|
|Max Charge Temperature| XXC |
|Min Discharge Temperature| XX C|
|Max Discharge Temperature| XX C|

## Battery IC Selection

I chose a BQ76972 for this project for these reasons:
- 16 cells
- Integrated linear regulator
- Integrated balancing
- Integrated temperature measurement
- Integrated overcurrent
- Wake function on charger connection
- High side Back to Back FET driver (to match original Faraday design).
- Very small quiescent power in sleep

The original design used an LT6802-G, which was a fine choice at the time but pushed many more critical functions to the processor. Thus, the processor code must be much more thoroughly validated and tested.

## Processor Selection

I chose a STM32L0 for this project for these reasons:
- Low sleep power (<1uA!!)
- Flexible wake inputs (need to wake with a button pull-down to match original faraday design)
- Standard complement of functions (UART, SPI, I2C)
- Independent watchdog
- Real time clock (though I didn't end up using it)
- Available on JLCPCB

## RS485 Driver Selection
This was a real challenge. There are surprisingly few RS485 drivers that meet these criteria:
- Low sleep power (<1uA)
- 3.3V I/O inputs
- Available on JLCPCB

## Power supplies

# Linear regulator

The STM32 and RS485 driver are powered off of REG1 from the BQ76972. REG1 has a recommended current of 30mA and an minimum of 47mA at 0V. Fortunately, the STM32 uses a maximum of 6mA. The power budget is:

|Function|Power|
|--------|-----|
|STM32L0 | 6mA |
|RS485 Driver | Vcc/2 / 120 Ohms = 13mA |
|Debug LEDs | 1mA * 4 |
|           | Total = 23mA|

REG1 dips a few hundred mV during heavy transmit periods, especially when using the implemented printf funtion for debugging.

At the maximum power draw (30mA), the linear regulator efficincy is laughable (1.3W!), but as long as the BJT used is well coupled to the PCB the deltaT is managable. In normal operation, the LEDs are unused and the RS485 driver is used intermittently.

"""Note - the linear regulator is quite sensitive to leakage currents from skin. Touching the 

## Charge pump

The charge pump takes in battery voltage and boosts it up 10V to provide a high-side drive for the back to back NFETs. This circuit is almost entirely based on the BS76972 reference schematic.

## Sleep/Wake

The design must allow the bike to be woken up in two ways
1. User presses wake button at rear of bike
2. User connects charge cable

To achieve button wake (1), the STM32 uses STOP mode and configures the button active-low as a wake interrupt. Upon waking, the STM32 resets, wakes up the BQ chip, configures it, checks for any error codes, then allows the BQ chip to turn on the back-to-back FETs if allowed.

To achieve charge cable wake (2), the BQ chip is placed into DEEPSLEEP. If a charger is connected (logic level voltage on CD pin), the BQ chip exists DEEPSLEEP and switches to NORMAL. The ALERT output from the BQ chip is configured to only go high when an ADC measurement is ready - this pin goes high a few 10s of milliseconds after the chip boots and is used as a wake interrupt for the STM32.

The corrolary of wake is sleep. The STM32, after a few minutes of inactivity or a user requests the off state, configures the BQ chip for DEEPSLEEP, sets the appropriate interrupts, then enters STOP mode. STOP mode is used because the other STM32 power modes do not support active-low wake interrupts.

The RS485 chip is put into sleep mode by setting RE high and DE low.

[Insert diagram]
![image](https://github.com/user-attachments/assets/34afd942-469b-46e9-9bf7-b9f8c9d57aac)





