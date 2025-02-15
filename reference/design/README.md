# faraday-rescue/reference/design Design Doc

## Philosophy

Much of this effort was born out of the tragically intricate and labor-intensive process of attempting to replace cells on a borked battery pack. The original design required a lot of free-hand welding, small, easy to pinch wires, and opportunities for shorting. The original battery protection system also had high quiescent draw which rapidly bricked the pack if left un-tended. In re-doing the design, the goals were:
- Simplify module welding and bussing
- Achieve a quiescent draw in the single digit uA to allow for multi-year storage
- Learn embedded software and the modern (as of 2025, anyways) options for hobby electronics projects.

In the PCB design, the goals were:
- Move all critical functions (overvoltage, undervoltage, overcurrent, balancing) to a validated, standard IC
- Use the reference design as much as possible to avoid nasty hardware bugs
- Keep it simple - for example, using a linear regulator and ultra low power processor rather than a more efficient switching regulator

In the software design, the goals were:
- No critical decision making, delgate all important functions to the Battery Management IC
- Reset at every use (avoid bugs from long-term operation)
- When in doubt, reset

## Module design

The original Faraday design featured two module groups - 8x 2P side-by-side groups and 4x 2P end-to-end modules. This arragement results in an intricate series of specialized nickel strips to build the voltage.

## Cell Selection

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

# Charge pump

The charge pump takes in battery voltage and boosts it up 10V to provide a high-side drive for the back to back NFETs. This circuit is almost entirely based on the BS76972 reference schematic.




