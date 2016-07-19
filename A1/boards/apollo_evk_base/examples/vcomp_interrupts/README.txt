This example initializes the voltage comparator, enables its interrupts
then captures transition interrupts and signal them to the base level
via a very simplistic method where it then reports the transitions on the
printf stream.

When connected to a suitable RC network on the control pins, this example
creates a relaxation oscillator. The Apollo EVK base board contains one
example of a relaxation oscillator circuit. One can monitor the voltage
waveform on pin 18 to see the relaxation oscillator charge/discharge cycle.

The RXO pins are defined in the BSP for this example.

This example assumes the voltage comparator EXT2 on pin 18 is used for
the comparator input. If EXT1 is used, then make suitable changes below.

This example turns on the following additional functions:
1. The voltage comparator
2. Voltage comparator RXO pin (GPIO output)
3. Voltage comparator RXO_CMP pin (analog input pin)
4. ITM/SWO output pin
5. 3 LED pins
6. The bandgap


