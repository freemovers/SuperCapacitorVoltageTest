# Super Capacitor Voltage Test
Program to read the internal voltage every 1 second to compare the different sleep modes.

The ATtiny 0- and 1-series are great for lower power applications. Peripherals can be used without running the core, and there are different sleep mode options.
This program awakes the ATtiny with the PIT every 1 second to measure the internal voltage and return the results using UART.
