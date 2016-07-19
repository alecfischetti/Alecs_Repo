This bootloader is intended to reside permanently in the beginning of flash
on an Apollo MCU used as a sensor hub like device, i.e. attached to a host
application processor.  The AP can download new applications to the flash
in the Apollo processor whenever it desires.

This bootloader implementation support the I/O slave and be conditionally
compiled to use either I2C mode or SPI mode of the I/O slave.

