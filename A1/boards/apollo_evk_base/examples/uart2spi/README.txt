This example accepts STXETX protocol packets from the buffered UART and 
turns them into SPI transactions on I/O Master 0.  In addition, it monitors
one GPIO line for interrupts from an Apollo's I/O slave.  It will also
handle the reset/SPICLK protocol to force a boot loader in to BL mode or
in to application mode. It is intended to run the HOST EMULATION side of a
pair of Apollo EVK boards. The other one is the sensor hub with boot 
loader installed.

UART2SPI EVK                     BOOTLOADER EVK
GPIO[5]  IOM SPI CLK            GPIO[0]  IOS SPI CLK  (also OVERRIDE PIN)
GPIO[6]  IOM SPI MISO           GPIO[1]  IOS SPI MISO
GPIO[7]  IOM SPI MOSI           GPIO[2]  IOS SPI MOSI
GPIO[4]  IOM SPI M0nCE5         GPIO[3]  IOS SPI CSn
GPIO[0]  IOM GPIO               GPIO[4]  IOS Interrupt
GPIO[1]  IOM GPIO               RSTn     Reset pin

