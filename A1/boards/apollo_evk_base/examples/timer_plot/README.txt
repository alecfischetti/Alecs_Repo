This example plots the value of a variable using AM Flash. This works by
configuring a timer interrupt, starting the timer, tracking in a variable
the number of times interrupts occur, and then plotting various bits from
that count. The value plotted depends on the axis.

SWO is configured in 1M baud, 8-n-1 mode.

