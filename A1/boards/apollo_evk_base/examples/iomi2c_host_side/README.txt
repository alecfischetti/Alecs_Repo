This is the main program. Refer to iomi2c_host_side_driver for the details
of the actions an Android or Nucleus kernel driver has to do to talk to the
Apollo slave for sensor hub applications.

This application simply provides stimulus for the driver code in
iomi2c_host_side_driver.c. This examples passes messages to the slave board
where they are echoed back.

See the i2cios_hub application as an example slave device to talk with.

