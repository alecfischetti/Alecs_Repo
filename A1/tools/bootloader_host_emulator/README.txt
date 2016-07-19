Both the ios_bootloader and the secure boot loader use the I/O slave to
provide a path for a host processor to download new programs and data into the
Apollo Flash.  In order to demonstrate the proper operation of these two boot
loaders, we provide this windows executable to drive and FTDI USB to MPSSE
cable.

Obtain an FTDI C232HM-DDHSL-0 cable and connect it before proceeding.
See the diagram in the ftdi_cable_pin_out.png file in this directory for
directions on how to connect the cable to the Apollo EVK.

The bootloader_host_emulator expects program.ota files as input. See the usage
information below but the parameter to this host emulator is the path to a
folder that contains one or more OTA files.  It will attempt to download all
*.ota files that it finds in the target directory.

Both of the bootloader_test_targetA and bootloader_test_targetB examples in
the apollo_evk_board have scripts that can be used to generate the required
OTA files for download. For convenience, these OTA files come pre-built with
the AmbiqSuite installation.

USAGE: bootlader_host_emulator.exe <folder that contains file to download>

e.g
> bootlader_host_emulator.exe ota_files
