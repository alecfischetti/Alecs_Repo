bootloader_targetA is an example that is intended to be downloaded by a
boot host into an Apollo chip running either ios_boot or the secure
boot loader. It is set to build at 0x8000 instead of 0x0.

This example prints a "Hello World" style message marked TARGET A 
over SWO at 1M baud. To see the output of this program, run AMFlash,
and configure the console for SWO. The example sleeps after it is done
printing.

It generates a different repeating output message than targetB.

bootloader_targetA is an example that is intended to be downloaded by a
boot host into an Apollo chip running either ios_boot or the secure
boot loader. It is set to build at 0x8000 instead of 0x0.
bootloader_targetA is an example that is intended to be downloaded by a
boot host into an Apollo chip running either ios_boot or the secure
boot loader. It is set to build at 0x8000 instead of 0x0.

Use the bash script generate_boot_image.sh to get an OTA file for download.

bootloader_targetA and bootloader_targetB are nearly identical programs.
The host program should download targetA into a fresh Apollo MCU with a 
brand new boot loader in it. After a reset it should be continuously
printing "TARGET A: I was downloaded via the boot loader". In order to 
test the host ability to over ride an existing program, one should
download bootloader_targetB to get "TARGET B: I was downloaded via the 
boot loader" in order to confirm a successful update operation.

