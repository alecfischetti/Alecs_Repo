#!/usr/bin/env python3

import argparse


#******************************************************************************
#
# CRC function that matches the CRC used by the Apollo bootloader.
#
#******************************************************************************
poly32 = 0x1EDC6F41
def crc32(L):
    rem = 0
    for b in L:
        rem = rem ^ (b << 24)
        for i in range(8):
            if rem & 0x80000000:
                rem = ((rem << 1) ^ poly32)
            else:
                rem = (rem << 1)

            rem = rem & 0xFFFFFFFF
    return rem



#******************************************************************************
#
# Read in the binary files and output the merged binary
#
#******************************************************************************
def process(boot_loader_filename, app_filename,  output):

    # Open the file, and read it into an array of integers.
    with open(boot_loader_filename, mode = 'rb') as f_bl:
        boot_loader_binarray = f_bl.read()
        f_bl.close()

    # Open the file, and read it into an array of integers.
    with open(app_filename, mode = 'rb') as f_app:
        app_binarray = f_app.read()
        f_app.close()

    boot_length = len(boot_loader_binarray)
    print("boot size ",boot_length)
    if boot_length > 16383:
	    print("ERROR boot loader image is too big");
	    return

    app_length  = len(app_binarray)

    pad_length  = 0x4000 - len(boot_loader_binarray)

    # this is where we will write the application size in FLASH
    flag_page_location = 0x3c00 - len(boot_loader_binarray)

    print("app_size ",len(app_binarray))
    print("flag_page_location ",flag_page_location)
    print("Modifying location",format(len(boot_loader_binarray) + flag_page_location, '#08x'))
    print("pad_length ",pad_length);

    #generate mutable byte array for the boot loader
    pad_binarray = bytearray([0]*pad_length);

    # put the application binary link address into the padding array @ 0x3c00
    pad_binarray[flag_page_location + 0]  = 0x00;
    pad_binarray[flag_page_location + 1]  = 0x40;
    pad_binarray[flag_page_location + 2]  = 0x00;
    pad_binarray[flag_page_location + 3]  = 0x00;

    # put the application binary size into the padding array @ 0x3c04
    pad_binarray[flag_page_location + 4]  = (app_length >>  0) & 0x000000ff
    pad_binarray[flag_page_location + 5]  = (app_length >>  8) & 0x000000ff
    pad_binarray[flag_page_location + 6]  = (app_length >> 16) & 0x000000ff
    pad_binarray[flag_page_location + 7]  = (app_length >> 24) & 0x000000ff

    # compute the CRC for the application and write it to 0x3c08
    app_crc = crc32(app_binarray)
    pad_binarray[flag_page_location + 8]  = (app_crc >>  0) & 0x000000ff
    pad_binarray[flag_page_location + 9]  = (app_crc >>  8) & 0x000000ff
    pad_binarray[flag_page_location + 10] = (app_crc >> 16) & 0x000000ff
    pad_binarray[flag_page_location + 11] = (app_crc >> 24) & 0x000000ff

    # choose an over ride pin number
    pad_binarray[flag_page_location + 12] = 0

    # choose an over ride pin polarity
    pad_binarray[flag_page_location + 16] = 1

    # put the application binary stack pointer into the padding array @ 0x3c14
    pad_binarray[flag_page_location + 20] = app_binarray[0];
    pad_binarray[flag_page_location + 21] = app_binarray[1];
    pad_binarray[flag_page_location + 22] = app_binarray[2];
    pad_binarray[flag_page_location + 23] = app_binarray[3];

    # put the application binary PC into the padding array @ 0x3c18
    pad_binarray[flag_page_location + 24] = app_binarray[4];
    pad_binarray[flag_page_location + 25] = app_binarray[5];
    pad_binarray[flag_page_location + 26] = app_binarray[6];
    pad_binarray[flag_page_location + 27] = app_binarray[7];


    # now output all three binary arrays in the proper order
    with open(output + '.bin', mode = 'wb') as out:
        out.write(boot_loader_binarray)
        out.write(pad_binarray)
        out.write(app_binarray)

#******************************************************************************
#
# Main program flow
#
#******************************************************************************
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description =
                                     'Combine two binary files in to a single download.')
    parser.add_argument('boot_binfile',
                        help = 'Bootloader binary file to read in as input.')

    parser.add_argument('app_binfile',
                        help = 'Application binary file to read in as input.')

    parser.add_argument('-o', dest = 'output', default = 'binary_array',
                        help = 'Output filename (without the extension)')

    args = parser.parse_args()

    process(args.boot_binfile, args.app_binfile, args.output)

