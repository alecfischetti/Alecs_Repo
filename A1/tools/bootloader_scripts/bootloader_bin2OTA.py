#!/usr/bin/env python3

import argparse


#******************************************************************************
#
# Output file templates.
#
#******************************************************************************
fileheader = '''
//*****************************************************************************
//
//! @file {name}.c
//!
//! @brief This is a generated file.
//
//*****************************************************************************

#include <stdint.h>
#include "{name}.h"

//*****************************************************************************
//
// Extracted binary array
//
//*****************************************************************************
const uint8_t g_pui8{varname}[{length}] =
{{
'''.strip()

filefooter = '''
};
'''.strip()

hfile = '''
//*****************************************************************************
//
//! @file {name}.h
//!
//! @brief This is a generated file.
//
//*****************************************************************************

#include <stdint.h>

#ifndef {macroname}_H
#define {macroname}_H

//*****************************************************************************
//
// Load address for the binary array.
//
//*****************************************************************************
#define {loadaddressmacro:35} {load_address}

//*****************************************************************************
//
// Length of the binary array in bytes.
//
//*****************************************************************************
#define {lengthmacro:35} {length}

//*****************************************************************************
//
// CRC-32C value calculated over the binary array.
//
//*****************************************************************************
#define {crcmacro:35} {crc}

//*****************************************************************************
//
// Extracted binary array.
//
//*****************************************************************************
extern const uint8_t g_pui8{varname}[{length}];

#endif // {macroname}_H
'''.strip()


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
def process(app_filename, load_address,  output):

    print("load_address ",hex(load_address), "(",load_address,")")

    # Open the file, and read it into an array of integers.
    with open(app_filename, mode = 'rb') as f_app:
        app_binarray = f_app.read()
        f_app.close()


    app_length  = len(app_binarray)

    print("app_size ",hex(app_length), "(",app_length,")")

    pad_length  =  36;

    print("pad_length ",pad_length);

    app_crc = crc32(app_binarray)

    print("crc =  ",hex(app_crc));




    #generate mutable byte array for the boot loader
    pad_binarray = bytearray([0]*pad_length);

    # mark this OTA file as being non-encrypted.
    pad_binarray[0]  = 0x00;
    pad_binarray[1]  = 0x00;
    pad_binarray[2]  = 0x00;
    pad_binarray[3]  = 0x00;

    # Insert the application binary load address.
    pad_binarray[4]  = (load_address >>  0) & 0x000000ff
    pad_binarray[5]  = (load_address >>  8) & 0x000000ff
    pad_binarray[6]  = (load_address >> 16) & 0x000000ff
    pad_binarray[7]  = (load_address >> 24) & 0x000000ff

    # put the application binary size into the padding array @ 0x3c04
    pad_binarray[8]  = (app_length >>  0)   & 0x000000ff
    pad_binarray[9]  = (app_length >>  8)   & 0x000000ff
    pad_binarray[10] = (app_length >> 16)   & 0x000000ff
    pad_binarray[11] = (app_length >> 24)   & 0x000000ff

    # compute the CRC for the application and write it to 0x3c08
    app_crc = crc32(app_binarray)
    pad_binarray[12] = (app_crc >>  0) & 0x000000ff
    pad_binarray[13] = (app_crc >>  8) & 0x000000ff
    pad_binarray[14] = (app_crc >> 16) & 0x000000ff
    pad_binarray[15] = (app_crc >> 24) & 0x000000ff



    # now output both binary arrays in the proper order
    with open(output + '.OTA', mode = 'wb') as out:
        out.write(pad_binarray)
        out.write(app_binarray)

    # Define a set of names to be used in the C output files.
    formatmap = {'length' : len(app_binarray),
                 'load_address' : hex(load_address),
                 'name' : output,
                 'macroname' : output.upper(),
                 'loadaddressmacro' : output.upper() + '_LOAD_ADDRESS',
                 'lengthmacro' : output.upper() + '_LENGTH',
                 'crcmacro' : output.upper() + '_CRC',
                 'varname' : output.title().replace('_', ''),
                }

    with open(output + '.c', mode = 'w') as out:
        # Print the global fileheader.
        print(fileheader.format(**formatmap), file=out)

        # S will be the string to accumulate the byte values in a C-friendly format
        S = '    '

        # Loop over binarray
        for n in range(len(app_binarray)):

            # Add the hex representation of each byte to S
            S += '0x{:02X}'.format(app_binarray[n])

            # Print in comma-separated lines, 8 bytes long each.
            if n % 8 == 7:
                S += ',\n    '
            else:
                S += ', '

        # The end of the previous loop will leave an extra comma and some amount of
        # whitespace after the last byte value. This line will remove that.
        S = S.rsplit(',', 1)[0]

        print(S, file=out)

        print(filefooter, file=out)

    formatmap['crc'] = '0x{:08X}'.format(crc32(app_binarray))

    with open(output + '.h', mode = 'w') as out:
        print(hfile.format(**formatmap), file=out)


#******************************************************************************
#
# Main program flow
#
#******************************************************************************
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description =
                                     'Generate an OTA file and C files for a boot loader host to consume.')
    parser.add_argument('src_binfile',
                        help = 'Application binary file to read in as input.')

    parser.add_argument('load_address',
                        help = 'Starting or Load address for this binary.')

    parser.add_argument('-o', dest = 'output', default = 'binary_array',
                        help = 'Output filename (without the extension)')

    args = parser.parse_args()

    process(args.src_binfile, int(args.load_address,16), args.output)
