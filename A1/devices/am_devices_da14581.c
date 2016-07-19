//*****************************************************************************
//
//! @file am_devices_da14581.c
//!
//! @brief Support functions for the Dialog Semiconductor DA14581 BTLE radio.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2016, Ambiq Micro
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 1.1.0 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_bsp.h"
#include "am_devices_da14581.h"


//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define DIALOG_BOOT_STX                     0x02
#define DIALOG_BOOT_SOH                     0x01
#define DIALOG_BOOT_ACK                     0x06
#define DIALOG_BOOT_NACK                    0x15



//*****************************************************************************
//
//! @brief Runs a UART based boot sequence for a Dialog radio device.
//!
//! @param pui8BinData - pointer to an array of bytes containing the firmware
//! for the DA14581
//!
//! @param ui32NumBytes - length of the DA14581 firmware image.
//!
//! This funciton allows the Ambiq device to program a "blank" DA14581 device
//! on startup. It will handle all of the necessary UART negotiation for the
//! Dialog boot procedure, and will verify that the CRC value for the
//! downloaded firmware image is correct.
//!
//! @return true if successful.
//
//*****************************************************************************
bool
am_devices_da14581_uart_boot(const uint8_t *pui8BinData, uint32_t ui32NumBytes)
{
    uint32_t ui32Index;
    uint8_t ui8CRCValue;
    char ui8RxChar;

    //
    // Poll the RX lines until we get some indication that the dialog radio is
    // present and ready to receive data.
    //
    do
    {
        am_hal_uart_char_receive_polled(&ui8RxChar);
    }
    while (ui8RxChar != DIALOG_BOOT_STX);

    //
    // Send the Start-of-Header signal and the length of the data download.
    //
    am_hal_uart_char_transmit_polled(DIALOG_BOOT_SOH);
    am_hal_uart_char_transmit_polled(ui32NumBytes & 0xFF);
    am_hal_uart_char_transmit_polled((ui32NumBytes & 0xFF00) >> 8);


    //
    // Poll for the 'ACK' from the dialog device that signifies that the header
    // was recieved correctly.
    //
    do
    {
        am_hal_uart_char_receive_polled(&ui8RxChar);
    }
    while (ui8RxChar != DIALOG_BOOT_ACK);

    //
    // Initialize the CRC value to zero.
    //
    ui8CRCValue = 0;

    //
    // Send the binary image over to the dialog device one byte at a time,
    // keeping track of the CRC as we go.
    //
    for (ui32Index = 0; ui32Index < ui32NumBytes; ui32Index++)
    {
        ui8CRCValue ^= pui8BinData[ui32Index];

        am_hal_uart_char_transmit_polled(pui8BinData[ui32Index]);
    }

    //
    // The Dialog device should respond back with a CRC value at the end of the
    // programming cycle. We should check here to make sure that they got the
    // same CRC result that we did. If it doesn't match, return with an error.
    //
    am_hal_uart_char_receive_polled(&ui8RxChar);

    if (ui8RxChar != ui8CRCValue)
    {
        return 1;
    }

    //
    // If all is well, send the final 'ACK' to tell the dialog device that its
    // new image is correct. After this point, the dialog device should start
    // running the downloaded firmware.
    //
    am_hal_uart_char_transmit_polled(DIALOG_BOOT_ACK);

    //
    // Wait until the FIFO is actually empty and the UART is no-longer busy.
    //
    while (!AM_BFR(UART, FR, TXFE) || AM_BFR(UART, FR, BUSY));

    return 0;
}


//*****************************************************************************
//
//! @brief Handles Initialization of the DA14581 after SPI HCI Download
//!
//! @param psDevice - pointer to device information structure for the DA14581
//!
//! This funciton looks for the DREADY line to be high and returns false if
//! not. It sends 1 byte read command over the IOM to read the protocol control
//! byte from the DA14581. If this is not 0x06 then false is returned.
//!
//! @return true if successful.
//
//*****************************************************************************
bool
am_devices_da14581_init(am_devices_da14581_t *psDevice)
{
    uint32_t ui32Buffer[1];
    uint8_t *ui8Command  = (uint8_t *)ui32Buffer;
    uint8_t *ui8Response = (uint8_t *)ui32Buffer;

    if (psDevice->ui32Mode)
    { // then SPI mode
        //
        // check DREADY state, must be asserted here or failure
        //
        if ( !am_hal_gpio_input_bit_read(psDevice->ui32DREADY) )
        {
            return false;
        }

        //
        //  enable the IOM
        //
        am_hal_iom_enable(psDevice->ui32IOMModule);

        //
        // assert chip select via GPIO
        //
        am_hal_gpio_out_bit_clear(psDevice->ui32IOMChipSelect);

        //
        // Use the IOM to read the one byte SPI XPORT direction control
        //
        ui8Command[0] = 0x08;
        am_hal_iom_spi_write(psDevice->ui32IOMModule,
                             0, // arbitrary chip select, we are using GPIO
                             ui32Buffer, 1, AM_HAL_IOM_RAW | AM_HAL_IOM_CS_LOW);
        am_hal_iom_spi_read(psDevice->ui32IOMModule,
                            0, // arbitrary chip select, we are using GPIO
                            ui32Buffer, 1, AM_HAL_IOM_RAW);

        //
        // remove chip select via GPIO
        //
        am_hal_gpio_out_bit_set(psDevice->ui32IOMChipSelect);

        //
        // We must have received a 0x6 token back from the DA14581
        //
        if (ui8Response[0] != AM_DEVICES_DA14581_SPI_XPORT_CTS)
        {
            return false;
        }


    }
    else // UART mode
    {
        //
        // NOT CURRENTLY SUPPORTED HERE
        //
        return false;
    }

    return true;
}



//*****************************************************************************
//
//! @brief Sends Command Packets to the DA14581 on the SPI Xport
//!
//! @param psDevice - pointer to device information structure for the DA14581
//!
//! @param type     - transport packet type
//!
//! @param pData    - Pointer to data buffer. Note this is uint8_t aligned.
//!
//! @param len      - Length of raw transfer
//!
//! This function sends a buffer of bytes to the DA14581
//!
//! NOTE: at this time this uses polled IOM transfers
//!
//! @return nothing
//
//*****************************************************************************
void
am_devices_da14581_spi_send(am_devices_da14581_t *psDevice, uint32_t options,
                            uint8_t type, uint8_t *pData, uint16_t len)
{
    int32_t i32I;
    uint32_t ui32IOMBuffer[16];
    uint8_t  *Command = (uint8_t *)ui32IOMBuffer;

    //
    //  for now we can only handle 64 byte max size xfers
    //
    if (len>63)
    {
        while (1);
    }

    //
    // Copy the values into our command buffer
    //
    Command[0] = type;
    for (i32I = 1; i32I < (len + 1); i32I++)
    {
        Command[i32I] = *pData++;
    }

    //
    // assert chip select via GPIO
    //
    am_hal_gpio_out_bit_clear(psDevice->ui32IOMChipSelect);

    //
    // Use the IOM to send the HCI command or data
    //
    am_hal_iom_spi_write(psDevice->ui32IOMModule,
                             0, // arbitrary chip select, we are using GPIO
                             ui32IOMBuffer, len + 1,
                             options | AM_HAL_IOM_RAW );

    //
    // Keep chip select low if requested
    //
    if ( !(options & AM_HAL_IOM_CS_LOW) )
    {
       //
       // remove chip select via GPIO
       //
       am_hal_gpio_out_bit_set(psDevice->ui32IOMChipSelect);
    }

}



//*****************************************************************************
//
//! @brief Recieve Bytes from the DA14581 on the SPI Xport
//!
//! @param psDevice - pointer to device information structure for the DA14581
//!
//! @param pData    - Pointer to data buffer. Note this is uint8_t aligned.
//!
//! @param len      - Length of raw transfer
//!
//! This function receives a buffer of bytes from the DA14581
//!
//! NOTE: at this time this uses polled IOM transfers
//!
//! @return nothing
//
//*****************************************************************************
void
am_devices_da14581_spi_receive_bytes(am_devices_da14581_t *psDevice,
                                     uint32_t options,
                                     uint8_t *pData, uint16_t len)
{
    int32_t i32I;
    uint32_t ui32IOMBuffer[16];
    uint8_t  *Response = (uint8_t *)ui32IOMBuffer;

    //
    //  for now we can only handle 64 byte max size xfers
    //
    if (len>64)
    {
        while (1);
    }

    //
    // assert chip select via GPIO
    //
    am_hal_gpio_out_bit_clear(psDevice->ui32IOMChipSelect);

    //
    // Use the IOM to receive bytes from the SPI interface
    //
    am_hal_iom_spi_read(psDevice->ui32IOMModule,
                        0, // arbitrary chip select, we are using GPIO
                        ui32IOMBuffer, len, (options | AM_HAL_IOM_RAW));

    //
    // remove chip select via GPIO
    //
    am_hal_gpio_out_bit_set(psDevice->ui32IOMChipSelect);

    //
    // Copy the values to caller's response buffer
    //
    for (i32I = 0; i32I < len; i32I++)
    {
        *pData++ = Response[i32I];
    }
}
