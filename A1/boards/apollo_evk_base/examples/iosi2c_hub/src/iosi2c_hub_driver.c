//*****************************************************************************
//
//! @file iosi2c_hub_driver.c
//!
//! @brief Slave-side functions for an I2C protocol between Apollo and a Host
//!
//! Slave-side functions for an I2C based communications channel between
//! Apollo and a host such as the application processor in an Android tablet
//! or cell phone.
//
// Layout of Apollo Slave Directly Addressable 120 Byte Register Space
//             ---------------------------------
//  0x77(119) |                                 |
//            |     unused                      |
//  0x6B(107) |                                 |
//             ---------------------------------
//  0x6A(106) |                                 |
//            |                                 |
//            |                                 |
//            |                                 |
//            |  Apollo to Host Message (64B)   |
//            |                                 |
//            |                                 |
//  0x26(38)  |                                 |
//             ---------------------------------
//  0x25(37)  |                                 |
//            |                                 |
//            |                                 |
//            |  Host to Apollo Message (32B)   |
//            |                                 |
//            |                                 |
//  0x2 (2)   |                                 |
//             ---------------------------------
//            |                                 |
//  0x1 (1)   | Host To Apollo Write Msg Length |  IOS Access Interrupt Enabled
//             ---------------------------------
//            |                                 |
//  0x0 (0)   | Host To Apollo Read Msg (H2A)   |  IOS Access Interrupt Enabled
//             ---------------------------------
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
#include "am_mcu_apollo.h"
#include "am_util_ring_buffer.h"
#include "iosi2c_hub_driver.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define SIGNAL_GPIO                         4
#define SLAVE_BUFFER_SIZE                   128

//*****************************************************************************
//
// Initialize the I/O Slave and get ready to commune with the hosts.
//
//*****************************************************************************
void
iosi2c_hub_driver_init(am_hal_ios_config_t *psIOSConfig)
{
    //
    // Configure the IOS based on the settings structure.
    //
    am_hal_ios_config(psIOSConfig);

    //
    // Clear out any IOS register-access interrupts that may be active, and
    // enable interrupts for the registers we're interested in.
    //
    am_hal_ios_access_int_clear(AM_HAL_IOS_ACCESS_INT_ALL);
    am_hal_ios_access_int_enable(  AM_HAL_IOS_ACCESS_INT_01
                                 | AM_HAL_IOS_ACCESS_INT_00);

    //
    // We aren't using the IOS FIFO and we aren't using the IOS control IRQs.
    //
    am_hal_ios_int_clear(AM_HAL_IOS_INT_ALL);

    //
    // Set the bit in the NVIC to accept access interrupts from the IO Slave.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSACC);
}

//*****************************************************************************
//
// Call this function from their I/O Slave Access ISR
//
//*****************************************************************************
void
iosi2c_hub_driver_acc_service(uint8_t *pui8Destination, uint32_t ui32Status)
{
    uint8_t ui8Size, ui8Index, ui8BufferIndex;

    //
    // Register access of LRAM location 0x4F means we have a new packet from the
    // host. These packets are variable sized and assumed to end at byte 0x4F.
    // This means their starting point for the first byte moves relative to
    // to the fixed location 0x4F. The host always writes the length of the
    // packet into 0x4F as its last write.
    //
    if (ui32Status & AM_HAL_IOS_ACCESS_INT_01)
    {
        //
        // Read the size of the new packet.
        //
        ui8Size = am_hal_ios_pui8LRAM[0x01];

        //
        // Find variable length packet starting address.
        //
        ui8Index = 0x02;

        //
        // Always copy to the beginning of their buffer.
        //
        ui8BufferIndex = 0;

        //
        // Copy the packet into our SRAM buffer.
        //
        for (ui8Index = 2; ui8Index < (ui8Size + 2); ui8Index++)
        {
            //
            // Keep track of our place in the buffer using a global index
            // variable, since we don't know yet if this is a complete packet.
            //
            pui8Destination[ui8BufferIndex++] = am_hal_ios_pui8LRAM[ui8Index];
        }
    }
}

//*****************************************************************************
//
// Call this function to format an SHTP message.
//
//*****************************************************************************
void
iosi2c_hub_driver_message_format(iosi2c_hub_driver_message_t *psMessage,
                                 uint8_t *pBuffer)
{
    uint8_t *pui8SrcPtr, *pui8DstPtr;
    uint32_t ui32I;

    //
    // Insert Length into the Header.
    //
    pBuffer[0] = (psMessage->Length >> 0);
    pBuffer[1] = (psMessage->Length >> 8);

    //
    // If the length is greater than 60 sit in a while(1).
    //
    if (psMessage->Length > 60)
    {
        while (1);
    }

    //
    // Insert Channel Number into the Header.
    //
    pBuffer[2] = psMessage->ChannelNumber;

    //
    // Insert Sequence Number into the Header.
    //
    pBuffer[3] = psMessage->SequenceNumber;

    //
    // Copy the payload into the target message string.
    //
    pui8SrcPtr = psMessage->Message;
    pui8DstPtr = &pBuffer[4];
    for (ui32I = 0; ui32I < psMessage->Length; ui32I++)
    {
        *pui8DstPtr++ = *pui8SrcPtr++;
    }
}

//*****************************************************************************
//
// Call this function to extract an SHTP message.
//
//*****************************************************************************
void
iosi2c_hub_driver_message_extract(iosi2c_hub_driver_message_t *psMessage,
                                  uint8_t *pBuffer)
{
    uint8_t *pui8SrcPtr, *pui8DstPtr;
    uint32_t ui32I;

    //
    // Extract Length from header.
    //
    psMessage->Length = (uint32_t)pBuffer[1] << 8;
    psMessage->Length |= pBuffer[0];

    //
    // Extract Channel Number.
    //
    psMessage->ChannelNumber = pBuffer[2];

    //
    // Extract Sequence Number.
    //
    psMessage->SequenceNumber = pBuffer[3];

    //
    // Copy the payload in to the target message string.
    //
    pui8SrcPtr = &pBuffer[4];
    pui8DstPtr = psMessage->Message;
    for (ui32I = 0; ui32I < (psMessage->Length - 4); ui32I++)
    {
        *pui8DstPtr++ = *pui8SrcPtr++;
    }
}

//*****************************************************************************
//
// Call this function to send a message to the host. The message should include
// the standard 4 byte header which includes a 2 byte length field.
//
//*****************************************************************************
void
iosi2c_hub_driver_write(uint8_t *pui8Data, uint8_t ui8Size)
{
    uint8_t ui8Index;
    uint8_t ui8BufferIndex = 0;

    //
    // "Send" the actual packet by copying the packet from our SRAM buffer to
    //  the IOS LRAM.
    //
    for (ui8Index = 0x26; ui8Index < (ui8Size + 0x26); ui8Index++)
    {
        //
        // Keep track of our place in the buffer using a global index
        // variable, since we don't know yet if this is a complete packet.
        //
        am_hal_ios_pui8LRAM[ui8Index] = pui8Data[ui8BufferIndex++];
    }
}

