//*****************************************************************************
//
//! @file iomi2c_host_side_driver.c
//!
//! @brief Master-side functions for an I2C protocol between host and Apollo
//!
//! Master-side functions for an I2C based communications channel between
//! Apollo and a host such as the application processor in an Android tablet
//! or cell phone. These driver routine mimics the actions of a host
//! processor and its kernel driver.
//
//
// We will use the Apollo I/O slave as a shared memory mail box. Apollo can
// signal the host of a message to read with a GPIO pin.  The host can signal
// Apollo that it has received the message by writing anything to location 0x0.
// The access interrupt is enabled for this byte so that any host writes to
// this shared location will cause an interrupt to the Apollo.
//
// In a similar way, the host first writes a message in the provided buffer.
// The host then writes the length of the message to location 0x1.
//
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
//            |  Apollo to Host Message (68B)   |
//            |  This is a 64 byte message      |
//            |  + a 4 byte header.             |
//            |                                 |
//  0x26(38)  |                                 |
//             ---------------------------------
//  0x25(37)  |                                 |
//            |                                 |
//            |                                 |
//            |  Host to Apollo Message (36B)   |
//            |  This is a 32 byte message      |
//            |  + a 4 byte header.             |
//  0x2 (2)   |                                 |
//             ---------------------------------
//            |                                 |
//  0x1 (1)   | Host To Apollo Write Msg Length |  IOS Access Interrupt Enabled
//             ---------------------------------
//            |                                 |
//  0x0 (0)   | Host To Apollo Read Msg (H2A)   |  IOS Access Interrupt Enabled
//             ---------------------------------
//
// The 4 byte header attached to each message is completely arbitrary and is
// provided here as an example of one to convey the size and target of a
// message.
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
// This is part of revision 0.0.36-r1.0 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "iomi2c_host_side_driver.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
//! This 7 bit I2C slave address gets shifted left by 1 in use so 0x4A --> 0x94
#define  SLAVE_I2C_ADDRESS (0x4A)

//*****************************************************************************
//
// Initialize this device driver module.
//
//*****************************************************************************
void
iomi2c_host_side_driver_init(void)
{
    uint32_t ui32IntStatusClear = 0x000000FF;
    uint32_t ui32IntEnable = 0x00000000;

    //
    // Clear the interrupt status bits in the host side of the I/O Slave.
    //
    am_hal_iom_i2c_write(0,                          // IOM 0
                         SLAVE_I2C_ADDRESS,          // I2C Bus Address
                         &ui32IntStatusClear, 1,     // data, length
                         AM_HAL_IOM_OFFSET(0x7A));   // offset reg in slave

    //
    // Clear the interrupt enable bits in the host side of the I/O Slave.
    //
    am_hal_iom_i2c_write(0,                          // IOM 0
                         SLAVE_I2C_ADDRESS,          // I2C Bus Address
                         &ui32IntEnable, 1,          // data, length
                         AM_HAL_IOM_OFFSET(0x78));   // offset reg in slave
}

//*****************************************************************************
//
// Receive a packet from the slave over I2C.
// This function uses polled I/O master I2C reads and writes.
//
// Note: Maximum packet size is 64 bytes.
//
//*****************************************************************************
void
iomi2c_host_side_driver_rcv_packet(iomi2c_host_side_driver_message_t *Msg)
{
    uint32_t ui32Header;
    uint32_t ui32Payload[36];
    uint32_t ui32Length;
    uint32_t ui32I;
    uint8_t *ui8PtrSrc = (uint8_t *)ui32Payload;
    uint8_t *ui8PtrDst = Msg->Message;

    //
    // Read 4 byte header from the FIFO.
    //
    am_hal_iom_i2c_read(0,                          // IOM 0
                        SLAVE_I2C_ADDRESS,          // I2C Bus Address
                        &ui32Header, 4,             // data, length
                        AM_HAL_IOM_OFFSET(0x26));   // offset reg in slave

    //
    // Extract the header info.
    //
    ui32Length = ((ui32Header >> 0) & 0x000000FF)
               | ((ui32Header >> 8) & 0x000000FF);
    Msg->Length = ui32Length;
    Msg->ChannelNumber  = ((ui32Header >> 16) & 0x000000FF);
    Msg->SequenceNumber = ((ui32Header >> 24) & 0x000000FF);

    //
    // Enforce the 64 byte limit, plus 4 byte header.
    //
    if (Msg->Length>(68))
    {
        while(1);   // Trap here in test code.
    }

    //
    // Now read the payload from the FIFO.
    //
    am_hal_iom_i2c_read(0,                              // IOM 0
                        SLAVE_I2C_ADDRESS,              // I2C Bus Address
                        ui32Payload, ui32Length - 4,    // data, length
                        AM_HAL_IOM_OFFSET(0x2A));       // offset reg in slave

    //
    // Copy the payload into the Message structure.
    //
    for (ui32I = 0; ui32I < (ui32Length - 4); ui32I++)
    {
        *ui8PtrDst++ = *ui8PtrSrc++;
    }
}

//*****************************************************************************
//
// Transmit a packet to the slave over I2C.
// This function uses polled I/O master I2C reads and writes.
// Assumes 2 Length bytes includes 4 for the header.
//
// Note: Maximum packet size is 32 bytes.
//
//*****************************************************************************
void
iomi2c_host_side_driver_xmit_packet(iomi2c_host_side_driver_message_t *Msg)
{
    uint32_t ui32Message[32];
    uint8_t *ui8PtrMessage = (uint8_t *)ui32Message;
    uint8_t *ui8PtrSrc = Msg->Message;
    uint32_t ui32I;

    //
    // Enforce the 32 byte limit, including 4 byte header
    //
    if (Msg->Length>(36))
    {
        while(1);   // trap here in test code
    }

    //
    // Format the message string for transmission.
    //
    *ui8PtrMessage++ = (Msg->Length >> 0) & 0x000000FF;
    *ui8PtrMessage++ = (Msg->Length >> 8) & 0x000000FF;
    *ui8PtrMessage++ = Msg->ChannelNumber;
    *ui8PtrMessage++ = Msg->SequenceNumber;

    //
    // Copy the payload into the Message string.
    //
    for (ui32I = 0; ui32I < (Msg->Length - 4); ui32I++)
    {
        *ui8PtrMessage++ = *ui8PtrSrc++;
    }

    //
    // Copy the message to the host to slave buffer in the IOS.
    //
    am_hal_iom_i2c_write(0,                             // IOM 0
                         SLAVE_I2C_ADDRESS,             // I2C Bus Address
                         ui32Message,                   // data,
                         Msg->Length,                   // length
                         AM_HAL_IOM_OFFSET(0x02));      // offset reg in slave

    //
    // Send the message to the slave by writing the length to offset 0x01.
    //
    ui32Message[0] = Msg->Length & 0x0000000FF;
    am_hal_iom_i2c_write(0,                             // IOM 0
                         SLAVE_I2C_ADDRESS,             // I2C Bus Address
                         ui32Message,                   // data,
                         1,                             // length
                         AM_HAL_IOM_OFFSET(0x01));      // offset reg in slave
}


//*****************************************************************************
//
// Transmit a packet to the slave over I2C to release the slave's buffer.
//
//
//*****************************************************************************
void
iomi2c_host_side_driver_release_slave_buffer(void)
{
    uint32_t ui32Message[32];

    //
    // write anything to offset zero in the slave
    //
    am_hal_iom_i2c_write(0,                             // IOM 0
                         SLAVE_I2C_ADDRESS,             // I2C Bus Address
                         ui32Message,                   // data,
                         1,                             // length
                         AM_HAL_IOM_OFFSET(0x00));      // offset reg in slave
}
