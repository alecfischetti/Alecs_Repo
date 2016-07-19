//*****************************************************************************
//
//! @file iomi2c_host_side_driver.h
//!
//! @brief Host-side (MASTR) functions for HOST to Apollo connections.
//!
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
#ifndef IOMI2C_HOST_SIDE_DRIVER_H
#define IOMI2C_HOST_SIDE_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif
//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define IOMI2C_HOST_SIDE_PAYLOAD_SIZE    (62)

//*****************************************************************************
//
// External variable definitions
//
//*****************************************************************************

//*****************************************************************************
//
// Message Header
//
//*****************************************************************************
typedef struct
{
    //
    // Length of the message including header bytes.
    //
    uint32_t Length;

    //
    // Channel Number on which this message flows.
    //
    uint8_t ChannelNumber;

    //
    // Sequence Number for this message.
    //
    uint8_t SequenceNumber;

    //
    // Pointer to the actual Message
    //
    uint8_t Message[IOMI2C_HOST_SIDE_PAYLOAD_SIZE];

} iomi2c_host_side_driver_message_t;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void iomi2c_host_side_driver_init(void);
extern uint8_t iomi2c_host_side_driver_read_int_status(void);
extern void iomi2c_host_side_driver_rcv_packet(
                                    iomi2c_host_side_driver_message_t *Msg);
extern void iomi2c_host_side_driver_xmit_packet(
                                    iomi2c_host_side_driver_message_t *Msg);
void iomi2c_host_side_driver_release_slave_buffer(void);

#ifdef __cplusplus
}
#endif

#endif // IOMI2C_HOST_SIDE_DRIVER_H
