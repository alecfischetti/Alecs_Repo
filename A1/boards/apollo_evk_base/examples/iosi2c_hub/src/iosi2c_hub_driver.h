//*****************************************************************************
//
//! @file iosi2c_hub_driver.h
//!
//! @brief Slave-side functions for the IOS based protocol.
//!
//! Slave-side functions for the IOS based SHTP-like protocol.
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
#ifndef IOSI2C_HUB_DRIVER_H
#define IOSI2C_HUB_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif
//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define IOSI2C_HUB_DRIVER_LAST_RCV_BYTE   0x4F
#define IOSI2C_HUB_DRIVER_FIFO_BASE       0x80
#define IOSI2C_HUB_DRIVER_FIFO_END        0x100
#define IOSI2C_HUB_DRIVER_PAYLOAD_SIZE    (62)

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
typedef struct iosi2c_hub_driver_message_s
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
    // Pointer to the actual message.
    //
    uint8_t Message[IOSI2C_HUB_DRIVER_PAYLOAD_SIZE];

} iosi2c_hub_driver_message_t;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void iosi2c_hub_driver_init(am_hal_ios_config_t *psIOSConfig);

extern void iosi2c_hub_driver_message_format(
                   iosi2c_hub_driver_message_t *psMessage, uint8_t *pBuffer);
extern void iosi2c_hub_driver_message_extract(
                   iosi2c_hub_driver_message_t *psMessage, uint8_t *pBuffer);

extern void iosi2c_hub_driver_fifo_service(uint32_t ui32Status);
extern void iosi2c_hub_driver_acc_service(uint8_t *pui8Destination,
                                            uint32_t ui32Status);
void iosi2c_hub_driver_release_inbound_hw_buffer(void);
extern void iosi2c_hub_driver_ios_int_service(void (*XmitMsgHandler)(void),
                                                uint32_t ui32Status);
extern void iosi2c_hub_driver_write(uint8_t *pui8Data, uint8_t ui8Size);
extern uint32_t iosi2c_hub_driver_read(uint8_t *pui8Data,
                                        uint32_t ui32MaxSize);
extern bool iosi2c_hub_driver_check(void);

#ifdef __cplusplus
}
#endif

#endif // IOSI2C_HUB_DRIVER_H
