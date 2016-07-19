//*****************************************************************************
//
//! @file uart2spi_fsm.c
//!
//! @brief Example that uses the UART to receive commands that control the SPI.
//!
//! This example accepts STXETX protocol packets from the buffered UART and 
//! turns them into SPI transactions on I/O Master 0.  In addition, it monitors
//! one GPIO line for interrupts from an Apollo's I/O slave.  It will also
//! handle the reset/SPICLK protocol to force a boot loader in to BL mode or
//! in to application mode. It is intended to run the HOST EMULATION side of a
//! pair of Apollo EVK boards. The other one is the sensor hub with boot 
//! loader installed.
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
// This is part of revision 1.1.0 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "uart2spi.h"
#include "uart2spi_fsm.h"

//*****************************************************************************
//
// Private work buffers for coding and decoding.
//
//*****************************************************************************
static uint32_t ui32WorkingBuffer[32];
static uint8_t ui32RxBuffer[256];

//*****************************************************************************
//
// Function to write to the SPI bus on IOM0.
//
//*****************************************************************************
static void
spi_write(int Length, uint8_t Offset, uint8_t *pui8Ptr)
{
    int i;
    uint8_t *pui8DstPtr = (uint8_t *)ui32WorkingBuffer;

    am_util_stdio_printf("\tSend this packet: Length = %d Offset = 0x%2.2x "
		         "first = 0x%2.2x\n", Length, Offset, *pui8Ptr);

    //
    // I/O Master transfers have to be 32-bit.
    //
    for(i=0; i<Length; i++) *pui8DstPtr++ = *pui8Ptr++;

    //
    // Now write the data to the spi bus.
    //
    am_hal_iom_spi_write(0, // I/O Master 0
		         5, // Chip Select
		         ui32WorkingBuffer,
                         Length,
                         AM_HAL_IOM_OFFSET(Offset));
}

//*****************************************************************************
//
// Function to read from SPI bus.
//
//*****************************************************************************
static void
spi_read(int Length, uint8_t Offset, uint8_t *pui8Ptr)
{

    uint8_t *pui8SrcPtr = (uint8_t *)ui32WorkingBuffer;

    am_util_stdio_printf("\tRead this packet: Length = %d Offset = 0x%2.2x\n",
		         Length,Offset);

    //
    // Prepare for a SPI read command.
    //
    am_hal_iom_enable(0);
    am_hal_iom_int_clear(0, AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);

    //
    // Do a SPI read transaction.
    //
    am_hal_iom_spi_read(0, // I/O Master 0
		        5, // Chip Select
		        ui32WorkingBuffer,
                        Length,
                        AM_HAL_IOM_OFFSET(Offset));

    //
    // Format the data and send it back to the host.
    //
    am_util_stxetx_tx(true, true, Length, &pui8SrcPtr);

}

//*****************************************************************************
//
// Function to reset this host emulation engine.
//
//*****************************************************************************
static void
host_emulation_reset(void)
{
    am_util_stdio_printf("Reset this Host Emulation Platform\n");
}

//*****************************************************************************
//
// Function to decode and process inbound packets.
//
//*****************************************************************************
void
uart2spi_fsm(int Length, uint8_t *pui8Packet)
{
    int i;
    eUART2SPIFSM_t packet_type = (eUART2SPIFSM_t) pui8Packet[1];
  

    am_util_stdio_printf("\n ==> uart2spi_fsm: PACKET Length = %d\n\t",Length);

    for(i=0; i<Length; i++)
    {
         am_util_stdio_printf(" 0x%2.2x",pui8Packet[i]);
         if( (i&7) == 7) am_util_stdio_printf("\n\t");
    }
    am_util_stdio_printf("\n");

    switch(packet_type)
    {
        case reset_host_emu:
            am_util_stdio_printf("\tuart2spi_fsm: reset host emulation\n");
            host_emulation_reset();
            break;
        case send_spi:
            am_util_stdio_printf("\tuart2spi_fsm: send_spi\n");
            spi_write((int) pui8Packet[0]-3,pui8Packet[2],&pui8Packet[3]);
            break;
        case read_spi:
            am_util_stdio_printf("\tuart2spi_fsm: read_spi\n");
            spi_read((int) pui8Packet[0]-3,pui8Packet[2],ui32RxBuffer);
            break;
        case reset_target:
            am_util_stdio_printf("\tuart2spi_fsm: reset target\n");
            target_reset(!!pui8Packet[2]);
            break;
        default:
            am_util_stdio_printf("ERROR: invalid packet type\n\n");
            break;
    }
}

