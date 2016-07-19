//*****************************************************************************
//
//! @file am_hal_ios.c
//!
//! @brief Functions for interfacing with the IO Slave module
//!
//! @addtogroup hal Hardware Abstraction Layer (HAL)
//! @addtogroup ios IO Slave (SPI/I2C)
//! @ingroup hal
//! @{
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

//*****************************************************************************
//
// SRAM Buffer structure
//
//*****************************************************************************
typedef struct
{
    volatile uint8_t *pui8Data;
    volatile uint32_t ui32WriteIndex;
    volatile uint32_t ui32ReadIndex;
    volatile uint32_t ui32Length;
    volatile uint32_t ui32Capacity;
}
am_hal_ios_buffer_t;

am_hal_ios_buffer_t g_sSRAMBuffer;

//*****************************************************************************
//
// Forward declarations of static funcitons.
//
//*****************************************************************************
static void am_hal_ios_buffer_init(am_hal_ios_buffer_t *psBuffer,
                                   void *pvArray, uint32_t ui32Bytes);

static bool am_hal_ios_buffer_write(am_hal_ios_buffer_t *psBuffer,
                                    void *pvSource, uint32_t ui32Bytes);

static bool am_hal_ios_buffer_read(am_hal_ios_buffer_t *psBuffer, void *pvDest,
                                   uint32_t ui32Bytes);

static void fifo_write(uint8_t *pui8Data, uint32_t ui32NumBytes);

//*****************************************************************************
//
// Function-like macros.
//
//*****************************************************************************
#define am_hal_ios_buffer_empty(psBuffer)                                   \
    ((psBuffer)->ui32Length == 0)

#define am_hal_ios_buffer_full(psBuffer)                                    \
    ((psBuffer)->ui32Length == (psBuffer)->ui32Capacity)

#define am_hal_ios_buffer_data_left(psBuffer)                               \
    ((psBuffer)->ui32Length)

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
volatile uint8_t * const am_hal_ios_pui8LRAM = (uint8_t *)REG_IOSLAVE_BASEADDR;
uint8_t *g_pui8FIFOBase = (uint8_t *) REG_IOSLAVE_BASEADDR;
uint8_t *g_pui8FIFOEnd = (uint8_t *) REG_IOSLAVE_BASEADDR;
uint8_t *g_pui8FIFOPtr = (uint8_t *) REG_IOSLAVE_BASEADDR;

//*****************************************************************************
//
//! @brief Return the space remaining in the buffer.
//!
//! This function returns the remaining space in the buffer.
//!
//! @note This function was originally implemented as a macro, thus it is
//!  declared as static.  It was converted to a function in order to resolve
//!  compiler warnings concerning order of volatile accesses.
//!
//! @return Space remaining in the buffer.
//
//*****************************************************************************
static uint32_t
am_hal_ios_buffer_space_left(am_hal_ios_buffer_t *psBuffer)
{
    uint32_t u32RetVal = psBuffer->ui32Capacity;
    u32RetVal -= psBuffer->ui32Length;
    return u32RetVal;
}


//*****************************************************************************
//
//! @brief Enables the IOS module
//!
//! This function enables the IOSLAVE module using the IFCEN bitfield in the
//! IOSLAVE_CFG register.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_enable(uint32_t ui32Module)
{
    AM_REGn(IOSLAVE, ui32Module, CFG) |= AM_REG_IOSLAVE_CFG_IFCEN(1);
}

//*****************************************************************************
//
//! @brief Disables the IOSLAVE module.
//!
//! This function disables the IOSLAVE module using the IFCEN bitfield in the
//! IOSLAVE_CFG register.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_disable(uint32_t ui32Module)
{
    AM_REGn(IOSLAVE, ui32Module, CFG) &= ~(AM_REG_IOSLAVE_CFG_IFCEN(1));
}

//*****************************************************************************
//
//! @brief Configure the IOS module.
//!
//! This function reads the an \e am_hal_ios_config_t structure and uses it to
//! set up the IO Slave module. Please see the information on the configuration
//! structure for more information on the parameters that may be set by this
//! function.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_config(am_hal_ios_config_t *psConfig)
{
    uint32_t ui32LRAMConfig;

    //
    // Record the FIFO parameters for later use.
    //
    g_pui8FIFOBase = (uint8_t *)(REG_IOSLAVE_BASEADDR + psConfig->ui32FIFOBase);
    g_pui8FIFOEnd = (uint8_t *)(REG_IOSLAVE_BASEADDR + psConfig->ui32RAMBase);

    //
    // Caluclate the value for the IO Slave FIFO configuration register.
    //
    ui32LRAMConfig = AM_REG_IOSLAVE_FIFOCFG_ROBASE(psConfig->ui32ROBase >> 3);
    ui32LRAMConfig |= AM_REG_IOSLAVE_FIFOCFG_FIFOBASE(psConfig->ui32FIFOBase >> 3);
    ui32LRAMConfig |= AM_REG_IOSLAVE_FIFOCFG_FIFOMAX(psConfig->ui32RAMBase >> 3);

    //
    // Just in case, disable the IOS
    //
    am_hal_ios_disable(0);

    //
    // Write the configuration register with the user's selected interface
    // characteristics.
    //
    AM_REG(IOSLAVE, CFG) = psConfig->ui32InterfaceSelect;

    //
    // Write the FIFO configuration register to set the memory map for the LRAM.
    //
    AM_REG(IOSLAVE, FIFOCFG) = ui32LRAMConfig;

    //
    // Enable the IOS. The following configuration options can't be set while
    // the IOS is disabled.
    //
    am_hal_ios_enable(0);

    //
    // Initialize the FIFO pointer to the beginning of the FIFO section.
    //
    am_hal_ios_fifo_ptr_set(psConfig->ui32FIFOBase);

    //
    // Write the FIFO threshold register.
    //
    AM_REG(IOSLAVE, FIFOTHR) = psConfig->ui32FIFOThreshold;
}

//*****************************************************************************
//
//! @brief Set bits in the HOST side IOINTCTL register.
//!
//! This function may be used to set an interrupt bit to the host.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_host_int_set(uint32_t ui32Interrupt)
{
    //
    // Set a bit that will cause an interrupt to the host.
    //
    AM_REG(IOSLAVE, IOINTCTL) = AM_REG_IOSLAVE_IOINTCTL_IOINTSET(ui32Interrupt);
}

//*****************************************************************************
//
//! @brief Clear bits in the HOST side IOINTCTL register.
//!
//! This function may be used to clear an interrupt bit to the host.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_host_int_clear(uint32_t ui32Interrupt)
{
    //
    // Clear bits that will cause an interrupt to the host.
    //
    AM_REG(IOSLAVE, IOINTCTL) = AM_REG_IOSLAVE_IOINTCTL_IOINTCLR(ui32Interrupt);
}

//*****************************************************************************
//
//! @brief Get the bits in the HOST side IOINTCTL register.
//!
//! This function may be used to read the host side interrupt bits.
//!
//! @return None.
//
//*****************************************************************************
uint32_t
am_hal_ios_host_int_get(void)
{
    //
    // return the value of the bits that will cause an interrupt to the host.
    //
    return AM_BFR(IOSLAVE, IOINTCTL, IOINT);
}

//*****************************************************************************
//
//! @brief Get the enable bits in the HOST side IOINTCTL register.
//!
//! This function may be used to read the host side interrupt bits.
//!
//! @return None.
//
//*****************************************************************************
uint32_t
am_hal_ios_host_int_enable_get(void)
{
    //
    // return the value of the bits that will cause an interrupt to the host.
    //
    return AM_BFR(IOSLAVE, IOINTCTL, IOINTEN);
}

//*****************************************************************************
//
//! @brief Enable an IOS Access Interrupt.
//!
//! This function may be used to enable an interrupt to the NVIC.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_access_int_enable(uint32_t ui32Interrupt)
{
    //
    // OR the desired interrupt into the enable register.
    //
    AM_REG(IOSLAVE, REGACCINTEN) |= ui32Interrupt;
}

//*****************************************************************************
//
//! @brief Return all enabled IOS Access Interrupts.
//!
//! This function may be used to return all enabled IOS Access interrupts.
//!
//! @return the enabled interrrupts.
//
//*****************************************************************************
uint32_t
am_hal_ios_access_int_enable_get(void)
{
    //
    // Return the enabled interrupts.
    //
    return AM_REG(IOSLAVE, REGACCINTEN);
}

//*****************************************************************************
//
//! @brief Disable an IOS Access Interrupt.
//!
//! This function may be used to disable an interrupt to the NVIC.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_access_int_disable(uint32_t ui32Interrupt)
{
    //
    // Clear the desired bit from the interrupt enable register.
    //
    AM_REG(IOSLAVE, REGACCINTEN) &= ~(ui32Interrupt);
}

//*****************************************************************************
//
//! @brief Clear an IOS Access Interrupt.
//!
//! This function may be used to clear an interrupt to the NVIC.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_access_int_clear(uint32_t ui32Interrupt)
{
    //
    // Use the interrupt clear register to deactivate the chosen interrupt.
    //
    AM_REG(IOSLAVE, REGACCINTCLR) = ui32Interrupt;
}

//*****************************************************************************
//
//! @brief Set an IOS Access Interrupt.
//!
//! This function may be used to set an interrupt to the NVIC.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_access_int_set(uint32_t ui32Interrupt)
{
    //
    // Use the interrupt clear register to deactivate the chosen interrupt.
    //
    AM_REG(IOSLAVE, REGACCINTSET) = ui32Interrupt;
}

//*****************************************************************************
//
//! @brief Check the status of an IOS Access Interrupt.
//!
//! @param bEnabledOnly - return only the enabled interrupt status.
//!
//! This function may be used to return the enabled interrupt status.
//!
//! @return the enabled interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_ios_access_int_status_get(bool bEnabledOnly)
{
    if (bEnabledOnly)
    {
        uint32_t u32RetVal = AM_REG(IOSLAVE, REGACCINTSTAT);
        return u32RetVal & AM_REG(IOSLAVE, REGACCINTEN);

    }
    else
    {
        return AM_REG(IOSLAVE, REGACCINTSTAT);
    }
}

//*****************************************************************************
//
//! @brief Enable an IOS Interrupt.
//!
//! @param ui32Interrupt - desired interrupts.
//!
//! This function may be used to enable an interrupt to the NVIC.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_int_enable(uint32_t ui32Interrupt)
{
    //
    // OR the desired interrupt into the enable register.
    //
    AM_REG(IOSLAVE, INTEN) |= ui32Interrupt;
}

//*****************************************************************************
//
//! @brief Return all enabled IOS Interrupts.
//!
//! This function may be used to return all enabled IOS interrupts.
//!
//! @return the enabled interrrupts.
//
//*****************************************************************************
uint32_t
am_hal_ios_int_enable_get(void)
{
    //
    // Return the enabled interrupts.
    //
    return AM_REG(IOSLAVE, INTEN);
}

//*****************************************************************************
//
//! @brief Disable an IOS Interrupt.
//!
//! @param ui32Interrupt - desired interrupts.
//!
//! This function may be used to disable an interrupt to the NVIC.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_int_disable(uint32_t ui32Interrupt)
{
    //
    // Clear the desired bit from the interrupt enable register.
    //
    AM_REG(IOSLAVE, INTEN) &= ~(ui32Interrupt);
}

//*****************************************************************************
//
//! @brief Clear an IOS Interrupt.
//!
//! @param ui32Interrupt - desired interrupts.
//!
//! This function may be used to clear an interrupt to the NVIC.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_int_clear(uint32_t ui32Interrupt)
{
    //
    // Use the interrupt clear register to deactivate the chosen interrupt.
    //
    AM_REG(IOSLAVE, INTCLR) = ui32Interrupt;
}

//*****************************************************************************
//
//! @brief Set an IOS Interrupt.
//!
//! @param ui32Interrupt - desired interrupts.
//!
//! This function may be used to set an interrupt to the NVIC.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_int_set(uint32_t ui32Interrupt)
{
    //
    // Use the interrupt clear register to deactivate the chosen interrupt.
    //
    AM_REG(IOSLAVE, INTSET) = ui32Interrupt;
}

//*****************************************************************************
//
//! @brief Write to the LRAM.
//!
//! @param ui32Offset - offset into the LRAM to write.
//! @param ui8Value - value to be written.
//!
//! This function writes ui8Value to offset ui32Offset inside the LRAM.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_lram_write(uint32_t ui32Offset, uint8_t ui8Value)
{
    //
    // Write the LRAM.
    //
    am_hal_ios_pui8LRAM[ui32Offset] = ui8Value;
}

//*****************************************************************************
//
//! @brief Read from the LRAM.
//!
//! @param ui32Offset - offset into the LRAM to read.
//!
//! This function read from offset ui32Offset inside the LRAM.
//!
//! @return the value at ui32Offset.
//
//*****************************************************************************
uint8_t
am_hal_ios_lram_read(uint32_t ui32Offset)
{
    //
    // Read the LRAM.
    //
    return am_hal_ios_pui8LRAM[ui32Offset];
}

//*****************************************************************************
//
//! @brief Check the status of an IOS Interrupt.
//!
//! @param bEnabledOnly - return only the enabled interrupt status.
//!
//! This function may be used to return the enabled interrupt status.
//!
//! @return the enabled interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_ios_int_status_get(bool bEnabledOnly)
{
    if (bEnabledOnly)
    {
        uint32_t u32RetVal = AM_REG(IOSLAVE, INTSTAT);
        return u32RetVal & AM_REG(IOSLAVE, INTEN);

    }
    else
    {
        return AM_REG(IOSLAVE, INTSTAT);
    }
}

//*****************************************************************************
//
//! @brief Check the amount of space left in the FIFO
//!
//! This function reads the IOSLAVE FIFOPTR register and determines the amount
//! of space left in the IOS LRAM FIFO.
//!
//! @return Bytes left in the IOS FIFO.
//
//*****************************************************************************
uint32_t
am_hal_ios_fifo_space_left(void)
{
    return (((uint32_t)g_pui8FIFOEnd - (uint32_t)g_pui8FIFOBase) -
            AM_BFR(IOSLAVE, FIFOPTR, FIFOSIZ));
}

//*****************************************************************************
//
// Helper function for managing IOS FIFO writes.
//
//*****************************************************************************
static void
fifo_write(uint8_t *pui8Data, uint32_t ui32NumBytes)
{
    while(ui32NumBytes)
    {
        //
        // Write the data to the FIFO
        //
        *g_pui8FIFOPtr++ = *pui8Data++;
        ui32NumBytes--;

        //
        // Make sure to wrap the FIFO pointer if necessary.
        //
        if(g_pui8FIFOPtr == g_pui8FIFOEnd)
        {
            g_pui8FIFOPtr = g_pui8FIFOBase;
        }
    }
}

//*****************************************************************************
//
//! @brief Transfer any available data from the IOS SRAM buffer to the FIFO.
//!
//! This function is meant to be called from an interrupt handler for the
//! ioslave module. It checks the IOS FIFO interrupt status for a threshold
//! event, and transfers data from an SRAM buffer into the IOS FIFO.
//!
//! @param ui32Status should be set to the ios interrupt status at the time of
//! ISR entry.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_fifo_service(uint32_t ui32Status)
{
    //
    // Check for FIFO size interrupts.
    //
    if(ui32Status & AM_HAL_IOS_INT_FSIZE)
    {
        //
        // While there is space in the FIFO....
        //
        while(am_hal_ios_fifo_space_left())
        {
            if(am_hal_ios_buffer_empty(&g_sSRAMBuffer))
            {
                //
                // If we run out of data to copy from SRAM to FIFO, we can
                // stop.
                //
                break;
            }

            //
            // Copy data from the ring buffer to the FIFO.
            //
            am_hal_ios_buffer_read(&g_sSRAMBuffer, g_pui8FIFOPtr, 1);

            //
            // Update the pointer
            //
            g_pui8FIFOPtr++;

            //
            // Make sure to wrap the FIFO pointer if necessary.
            //
            if(g_pui8FIFOPtr == g_pui8FIFOEnd)
            {
                g_pui8FIFOPtr = g_pui8FIFOBase;
            }
        }
    }
}

//*****************************************************************************
//
//! @brief Writes the specified number of bytes to the IOS fifo.
//!
//! @param pui8Data is a pointer to the data to be written to the fifo.
//! @param ui32NumBytes is the number of bytes to send.
//!
//! This function will write data from the caller-provided array to the IOS
//! LRAM FIFO. If there is no space in the LRAM FIFO, the data will be copied
//! to a temporary SRAM buffer instead.
//!
//! The maximum message size for the IO Slave is 1024 bytes.
//!
//! @note In order for SRAM copy operations in the function to work correctly,
//! the \e am_hal_ios_buffer_service() function must be called in the ISR for
//! the ioslave module.
//!
//! @return
//
//*****************************************************************************
void
am_hal_ios_fifo_write(uint8_t *pui8Data, uint32_t ui32NumBytes)
{
    uint32_t ui32FIFOSpace;
    uint32_t ui32SRAMSpace;

    //
    // This operation will only work properly if an SRAM buffer has been
    // allocated. Make sure that am_hal_ios_fifo_buffer_init() has been called,
    // and the buffer pointer looks valid.
    //
    am_hal_debug_assert(g_sSRAMBuffer.pui8Data != 0);

    //
    // Check the FIFO and the SRAM buffer to see where we have space.
    //
    ui32FIFOSpace = am_hal_ios_fifo_space_left();
    ui32SRAMSpace = am_hal_ios_buffer_space_left(&g_sSRAMBuffer);

    //
    // If the SRAM buffer is empty, we should just write directly to the FIFO.
    //
    if(am_hal_ios_buffer_empty(&g_sSRAMBuffer))
    {
        //
        // If the whole message fits, send it now.
        //
        if(ui32NumBytes <= ui32FIFOSpace)
        {
            fifo_write(pui8Data, ui32NumBytes);
            return;
        }
        else
        {
            fifo_write(pui8Data, ui32FIFOSpace);
            ui32NumBytes -= ui32FIFOSpace;
            pui8Data += ui32FIFOSpace;
        };
    }

    //
    // If there's still data, write it to the SRAM buffer.
    //
    if(ui32NumBytes)
    {
        //
        // Make sure that the data will fit inside the SRAM buffer.
        //
        if(ui32SRAMSpace >= ui32NumBytes)
        {
            //
            // If the data will fit, write it to the SRAM buffer.
            //
            am_hal_ios_buffer_write(&g_sSRAMBuffer, pui8Data, ui32NumBytes);
        }
        else
        {
            //
            // We've overflowed the SRAM buffer. Either the SRAM buffer isn't
            // big enough, or there's a data throughput issue somewhere in the
            // chain.
            //
            am_hal_debug_assert_msg(0, "The SRAM buffer wasn't big enough to"
                                       "hold the requested IOS FIFO transfer.");
        }
    }

    //
    // Tell the FIFO that there is data ready to send.
    //
    AM_REG(IOSLAVE, FIFOINC) = ui32NumBytes;
}

//*****************************************************************************
//
//! @brief Writes the specified number of bytes to the IOS fifo simply.
//!
//! @param pui8Data is a pointer to the data to be written to the fifo.
//! @param ui32NumBytes is the number of bytes to send.
//!
//! This function will write data from the caller-provided array to the IOS
//! LRAM FIFO. This simple routine does not use SRAM buffering for large
//! messages.
//!
//! The maximum message size for the IO Slave is 128 bytes.
//!
//! @note Do note call the \e am_hal_ios_buffer_service() function in the ISR for
//! the ioslave module.
//!
//! @return
//
//*****************************************************************************
void
am_hal_ios_fifo_write_simple(uint8_t *pui8Data, uint32_t ui32NumBytes)
{
    uint32_t ui32FIFOSpace;

    //
    // Check the FIFO and the SRAM buffer to see where we have space.
    //
    ui32FIFOSpace = am_hal_ios_fifo_space_left();

    //
    // If the whole message fits, send it now.
    //
    if(ui32NumBytes <= ui32FIFOSpace)
    {
        fifo_write(pui8Data, ui32NumBytes);
        return;
    }
    else
    {
        //
        // The message didn't fit. Try using am_hal_ios_fifo_write() instead.
        //
        am_hal_debug_assert_msg(0, "The requested IOS transfer didn't fit in"
                                   "the LRAM FIFO. Try using am_hal_ios_fifo_write().");
    }
}

//*****************************************************************************
//
//! @brief Sets the IOS FIFO pointer to the specified LRAM offset.
//!
//! @param ui32Offset is LRAM offset to set the FIFO pointer to.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_fifo_ptr_set(uint32_t ui32Offset)
{
    //
    // Set the FIFO Update bit.
    //
    AM_REG(IOSLAVE, FUPD) = 0x1;

    //
    // Change the FIFO offset.
    //
    AM_REG(IOSLAVE, FIFOPTR) = ui32Offset;

    //
    // Clear the FIFO update bit.
    //
    AM_REG(IOSLAVE, FUPD) = 0x0;

    //
    // Set the global FIFO-pointer tracking variable.
    //
    g_pui8FIFOPtr = (uint8_t *) (REG_IOSLAVE_BASEADDR + ui32Offset);
}

//*****************************************************************************
//
// Initialize an SRAM buffer for use with the IO Slave.
//
//*****************************************************************************
static void
am_hal_ios_buffer_init(am_hal_ios_buffer_t *psBuffer, void *pvArray,
                       uint32_t ui32Bytes)
{
    psBuffer->ui32WriteIndex = 0;
    psBuffer->ui32ReadIndex = 0;
    psBuffer->ui32Length = 0;
    psBuffer->ui32Capacity = ui32Bytes;
    psBuffer->pui8Data = (uint8_t *)pvArray;
}

//*****************************************************************************
//
// Write data into the IOS SRAM buffer.
//
//*****************************************************************************
static bool
am_hal_ios_buffer_write(am_hal_ios_buffer_t *psBuffer, void *pvSource,
                        uint32_t ui32Bytes)
{
    uint32_t i;
    uint8_t *pui8Source;

    pui8Source = (uint8_t *) pvSource;

    //
    // Check to make sure that the buffer isn't already full
    //
    if(am_hal_ios_buffer_space_left(psBuffer) >= ui32Bytes)
    {
        //
        // Loop over the bytes in the source array.
        //
        for(i = 0; i < ui32Bytes; i++)
        {
            uint32_t ui32WrIdx, ui32Cap;

            //
            // Write the value to the buffer.
            //
            psBuffer->pui8Data[psBuffer->ui32WriteIndex] = pui8Source[i];

            //
            // Advance the write index, making sure to wrap if necessary.
            //
            ui32WrIdx = psBuffer->ui32WriteIndex + 1;
            ui32Cap   = psBuffer->ui32Capacity;
            psBuffer->ui32WriteIndex = ui32WrIdx % ui32Cap;
        }

        //
        // Update the length value appropriately.
        //
        psBuffer->ui32Length += ui32Bytes;

        //
        // Report a success.
        //
        return true;
    }
    else
    {
        //
        // The ring buffer can't fit the amount of data requested. Return a
        // failure.
        //
        return false;
    }
}

//*****************************************************************************
//
// Read data back from the IOS SRAM buffer.
//
//*****************************************************************************
static bool
am_hal_ios_buffer_read(am_hal_ios_buffer_t *psBuffer, void *pvDest,
                       uint32_t ui32Bytes)
{
    uint32_t i;
    uint8_t *pui8Dest;

    pui8Dest = (uint8_t *) pvDest;

    //
    // Check to make sure that the buffer isn't empty
    //
    if(am_hal_ios_buffer_data_left(psBuffer) >= ui32Bytes)
    {
        //
        // Loop over the bytes in the destination array.
        //
        for(i = 0; i < ui32Bytes; i++)
        {
            uint32_t ui32RdIdx, ui32Cap;

            //
            // Grab the next value from the buffer.
            //
            pui8Dest[i] = psBuffer->pui8Data[psBuffer->ui32ReadIndex];

            //
            // Advance the read index, wrapping if needed.
            //
            ui32RdIdx = psBuffer->ui32ReadIndex + 1;
            ui32Cap   = psBuffer->ui32Capacity;
            psBuffer->ui32ReadIndex = ui32RdIdx % ui32Cap;
        }

        //
        // Adjust the length value to reflect the change.
        //
        psBuffer->ui32Length--;

        //
        // Report a success.
        //
        return true;
    }
    else
    {
        //
        // If the buffer didn't have enough data, just return a zero.
        //
        return false;
    }
}

//*****************************************************************************
//
//! @brief Poll for all host side read activity to complete.
//!
//! Poll for all host side read activity to complete. Use this before
//! calling am_hal_ios_fifo_write_simple().
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_read_poll_complete(void)
{
    while(AM_REG(IOSLAVE, FUPD) & AM_REG_IOSLAVE_FUPD_IOREAD_M);
}

//*****************************************************************************
//
//! @brief Initializes an SRAM buffer for the IOS FIFO.
//!
//! @param pui8Buffer is the SRAM buffer that will be used for IOS fifo data.
//! @param ui32BufferSize is the size of the SRAM buffer.
//!
//! This function provides the IOS HAL functions with working memory for
//! managing outgoing IOS FIFO transactions. It needs to be called at least
//! once before am_hal_ios_fifo_write() may be used.
//!
//! The recommended buffer size for the IOS FIFO is 1024 bytes.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_ios_fifo_buffer_init(uint8_t *pui8Buffer, uint32_t ui32NumBytes)
{
    //
    // Initialize the global SRAM buffer.
    //
    am_hal_ios_buffer_init(&g_sSRAMBuffer, pui8Buffer, ui32NumBytes);
}

//*****************************************************************************
//
//  End the doxygen group
//! @}
//
//*****************************************************************************
