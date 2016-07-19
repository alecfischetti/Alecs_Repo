//*****************************************************************************
//
//! @file am_hal_i2c_bit_bang.c
//!
//! @brief I2C bit bang module.
//!
//! These functions implement the I2C bit bang utility
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
#include "am_util.h"
#include "am_hal_i2c_bit_bang.h"

//#define I2C_BB_DEBUG

//*****************************************************************************
//
// How many microseconds in a half bit cell or quarter bit cell
//
//*****************************************************************************
#define HALF_BIT_TIME_IN_USECS (2)
#define QUARTER_BIT_TIME_IN_USECS (1)

//*****************************************************************************
//
// I2C Bit Bang Private Data Structure
//
//*****************************************************************************
typedef struct am_util_bit_bang_priv
{
    bool  start_flag;
    uint32_t sck_gpio_number;
    uint32_t sda_gpio_number;


} am_hal_i2c_bit_bang_priv_t;
static am_hal_i2c_bit_bang_priv_t am_hal_i2c_bit_bang_priv;

//*****************************************************************************
//
//! @brief Initialize i2c bit bang private data structure
//!
//! @param sck_gpio_number is the GPIO # for the I2C SCK clock pin
//! @param sda_gpio_number is the GPIO # for the I2C SDA data pin
//!
//! This function initializes the I2C bit bang utility's internal data struct.
//!
//! returns nothing
//
//*****************************************************************************
void
am_hal_i2c_bit_bang_init(uint32_t sck_gpio_number,
                         uint32_t sda_gpio_number)
{
    //
    // remember GPIO pin assignments for I2C bus signals
    //
    am_hal_i2c_bit_bang_priv.sck_gpio_number = sck_gpio_number;
    am_hal_i2c_bit_bang_priv.sda_gpio_number = sda_gpio_number;

    //
    // Set up SCK GPIO configuration bidirection, input
    //
    am_hal_gpio_pin_config(sck_gpio_number, AM_HAL_PIN_OPENDRAIN);

    //
    // Set SCK GPIO data bit high so we aren't pulling down the clock
    //
    am_hal_gpio_out_bit_set(sck_gpio_number);

    //
    // Set up SCK GPIO configuration bidirection, input
    //
    am_hal_gpio_pin_config(sda_gpio_number, AM_HAL_PIN_OPENDRAIN);

    //
    // Set SDA GPIO data bit high so we aren't pulling down the data line
    //
    am_hal_gpio_out_bit_set(sda_gpio_number);

    //
    // Wait for the pin state to settle down
    //
    am_util_delay_ms(10);
}

//*****************************************************************************
//
//! @brief Receive one data byte from an I2C device
//!
//!
//! This function handles sending one byte to a slave device
//!
//! returns the byte received
//
//*****************************************************************************
static uint8_t
i2c_receive_byte(void)
{
    int i;
    uint8_t data_byte = 0;

    //
    // Loop through receiving 8 bits
    //
    for (i = 0; i < 8; i++)
    {
        //
        // Pull down on clock line
        //
        am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sck_gpio_number);

        //
        // Delay for 1/4 bit cell time to start the clock
        //
        am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

        //
        // release the data line from from the previous ACK
        //
        am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sda_gpio_number);

        //
        // Release the clock line
        //
        am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sck_gpio_number);

        //
        // Wait for any stretched clock to go high
        //
        while( !am_hal_gpio_input_bit_read(am_hal_i2c_bit_bang_priv.sck_gpio_number) );

        //
        // grab the data bit here
        //
        if(am_hal_gpio_input_bit_read(am_hal_i2c_bit_bang_priv.sda_gpio_number))
        {
            //
            // set the bit in the data byte to be returned
            //
            data_byte |=  (0x80 >> i);
        }

        //
        // Delay for 1/2 bit cell time while clock is high
        //
        am_util_delay_us(HALF_BIT_TIME_IN_USECS);
    }

    //
    // Pull down on clock line
    //
    am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Delay for 1/4 bit cell time before sending ACK to device
    //
    am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

    //
    // pull the data line down so we can ACK the byte we just received
    //
    am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Delay for 1/4 bit cell time before sending ACK to device
    //
    am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

    //
    // Release the clock line
    //
    am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Wait for any stretched clock to go high
    //
    while( !am_hal_gpio_input_bit_read(am_hal_i2c_bit_bang_priv.sck_gpio_number) );

    //
    // Give the received data byte back to them
    //
    return data_byte;
}

//*****************************************************************************
//
//! @brief Send one data bytes to an I2C device
//!
//! @param one_byte the byte to send, could be address could be data
//!
//! This function handles sending one byte to a slave device
//!
//! returns I2C BB ENUM
//!     {
//!     AM_HAL_I2C_BIT_BANG_SUCCESS,
//!     AM_HAL_I2C_BIT_BANG_ADDRESS_NAKED
//!     }
//
//*****************************************************************************
static am_hal_i2c_bit_bang_enum_t
i2c_send_byte(uint8_t one_byte)
{
    int i;

    //
    // Loop through sending 8 bits
    //
    for (i = 0; i < 8; i++)
    {
        //
        // Pull down on clock line
        //
        am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sck_gpio_number);

        //
        // Delay for 1/4 bit cell time to start the clock
        //
        am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

        //
        // output the next data bit
        //
        if(one_byte & (0x80 >> i))
        {
            am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sda_gpio_number);
        }
        else
        {
            am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sda_gpio_number);
        }

        //
        // Delay for 1/4 bit cell time to start the clock
        //
        am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

        //
        // Release the clock line
        //
        am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sck_gpio_number);

        //
        // Wait for any stretched clock to go high
        //
        while( !am_hal_gpio_input_bit_read(am_hal_i2c_bit_bang_priv.sck_gpio_number) );

        //
        // Delay for 1/2 bit cell time while clock is high
        //
        am_util_delay_us(HALF_BIT_TIME_IN_USECS);
    }

    //
    // Pull down on clock line
    //
    am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Delay for 1/2 bit cell time to start the clock
    //
    am_util_delay_us(HALF_BIT_TIME_IN_USECS);

    //
    // Release the clock line
    //
    am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Wait for any stretched clock to go high
    //
    while( !am_hal_gpio_input_bit_read(am_hal_i2c_bit_bang_priv.sck_gpio_number) );

    //
    // Grab the state of the ACK bit and return it
    //
    if(am_hal_gpio_input_bit_read(am_hal_i2c_bit_bang_priv.sda_gpio_number))
    {
#ifdef I2C_BB_DEBUG
        return AM_HAL_I2C_BIT_BANG_SUCCESS;
#else
        return AM_HAL_I2C_BIT_BANG_ADDRESS_NAKED;
#endif
    }
    else
    {
        return AM_HAL_I2C_BIT_BANG_SUCCESS;
    }
}

//*****************************************************************************
//
//! @brief Receuve a string of data bytes from an I2C device
//!
//! @param address (only 8 bit I2C addresses are supported)
//!        LSB is I2C R/W
//! @param number_of_bytes to transfer (# payload bytes)
//! @param pData pointer to data buffer to receive payload
//!
//! This function handles receiving a payload from a slave device
//!
//! returns ENUM{AM_HAL_I2C_BIT_BANG_SUCCESS,AM_HAL_I2C_BIT_BANG_ADDRESS_NAKED}
//
//*****************************************************************************
am_hal_i2c_bit_bang_enum_t
am_hal_i2c_bit_bang_receive(uint8_t address, uint32_t number_of_bytes,
                            uint8_t *pData, uint8_t ui8Offset,
                            bool bUseOffset)
{
    uint32_t ui32I;

    //
    // Pull down on data line with clock high --> START CONDITION
    //
    am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sda_gpio_number);

    //
    // Delay for 1/4 bit cell time to start the clock
    //
    am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

    //
    // send the address byte and wait for the ACK/NAK
    //
    if(i2c_send_byte(address) != AM_HAL_I2C_BIT_BANG_SUCCESS)
    {
        return AM_HAL_I2C_BIT_BANG_ADDRESS_NAKED;
    }

    if(bUseOffset)
    {
        if(i2c_send_byte(ui8Offset) != AM_HAL_I2C_BIT_BANG_SUCCESS)
        {
            return AM_HAL_I2C_BIT_BANG_DATA_NAKED;
        }
    }

    //
    // receive the requested number of data bytes
    //
    for (ui32I = 0; ui32I < number_of_bytes; ui32I++)
    {
        //
        // receive the data bytes and send ACK for each one
        //
        *pData++ = i2c_receive_byte();
    }

    //********************
    // Send stop condition
    //********************

    //
    // Pull down on clock line
    //
    am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Delay for 1/4 bit cell time
    //
    am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

    //
    // Pull down on data line with clock low
    //
    am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sda_gpio_number);

    //
    // Delay for 1/4 bit cell time
    //
    am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

    //
    // Release the clock line
    //
    am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Wait for any stretched clock to go high
    //
    while( !am_hal_gpio_input_bit_read(am_hal_i2c_bit_bang_priv.sck_gpio_number) );

    //
    // Delay for 1/2 bit cell time while clock is high
    //
    am_util_delay_us(HALF_BIT_TIME_IN_USECS);

    //
    // release data line with clock high --> STOP CONDITION
    //
    am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sda_gpio_number);

    //
    // mesage successfully received (how could we fail???)
    //
    return AM_HAL_I2C_BIT_BANG_SUCCESS;
}

//*****************************************************************************
//
//! @brief Send a string of data bytes to an I2C device
//!
//! @param address (only 8 bit I2C addresses are supported)
//!        LSB is I2C R/W
//! @param number_of_bytes to transfer (# payload bytes)
//! @param pData pointer to data buffer containing payload
//!
//! This function handles sending a payload to a slave device
//!
//! returns ENUM {AM_HAL_I2C_BIT_BANG_SUCCESS, AM_HAL_I2C_BIT_BANG_DATA_NAKED,
//!               AM_HAL_I2C_BIT_BANG_ADDRESS_NAKED}
//
//*****************************************************************************
am_hal_i2c_bit_bang_enum_t
am_hal_i2c_bit_bang_send(uint8_t address, uint32_t number_of_bytes,
                         uint8_t *pData, uint8_t ui8Offset,
                         bool bUseOffset)
{
    uint32_t ui32I;
    am_hal_i2c_bit_bang_enum_t eRC;
    bool data_naked = false;

    //
    // Pull down on data line with clock high --> START CONDITION
    //
    am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sda_gpio_number);

    //
    // Delay for 1/4 bit cell time to start the clock
    //
    am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

    //
    // send the address byte and wait for the ACK/NAK
    //
    if(i2c_send_byte(address) != AM_HAL_I2C_BIT_BANG_SUCCESS)
    {
        return AM_HAL_I2C_BIT_BANG_ADDRESS_NAKED;
    }

    if(bUseOffset)
    {
        if(i2c_send_byte(ui8Offset) != AM_HAL_I2C_BIT_BANG_SUCCESS)
        {
            return AM_HAL_I2C_BIT_BANG_DATA_NAKED;
        }
    }

    //
    // send the requested number of data bytes
    //
    for (ui32I = 0; ui32I < number_of_bytes; ui32I++)
    {
        //
        // send out the data bytes while watching for premature NAK
        //
        eRC =  i2c_send_byte(*pData++);
        if(    (eRC   != AM_HAL_I2C_BIT_BANG_SUCCESS)
               && (ui32I != (number_of_bytes-1))
          )
        {
            data_naked = true;
            break;
        }
    }

    //********************
    // Send stop condition
    //********************

    //
    // Pull down on clock line
    //
    am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Delay for 1/4 bit cell time
    //
    am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

    //
    // Pull down on data line with clock low
    //
    am_hal_gpio_out_bit_clear(am_hal_i2c_bit_bang_priv.sda_gpio_number);

    //
    // Delay for 1/4 bit cell time
    //
    am_util_delay_us(QUARTER_BIT_TIME_IN_USECS);

    //
    // Release the clock line
    //
    am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sck_gpio_number);

    //
    // Wait for any stretched clock to go high
    //
    while( !am_hal_gpio_input_bit_read(am_hal_i2c_bit_bang_priv.sck_gpio_number) );

    //
    // Delay for 1/2 bit cell time while clock is high
    //
    am_util_delay_us(HALF_BIT_TIME_IN_USECS);

    //
    // release data line with clock high --> STOP CONDITION
    //
    am_hal_gpio_out_bit_set(am_hal_i2c_bit_bang_priv.sda_gpio_number);

    if(data_naked)
    {
        return AM_HAL_I2C_BIT_BANG_DATA_NAKED;  // if it happens early
    }

    //
    // mesage successfully sent
    //
    return AM_HAL_I2C_BIT_BANG_SUCCESS;
}
