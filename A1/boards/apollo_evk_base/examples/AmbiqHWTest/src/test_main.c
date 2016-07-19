//*****************************************************************************
//
//! @file test_main.c
//!
//! @brief Has functions to test peripherals on Ambiq's Apollo board
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
#include "test_main.h"

//*****************************************************************************
//
// Insert compiler version at compile time.
//
//*****************************************************************************
#define STRINGIZE_VAL(n)                    STRINGIZE_VAL2(n)
#define STRINGIZE_VAL2(n)                   #n

//*****************************************************************************
//
// UART configuration settings.
//
//*****************************************************************************
am_hal_uart_config_t g_sUartConfig =
{
    .ui32BaudRate = 115200,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .bTwoStopBits = false,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32FlowCtrl = AM_HAL_UART_FLOW_CTRL_NONE,
};

//*****************************************************************************
//
// Initialize the UART
//
//*****************************************************************************
void
uart_init(void)
{
    //
    // Make sure the UART RX and TX pins are enabled.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);

    //
    // Start the UART interface, apply the desired configuration settings, and
    // enable the FIFOs.
    //
    am_hal_uart_clock_enable();

    //
    // Disable the UART before configuring it.
    //
    am_hal_uart_disable();

    //
    // Configure the UART.
    //
    am_hal_uart_config(&g_sUartConfig);

    //
    // Enable the UART FIFO.
    //
    am_hal_uart_fifo_config(AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

    //
    // Enable the UART.
    //
    am_hal_uart_enable();
}

//*****************************************************************************
//
// Disable the UART
//
//*****************************************************************************
void
uart_disable(void)
{
      //
      // Clear all interrupts before sleeping as having a pending UART interrupt
      // burns power.
      //
      am_hal_uart_int_clear(0xFFFFFFFF);

      //
      // Disable the UART.
      //
      am_hal_uart_disable();

      //
      // Disable the UART pins.
      //
      am_bsp_pin_disable(COM_UART_TX);
      am_bsp_pin_disable(COM_UART_RX);

      //
      // Disable the UART clock.
      //
      am_hal_uart_clock_disable();
}

//*****************************************************************************
//
// Transmit delay waits for busy bit to clear to allow
// for a transmission to fully complete before proceeding.
//
//*****************************************************************************
void
uart_transmit_delay(void)
{
  //
  // Wait until busy bit clears to make sure UART fully transmitted last byte
  //
  while ( am_hal_uart_flags_get() & AM_HAL_UART_FR_BUSY );
}

void print_test_menu(void);

char rxBuffer[128];

void print_test_menu(void)
{
   am_util_stdio_terminal_clear();
   am_util_stdio_printf("Ambiq Hardware/Software Test Menu \n\n");
   uart_transmit_delay();
   am_util_stdio_printf("Select Test by entering a number \n");
   uart_transmit_delay();
   am_util_stdio_printf("1. Gpio \n");
   uart_transmit_delay();
   am_util_stdio_printf("2. I2c \n");
   uart_transmit_delay();
   am_util_stdio_printf("3. Spi \n");
   uart_transmit_delay();
   am_util_stdio_printf("4. Uart \n");
   uart_transmit_delay();
   am_util_stdio_printf("5. Isr \n");
   uart_transmit_delay();
   am_util_stdio_printf("7. Timer \n");
   uart_transmit_delay();
   am_util_stdio_printf("8. Floating point \n");
   uart_transmit_delay();
   am_util_stdio_printf("9. Throughput With Characteristic \n");
   uart_transmit_delay();
   am_util_stdio_printf("10. Throughput With Notification \n");
   uart_transmit_delay();
   uart_transmit_delay();

}


//*****************************************************************************
//
// Read from the UART for get the command
//
//*****************************************************************************
uint8_t readCmd(void)
{
   uint8_t cmd = 0;
   am_hal_uart_line_receive_polled((uint32_t)8, &rxBuffer[0]);
   am_util_stdio_printf("Val: %s\n", rxBuffer);

   return cmd;
}

void processCmd(uint8_t TestCommands)
{
    switch(TestCommand1)
    {
        case CMD_TEST_GPIO:

        break;

        case CMD_TEST_I2C:
   

        break;


        case CMD_TEST_SPI:


        break;

        case CMD_TEST_UART:

        break;


        case CMD_TEST_ISR:


        break;


        case CMD_TEST_TIMER:

        break;


        case CMD_TEST_FLOATING_POINT:

        break;


        case CMD_TEST_THROUGHPUT_NOTFI:

        break;

    }
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
	uint8_t TestCommands = 0;
   // am_hal_mcuctrl_device_t  mcu_dev;

    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Initialize the BSP.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for UART output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t)
      am_hal_uart_string_transmit_polled);

    //
    // Configure and enable the UART.
    //
    uart_init();

    print_test_menu();

    //
    // Loop forever while sleeping.
    //
    while (1)
    {

        // reading from uart to run a test
    	TestCommands = readCmd();
        processCmd(TestCommands);
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
