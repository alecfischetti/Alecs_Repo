//*****************************************************************************
//
//! @file uart2spi.c
//!
//! @brief Example that uses the UART receive commands that are driven out SPI.
//!
//! This example accepts STXETX protocol packets from the buffered UART and 
//! turns them into SPI transactions on I/O Master 0.  In addition, it monitors
//! one GPIO line for interrupts from an Apollo's I/O slave.  It will also
//! handle the reset/SPICLK protocol to force a boot loader in to BL mode or
//! in to application mode. It is intended to run the HOST EMULATION side of a
//! pair of Apollo EVK boards. The other one is the sensor hub with boot 
//! loader installed.
//!
//!    UART2SPI EVK                     BOOTLOADER EVK
//!  GPIO[5]  IOM SPI CLK            GPIO[0]  IOS SPI CLK  (also OVERRIDE PIN)
//!  GPIO[6]  IOM SPI MISO           GPIO[1]  IOS SPI MISO
//!  GPIO[7]  IOM SPI MOSI           GPIO[2]  IOS SPI MOSI
//!  GPIO[4]  IOM SPI M0nCE5         GPIO[3]  IOS SPI CSn
//!  GPIO[0]  IOM GPIO               GPIO[4]  IOS Interrupt
//!  GPIO[1]  IOM GPIO               RSTn     Reset pin
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

#include "uart2spi_fsm.h"



//*****************************************************************************
//
// GPIO pin rising and falling interrupt markers.
//
//*****************************************************************************
bool g_bGPIORising  = true;  // true == rising detected.
bool g_bGPIOFalling = false;  // true == falling detected.
bool g_bGPIODirection = true; // true == rising expected.



//*****************************************************************************
//
// UART buffers for talking to the PC over the UART.
//
//*****************************************************************************
uint8_t g_pui8TxArray[1024];
uint8_t g_pui8RxArray[1024];

//*****************************************************************************
//
// Flags to alert the main application of various interrupt conditions.
//
//*****************************************************************************
volatile bool g_bUARTIdle = true;


//*****************************************************************************
//
// Receive and depacketized buffer.
//
//*****************************************************************************
uint8_t g_ui8RcvBuffer[512];

//*****************************************************************************
//
// Configure the necessary pins for communication with the PC via the UART.
//
//*****************************************************************************
void
configure_uart_pins(void)
{
    //
    // Enable the necessary pins for talking to the PC via the UART.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);
    am_bsp_pin_enable(COM_UART_RTS);
    am_bsp_pin_enable(COM_UART_CTS);
}

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
    .ui32FlowCtrl = AM_HAL_UART_FLOW_CTRL_NONE
};

//*****************************************************************************
//
// Configure the necessary pins for communication with the radio.
//
//*****************************************************************************
void
configure_uart(void)
{
    //
    // Enable the clock to the UART.
    //
    am_hal_uart_clock_enable();

    //
    // Disable and configure the UART.
    //
    am_hal_uart_disable();
    am_hal_uart_config(&g_sUartConfig);
    am_hal_uart_fifo_config(AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

    //
    // Enable the UART.
    //
    am_hal_uart_enable();
}

//*****************************************************************************
//
// Enable Buffered UART Communication.
//
//*****************************************************************************
void
uart_enable(void)
{
    //
    // Set the global variable to indicate that the UART is on.
    //
    g_bUARTIdle = false;

    //
    // Enable the UART pins.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);
    am_bsp_pin_enable(COM_UART_RTS);
    am_bsp_pin_enable(COM_UART_CTS);

    //
    // Enable the clock to the UART.
    //
    am_hal_uart_clock_enable();
    am_hal_uart_config(&g_sUartConfig);
    am_hal_uart_enable();
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_UART);
    am_hal_uart_int_enable(AM_HAL_UART_INT_TX);
    am_hal_uart_int_enable(AM_HAL_UART_INT_RX);
    am_hal_uart_int_enable(AM_HAL_UART_INT_RX_TMOUT);
}

//*****************************************************************************
//
// Disable the Buffered UART.
//
//*****************************************************************************
void
uart_disable(void)
{
    //
    // Set the global variable to indicate that the UART is off.
    //
    g_bUARTIdle = true;

    //
    // Wait for the UART lines to stop toggling, and make sure we have
    // absolutely no interrupts pending.
    //
    while (am_hal_uart_flags_get() & AM_REG_UART_FR_BUSY_BUSY);
    am_hal_uart_int_clear(0xFFFFFFFF);

    //
    // Disable the UART pins.
    //
    am_bsp_pin_disable(COM_UART_TX);
    am_bsp_pin_disable(COM_UART_RX);
    am_bsp_pin_disable(COM_UART_RTS);
    am_bsp_pin_disable(COM_UART_CTS);

    //
    // Disable the clock to the UART.
    //
    am_hal_uart_disable();
    am_hal_uart_clock_disable();
}


//*****************************************************************************
//
// Interrupt handler for the UART
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t ui32Status;

    //
    // Read the masked interrupt status from the UART.
    //
    ui32Status = am_hal_uart_int_status_get(true);

    //
    // Service the buffered UART.
    //
    am_hal_uart_service_buffered(ui32Status);

    //
    // Clear the UART interrupts.
    //
    am_hal_uart_int_clear(ui32Status);

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

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
const am_hal_iom_config_t g_sIOMConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_100KHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 0,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// Configure GPIOs for this example
//
//*****************************************************************************
void
configure_pins(void)
{
    //
    // Configure I/O Master 0 as SPI
    //
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCK);
    am_hal_gpio_pin_config(6, AM_HAL_PIN_6_M0SDA);
    am_hal_gpio_pin_config(7, AM_HAL_PIN_7_M0MOSI);
    am_hal_gpio_pin_config(4, AM_HAL_PIN_4_M0nCE5);

    //
    // Configure the reset pin for the target
    //
    am_hal_gpio_out_bit_set(1);
    am_hal_gpio_pin_config(1, AM_HAL_PIN_OUTPUT);

    //
    // Configure the I/O Slave interrupt pin.
    // Rising and falling edges will be reported to host.
    //
    am_hal_gpio_pin_config(0, AM_HAL_PIN_INPUT);

    //
    // Enable a GPIO interrupt for positive edges on GPIO[0] which is 
    // connected to the target's I/O Slave interrupt pin.
    //
    am_hal_gpio_int_polarity_bit_set(0,AM_HAL_GPIO_RISING);
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(0));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(0));
}

//*****************************************************************************
//
// Configure IOM pins for reset
//
//*****************************************************************************
void
target_reset(bool bEnterBootloader)
{
    //
    // Set I/O Master 0 SPI Clock pin GPIO gpio state.
    //
    if(bEnterBootloader) am_hal_gpio_out_bit_set(5);
    else                 am_hal_gpio_out_bit_clear(5);

    //
    // Configure I/O Master 0 SPI Clock as GPIO output.
    //
    am_hal_gpio_pin_config(5, AM_HAL_PIN_OUTPUT);

    //
    // Assert reset to the target.
    //
    am_hal_gpio_out_bit_clear(1);

    //
    // Short delay.
    //
    am_util_delay_us(10);

    //
    // De-assert reset to the target.
    //
    am_hal_gpio_out_bit_set(1);

    //
    // Wait for target to come out of reset.
    //
    am_util_delay_ms(1000);

    //
    // Restore SPI Clock configuration.
    //
    am_hal_gpio_pin_config(5, AM_HAL_PIN_5_M0SCK);
}


//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint64_t ui64Status;

    //
    // Read and clear the GPIO interrupt status.
    //
    ui64Status = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64Status);

    //
    // Check to make sure that this is the correct interrupt.
    //
    if (ui64Status & AM_HAL_GPIO_BIT(0))
    {
        if(g_bGPIODirection) // rising expected.
	{
            //
            // mark it as detected for use in the base level.
            //
            g_bGPIORising  = true;

            //
            // flip to the other polarity.
            //
            am_hal_gpio_int_polarity_bit_set(0,AM_HAL_GPIO_FALLING);
            g_bGPIODirection = false;
	}
	else // falling expected.
	{
            //
            // mark it as detected for use in the base level.
            //
            g_bGPIOFalling = true;

            //
            // flip to the other polarity.
            //
            am_hal_gpio_int_polarity_bit_set(0,AM_HAL_GPIO_RISING);
            g_bGPIODirection = true;
	}
    }
}


//*****************************************************************************
//
// Interrupt handler for IOM0
//
//*****************************************************************************
void
am_iomaster0_isr(void)
{
    uint32_t ui32IntStatus;

    //
    // Read and clear the interrupt status.
    //
    ui32IntStatus = am_hal_iom_int_status_get(0, false);
    am_hal_iom_int_clear(0, ui32IntStatus);

    //
    // Service FIFO interrupts as necessary, and call IOM callbacks as
    // transfers are completed.
    //
    am_hal_iom_int_service(0, ui32IntStatus);
}




//*****************************************************************************
//
// Hex dump the payload of a packet to the ITM.
//
//*****************************************************************************
void packet_hex_dump(char *PType, int Length,uint8_t *ui8Ptr)
{
    int i;

    //
    // Dump it to printf.
    //
    am_util_stdio_printf("\n%s PACKET(%d):\n\r", PType, Length);

    //
    // Unless it is an invalid payload.
    //
    if(Length <0) 
    {
        am_util_stdio_printf(" INVALID PACKET LENGTH\n\r");
        return;
    }

    am_util_stdio_printf("\t");

    for(i=0; i<Length; i++)
    {
        am_util_stdio_printf(" 0x%2.2x",*ui8Ptr);
	if( (i&7) ==7) am_util_stdio_printf("\n\r\t");
        ui8Ptr++;
    }
    am_util_stdio_printf("\n\r\n\r");
}



//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    int rc;
    uint8_t *ui8Ptr2RXBuffer;

    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_24MHZ);

    //
    // Initialize peripherals as specified in the BSP.
    //
    am_bsp_low_power_init();
    
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_hal_itm_print);

    //
    // Initialize the SWO GPIO pin
    //
    am_bsp_pin_enable(ITM_SWO);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("UART to SPI Example\n");

    //
    // Configure UART pins.
    //
    configure_uart_pins();

    //
    // Initialize and Enable the UART.
    //
    configure_uart();
    uart_enable();

    //
    // Now crank up the ring buffered interface to the UART.
    //
    am_hal_uart_init_buffered(g_pui8RxArray, 1024, g_pui8TxArray, 1024);

    //
    // Configure the STXETX protocol engine
    //
    am_util_stxetx_init(NULL);

    //
    // Configure the I/O Master 0 and GPIO pins we will need.
    //
    configure_pins();

    //
    // Enable interrupts before entering application loop.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER0);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
    am_hal_interrupt_master_enable();

    //
    // Initialize IOM 0 in SPI mode at 100KHz.
    //
    am_hal_iom_config(0, &g_sIOMConfig);
    am_hal_iom_enable(0);

    //
    // Loop forever writing for packets from the PC to come over the UART interface.
    // BUT, if a GPIO interrupt comes in from the sensor hub then handle that too.
    //
    while (1)
    {
	//
	// Report Rising GPIO pin.
	//
        if(g_bGPIORising)
	{
            g_ui8RcvBuffer[0] = 2;
            g_ui8RcvBuffer[1] = gpio_went_high;
            ui8Ptr2RXBuffer = g_ui8RcvBuffer;
            am_util_stxetx_tx(true, true, 2, &ui8Ptr2RXBuffer);
            g_bGPIORising  = false; // clear interrupt flag.
	}

	//
	// Report Falling GPIO pin.
	//
        if(g_bGPIOFalling)
	{
            g_ui8RcvBuffer[0] = 2;
            g_ui8RcvBuffer[1] = gpio_went_low;
            ui8Ptr2RXBuffer = g_ui8RcvBuffer;
            am_util_stxetx_tx(true, true, 2, &ui8Ptr2RXBuffer);
            g_bGPIOFalling = false; // clear interrupt flag.
	}
	

	//
	// Wait for the arrival of a valid STX byte, for packet start.
	//
        while( ! am_util_stxetx_rx_wait4start());
        am_util_stdio_printf("\nWe are started\n");

	//
	// Go get the packet payload.
	//
        rc = am_util_stxetx_rx(512, g_ui8RcvBuffer);

	//
	// If there was anything to it, then process it.
	//
	if(rc>0)
	{
             am_util_stdio_printf("\n Got a packet\n\r");

	     //
	     // If there was anything in it, then return to sender.
	     //
             g_ui8RcvBuffer[1] ^= 0xFF; // make it different from what we sent.
             ui8Ptr2RXBuffer = g_ui8RcvBuffer;
             am_util_stxetx_tx(true, true, rc, &ui8Ptr2RXBuffer);
             g_ui8RcvBuffer[1] ^= 0xFF; // put the command back to original.

	     //
	     // Decode it and act on it.
	     //
             uart2spi_fsm(rc, g_ui8RcvBuffer);

	     //
	     // Print the contents of the RX packet.
	     //
	     packet_hex_dump("RX",rc,g_ui8RcvBuffer);
	}
    }
}
