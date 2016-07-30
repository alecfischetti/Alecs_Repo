//*****************************************************************************
//
//! @file adxl362.c
//!
//! @brief This module contains high level functions to sample adxl362.
//!
//! Configures the adxl362 to sample at 100Hz and set its watermark based
//! on the ADXL362_SAMPLE_SIZE define. When the adxl362 FIFO hits its
//! watermark, an interrupt line asserts causing the Apollo MCU to interrupt
//! and begin draining the FIFO while sleeping and periodically waking to
//! empty the internal IOM. The samples are plotted over the ITM.
//!
//! While moving the EVK board, use AM Flash (click "Show Plot Window") to
//! view the real-time plot.
//!
//! Note:
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices.h"
#include "am_util.h"
#include <stdbool.h>

//*****************************************************************************
//
// Forward declaration for the data handler function.
//
//*****************************************************************************
void data_handler_adxl362(void);

//*****************************************************************************
//
// Buffer for holding ADXL samples.
//
//*****************************************************************************
#define ADXL362_SAMPLE_SIZE                             4
static am_devices_adxl362_sample(ADXL362_SAMPLE_SIZE)
g_psSampleBuffer;

//*****************************************************************************
//
// Flags to alert the main application of various interrupt conditions.
//
//*****************************************************************************
volatile bool g_bIOMIdle = true;
static bool exitLoop = false;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
const am_hal_iom_config_t g_sIOMConfig_adxl362 = { .ui32InterfaceMode =
		AM_HAL_IOM_SPIMODE, .ui32ClockFrequency = AM_HAL_IOM_8MHZ, .bSPHA = 0,
		.bSPOL = 0, .ui8WriteThreshold = 0, .ui8ReadThreshold = 60, };

//*****************************************************************************
//
// Device structure for the ADXL.
//
//*****************************************************************************
const am_devices_adxl362_t g_sADXL_adxl362 = { .ui32IOMModule =
		AM_BSP_ADXL362_IOM, .ui32ChipSelect = AM_BSP_ADXL362_CS, .ui32Samples =
		ADXL362_SAMPLE_SIZE, .ui32SampleRate = AM_DEVICES_ADXL362_400HZ,
		.ui32Range = AM_DEVICES_ADXL362_2G, .bHalfBandwidth = false,
		.bSyncMode = false, };

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_adxl362_isr(void) {
	//
	// Alert the base level that the IOM is active.
	//
	g_bIOMIdle = true;

	//
	// Start a SPI read command to retrieve the samples from the ADXL
	//
	am_hal_iom_enable(AM_BSP_ADXL362_IOM);
	am_hal_iom_int_clear(AM_BSP_ADXL362_IOM,
	AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
	am_hal_iom_int_enable(AM_BSP_ADXL362_IOM,
	AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
	//
	// Start a SPI read command to retrieve the samples from the
	// gyro.
	//
	am_devices_adxl362_sample_get(&g_sADXL_adxl362, ADXL362_SAMPLE_SIZE,
			g_psSampleBuffer.words, data_handler_adxl362);
}

//*****************************************************************************
//
// Function to handle incoming data from the ADXL.
//
//*****************************************************************************
void data_handler_adxl362(void) {
	//
	// Now we have the accel data, so we can disable the IOM.
	//
	am_hal_iom_disable(AM_BSP_L3GD20H_IOM);
	g_bIOMIdle = false;

	//
	// Enable the ITM and plot the data
	//
	am_bsp_debug_printf_enable();

	// Get the data
	am_util_stdio_printf("%d\n",
			AM_DEVICES_ADXL362_VALUE(g_psSampleBuffer.samples[0]));
	am_util_stdio_printf("%d\n",
			AM_DEVICES_ADXL362_VALUE(g_psSampleBuffer.samples[1]));
	am_util_stdio_printf("%d\n\n",
			AM_DEVICES_ADXL362_VALUE(g_psSampleBuffer.samples[2]));

	exitLoop = true;
	am_hal_interrupt_disable(AM_HAL_INTERRUPT_GPIO);
}

//*****************************************************************************
//
// Perform all of the tasks necessary to initialize the ADXL and put it in
// measurement mode.
//
//*****************************************************************************
void start_adxl362(void) {
	//
	// Enable the IOM so we can talk to the ADXL.
	//
	am_hal_iom_enable(AM_BSP_ADXL362_IOM);

	//
	// Set ADXL registers to the default settings from the driver (asynchronous
	// mode) and set "measurement mode" to get the ADXL to start taking
	// samples.
	//
	am_devices_adxl362_config(&g_sADXL_adxl362);
	am_devices_adxl362_measurement_mode_set(&g_sADXL_adxl362);

	//
	// Wait until the data has actually gone out over the SPI lines.
	//
	am_hal_iom_poll_complete(AM_BSP_ADXL362_IOM);

	//
	// Disable the IOM to save power. We won't need it again until the ADXL
	// actually has data.
	//
	am_hal_iom_disable(AM_BSP_ADXL362_IOM);
}

//*****************************************************************************
//
// unconfigure GPIOs for communicating with the ADXL
//
//*****************************************************************************
void unconfigure_adxl_pins(void) {
	//
	// disable IOM pins needed for the ADXL362.
	//
	am_bsp_iom_spi_pins_disable(AM_BSP_ADXL362_IOM);

	//
	// disable ADXL interrupt and chip select.
	//
	am_bsp_pin_disable(ADXL362_CS);
	am_bsp_pin_disable(ADXL362_INT1);

	//
	// disable ITM pin for plotting
	//
	//am_bsp_pin_disable(ITM_SWO);

	//
	// disable a GPIO interrupt for positive edges on ADXL's INT1 pin.
	//
	am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_ADXL362_INT1));

}

//*****************************************************************************
//
// Configure GPIOs for communicating with the ADXL
//
//*****************************************************************************
void configure_adxl_pins(void) {
	//
	// Set up the IOM pins needed for the ADXL362.
	//
	am_bsp_iom_spi_pins_enable(AM_BSP_ADXL362_IOM);

	//
	// Set up ADXL interrupt and chip select.
	//
	am_bsp_pin_enable(ADXL362_CS);
	am_bsp_pin_enable(ADXL362_INT1);

	//
	// Setup ITM pin for plotting
	//
	//am_bsp_pin_enable(ITM_SWO);

	//
	// Enable a GPIO interrupt for positive edges on ADXL's INT1 pin.
	//
	am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_ADXL362_INT1));
	am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_ADXL362_INT1));
}

void exit_accel() {
	g_bIOMIdle = false;
	unconfigure_adxl_pins();
	am_hal_interrupt_master_enable();
	am_util_stdio_printf("\n");
	return;
}

//*****************************************************************************
//
// adxl362 Test.
//
//*****************************************************************************
void adxl362_test(uint32_t TestCommand2) {
	//
	// Configure the IOM for talking with the ADXL.
	//
	am_hal_iom_config(AM_BSP_ADXL362_IOM, &g_sIOMConfig_adxl362);

	//
	// Configure the GPIOs to work with the ADXL.
	//
	configure_adxl_pins();

	//
	// Enable an LED so we can see whether we're awake or asleep
	//
	am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
	am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);

	//
	// Configure the ADXL, and start the data-taking process.
	//
	start_adxl362();

	//
	// Enable interrupts before entering application loop
	//
	am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER1);
	am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
	am_hal_interrupt_master_enable();

	exitLoop = false;

	// Wait for the Accelerometer to be polled once
	while (exitLoop == false) {
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
	}

}
