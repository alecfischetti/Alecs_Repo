//*****************************************************************************
//
//! @file am_bsp.c
//!
//! @brief Top level functions for performing board initialization.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @addtogroup apollo_evk_btle_sensor_bsp BSP for the Apollo EVK BTLE + SENSOR board
//! @ingroup BSP
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"

//*****************************************************************************
//
// LEDs
//
//*****************************************************************************
am_devices_led_t am_bsp_psLEDs[AM_BSP_NUM_LEDS] =
{
    {AM_BSP_GPIO_LED0, AM_DEVICES_LED_ON_LOW},
    {AM_BSP_GPIO_LED1, AM_DEVICES_LED_ON_LOW},
    {AM_BSP_GPIO_LED2, AM_DEVICES_LED_ON_LOW},
    {AM_BSP_GPIO_LED3, AM_DEVICES_LED_ON_LOW},
};

//*****************************************************************************
//
// Buttons.
//
//*****************************************************************************
am_devices_button_t am_bsp_psButtons[AM_BSP_NUM_BUTTONS] =
{
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON0, AM_DEVICES_BUTTON_NORMAL_HIGH),
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON1, AM_DEVICES_BUTTON_NORMAL_HIGH),
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON2, AM_DEVICES_BUTTON_NORMAL_HIGH),
    AM_DEVICES_BUTTON(AM_BSP_GPIO_BUTTON3, AM_DEVICES_BUTTON_NORMAL_HIGH),
};

//*****************************************************************************
//
//! @brief Enable the necessary pins for the given IOM module.
//!
//! @param ui32Module is the module number of the IOM interface to be used.
//!
//! Uses BSP pin definitions to configure the correct interface for the
//! selected IOM module.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_spi_pins_enable(uint32_t ui32Module)
{
    switch(ui32Module)
    {
#ifdef AM_BSP_GPIO_IOM0_SCK
        case 0:
            am_bsp_pin_enable(IOM0_SCK);
            am_bsp_pin_enable(IOM0_MISO);
            am_bsp_pin_enable(IOM0_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM1_SCK
        case 1:
            am_bsp_pin_enable(IOM1_SCK);
            am_bsp_pin_enable(IOM1_MISO);
            am_bsp_pin_enable(IOM1_MOSI);
            break;
#endif
        //
        // If we get here, the caller's selected IOM interface couldn't be
        // found in the BSP GPIO definitions. Halt program execution for
        // debugging.
        //
        default:
            while(1);
    }
}

//*****************************************************************************
//
//! @brief Disable the necessary pins for the given IOM module.
//!
//! @param ui32Module is the module number of the IOM interface to be used.
//!
//! Uses BSP pin definitions to configure the correct interface for the
//! selected IOM module.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_iom_spi_pins_disable(uint32_t ui32Module)
{
    switch(ui32Module)
    {
#ifdef AM_BSP_GPIO_IOM0_SCK
        case 0:
            am_bsp_pin_disable(IOM0_SCK);
            am_bsp_pin_disable(IOM0_MISO);
            am_bsp_pin_disable(IOM0_MOSI);
            break;
#endif

#ifdef AM_BSP_GPIO_IOM1_SCK
        case 1:
            am_bsp_pin_disable(IOM1_SCK);
            am_bsp_pin_disable(IOM1_MISO);
            am_bsp_pin_disable(IOM1_MOSI);
            break;
#endif
        //
        // If we get here, the caller's selected IOM interface couldn't be
        // found in the BSP GPIO definitions. Halt program execution for
        // debugging.
        //
        default:
            while(1);
    }
}

//*****************************************************************************
//
//! @brief Prepare the MCU for low power operation.
//!
//! This function enables several power-saving features of the MCU, and
//! disables some of the less-frequently used peripherals. It also sets the
//! system clock to 24 MHz.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_low_power_init(void)
{
    //
    // Enable internal buck converters.
    //
    am_hal_mcuctrl_bucks_enable();

    //
    // Turn off the voltage comparator as this is enabled on reset.
    //
    am_hal_vcomp_disable();

    //
    // Run the RTC off the LFRC.
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC);

    //
    // Stop the XT and LFRC.
    //
    am_hal_clkgen_osc_stop(AM_HAL_CLKGEN_OSC_XT);
    am_hal_clkgen_osc_stop(AM_HAL_CLKGEN_OSC_LFRC);

    //
    // Disable the RTC.
    //
    am_hal_rtc_osc_disable();

    //
    // Disable the bandgap.
    //
    am_hal_mcuctrl_bandgap_disable();
}

//*****************************************************************************
//
//! @brief Enable the TPIU and ITM for debug printf messages.
//!
//! This function enables TPIU registers for debug printf messages and enables
//! ITM GPIO pin to SWO mode. This function should be called after reset and
//! after waking up from deep sleep.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_debug_printf_enable(void)
{
    if (g_ui32HALflags & AM_HAL_FLAGS_ITMSKIPENABLEDISABLE_M)
    {
        return;
    }
    am_hal_tpiu_enable();
    am_bsp_pin_enable(ITM_SWO);
}

//*****************************************************************************
//
//! @brief Enable the TPIU and ITM for debug printf messages.
//!
//! This function disables TPIU registers for debug printf messages and
//! enables ITM GPIO pin to GPIO mode and prepares the MCU to go to deep sleep.
//!
//! @return None.
//
//*****************************************************************************
void
am_bsp_debug_printf_disable(void)
{
    if (g_ui32HALflags & AM_HAL_FLAGS_ITMSKIPENABLEDISABLE_M)
    {
        return;
    }
    am_hal_itm_not_busy();
    am_hal_gpio_pin_config(AM_BSP_GPIO_ITM_SWO, AM_HAL_GPIO_OUTPUT);
    am_hal_tpiu_disable();
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
