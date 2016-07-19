//*****************************************************************************
//
//! @file lsm6dsl.c
//!
//! @brief This module contains high level functions to sample lsm6dsl.
//!
//! Configures the lsm6dsl to sample at 100Hz and interrupt on data-ready
//! causing the Apollo MCU to interrupt and retrieve the sample. The samples
//! are plotted over the ITM.
//!
//! While moving the EVK board, use AM Flash (click "Show Plot Window") to
//! view the real-time plot.
//!
//! Note: 
//
//*****************************************************************************


#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices.h"
#include "am_util.h"

extern void uart_transmit_delay(void);

//*****************************************************************************
//
// Forward declaration for the data handler function.
//
//*****************************************************************************
void data_handler_lsm6dsl(void);

//*****************************************************************************
//
// Data buffer for the most recent magnetometer sample.
// Note: The lsm6dsl does not have an internal fifo so setting a sample
// size larger than one has no effect on sampling.
//
//*****************************************************************************
#define LSM6DSL_SAMPLE_SIZE                     1
am_devices_lsm6dsl_sample(LSM6DSL_SAMPLE_SIZE)  g_sMagData_lsm6dsl;

//*****************************************************************************
//
// Flags to alert the main application of various interrupt conditions.
//
//*****************************************************************************
//volatile bool g_bMagDataReady = false;
volatile bool g_bIOMIdle_lsm6dsl = true;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
am_hal_iom_config_t g_sIOMConfig_lsm6dsl =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_8MHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 0,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// Device structure for the ADXL
//
//*****************************************************************************
am_devices_lsm6dsl_t g_sMAG_lsm6dsl =
{
    .ui32IOMModule = AM_BSP_LSM6DSL_IOM,
    .ui32ChipSelect = AM_BSP_LSM6DSL_CS,
};

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_lsm6dsl_isr(void)
{
    //
    // Alert the base level that the IOM is active.
    //
    g_bIOMIdle_lsm6dsl = true;

    //
    // Make sure the IOM is enabled, and that interrupts are configured.
    //
    am_hal_iom_enable(AM_BSP_LSM6DSL_IOM);
    am_hal_iom_int_clear(AM_BSP_LSM6DSL_IOM,
                         AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
    am_hal_iom_int_enable(AM_BSP_LSM6DSL_IOM,
                          AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);

    //
    // Start a SPI read command to retrieve the samples from the
    // magnetometer.
    //
    am_devices_lsm6dsl_sample_get(&g_sMAG_lsm6dsl, g_sMagData_lsm6dsl.words,
                                  data_handler_lsm6dsl);
}

//*****************************************************************************
//
// Function to handle incoming data from the magnetometer.
//
//*****************************************************************************
void data_handler_lsm6dsl(void)
{
    //
    // Now we have the magnetometer data, so we can disable the IOM.
    //
    am_hal_iom_disable(AM_BSP_LSM6DSL_IOM);
    g_bIOMIdle_lsm6dsl = false;
    am_util_stdio_printf(" lsm6dsl Data Rcvd \n");
    uart_transmit_delay();

#if 0
    //
    // Enable the ITM and plot the data
    //
    am_bsp_debug_printf_enable();

    am_util_plot_int(AM_UTIL_PLOT_0, g_sMagData_lsm6dsl.samples[0]);
    am_util_plot_int(AM_UTIL_PLOT_1, g_sMagData_lsm6dsl.samples[1]);
    am_util_plot_int(AM_UTIL_PLOT_2, g_sMagData_lsm6dsl.samples[2]);
#endif
}

//*****************************************************************************
//
// Perform all of the tasks necessary to initialize the LSM6DSL and put it in
// measurement mode.
//
//*****************************************************************************
void start_lsm6dsl(void)
{
    //
    // Enable the IOM so we can talk to the LIS.
    //
    am_hal_iom_enable(AM_BSP_LSM6DSL_IOM);

    //
    // Initialize the ADXL362 driver.
    //
    am_devices_lsm6dsl_config(&g_sMAG_lsm6dsl);

    //
    // Wait until the data has actually gone out over the SPI lines, and then
    // disable the IOM. We won't need it again until the LIS actually has data.
    //
    am_hal_iom_poll_complete(AM_BSP_LSM6DSL_IOM);
    am_hal_iom_disable(AM_BSP_LSM6DSL_IOM);
}

//*****************************************************************************
//
// unconfigure GPIOs for communicating with the lsm6dsl
//
//*****************************************************************************
void unconfigure_lsm6dsl_pins(void)
{
    //
    // disable IOM pins needed for the LSM6DSL.
    //
    am_bsp_iom_spi_pins_disable(AM_BSP_LSM6DSL_IOM);

    //
    // disable interrupt and chip select.
    //
    am_bsp_pin_disable(LSM6DSL_CS);
    am_bsp_pin_disable(LSM6DSL_INT1);

    //
    // disable ITM pin for plotting
    //
    //am_bsp_pin_disable(ITM_SWO);

    //
    // disable the chip-select and data-ready pins for the LSM6DSL.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LSM6DSL_DRDY));

}

//*****************************************************************************
//
// Configure GPIOs for communicating with the LSM6DSL
//
//*****************************************************************************
void configure_lsm6dsl_pins(void)
{
    //
    // Set up IOM1 SPI pins.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_LSM6DSL_IOM);

    //
    // Enable the chip-select and data-ready pins for the LSM6DSL
    //
    am_bsp_pin_enable(LSM6DSL_CS);
    am_bsp_pin_enable(LSM6DSL_DRDY);

    //
    // Setup ITM pin for plotting
    //
    //am_bsp_pin_enable(ITM_SWO);

    //
    // Enable a GPIO interrupt for positive edges on the DRDY pin.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LSM6DSL_DRDY));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LSM6DSL_DRDY));
}

//*****************************************************************************
//
// lsm6dsl Test.
//
//*****************************************************************************
void lsm6dsl_test(uint32_t TestCommand2)
{
    static uint32_t loopInteration;

    //
    // Configure IOM1 for talking with the LSM6DSL.
    //
    am_hal_iom_config(AM_BSP_LSM6DSL_IOM, &g_sIOMConfig_lsm6dsl);

    //
    // Configure the GPIOs to work with the LSM6DSL.
    //
    configure_lsm6dsl_pins();

    //
    // Enable an LED so we can see whether we're awake or asleep
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);

    //
    // Configure the LSM6DSL, and start the data-taking process.
    //
    start_lsm6dsl();

    //
    // Initialize the plotting interface.
    //
    //am_util_plot_init();

    //
    // Enable interrupts before entering applicaiton loop
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER1);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
    am_hal_interrupt_master_enable();

    //
    // Loop forever, waiting for events.
    //

    loopInteration = TestCommand2;
    if(loopInteration < 0)
    {
        loopInteration = 1000;
    }

    while (loopInteration)
    {
       loopInteration--;

       am_util_stdio_printf(" %d \n",loopInteration);
       uart_transmit_delay();
        //
        // Disable interrupts temporarily. We need to make sure that the global
        // flags aren't changing while we are checking them.
        //
        am_hal_interrupt_master_disable();

        if( loopInteration == 0)
        {
            g_bIOMIdle_lsm6dsl = false;
            unconfigure_lsm6dsl_pins();
            am_hal_interrupt_master_enable();
            am_util_stdio_printf("1: %d \n",loopInteration);
            return;
        }
        //
        // Pick a sleep mode. If the IOM is active, we will need to wake up
        // again very soon, so we will use normal sleep mode. Otherwise, we
        // should use deep sleep.
        //
        if (g_bIOMIdle_lsm6dsl)
        {
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED0);
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
            am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);
        }
        else
        {
            //
            // Diasable ITM before going to deep sleep
            //
            am_bsp_debug_printf_disable();

            //
            // Go to deep sleep
            //
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED0);
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
            am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);
        }

        //
        // Re-enable interrupts so we can handle any new events that have come
        // in.
        //
        am_hal_interrupt_master_enable();
    }
}
