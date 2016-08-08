//*****************************************************************************
//
//! @file psikick1001.c
//!
//! @brief This module contains high level functions to interface with psikick.
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
#include "test_main.h"


//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_psikick_isr(void)
{
    uint32_t retStatus=5;
    am_util_stdio_printf(" Got Psikick Int. Now Wake rest of devices\n");

    print_test_menu();

    // remember that you have to execute commands in order to exit from below
    // while loop
    while(retStatus)
    {
        retStatus = testMain();
        if(retStatus == CMD_TEST_EXIT)
        {
            retStatus = 0;
        }
        am_util_stdio_printf("retStatus:%d\n",retStatus);

    }
}

//*****************************************************************************
//
// lsm6dsl Test.
//
//*****************************************************************************
void psikick_test(void)
{
	static uint32_t loopInteration=0;
    am_util_stdio_printf(" Psikick Test.........Ambique going into deep sleep \n");

    // Configure the board for low power.
    //
    //am_bsp_low_power_init();

    //
    // Enable an LED so we can see whether we're awake or asleep
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);


    // Power down all but the first SRAM banks.
    //
    am_hal_mcuctrl_sram_power_set(AM_HAL_MCUCTRL_SRAM_POWER_DOWN_1 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_2 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_3 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_4 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_5 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_6 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_7,
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_1 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_2 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_3 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_4 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_5 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_6 |
                                  AM_HAL_MCUCTRL_SRAM_POWER_DOWN_7);
   

    //
    // Antenna switch control Low selects BT DA14581 RF RF; High selects PK1001 Receiver
    // GPIO 15 set to high to select PK1001 Receiver

    //am_hal_gpio_out_bit_set(AM_BSP_GPIO_PSIKICK_SELECT);
    //am_hal_gpio_out_bit_clear(AM_BSP_GPIO_PSIKICK_SELECT);
    //
    // Configure the Psikick interrupt GPIO pin 16 as an input.
    //
    am_bsp_pin_enable(PSIKICK_INT);
    am_hal_gpio_int_polarity_bit_set(AM_BSP_GPIO_PSIKICK_INT, AM_HAL_GPIO_RISING);

   // am_hal_gpio_pin_config(AM_BSP_GPIO_PSIKICK_INT, AM_HAL_GPIO_INPUT);

    //
    // Enable GPIO interrupts to the NVIC.
    // 

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_PSIKICK_INT));

    //
    // Enable the GPIO/button interrupt.
    //
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_PSIKICK_INT));

    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER1);

    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);     
    //am_hal_interrupt_enable(AM_HAL_INTERRUPT_CLKGEN);
  
    //
    // Enable interrupts to the core.
    //
    am_hal_interrupt_master_enable();
 
    while (1)
    {
    	loopInteration++;
        am_util_stdio_printf("AAAAA: %d \n",loopInteration);

        am_hal_interrupt_master_disable();
        //
        // Go to Deep Sleep.
        //
        am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED0);
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);


        am_hal_interrupt_master_enable();
    }
}
