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
//#include <stdint.h>
//#include <stdbool.h>
//#include <stdlib.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "test_main.h"

// extern void myHello(void);
void print_test_menu(void);
void uart_transmit_delay();

char rxBuffer[128];

void test_printf(char* string) {
	am_util_stdio_printf(string);
	uart_transmit_delay();
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
    .ui32FlowCtrl = AM_HAL_UART_FLOW_CTRL_NONE,
};


//*****************************************************************************
//
// Interrupt handler for IOM0
//
//*****************************************************************************
void am_iomaster0_isr(void)
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
// Interrupt handler for IOM1
//
//*****************************************************************************
void am_iomaster1_isr(void)
{
    uint32_t ui32IntStatus;

    //
    // Read and clear the interrupt status.
    //
    ui32IntStatus = am_hal_iom_int_status_get(1, false);
    am_hal_iom_int_clear(1, ui32IntStatus);

    //
    // Service FIFO interrupts as necessary, and call IOM callbacks as
    // transfers are completed.
    //
    am_hal_iom_int_service(1, ui32IntStatus);
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_isr(void)
{
    uint64_t ui64Status;

    //
    // Delay for debounce.
    //
    am_util_delay_ms(200);

    //
    // Read and clear the GPIO interrupt status.
    //
    ui64Status = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64Status);

    //
    // Check to make sure that this is the correct interrupt, and then start
    // the data transfer.
    //
    if (ui64Status & AM_HAL_GPIO_BIT(AM_BSP_GPIO_L3GD20H_INT2))
    {
        am_gpio_l3gd20h_isr();
    }
    else if (ui64Status & AM_HAL_GPIO_BIT (AM_BSP_GPIO_LIS3MDL_DRDY)) 
    {
        am_gpio_lis3mdl_isr();
    }
    else if (ui64Status & AM_HAL_GPIO_BIT (AM_BSP_GPIO_ADXL362_INT1)) 
    {
        am_gpio_adxl362_isr(); 
    }
    else if (ui64Status & AM_HAL_GPIO_BIT (AM_BSP_GPIO_BMI160_INT1)) 
    {
        am_gpio_bmi160_isr();
    }
    else if (ui64Status & AM_HAL_GPIO_BIT (AM_BSP_GPIO_LIS2DH12_INT1)) 
    {
        am_gpio_lis2dh12_isr();
    }
    else if (ui64Status & AM_HAL_GPIO_BIT (AM_BSP_GPIO_PSIKICK_INT))
    {
        am_gpio_psikick_isr();
    }

}

//*****************************************************************************
//
// Initialize the UART
//
//*****************************************************************************
void uart_init(void)
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
void uart_disable(void)
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
void uart_transmit_delay(void)
{
  //
  // Wait until busy bit clears to make sure UART fully transmitted last byte
  //
  while ( am_hal_uart_flags_get() & AM_HAL_UART_FR_BUSY );
}

void print_test_menu(void)
{
   am_util_stdio_terminal_clear();
   test_printf("          Ambiq Hardware/Software Test Menu \n\n");
   test_printf("          Select Test\n");
   test_printf("          1.   Gpio Get <1 Gpio# >, Returns current Value of this Gpio  \n");
   test_printf("          2.   Gpio Set <2 Gpio#> Changes this Gpio pin to high\n");
   test_printf("          3.   Gpio Clear <3 Gpio#> Changes this Gpio pin to ground\n");
   test_printf("          4.   Gpio Toggle <4 Gpio#> Change the state of specified gpio\n");
   test_printf("          5.   Gpio Replace <5 Gpio# (0=Low 1=High)> Set this Gpio pin high\n");
   test_printf("          6.   Gpio Set Direction <6 Gpio# In (0) or Out(1)> Sets Direction of this Gpio \n");
   test_printf("          7.   I2c <7 deviceID 0 or 1 (Read or Write)> \n");
   test_printf("          8.   Spi \n");
   test_printf("          9.   Uart \n");
   test_printf("          10.  Isr \n");
   test_printf("          11.  Timer \n");
   test_printf("          12.  Floating point \n");
   test_printf("          13.  Test ADX1362 (13 NumOfIterations, default 1000>\n");
   test_printf("          14.  Test BMI160 (14 NumOfIterations, default 1000>\n");
   test_printf("          15.  Test LIS3MDL (15 NumOfIterations, default 1000>\n");
   test_printf("          16.  Test LIS2DH12 (16 NumOfIterations, default 1000>\n");
   test_printf("          17.  Test L3GD20H (17 NumOfIterations, default 1000>\n");
   test_printf("          18.  Test LSM6DSL (18 NumOfIterations, default 1000>\n");
   test_printf("          19.  Throughput With Characteristic \n");              
   test_printf("          20.  Throughput With Notification \n");
   test_printf("          21.  Test Deep Deep Sleep \n");
   test_printf("          22.  Test Normal Sleep \n");
   test_printf("          23.  Test Psikick \n");
   test_printf("          24.  Flash Write \n");
   test_printf("          25.  Exit Test Menu  \n");
}

uint32_t convertStrToDec(char *str, uint32_t *len)
{
    uint32_t dec=0, i=0;

    while((str[i] != ' ') && (str[i] != '\0'))
    {
        dec = dec * 10 + (str[i] - '0');
        i++;
    }
    *len = i+1;
    return dec;
}

//*****************************************************************************
//
// Read from the UART for get the command
//
//*****************************************************************************
void readCmd(cmd_test_enum *cmd1, uint32_t *cmd2, uint32_t *cmd3)
{
  // uint32_t numOfIter,numOfIter1 = 0;
   uint32_t command, i;
   uint32_t len, index=0;
   bool validInput = true;
   
   // Empty the buffer
   for(i=0; i<16; i++)
   {
       rxBuffer[i] = ' ';
   }
   
   // Receive the input into rxBuffer
   // Max: 16 chars
   am_hal_uart_line_receive_polled((uint32_t)16, &rxBuffer[0]);

   // am_util_stdio_printf will terminate at the first null char
   am_util_stdio_printf("Val: %c,%c,%c,%c,%c,%c,%c,%c,%c,%c", rxBuffer[0], 
           rxBuffer[1], rxBuffer[2], rxBuffer[3], rxBuffer[4], 
           rxBuffer[5],rxBuffer[6],rxBuffer[7],rxBuffer[8],rxBuffer[9]);
   am_util_stdio_printf("\n");

   // Parse input
   uart_transmit_delay();
   command = convertStrToDec(&rxBuffer[0], &len);
   am_util_stdio_printf(" %d \n", command);
   uart_transmit_delay();
   if (command > 0 || command <= 23)
   {
	   *cmd1 = (cmd_test_enum)(command - 1); // necessary to do so to make enum work
   }
   else
   {
	   validInput = false;
   }
   index = len;
   command = convertStrToDec(&rxBuffer[index], &len);

   am_util_stdio_printf(" %d \n", command);
   uart_transmit_delay();
   *cmd2 = command;

   // Collect third command for commands 2. GPIO Set Value
   // and 3. GPIO Set Direction
   if(*cmd1 == CMD_TEST_GPIO_REPLACE ||
	  *cmd1 == CMD_TEST_GPIO_SET_DIR ||
	  *cmd1 == CMD_TEST_I2C)
   {
	   index = index + len;
	   command = convertStrToDec(&rxBuffer[index], &len);
	   am_util_stdio_printf(" %d \n", command);
	   uart_transmit_delay();
	   *cmd3 = command;
   }

}

void processCmd(cmd_test_enum TestCommand1, uint32_t TestCommand2, uint32_t TestCommand3)
{

    uint32_t value;

    switch(TestCommand1)
    {
        case CMD_TEST_GPIO_GET:
            am_util_stdio_printf("Get Current State Of GPIO \n");
            value = am_hal_gpio_input_bit_read(TestCommand2);
            am_util_stdio_printf("The state of gpio %d is %d \n", TestCommand2,value);
        break;

        // Set the specified pin high
        case CMD_TEST_GPIO_SET:
            am_util_stdio_printf("Set State Of GPIO %d to 1", TestCommand2);
            // TODO: make sure correct pin is set based off of test command2
            am_hal_gpio_out_bit_set(TestCommand2);
            uart_transmit_delay();
        break;

        // Set the specified pin low
        case CMD_TEST_GPIO_CLEAR:
            am_util_stdio_printf("Set State Of GPIO %d to 0", TestCommand2);
            am_hal_gpio_out_bit_clear(TestCommand2);
            uart_transmit_delay();
        break;

        // Change the state of the specified pin
        case CMD_TEST_GPIO_TOGGLE:
            am_util_stdio_printf("Change State Of GPIO %d to !GPIO %d", TestCommand2, TestCommand2);
            am_hal_gpio_out_bit_toggle(TestCommand2);
            uart_transmit_delay();
        break;

        // Change the state of the specified pin to given value
        case CMD_TEST_GPIO_REPLACE:
            am_util_stdio_printf("Set State Of GPIO %d to %d", TestCommand2, TestCommand3);
            am_hal_gpio_out_bit_replace(TestCommand2, TestCommand3);
            uart_transmit_delay();
        break;

        case CMD_TEST_GPIO_SET_DIR:
            am_util_stdio_printf("Set Direction Of GPIO %d to %d\n", TestCommand2, TestCommand3);
            if (TestCommand3 == 0)
            {
            	am_hal_gpio_out_enable_bit_clear(TestCommand2);
            	am_hal_gpio_pin_config(TestCommand2,  AM_HAL_GPIO_INPUT);
            }
            else
            {
            	am_hal_gpio_out_enable_bit_set(TestCommand2);
            	am_hal_gpio_pin_config(TestCommand2,  AM_HAL_GPIO_OUTPUT);
            }
            uart_transmit_delay();
        break;

        case CMD_TEST_I2C:
            am_util_stdio_printf("No I2C Device Connected\n");
            uart_transmit_delay();
        break;

        case CMD_TEST_SPI:
            am_util_stdio_printf("Not Supported Yet \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_UART:
            am_util_stdio_printf("Not Supported Yet \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_ISR:
            am_util_stdio_printf("Not Supported Yet \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_TIMER:
            am_util_stdio_printf("Not Supported Yet \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_FLOATING_POINT:
            am_util_stdio_printf("Not Supported Yet \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_ADX1362:
            am_util_stdio_printf("Runing Test ADX1362 \n");
            uart_transmit_delay();
            adxl362_test(TestCommand2);
            am_util_stdio_printf("End Test ADX1362 \n");
            uart_transmit_delay();

        break;

        case CMD_TEST_BMI160:
            am_util_stdio_printf("Runing Test BMI160 \n");
            uart_transmit_delay();
            bmi160_test(TestCommand2);
            am_util_stdio_printf("End Test BMI160 \n");
            uart_transmit_delay();
        break;

        case CMD_TSET_LIS3MDL:
            am_util_stdio_printf("Runing Test LIS3MDL \n");
            uart_transmit_delay();
            lis3mdl_test(TestCommand2);
            am_util_stdio_printf("End Test LIS3MDL \n");
            uart_transmit_delay();
        break;

        case CMD_TSET_LIS2DH12:
            am_util_stdio_printf("Runing Test LIS2DH12 \n");
            uart_transmit_delay();
            lis2dh12_test(TestCommand2);
            am_util_stdio_printf("End Test LIS2DH12 \n");
            uart_transmit_delay();
        break;
       
        case CMD_TSET_L3GD20H:
            am_util_stdio_printf("Runing Test L3GD20H \n");
            uart_transmit_delay();
            l3gd20h_test(TestCommand2);
            am_util_stdio_printf("End Test L3GD20H \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_LSM6DSL:
            am_util_stdio_printf("Runing Test LSM6DSL \n");
            uart_transmit_delay();
            lsm6dsl_test(TestCommand2);
            am_util_stdio_printf("End Test LSM6DSL \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_THROUGHPUT_CHAR:
            am_util_stdio_printf("Not Supported Yet \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_THROUGHPUT_NOTFI:
            am_util_stdio_printf("Not Supported Yet \n");
            uart_transmit_delay();
        break;

        case CMD_TEST_DEEP_DEEP_SLEEP:
            am_util_stdio_printf("Runing Deep Deep Sleep Test \n \n");
            uart_transmit_delay();

            //
            // Configure the board for low power.
            //
            am_bsp_low_power_init();

            //
            // Power down all SRAM banks.
            //
            am_hal_mcuctrl_sram_power_set(AM_HAL_MCUCTRL_SRAM_POWER_DOWN_ALL,
                                          AM_HAL_MCUCTRL_SRAM_POWER_DOWN_ALL);

            am_util_delay_ms(10);

            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        break;
        
        case CMD_TEST_NORMAL_SLEEP:
            am_util_stdio_printf("Runing Normal Sleep Test \n \n");
            uart_transmit_delay();


            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
        break;

        case CMD_TEST_PSIKICK:
            am_util_stdio_printf("Testing Psikick \n");
            uart_transmit_delay();
            psikick_test();
        break;

	    case CMD_TEST_WRITE_FLASH:
		    test_printf("Testing Flash-write \n");
		    test_write_flash();
		break;

		case CMD_TEST_EXIT:
		    test_printf("Exiting from Ambiq Hardware/Software Test Menu \n");
		break;

        default:
            am_util_stdio_printf("Invalid command selected\n\n");
        break;
    }
    print_test_menu();
}

uint32_t testMain(void)
{
	cmd_test_enum command1;
    uint32_t command2;
    uint32_t command3;
    
    // reading from UART to run a test
    readCmd(&command1, &command2, &command3);
    processCmd(command1, command2, command3);
    // delay 10 milliseconds
    am_util_delay_ms(10);
    return ((uint32_t)command1);
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
    uint32_t retStatus=0;
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
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_hal_uart_string_transmit_polled);

    //
    // Configure and enable the UART.
    //
    uart_init();

    print_test_menu();

    // myHello();

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        retStatus = testMain();
        // delay 10 milliseconds
        am_util_delay_ms(10);
    }
}
