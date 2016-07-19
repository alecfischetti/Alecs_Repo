//*****************************************************************************
//
//! @file am_bsp_gpio.h
//!
//! @brief Functions to aid with configuring the GPIOs.
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
#ifndef AM_BSP_GPIO_H
#define AM_BSP_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Miscellaneous pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_BUTTON0             39
#define AM_BSP_GPIO_BUTTON1             26
#define AM_BSP_GPIO_BUTTON2             46
#define AM_BSP_GPIO_LED0                43
#define AM_BSP_GPIO_LED1                44
#define AM_BSP_GPIO_LED2                45
#define AM_BSP_GPIO_LED3                47

//*****************************************************************************
//
// ADXL362 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_ADXL362_CS          12
#define AM_BSP_GPIO_CFG_ADXL362_CS      AM_HAL_PIN_12_M1nCE0
#define AM_BSP_GPIO_ADXL362_INT1        31
#define AM_BSP_GPIO_CFG_ADXL362_INT1    AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_ADXL362_INT2        19
#define AM_BSP_GPIO_CFG_ADXL362_INT2    AM_HAL_PIN_19_TCTB1

//*****************************************************************************
//
// BMI160 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_BMI160_CS           30
#define AM_BSP_GPIO_CFG_BMI160_CS       AM_HAL_PIN_30_M1nCE7
#define AM_BSP_GPIO_BMI160_INT1         33
#define AM_BSP_GPIO_CFG_BMI160_INT1     AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_BMI160_INT2         34
#define AM_BSP_GPIO_CFG_BMI160_INT2     AM_HAL_PIN_INPUT

//*****************************************************************************
//
// COM_UART pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_RX         36
#define AM_BSP_GPIO_CFG_COM_UART_RX     AM_HAL_PIN_36_UARTRX
#define AM_BSP_GPIO_COM_UART_TX         35
#define AM_BSP_GPIO_CFG_COM_UART_TX     AM_HAL_PIN_35_UARTTX

//*****************************************************************************
//
// FLASH pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_FLASH_CS            29
#define AM_BSP_GPIO_CFG_FLASH_CS        AM_HAL_PIN_29_M1nCE6

//*****************************************************************************
//
// I2C_MUX pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_I2C_MUX_ENABLE      40
#define AM_BSP_GPIO_CFG_I2C_MUX_ENABLE  AM_HAL_PIN_OUTPUT

//*****************************************************************************
//
// IOM1 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_MISO           9
#define AM_BSP_GPIO_CFG_IOM1_MISO       AM_HAL_PIN_9_M1MISO
#define AM_BSP_GPIO_IOM1_MOSI           10
#define AM_BSP_GPIO_CFG_IOM1_MOSI       AM_HAL_PIN_10_M1MOSI
#define AM_BSP_GPIO_IOM1_SCK            8
#define AM_BSP_GPIO_CFG_IOM1_SCK        (AM_HAL_PIN_8_M1SCK | AM_HAL_GPIO_HIGH_DRIVE)
#define AM_BSP_GPIO_IOM1_SCL            8
#define AM_BSP_GPIO_CFG_IOM1_SCL        (AM_HAL_PIN_8_M1SCL | AM_HAL_GPIO_HIGH_DRIVE)
#define AM_BSP_GPIO_IOM1_SDA            9
#define AM_BSP_GPIO_CFG_IOM1_SDA        AM_HAL_PIN_9_M1SDA

//*****************************************************************************
//
// IOS pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_CS              3
#define AM_BSP_GPIO_CFG_IOS_CS          AM_HAL_PIN_3_SLnCE
#define AM_BSP_GPIO_IOS_INT             4
#define AM_BSP_GPIO_CFG_IOS_INT         AM_HAL_PIN_4_SLINT
#define AM_BSP_GPIO_IOS_MISO            1
#define AM_BSP_GPIO_CFG_IOS_MISO        AM_HAL_PIN_1_SLMISO
#define AM_BSP_GPIO_IOS_MOSI            2
#define AM_BSP_GPIO_CFG_IOS_MOSI        AM_HAL_PIN_2_SLMOSI
#define AM_BSP_GPIO_IOS_SCK             0
#define AM_BSP_GPIO_CFG_IOS_SCK         AM_HAL_PIN_0_SLSCK
#define AM_BSP_GPIO_IOS_SCL             0
#define AM_BSP_GPIO_CFG_IOS_SCL         AM_HAL_PIN_0_SLSCL
#define AM_BSP_GPIO_IOS_SDA             1
#define AM_BSP_GPIO_CFG_IOS_SDA         AM_HAL_PIN_1_SLSDA

//*****************************************************************************
//
// ITM pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_ITM_SWO             41
#define AM_BSP_GPIO_CFG_ITM_SWO         AM_HAL_PIN_41_SWO

//*****************************************************************************
//
// L3GD20H pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_L3GD20H_CS          14
#define AM_BSP_GPIO_CFG_L3GD20H_CS      AM_HAL_PIN_14_M1nCE2
#define AM_BSP_GPIO_L3GD20H_INT1        16
#define AM_BSP_GPIO_CFG_L3GD20H_INT1    AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_L3GD20H_INT2        48
#define AM_BSP_GPIO_CFG_L3GD20H_INT2    AM_HAL_PIN_INPUT

//*****************************************************************************
//
// LIS2DH12 pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_LIS2DH12_CS         13
#define AM_BSP_GPIO_CFG_LIS2DH12_CS     AM_HAL_PIN_13_M1nCE1
#define AM_BSP_GPIO_LIS2DH12_INT1       46
#define AM_BSP_GPIO_CFG_LIS2DH12_INT1   AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_LIS2DH12_INT2       22
#define AM_BSP_GPIO_CFG_LIS2DH12_INT2   AM_HAL_PIN_INPUT

//*****************************************************************************
//
// LIS3MDL pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_LIS3MDL_CS          28
#define AM_BSP_GPIO_CFG_LIS3MDL_CS      AM_HAL_PIN_28_M1nCE5
#define AM_BSP_GPIO_LIS3MDL_DRDY        32
#define AM_BSP_GPIO_CFG_LIS3MDL_DRDY    AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_LIS3MDL_INT1        25
#define AM_BSP_GPIO_CFG_LIS3MDL_INT1    AM_HAL_PIN_INPUT

//*****************************************************************************
//
// PWM pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_PWM_LED             43
#define AM_BSP_GPIO_CFG_PWM_LED         AM_HAL_PIN_43_TCTB0

//*****************************************************************************
//
// RTC pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_RTC_CLKOUT          49
#define AM_BSP_GPIO_CFG_RTC_CLKOUT      AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_RTC_EXT1            23
#define AM_BSP_GPIO_CFG_RTC_EXT1        AM_HAL_PIN_OUTPUT
#define AM_BSP_GPIO_RTC_FOUT            15
#define AM_BSP_GPIO_CFG_RTC_FOUT        AM_HAL_PIN_INPUT

//*****************************************************************************
//
// SPI_HDR pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_SPI_HDR_CS          27
#define AM_BSP_GPIO_CFG_SPI_HDR_CS      AM_HAL_PIN_27_M1nCE4
#define AM_BSP_GPIO_SPI_HDR_INT         2
#define AM_BSP_GPIO_CFG_SPI_HDR_INT     AM_HAL_PIN_INPUT

//*****************************************************************************
//
// UART pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART_CTS            38
#define AM_BSP_GPIO_CFG_UART_CTS        AM_HAL_PIN_38_UACTS
#define AM_BSP_GPIO_UART_RTS            37
#define AM_BSP_GPIO_CFG_UART_RTS        AM_HAL_PIN_37_UARTS

//*****************************************************************************
//
// VCOMP pins.
//
//*****************************************************************************
#define AM_BSP_GPIO_VCOMP_RXO           42
#define AM_BSP_GPIO_CFG_VCOMP_RXO       AM_HAL_PIN_INPUT
#define AM_BSP_GPIO_VCOMP_RXO_CMP       18
#define AM_BSP_GPIO_CFG_VCOMP_RXO_CMP   AM_HAL_PIN_INPUT

//*****************************************************************************
//
// Convenience macros for enabling and disabling pins by function.
//
//*****************************************************************************
#define am_bsp_pin_enable(name)                                               \
    am_hal_gpio_pin_config(AM_BSP_GPIO_ ## name, AM_BSP_GPIO_CFG_ ## name);

#define am_bsp_pin_disable(name)                                              \
    am_hal_gpio_pin_config(AM_BSP_GPIO_ ## name, AM_HAL_PIN_DISABLE);

#ifdef __cplusplus
}
#endif

#endif // AM_BSP_GPIO_H
