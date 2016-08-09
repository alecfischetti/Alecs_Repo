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
// lsm6dsl Test.
//
//*****************************************************************************
void lsm6dsl_test(void)
{
    // initialize the sensor
    // setup gpio 11 and 12 for i2c bit banging
    uint8_t setClear = 1;
    uint8_t sensorId, wakeUpDur, masterCmdCode; 
    uint32_t ui32NumBytes = 1;

    am_devices_lsm6dsl_config();

    // power up the sensor
    am_devices_lsm6dsl_set_clear(setClear);
    am_devices_lsm6dsl_reg_read((uint8_t)AM_DEVICES_LSM6DSL_WHO_AM_I_REG, ui32NumBytes, &sensorId);
    am_devices_lsm6dsl_reg_read((uint8_t)AM_DEVICES_LSM6DSL_WAKEUP_DUR, ui32NumBytes, &wakeUpDur);
    am_devices_lsm6dsl_reg_read((uint8_t)AM_DEVICES_LSM6DSL_MASTER_CMD_CODE, ui32NumBytes, &masterCmdCode);

    am_util_stdio_printf("1. Wake Up Dur: %d \n",wakeUpDur);
    am_util_stdio_printf("1. Maset Cmd Code: %d \n",masterCmdCode);

    wakeUpDur = 0xAA;
    masterCmdCode = 0xCC;
    am_devices_lsm6dsl_reg_write((uint8_t)AM_DEVICES_LSM6DSL_WAKEUP_DUR, ui32NumBytes, &wakeUpDur);
    am_devices_lsm6dsl_reg_write((uint8_t)AM_DEVICES_LSM6DSL_MASTER_CMD_CODE, ui32NumBytes, &masterCmdCode);

    am_devices_lsm6dsl_reg_read((uint8_t)AM_DEVICES_LSM6DSL_WAKEUP_DUR, ui32NumBytes, &wakeUpDur);
    am_devices_lsm6dsl_reg_read((uint8_t)AM_DEVICES_LSM6DSL_MASTER_CMD_CODE, ui32NumBytes, &masterCmdCode);

    am_util_stdio_printf("2. Wake Up Dur: %d \n",wakeUpDur);
    am_util_stdio_printf("2. Maset Cmd Code: %d \n",masterCmdCode);
}
