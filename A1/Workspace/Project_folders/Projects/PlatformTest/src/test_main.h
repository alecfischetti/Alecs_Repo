

/**************************************************************************************************
  Filename:       test_main.h
  
  Description:    This file contains prototypes and defines  
                  
  Copyright 2016 Flextronics. All rights reserved.

 **************************************************************************************************/
#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum cmd_test_enum
{
	CMD_TEST_GPIO_GET,
	CMD_TEST_GPIO_SET,
	CMD_TEST_GPIO_CLEAR,
	CMD_TEST_GPIO_TOGGLE,
	CMD_TEST_GPIO_REPLACE,
	CMD_TEST_GPIO_SET_DIR,
	CMD_TEST_I2C,
	CMD_TEST_SPI,
	CMD_TEST_UART,
	CMD_TEST_ISR,
	CMD_TEST_TIMER,
	CMD_TEST_FLOATING_POINT,
	CMD_TEST_ADX1362,
	CMD_TEST_BMI160,
	CMD_TSET_LIS3MDL,
	CMD_TSET_LIS2DH12,
	CMD_TSET_L3GD20H,
	CMD_TEST_LSM6DSL,
	CMD_TEST_THROUGHPUT_CHAR,
	CMD_TEST_THROUGHPUT_NOTFI,
	CMD_TEST_DEEP_DEEP_SLEEP,
	CMD_TEST_NORMAL_SLEEP,
	CMD_TEST_PSIKICK,
	CMD_TEST_WRITE_FLASH,
	CMD_TEST_EXIT,
	CMD_TEST_ENUM_MAX
} cmd_test_enum;

extern void adxl362_test(uint32_t TestCommand2);
extern void bmi160_test(uint32_t TestCommand2);
extern void lis2dh12_test(uint32_t TestCommand2);
extern void lis3mdl_test(uint32_t TestCommand2);
extern void l3gd20h_test(uint32_t TestCommand2);
extern void lsm6dsl_test(uint32_t TestCommand2);
extern void test_write_flash(void);
extern void psikick_test(void);
extern void am_gpio_psikick_isr(void);
extern void am_gpio_l3gd20h_isr(void);
extern void am_gpio_lis3mdl_isr(void);
extern void am_gpio_adxl362_isr(void);
extern void am_gpio_bmi160_isr(void);
extern void am_gpio_lis2dh12_isr(void);
extern uint32_t testMain(void);
extern void print_test_menu(void);
extern void ambiq_config_deep_sleep(void);
    
#ifdef __cplusplus
}
#endif

#endif /* TEST_MAIN_H */

