

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

//#define    CMD_TEST_GPIO_GET                        0x0001
//#define    CMD_TEST_GPIO_SET                        0x0002
//#define    CMD_TEST_GPIO_CLEAR                      0x0002
//#define    CMD_TEST_GPIO_SET                        0x0002
//#define    CMD_TEST_GPIO_SET                        0x0002
//#define    CMD_TEST_GPIO_SET_DIR                    0x0003
//#define    CMD_TEST_I2C                             0x0004
//#define    CMD_TEST_SPI                             0x0005
//#define    CMD_TEST_UART                            0x0006
//#define    CMD_TEST_ISR                             0x0007
//#define    CMD_TEST_TIMER                           0x0008
//#define    CMD_TEST_FLOATING_POINT                  0x0009
//#define    CMD_TEST_ADX1362                         0x000A
//#define    CMD_TEST_BMI160                          0x000B
//#define    CMD_TSET_LIS3MDL                         0x000C
//#define    CMD_TSET_LIS2DH12                        0x000D
//#define    CMD_TSET_L3GD20H                         0x000E
//#define    CMD_TEST_LSM6DSL                         0x000F
//#define    CMD_TEST_THROUGHPUT_CHAR                 0x0010
//#define    CMD_TEST_THROUGHPUT_NOTFI                0x0011
//#define    CMD_TEST_DEEP_DEEP_SLEEP                 0x0012
//#define    CMD_TEST_NORMAL_SLEEP                    0x0013
//#define    CMD_TEST_PSIKICK                         0x0014

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
	CMD_TEST_ENUM_MAX
} cmd_test_enum;

extern void adxl362_test(uint32_t TestCommand2);
extern void bmi160_test(uint32_t TestCommand2);
extern void lis2dh12_test(uint32_t TestCommand2);
extern void lis3mdl_test(uint32_t TestCommand2);
extern void l3gd20h_test(uint32_t TestCommand2);
extern void lsm6dsl_test(uint32_t TestCommand2);
extern void am_gpio_l3gd20h_isr(void);
extern void am_gpio_lis3mdl_isr(void);
extern void am_gpio_adxl362_isr(void);
extern void am_gpio_bmi160_isr(void);
extern void am_gpio_lis2dh12_isr(void);

    
#ifdef __cplusplus
}
#endif

#endif /* TEST_MAIN_H */

