/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Generic ARM M class core system startup for GCC compiler directives.
 *
 *  $Date: 2014-07-03 14:38:57 -0700 (Thu, 03 Jul 2014) $
 *  $Revision: 1632 $
 *
 *  Copyright (c) 2013 Wicentric, Inc., all rights reserved.
 *  Wicentric confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact Wicentric, Inc. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include "wsf_types.h"
#include "wsf_timer.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Linker memory section definition. */
#define SECTION(x)  __attribute__ ((section(x)))

/*! Weak symbol reference. */
#define WEAK        __attribute__ ((weak))

/**************************************************************************************************
  Functions
**************************************************************************************************/

extern int main(void);
extern void SystemInit(void);
static void SystemDefaultHandler(void);

/* Core vectors. */
void WEAK Reset_Handler(void);
void WEAK NMI_Handler(void);
void WEAK HardFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK BusFault_Handler(void);
void WEAK UsageFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK SVC_Handler(void);
void WEAK DebugMon_Handler(void);
void WEAK PendSV_Handler(void);
void WEAK SysTick_Handler(void);
void WEAK am_brownout_isr(void);
void WEAK am_watchdog_isr(void);
void WEAK am_clk_gen_isr(void);
void WEAK am_vcomp_isr(void);
void WEAK am_ioslave_ios_isr(void);
void WEAK am_ioslave_acc_isr(void);
void WEAK am_iomaster0_isr(void);
void WEAK am_iomaster1_isr(void);
void WEAK am_adc_isr(void);
void WEAK am_gpio_isr(void);
void WEAK am_ctimer_isr(void);
void WEAK am_uart_isr(void);

/* Assign default weak references. Override these values by defining a new function with the same name. */
#pragma weak NMI_Handler        = SystemDefaultHandler
#pragma weak HardFault_Handler  = SystemDefaultHandler
#pragma weak MemManage_Handler  = SystemDefaultHandler
#pragma weak BusFault_Handler   = SystemDefaultHandler
#pragma weak UsageFault_Handler = SystemDefaultHandler
#pragma weak SVC_Handler        = SystemDefaultHandler
#pragma weak DebugMon_Handler   = SystemDefaultHandler
#pragma weak PendSV_Handler     = SystemDefaultHandler
#pragma weak SysTick_Handler    = SystemDefaultHandler
#pragma weak am_brownout_isr    = SystemDefaultHandler
#pragma weak am_watchdog_isr    = SystemDefaultHandler
#pragma weak am_clk_gen_isr     = SystemDefaultHandler
#pragma weak am_vcomp_isr       = SystemDefaultHandler
#pragma weak am_ioslave_ios_isr = SystemDefaultHandler
#pragma weak am_ioslave_acc_isr = SystemDefaultHandler
#pragma weak am_iomaster0_isr   = SystemDefaultHandler
#pragma weak am_iomaster1_isr   = SystemDefaultHandler
#pragma weak am_adc_isr         = SystemDefaultHandler
#pragma weak am_gpio_isr        = SystemDefaultHandler
#pragma weak am_ctimer_isr      = SystemDefaultHandler
#pragma weak am_uart_isr        = SystemDefaultHandler

/**************************************************************************************************
  Global variables
**************************************************************************************************/

/* Defined by linker. */
extern unsigned long __etext;
extern unsigned long __data_start__;
extern unsigned long __data_end__;
extern unsigned long __bss_start__;
extern unsigned long __bss_end__;
extern unsigned long __stack;

/*! Core vector table (must be placed at address VTOR, i.e. 0x00000000). */
SECTION(".isr_vector")
void (* const systemVectors[])(void) =
{
  (void (*)(void))(&__stack),             /*  0: The initial stack pointer */
  Reset_Handler,                          /*  1: The reset handler */
  NMI_Handler,                            /*  2: The NMI handler */
  HardFault_Handler,                      /*  3: The hard fault handler */
  MemManage_Handler,                      /*  4: The MPU fault handler */
  BusFault_Handler,                       /*  5: The bus fault handler */
  UsageFault_Handler,                     /*  6: The usage fault handler */
  0,                                      /*  7: Reserved */
  0,                                      /*  8: Reserved */
  0,                                      /*  9: Reserved */
  0,                                      /* 10: Reserved */
  SVC_Handler,                            /* 11: SVCall handler */
  DebugMon_Handler,                       /* 12: Debug monitor handler */
  0,                                      /* 13: Reserved */
  PendSV_Handler,                         /* 14: The PendSV handler */
  SysTick_Handler,                        /* 15: The SysTick handler */

  /* External interrupts not defined */
  am_brownout_isr,                        //  0: Brownout
  am_watchdog_isr,                        //  1: Watchdog
  am_clk_gen_isr,                         //  2: CLK_GEN
  am_vcomp_isr,                           //  3: Voltage Comparator
  am_ioslave_ios_isr,                     //  4: I/O Slave general
  am_ioslave_acc_isr,                     //  5: I/O Slave access
  am_iomaster0_isr,                       //  6: I/O Master 0
  am_iomaster1_isr,                       //  7: I/O Master 1
  am_adc_isr,                             //  8: ADC
  am_gpio_isr,                            //  9: GPIO
  am_ctimer_isr,                          // 10: CTIMER
  am_uart_isr,                            // 11: UART
};

/*************************************************************************************************/
/*!
 *  \brief      Reset handler.
 *
 *  \param      None.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void Reset_Handler(void)
{
  unsigned long *pSrc, *pDst;

  /* Copy data segment initializers from flash to SRAM. */
  pSrc = &__etext;
  pDst = &__data_start__;
  while (pDst < &__data_end__)
  {
    *pDst++ = *pSrc++;
  }

  /* Fill BSS segment with zeros. */
  pDst = &__bss_start__;
  while (pDst < &__bss_end__)
  {
    *pDst++ = 0;
  }

  /* Core initialization. */
  SystemInit();

  /* Application entry. */
  main();

  /* Invoke semihosting exit on main() return. */
  __asm volatile ("mov    r0,#0x18");
  __asm volatile ("bkpt   0xAB");
}

/*************************************************************************************************/
/*!
 *  \brief      Default vector handler.
 *
 *  \param      None.
 *
 *  \return     None.
 */
/*************************************************************************************************/
static void SystemDefaultHandler(void)
{
  volatile unsigned int forever = 1;
  while (forever);
}

/*************************************************************************************************/
/*!
 *  \brief      Generic system initialization.
 *
 *  \param      None.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void WEAK SystemInit(void)
{
  /* Stub function for generic initialization. */
}
