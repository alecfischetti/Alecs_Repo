###################################################################################################
#
# Source and include definition
# 
#         $Date: 2015-06-12 04:19:18 -0700 (Fri, 12 Jun 2015) $
#         $Revision: 3061 $
#  
# Copyright (c) 2012 Wicentric, Inc., all rights reserved.
# Wicentric confidential and proprietary.
#
# IMPORTANT.  Your use of this file is governed by a Software License Agreement
# ("Agreement") that must be accepted in order to download or otherwise receive a
# copy of this file.  You may not use or copy this file for any purpose other than
# as described in the Agreement.  If you do not agree to all of the terms of the
# Agreement do not use this file and delete all copies in your possession or control;
# if you do not have a copy of the Agreement, you must contact Wicentric, Inc. prior
# to any use, copying or further distribution of this software.
#
###################################################################################################

INC_DIRS += \
	$(ROOT_DIR)/sw/apps/app \
	$(ROOT_DIR)/sw/apps/app/include \
	$(ROOT_DIR)/sw/apps/fit \
	$(ROOT_DIR)/sw/hci/dual_chip \
	$(ROOT_DIR)/sw/hci/include \
	$(ROOT_DIR)/sw/profiles/bas \
	$(ROOT_DIR)/sw/profiles/hrps \
	$(ROOT_DIR)/sw/services \
	$(ROOT_DIR)/sw/stack/cfg \
	$(ROOT_DIR)/sw/stack/hci \
	$(ROOT_DIR)/sw/stack/include \
	$(ROOT_DIR)/sw/util \
	$(ROOT_DIR)/sw/wsf/common \
	$(ROOT_DIR)/sw/wsf/generic \
	$(ROOT_DIR)/sw/wsf/include \
	$(ROOT_DIR)/projects/ambiq/board \
	$(ROOT_DIR)/platform/ambiq/boards/$(BOARD)/bsp \
	$(ROOT_DIR)/platform/ambiq/devices \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/regs \
	$(ROOT_DIR)/platform/ambiq/utils \
	$(ROOT_DIR)/platform/ambiq/devices \
	$(ROOT_DIR)/platform/ambiq/third_party/CMSIS/CMSIS/Include \
	$(ROOT_DIR)/platform/ambiq/third_party/CMSIS/Device/Ambiq/APOLLO/Include

C_FILES += \
	$(ROOT_DIR)/sw/apps/app/app_main.c \
	$(ROOT_DIR)/sw/apps/app/app_server.c \
	$(ROOT_DIR)/sw/apps/app/app_slave.c \
	$(ROOT_DIR)/sw/apps/app/generic/app_db.c \
	$(ROOT_DIR)/sw/apps/app/generic/app_hw.c \
	$(ROOT_DIR)/sw/apps/app/generic/app_ui.c \
	$(ROOT_DIR)/sw/apps/fit/fit_main.c \
	$(ROOT_DIR)/sw/hci/generic/hci_core.c \
	$(ROOT_DIR)/sw/hci/dual_chip/hci_cmd.c \
	$(ROOT_DIR)/sw/hci/dual_chip/hci_core_ps.c \
	$(ROOT_DIR)/sw/hci/dual_chip/hci_evt.c \
	$(ROOT_DIR)/sw/hci/dual_chip/hci_tr.c \
	$(ROOT_DIR)/sw/hci/dual_chip/hci_vs.c \
	$(ROOT_DIR)/sw/profiles/bas/bas_main.c \
	$(ROOT_DIR)/sw/profiles/hrps/hrps_main.c \
	$(ROOT_DIR)/sw/services/svc_batt.c \
	$(ROOT_DIR)/sw/services/svc_core.c \
	$(ROOT_DIR)/sw/services/svc_dis.c \
	$(ROOT_DIR)/sw/services/svc_hrs.c \
	$(ROOT_DIR)/sw/stack/cfg/cfg_stack.c \
	$(ROOT_DIR)/sw/stack/hci/hci_main.c \
	$(ROOT_DIR)/sw/util/bda.c \
    $(ROOT_DIR)/sw/util/bstream.c \
	$(ROOT_DIR)/sw/util/calc128.c \
	$(ROOT_DIR)/sw/wsf/common/wsf_buf.c \
	$(ROOT_DIR)/sw/wsf/common/wsf_msg.c \
	$(ROOT_DIR)/sw/wsf/common/wsf_queue.c \
	$(ROOT_DIR)/sw/wsf/common/wsf_sec.c \
	$(ROOT_DIR)/sw/wsf/common/wsf_sec_aes.c \
	$(ROOT_DIR)/sw/wsf/common/wsf_timer.c \
	$(ROOT_DIR)/sw/wsf/generic/wsf_assert.c \
	$(ROOT_DIR)/sw/wsf/generic/wsf_os.c \
	$(ROOT_DIR)/sw/wsf/generic/wsf_trace.c \
	$(ROOT_DIR)/projects/ambiq/board/board_apollo.c \
	$(ROOT_DIR)/projects/ambiq/board/da14581_hci_image.c \
	$(ROOT_DIR)/projects/ambiq/board/startup_gcc.c \
	$(ROOT_DIR)/projects/ambiq/board/retarget_gcc.c \
	$(ROOT_DIR)/projects/ambiq/board/hci_drv.c \
	$(ROOT_DIR)/projects/ambiq/fit/main.c \
	$(ROOT_DIR)/platform/ambiq/boards/$(BOARD)/bsp/am_bsp_button.c \
	$(ROOT_DIR)/platform/ambiq/boards/$(BOARD)/bsp/am_bsp_gpio.c \
	$(ROOT_DIR)/platform/ambiq/boards/$(BOARD)/bsp/am_bsp_iom.c \
	$(ROOT_DIR)/platform/ambiq/boards/$(BOARD)/bsp/am_bsp_led.c \
	$(ROOT_DIR)/platform/ambiq/boards/$(BOARD)/bsp/am_bsp_uart.c \
	$(ROOT_DIR)/platform/ambiq/boards/$(BOARD)/bsp/am_bsp.c \
	$(ROOT_DIR)/platform/ambiq/devices/am_devices_da14580.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_adc.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_clkgen.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_ctimer.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_flash.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_gpio.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_interrupt.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_iom.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_ios.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_itm.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_mcuctrl.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_rtc.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_sysctrl.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_systick.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_tpiu.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_uart.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_vcomp.c \
	$(ROOT_DIR)/platform/ambiq/mcu/apollo/hal/am_hal_wdt.c \
	$(ROOT_DIR)/platform/ambiq/utils/am_util_delay.c \
	$(ROOT_DIR)/platform/ambiq/utils/am_util_math.c \
	$(ROOT_DIR)/platform/ambiq/utils/am_util_ring_buffer.c \
	$(ROOT_DIR)/platform/ambiq/utils/am_util_stdio.c