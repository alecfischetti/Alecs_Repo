#******************************************************************************
#
# Makefile - Rules for building the libraries, examples and docs.
#
# Copyright (c) 2016, Ambiq Micro
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# This is part of revision 1.1.0 of the AmbiqSuite Development Package.
#
#******************************************************************************
TARGET = exactle_fit

#### Setup ####
TOOLCHAIN = arm-none-eabi
PART = apollo
CPU = cortex-m4
FPU = fpv4-sp-d16
FABI = softfp
WITHCC=w/GCC

LINKER_FILE = ./exactle_fit.ld
STARTUP_FILE = ./startup_gcc.c

#### Required Executables ####
CC = $(TOOLCHAIN)-gcc
GCC = $(TOOLCHAIN)-gcc
CPP = $(TOOLCHAIN)-cpp
LD = $(TOOLCHAIN)-ld
CP = $(TOOLCHAIN)-objcopy
OD = $(TOOLCHAIN)-objdump
RD = $(TOOLCHAIN)-readelf
AR = $(TOOLCHAIN)-ar
SIZE = $(TOOLCHAIN)-size

EXECUTABLES = CC LD CP OD AR RD SIZE GCC
K := $(foreach exec,$(EXECUTABLES),\
        $(if $(shell which $($(exec))),,\
        $(info $(exec) not found on PATH ($($(exec))).)$(exec)))
$(if $(strip $(value K)),$(info Required Program(s) $(strip $(value K)) not found))

ifneq ($(strip $(value K)),)
all clean:
	$(info Tools $(WITHCC) not installed.)
else
DEFINES = -DPART_$(PART)
DEFINES+= -Dgcc
DEFINES+= -DAM_PACKAGE_BGA
DEFINES+= -DHCI_TR_UART

INCLUDES = -I../../../../../devices
INCLUDES+= -I../../../../../mcu/apollo
INCLUDES+= -I../../../../../third_party/exactle/sw/apps/app
INCLUDES+= -I../../../../../third_party/exactle/sw/apps/app/include
INCLUDES+= -I../../../../../third_party/exactle/sw/apps/fit
INCLUDES+= -I../../../../../third_party/exactle/sw/hci/ambiq
INCLUDES+= -I../../../../../third_party/exactle/sw/hci/include
INCLUDES+= -I../../../../../third_party/exactle/sw/profiles/bas
INCLUDES+= -I../../../../../third_party/exactle/sw/profiles/hrps
INCLUDES+= -I../../../../../third_party/exactle/sw/services
INCLUDES+= -I../../../../../third_party/exactle/sw/stack/cfg
INCLUDES+= -I../../../../../third_party/exactle/sw/stack/hci
INCLUDES+= -I../../../../../third_party/exactle/sw/stack/include
INCLUDES+= -I../../../../../third_party/exactle/sw/util
INCLUDES+= -I../../../../../third_party/exactle/sw/wsf/ambiq
INCLUDES+= -I../../../../../third_party/exactle/sw/wsf/include
INCLUDES+= -I../../../../../utils
INCLUDES+= -I../../../bsp
INCLUDES+= -I../src

VPATH = ../../../../../utils
VPATH+=:../../../../../devices
VPATH+=:../../../../../third_party/exactle/sw/apps/app
VPATH+=:../../../../../third_party/exactle/sw/apps/app/generic
VPATH+=:../../../../../third_party/exactle/sw/apps/fit
VPATH+=:../../../../../third_party/exactle/sw/hci/ambiq
VPATH+=:../../../../../third_party/exactle/sw/profiles/bas
VPATH+=:../../../../../third_party/exactle/sw/profiles/hrps
VPATH+=:../../../../../third_party/exactle/sw/services
VPATH+=:../../../../../third_party/exactle/sw/stack/cfg
VPATH+=:../../../../../third_party/exactle/sw/stack/hci
VPATH+=:../../../../../third_party/exactle/sw/util
VPATH+=:../../../../../third_party/exactle/sw/wsf/ambiq
VPATH+=:../../../../../third_party/exactle/sw/wsf/common
VPATH+=:../src

SRC = app_disc.c
SRC+= app_main.c
SRC+= app_master.c
SRC+= app_server.c
SRC+= app_slave.c
SRC+= app_db.c
SRC+= app_hw.c
SRC+= app_ui.c
SRC+= fit_main.c
SRC+= da14581_hci_image.c
SRC+= hci_cmd.c
SRC+= hci_core.c
SRC+= hci_core_ps.c
SRC+= hci_drv.c
SRC+= hci_evt.c
SRC+= hci_tr.c
SRC+= hci_vs.c
SRC+= bas_main.c
SRC+= hrps_main.c
SRC+= svc_batt.c
SRC+= svc_bps.c
SRC+= svc_core.c
SRC+= svc_dis.c
SRC+= svc_gls.c
SRC+= svc_hidg.c
SRC+= svc_hidkb.c
SRC+= svc_hidm.c
SRC+= svc_hrs.c
SRC+= svc_hts.c
SRC+= svc_px.c
SRC+= svc_wp.c
SRC+= svc_wss.c
SRC+= cfg_stack.c
SRC+= hci_main.c
SRC+= bda.c
SRC+= bstream.c
SRC+= calc128.c
SRC+= crc32.c
SRC+= wstr.c
SRC+= wsf_assert.c
SRC+= wsf_math.c
SRC+= wsf_os.c
SRC+= wsf_trace.c
SRC+= wsf_buf.c
SRC+= wsf_efs.c
SRC+= wsf_msg.c
SRC+= wsf_queue.c
SRC+= wsf_sec.c
SRC+= wsf_sec_aes.c
SRC+= wsf_sec_cmac.c
SRC+= wsf_sec_ecc_debug.c
SRC+= wsf_timer.c
SRC+= exactle_fit.c
SRC+= am_util_delay.c
SRC+= am_util_faultisr.c
SRC+= am_util_stdio.c
SRC+= am_devices_adxl362.c
SRC+= am_devices_da14581.c
SRC+= am_devices_led.c
SRC+= startup_gcc.c

CSRC = $(filter %.c,$(SRC))
ASRC = $(filter %.s,$(SRC))

OBJS = $(CSRC:%.c=bin/%.o)
OBJS+= $(ASRC:%.s=bin/%.o)

DEPS = $(CSRC:%.c=bin/%.d)
DEPS+= $(ASRC:%.s=bin/%.d)

LIBS = ../../../bsp/eclipse_gcc/bin/libam_bsp.a
LIBS+= ../../../../../mcu/apollo/hal/eclipse_gcc/bin/libam_hal.a
LIBS+= ../../../../../third_party/exactle/projects/generic/stacklib/gcc/bin-cortex-m4/libstacklib.a

CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections
CFLAGS+= -MMD -MP -std=c99 -Wall -g
CFLAGS+= -O0
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)
CFLAGS+= 
GCCFLAGS = $(CFLAGS)

LIB_GCC_QPATH:='$(shell $(GCC) $(GCCFLAGS) -print-libgcc-file-name)'
LIB_GCC_EPATH:=$(shell $(GCC) $(GCCFLAGS) -print-libgcc-file-name | sed 's/ /\\ /g')
LIBC_QPATH:='$(shell $(GCC) $(GCCFLAGS) -print-file-name=libc.a)'
LIBC_EPATH:=$(shell $(GCC) $(GCCFLAGS) -print-file-name=libc.a | sed 's/ /\\ /g')
LIBM_QPATH:='$(shell $(GCC) $(GCCFLAGS) -print-file-name=libm.a)'
LIBM_EPATH:=$(shell $(GCC) $(GCCFLAGS) -print-file-name=libm.a | sed 's/ /\\ /g')
LFLAGS = --gc-sections --entry am_reset_isr -Map bin/$(TARGET).map

CPFLAGS = -Obinary

ODFLAGS = -S

#### Rules ####
.PHONY: all clean
all: bin bin/$(TARGET).bin

bin:
	mkdir -p $@

bin/%.o: %.c bin/%.d
	@+echo " Compiling $(WITHCC) $<" ;\
	$(CC) -c $(CFLAGS) $< -o $@

bin/%.o: %.s bin/%.d
	@+echo " Assembling $(WITHCC) $<" ;\
	$(CC) -c -mcpu=cortex-m4 -mthumb $< -o $@

bin/$(TARGET).axf: $(OBJS) $(LIBS) $(LIBM_EPATH) $(LIBC_EPATH) $(LIB_GCC_EPATH)
	@+echo " Linking $(WITHCC) $@" ;\
	$(LD) -T $(LINKER_FILE) $(LFLAGS) -o $@ $(OBJS) $(LIBS) $(LIBM_QPATH) $(LIBC_QPATH) $(LIB_GCC_QPATH)

bin/$(TARGET).bin: bin/$(TARGET).axf
	@+echo " Copying $(WITHCC) $@..." ;\
	$(CP) $(CPFLAGS) $< $@ ;\
	$(OD) $(ODFLAGS) $< > bin/$(TARGET).lst

clean:
	@+echo "Cleaning..." ;\
	rm -f $(OBJS) $(DEPS) \
		bin/$(TARGET).bin bin/$(TARGET).axf \
		bin/$(TARGET).lst bin/$(TARGET).map

bin/%.d: ;

# Automatically include any generated dependencies
-include $(DEPS)
endif

