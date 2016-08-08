#******************************************************************************
#
# Makefile - Rules for building the libraries, examples and docs.
#
# ${copyright}
#
# This is part of revision ${version} of the AmbiqSuite Development Package.
#
#******************************************************************************
TARGET = PlatformTest

#### Setup ####
TOOLCHAIN = arm-none-eabi
PART = apollo
CPU = cortex-m4
FPU = fpv4-sp-d16
FABI = softfp
WITHCC=w/GCC

LINKER_FILE = ./PlatformTest.ld
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

INCLUDES = -I../src
INCLUDES = -I../../../bsp
INCLUDES+= -I../../../../../mcu/apollo
INCLUDES+= -I../../../../../utils
INCLUDES+= -I../../../../../devices

VPATH+=:../../../../../utils
VPATH+=:../../../../../devices
VPATH+=:../src

SRC = test_main.c
SRC+= adxl362.c
SRC+= bmi160.c
SRC+= l3gd20h.c
SRC+= lis2dh12.c
SRC+= lis3mdl.c
SRC+= lsm6dsl.c
SRC+= psikick1001.c
SRC+= write_flash.c
SRC+= am_devices_da14581.c
SRC+= am_devices_adxl362.c
SRC+= am_devices_bmi160.c
SRC+= am_devices_l3gd20h.c
SRC+= am_devices_lis2dh12.c
SRC+= am_devices_lis3mdl.c
SRC+= am_devices_lsm6dsl.c
SRC+= am_devices_led.c
SRC+= am_util_delay.c
SRC+= am_util_faultisr.c
SRC+= am_util_stdio.c
SRC+= am_util_plot.c
SRC+= startup_gcc.c

CSRC = $(filter %.c,$(SRC))
ASRC = $(filter %.s,$(SRC))

OBJS = $(CSRC:%.c=bin/%.o)
OBJS+= $(ASRC:%.s=bin/%.o)

DEPS = $(CSRC:%.c=bin/%.d)
DEPS+= $(ASRC:%.s=bin/%.d)

LIBS = ../../../bsp/eclipse_gcc/bin/libam_bsp.a
##LIBS+= ../../MyHello/eclipse_gcc/bin/MyHello.a
LIBS+= ../../../../../mcu/apollo/hal/eclipse_gcc/bin/libam_hal.a

CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections
CFLAGS+= -MMD -MP -std=c99 -Wall -g
#CFLAGS+= -O3 -funroll-loops

CFLAGS+= -O -funroll-loops
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
	$(CC) -c -v $(CFLAGS) $< -o $@

bin/%.o: %.s bin/%.d
	@+echo " Assembling $(WITHCC) $<" ;\
	$(CC) -c -v -mcpu=cortex-m4 -mthumb $< -o $@

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
