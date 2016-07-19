###################################################################################################
#
# Makefile for building the sample application
# 
#         $Date: 2015-03-31 09:29:12 -0700 (Tue, 31 Mar 2015) $
#         $Revision: 2628 $
#  
# Copyright (c) 2013 Wicentric, Inc., all rights reserved.
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

#--------------------------------------------------------------------------------------------------
#     Project
#--------------------------------------------------------------------------------------------------

# GCC ARM cross toolchain
CROSS_COMPILE := arm-none-eabi-
CPU := cortex-m4

# Toolchain
OBJCOPY	   := $(CROSS_COMPILE)objcopy
SIZE       := $(CROSS_COMPILE)size

# Inputs
ROOT_DIR   := $(shell pwd)/../../../..
STACK_PRJ  := $(ROOT_DIR)/projects/generic/stacklib/gcc
LIBS       := $(STACK_PRJ)/bin-$(CPU)/libstacklib.a

# Output
INT_DIR    := obj-$(CPU)
BIN_DIR    := bin-$(CPU)
BIN        := $(BIN_DIR)/fit-obj.elf

# Options
BOARD      := evk-apolloBGA-da14581-sh
DEBUG      := 0
TOKEN      := 0

export DEBUG

#--------------------------------------------------------------------------------------------------
#     Targets
#--------------------------------------------------------------------------------------------------

all: $(LIBS) $(BIN) gentokens
	@$(OBJCOPY) -O binary $(BIN) $(BIN:.elf=.bin)
	@$(OBJCOPY) -O ihex $(BIN) $(BIN:.elf=.hex)
	@$(OBJDUMP) -t $(BIN) > $(BIN:.elf=.sym)
	@echo "+++ Object summary: $(INT_DIR)"
	@-find $(INT_DIR) -name *.o | xargs $(SIZE) -t
	@echo "+++ Binary summary: $(BIN)"
	@-$(SIZE) $(BIN)

ifeq ($(TOKEN),1)
gentokens: token
	@$(MAKE) -C $(STACK_PRJ) token
	@echo "+++ Generating token defintion file: $(TOKEN_FILE)"
	@cat $(STACK_LIB:.a=.tokens) >> $(TOKEN_FILE)
else
gentokens:
endif

ifeq ($(TOKEN),1)
clean: token.clean
else
clean:
endif
	@rm -rf $(INT_DIR)
	@rm -rf $(BIN_DIR)

include $(ROOT_DIR)/projects/ambiq/board/build.mk
include $(ROOT_DIR)/projects/ambiq/board/install.mk
ifeq ($(TOKEN),1)
include $(ROOT_DIR)/projects/generic/common/gcc/token.mk
endif

.PHONY: all gentokens clean
