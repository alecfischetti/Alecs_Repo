###################################################################################################
#
# Token generation make targets
# 
#         $Date: 2015-09-09 09:35:06 -0700 (Wed, 09 Sep 2015) $
#         $Revision: 3809 $
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

# Parent makefile must export the following variables
#    CROSS_COMPILE
#    ROOT_DIR
#    LIBS
#    DEBUG
#    INIT_MASTER
#    BIN
#    STACK_PRJ
#    CPU
#    CFG_STACK

# Toolchain
CC         := $(CROSS_COMPILE)gcc
AR         := $(CROSS_COMPILE)ar
LD         := $(CROSS_COMPILE)gcc
DEP        := $(CROSS_COMPILE)gcc
OBJDUMP	   := $(CROSS_COMPILE)objdump

#--------------------------------------------------------------------------------------------------
#     Sources
#--------------------------------------------------------------------------------------------------

include sources*.mk

# Object file list
OBJ_FILES  := $(C_FILES:.c=.o)
OBJ_FILES  := $(subst $(ROOT_DIR)/,$(INT_DIR)/,$(OBJ_FILES))
OBJ_FILES  := $(subst ./,$(INT_DIR)/,$(OBJ_FILES))
DEP_FILES  := $(OBJ_FILES:.o=.d)

#--------------------------------------------------------------------------------------------------
#     Compilation flags
#--------------------------------------------------------------------------------------------------

# Hardware configuration
CFG_DEV    := HCI_TR_UART gcc AM_PACKAGE_BGA
#CFG_DEV    := HCI_TR_SPI
ifeq ($(INIT_MASTER),1)
CFG_DEV    += INIT_MASTER
endif

# Compiler flags
C_FLAGS    := -std=c99
C_FLAGS    += -Wall #-pedantic #-Werror
C_FLAGS    += -fno-common -fomit-frame-pointer
C_FLAGS    += -mcpu=$(CPU) -mthumb -mlittle-endian
C_FLAGS    += $(addprefix -I,$(INC_DIRS))
C_FLAGS    += $(addprefix -D,$(CFG_DEV))
ifeq ($(DEBUG),1)
C_FLAGS    += -O0 -g
C_FLAGS    += -DWSF_ASSERT_ENABLED=TRUE
#C_FLAGS    += -DWSF_BUF_FREE_CHECK=TRUE
C_FLAGS    += -DWSF_BUF_STATS=TRUE
else
C_FLAGS    += -Os
endif

# Linker flags
LD_FLAGS   := -T$(ROOT_DIR)/projects/ambiq/board/apollo.ld
LD_FLAGS   += -mthumb -mcpu=$(CPU)
LD_FLAGS   += -Wl,-Map=$(BIN:.elf=.map)
LD_FLAGS   += -Wl,--gc-sections

# Dependency flags
DEP_FLAGS  := $(C_FLAGS) -MM -MF

#--------------------------------------------------------------------------------------------------
#     Targets
#--------------------------------------------------------------------------------------------------

$(BIN): $(OBJ_FILES) $(LIBS)
	@echo "+++ Linking: $@"
	@mkdir -p $(BIN_DIR)
	@$(LD) -o $(BIN) $(LD_FLAGS) $(OBJ_FILES) $(LIBS)
	@$(OBJDUMP) -t $(BIN) > $(BIN:.elf=.sym)

$(STACK_LIB):
	@$(MAKE) -C $(STACK_PRJ) CROSS_COMPILE=$(CROSS_COMPILE) CPU=$(CPU) CFG_STACK="$(CFG_STACK)"

$(INT_DIR)/%.o: $(ROOT_DIR)/%.c
	@echo "+++ Compiling: $<"
	@mkdir -p $(dir $@)
	@$(CC) $(C_FLAGS) -DMODULE_ID=$(call FILE_HASH,$<) -c -o $@ $<
	@$(if $(DEP),$(DEP) $(DEP_FLAGS) $(subst .o,.d,$@) -MP -MT $@ $<,)

-include $(DEP_FILES)

.PHONY: $(LIBS)
