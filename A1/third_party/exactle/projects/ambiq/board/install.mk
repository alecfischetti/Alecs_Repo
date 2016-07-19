###################################################################################################
#
# Token generation make targets
# 
#         $Date: 2014-08-22 00:35:23 -0700 (Fri, 22 Aug 2014) $
#         $Revision: 1754 $
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
#    BIN

# Toolchain
GDB        := $(CROSS_COMPILE)gdb
JLINKEXE   := JLinkExe

#--------------------------------------------------------------------------------------------------
#     Targets
#--------------------------------------------------------------------------------------------------

install:
	@echo "+++ Execute: $(BIN)"
	@$(GDB) --se $(BIN) --command=$(ROOT_DIR)/projects/ambiq/board/ocd_rom.gdbinit
	
install.server:
	@cd $(ROOT_DIR)/platform/ambiq/tools/OpenOCD/code/tcl && \
		../../bin/linux64/usr/local/bin/openocd -f ../../openocd.cfg

install.dialog:
	@cd $(ROOT_DIR)/projects/ambiq/board && \
		$(JLINKEXE) -CommandFile load_da14580.jlinkexe; [ $$? -eq 1 ]

.PHONY: install install.server install.dialog
