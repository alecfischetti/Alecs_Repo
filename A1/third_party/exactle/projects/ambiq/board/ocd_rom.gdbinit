###################################################################################################
#
# J-Link GDB startup script.
# 
#         $Date: 2014-08-26 12:42:22 -0700 (Tue, 26 Aug 2014) $
#         $Revision: 1763 $
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

# Macros
define restart
  monitor reset 0
  monitor reg r13 = (0x00000000)
  monitor reg pc = (0x00000004)
  continue
end

# Environment
#set verbose on

# Connect to device
target remote localhost:2331
monitor interface swd
monitor speed 10000

# Enable flash options
monitor endian little
monitor flash device = NRF51822
monitor clrbp

# Halt CPU and download image
monitor reset 0
load

# Setup windows
layout sources
tabset 2

# Run to main
break main
restart
