##############################################################################
# Copyright (C) 2019 Mono Wireless Inc. All Rights Reserved.
# Released under MW-SLA-*J,*E (MONO WIRELESS SOFTWARE LICENSE
# AGREEMENT). 
##############################################################################
# USER PROJECT BUILD DEFINITION.
##############################################################################

#####################################################################
### set TWELITE model
TWELITE ?= BLUE
#TWELITE ?= RED

#####################################################################
### set application version (MUST SET THIS.)
VERSION_MAIN = 1
VERSION_SUB  = 4
VERSION_VAR  = 8

#####################################################################
### set an additional source file
###   the default file name is dirname.

### for C files compiled with gcc (must have .c suffix)
APPSRC += common.c
APPSRC += flash.c
APPSRC += cmd_config.c
APPSRC += cmd_misc.c
APPSRC += Interactive.c
APPSRC += sercmd_plus3.c
APPSRC += sercmd_ascii.c
APPSRC += sercmd_binary.c
APPSRC += sercmd_chat.c
APPSRC += sercmd_timeout.c
APPSRC += input_string.c
APPSRC += duplicate_checker.c
APPSRC += ser_out.c

### Additional Src/Include Path
# if set, find source files from given dirs.
#
APP_COMMON_SRC_DIR_ADD1 = ../../Common
#APP_COMMON_SRC_DIR_ADD2 = 
#APP_COMMON_SRC_DIR_ADD3 = 
#APP_COMMON_SRC_DIR_ADD4 =

#####################################################################
### set misc option for compiler

### C flags passed to gcc
# e.g. CFLAGS += -DMY_DEFS
CFLAGS += -DDEBUG_OUTPUT
CFLAGS += -DCONFIG_NORMAL

ifeq ($(BOARD),TWELITEUART)
	CFLAGS += -DTWELITEUART
endif

#ifneq ($(APP_UART_CONFIG),)
#  CFLAGS += -D$(APP_UART_CONFIG)
#endif
#
#ifneq ($(APP_UART_CONFIG),)
#ifneq ($(APP_UART_CONFIG),CONFIG_NORMAL)
#  TARGET_SUFF += _X$(APP_UART_CONFIG)X
#endif
#endif

### include opts
# e.g. INCFLAGS += -I../my_common_src/
#INCFLAGS +=

### optimize flag (default is -Os, normally no need to change)
#OPTFLAG=-O2

#####################################################################
### set MWSDK install path and include makefile.
MWSDK_PATH=$(realpath $(MWSDK_ROOT))
### must include mwx.mk (the makefile body part.)
include $(MWSDK_PATH)/MkFiles/mw.mk
#####################################################################
