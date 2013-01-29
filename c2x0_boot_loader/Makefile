#
# (C) Copyright 2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# (C) Copyright 2002,2003 Motorola Inc.
# Xianghua Xiao,X.Xiao@motorola.com
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk
include ./config.mk

LIB	= $(obj)lib$(CPU).o

START	= start.o resetvec.o
COBJS   += uboot_init.o
COBJS   += uboot_print.o
COBJS   += vsprintf.o
COBJS   += fsl_c2x0_fw.o

ifeq ($(HIGH_PERF_MODE),y)
CFLAGS += -DHIGH_PERF
endif

ifeq ($(EP_TYPE),C293)
CFLAGS += -DC293_EP_TYPE
endif

ifeq ($(EP_TYPE), P4080)
CFLAGS += -DP4080_EP_TYPE
endif

ifeq ($(ENHANCE_KERNEL_TEST), y)
CFLAGS += -DENHANCE_KERNEL_TEST
endif

SRCS	:= $(START:.o=.S) $(SOBJS:.o=.S) $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(SOBJS) $(COBJS))
START	:= $(addprefix $(obj),$(START))

all:	$(obj).depend $(START) $(LIB)

$(LIB):	$(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
