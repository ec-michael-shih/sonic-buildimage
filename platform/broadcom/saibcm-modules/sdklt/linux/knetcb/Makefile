#
# $Copyright: Copyright 2018-2023 Broadcom. All rights reserved.
# The term 'Broadcom' refers to Broadcom Inc. and/or its subsidiaries.
# 
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License 
# version 2 as published by the Free Software Foundation.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# A copy of the GNU General Public License version 2 (GPLv2) can
# be found in the LICENSES folder.$
#
# Linux KNET Callback module.
#

# Include PMD library by default
ifneq (0,$(KPMD))

# Kernel module source directory
KMODDIR = $(CURDIR)

# Avoid creating links in original kernel module source directory
GENDIR = $(KMODDIR)/generated
ifneq ($(OUTPUT_DIR),)
GENDIR = $(OUTPUT_DIR)/knet/generated/knetcb
endif
knetcb: kpmd
	$(MAKE) -C $(GENDIR) all

# SDK make helper for stand-alone PMD kernel module
include $(SDK)/make/kpmd.mk

distclean::
	rm -rf $(GENDIR)

endif # KPMD

include Kbuild

ifeq ($(KERNELRELEASE),)

MOD_NAME = linux_ngknetcb

include $(SDK)/make/lkm.mk

endif

.PHONY: distclean

distclean::
