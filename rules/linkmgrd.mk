# SONiC LINK ManaGeR Daemon package

SONIC_LINKMGRD_VERSION = 1.0.0-1
SONIC_LINKMGRD_PKG_NAME = linkmgrd

export SONIC_LINKMGRD_VERSION SONIC_LINKMGRD_PKG_NAME

SONIC_LINKMGRD = sonic-$(SONIC_LINKMGRD_PKG_NAME)_$(SONIC_LINKMGRD_VERSION)_$(CONFIGURED_ARCH).deb
$(SONIC_LINKMGRD)_SRC_PATH = $(SRC_PATH)/$(SONIC_LINKMGRD_PKG_NAME)
$(SONIC_LINKMGRD)_DEPENDS = $(LIBSWSSCOMMON_DEV) $(LIBSWSSCOMMON) $(LIBHIREDIS_DEV) $(LIBHIREDIS)

SONIC_DPKG_DEBS += $(SONIC_LINKMGRD)

SONIC_LINKMGRD_DBG = sonic-$(SONIC_LINKMGRD_PKG_NAME)-dbgsym_$(SONIC_LINKMGRD_VERSION)_$(CONFIGURED_ARCH).deb
$(SONIC_LINKMGRD)_DBG_DEPENDS = $(LIBSWSSCOMMON_DEV) $(LIBSWSSCOMMON_DBG) $(LIBHIREDIS_DEV) $(LIBHIREDIS_DBG)
$(eval $(call add_derived_package,$(SONIC_LINKMGRD),$(SONIC_LINKMGRD_DBG)))

export SONIC_LINKMGRD SONIC_LINKMGRD_DBG
