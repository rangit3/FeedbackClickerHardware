#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/tirtos_cc13xx_cc26xx_2_18_00_03/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_00_03/products/tidrivers_cc13xx_cc26xx_2_16_01_13/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_00_03/products/bios_6_45_02_31/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_00_03/products/uia_2_00_05_50/packages;C:/ti/ccsv6/ccs_base;C:/workspace_v6_2/FeedbackClickerHardware/project_zero_app_cc2650launchxl/.config
override XDCROOT = C:/ti/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/tirtos_cc13xx_cc26xx_2_18_00_03/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_00_03/products/tidrivers_cc13xx_cc26xx_2_16_01_13/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_00_03/products/bios_6_45_02_31/packages;C:/ti/tirtos_cc13xx_cc26xx_2_18_00_03/products/uia_2_00_05_50/packages;C:/ti/ccsv6/ccs_base;C:/workspace_v6_2/FeedbackClickerHardware/project_zero_app_cc2650launchxl/.config;C:/ti/xdctools_3_32_00_06_core/packages;..
HOSTOS = Windows
endif
