/*
 * Copyright (C)  Michael Shih <michael_shih@edge-core.com>
 *
 * This module supports the accton fpga via pcie that read/write reg
 * mechanism to get QSFP/SFP status, eeprom info...etc.
 * This includes the:
 *     Accton as9736_64d UDB/LDB
 *
 * Based on:
 *    optoe.c fromDON BOLLINGER <don@thebollingers.org>
 * Copyright (C) 2017 Finisar Corp.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c-mux.h>
#include <linux/version.h>
#include <linux/stat.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/pci.h>

/***********************************************
 *       variable define
 * *********************************************/
#define DRVNAME               "as9736_64d_fpga"
/*
 * PCIE BAR0 address of UDB and LDB
 */
#define FPGA_PCIE_UDB_BAR0_ADDRESS    0xFB700000
#define FPGA_PCIE_LDB_BAR0_ADDRESS    0xFB400000
#define FPGA_PCIE_SMB_BAR0_ADDRESS    0xFB100000

#define PCI_VENDOR_ID_ACCTON          0x1113
#define PCI_DEVICE_ID_ACCTON          0x8664
#define PCI_SUBSYSTEM_ID_UDB          0x0000
#define PCI_SUBSYSTEM_ID_LDB          0x0001
#define PCI_SUBSYSTEM_ID_SMB          0x0002

#define QSFP_PRESENT_REG_OFFSET       0x1500
#define QSFP_LPMODE_REG_OFFSET        0x1550
#define QSFP_RESET_REG_OFFSET         0x1560

#define SFP_LDB_GPIO1_DATA_EN         0x1000
#define SFP_LDB_GPIO1_DATA_OUT        0x1004
#define SFP_LDB_GPIO1_DATA_IN         0x1008

#define ASLPC_DEV_UDB_CPLD1_PCIE_START_OFFST 0x400
#define ASLPC_DEV_UDB_CPLD2_PCIE_START_OFFST 0x500
#define ASLPC_DEV_LDB_CPLD1_PCIE_START_OFFST 0x400
#define ASLPC_DEV_LDB_CPLD2_PCIE_START_OFFST 0x500
#define ASLPC_DEV_SMB_CPLD_PCIE_START_OFFST  0x200

#define   BIT(x)                      x
#define   SFP_PORT0_TXDIS(x)          ((x) >> (11))
#define   SFP_PORT0_ABS(x)            ((x) >> (10))
#define   SFP_PORT0_TXFLT(x)          ((x) >> (9))
#define   SFP_PORT0_RXLOS(x)          ((x) >> (8))
#define   SFP_PORT1_TXDIS(x)          ((x) >> (3))
#define   SFP_PORT1_ABS(x)            ((x) >> (2))
#define   SFP_PORT1_TXFLT(x)          ((x) >> (1))
#define   SFP_PORT1_RXLOS(x)          ((x) >> (0))

#define QSFP_NUM_OF_PORT 64
#define SFP_NUM_OF_PORT  2
#define FPGA_NUM 3

#define TRANSCEIVER_PRESENT_ATTR_ID(index)    MODULE_PRESENT_##index
#define TRANSCEIVER_LPMODE_ATTR_ID(index)     MODULE_LPMODE_##index
#define TRANSCEIVER_RESET_ATTR_ID(index)      MODULE_RESET_##index
#define TRANSCEIVER_TX_DISABLE_ATTR_ID(index) MODULE_TX_DISABLE_##index
#define TRANSCEIVER_TX_FAULT_ATTR_ID(index)   MODULE_TX_FAULT_##index
#define TRANSCEIVER_RX_LOS_ATTR_ID(index)     MODULE_RX_LOS_##index

/*
 *PCIE port dev define
 */
#define EEPROM_SYSFS_NAME     "eeprom"

#define FPGA_UDB_QSFP_PORT_NUM  32
#define FPGA_LDB_QSFP_PORT_NUM  32
#define FPGA_QSFP_PORT_NUM      (FPGA_UDB_QSFP_PORT_NUM + FPGA_LDB_QSFP_PORT_NUM)
#define FPGA_LDB_SFP_PORT1_NO   65
#define FPGA_LDB_SFP_PORT2_NO   66
#define FPGA_LDB_SFP_PORT_NUM   2

#define QSFPDD_TYPE     0x18
/* fundamental unit of addressing for EEPROM */
#define OPTOE_PAGE_SIZE 128
/*
 * Single address devices (eg QSFP, CMIS) have 256 pages, plus the unpaged
 * low 128 bytes.  If the device does not support paging, it is
 * only 2 'pages' long.
 */
#define OPTOE_ARCH_PAGES 256
#define ONE_ADDR_EEPROM_SIZE ((1 + OPTOE_ARCH_PAGES) * OPTOE_PAGE_SIZE)
#define ONE_ADDR_EEPROM_UNPAGED_SIZE (2 * OPTOE_PAGE_SIZE)
/*
 * Dual address devices (eg SFP) have 256 pages, plus the unpaged
 * low 128 bytes, plus 256 bytes at 0x50.  If the device does not
 * support paging, it is 4 'pages' long.
 */
#define TWO_ADDR_EEPROM_SIZE ((3 + OPTOE_ARCH_PAGES) * OPTOE_PAGE_SIZE)
#define TWO_ADDR_EEPROM_UNPAGED_SIZE (4 * OPTOE_PAGE_SIZE)
#define TWO_ADDR_NO_0X51_SIZE (2 * OPTOE_PAGE_SIZE)

/* a few constants to find our way around the EEPROM */
#define OPTOE_PAGE_SELECT_REG   0x7F
#define ONE_ADDR_PAGEABLE_REG   0x02
#define QSFP_NOT_PAGEABLE      (1<<2)
#define CMIS_NOT_PAGEABLE      (1<<7)
#define TWO_ADDR_PAGEABLE_REG   0x40
#define TWO_ADDR_PAGEABLE      (1<<4)
#define TWO_ADDR_0X51_REG       92
#define TWO_ADDR_0X51_SUPP     (1<<6)
#define OPTOE_READ_OP           0
#define OPTOE_WRITE_OP          1
#define OPTOE_EOF               0  /* used for access beyond end of device */
#define TWO_ADDR_0X51           0x51
#define EEPROM_ALLOW_SET_LEN    1

/*
 * flags to distinguish one-address (QSFP family) from two-address (SFP family)
 * and one-address Common Management Interface Specification (CMIS family)
 */
#define ONE_ADDR 1
#define TWO_ADDR 2
#define CMIS_ADDR 3

/* I2C Controller Management Registers */
#define PCIE_FPGA_I2C_MGMT_RTC0_PROFILE_0         0x2008

/* I2C Real Time Control Registers */
#define PCIE_FPGA_I2C_CONTROL_RTC0_CONFIG_0       0x2050
#define PCIE_FPGA_I2C_CONTROL_RTC0_CONFIG_1       0x2054
#define PCIE_FPGA_I2C_CONTROL_RTC0_STATUS_0       0x2060

/* I2C RTC Data Block */
#define PCIE_FPGA_I2C_RTC_WRITE_DATA_REG_0        0x5000
#define PCIE_FPGA_I2C_RTC_READ_DATA_REG_0         0xA000

#define PCIE_FPGA_I2C_MAX_LEN                     128
#define PCIE_FPGA_I2C_NEW_TRIGGER_VALUE           0x80000000

/***********************************************
 *       structure & variable declare
 * *********************************************/
static char FPGA_NAME[FPGA_NUM][10] = {"UDB FPGA", "LDB FPGA", "SMB FPGA"};

typedef struct pci_fpga_device_s {
    u32  base_addr;
    u16  id;
    u16  size;
    u32  qsfp_present;
    u32  qsfp_lpmode;
    u32  qsfp_reset;
    u32  sfp_input_data;
    u32  sfp_output_data;
    u16  aslpc_cpld1_offset;
    u16  aslpc_cpld2_offset;
} pci_fpga_device_t;

/*fpga port status*/
struct as9736_64d_fpga_data {
    struct mutex              driver_lock;
    struct platform_device    *pdev;
    pci_fpga_device_t         pci_fpga_dev[3]; /*UDB, LDB and SMB*/
    char                      valid;           /* != 0 if registers are valid */
    u32                       udb_version;
    u32                       ldb_version;
    u32                       smb_version;
    unsigned long             last_updated;    /* In jiffies */
};

static struct as9736_64d_fpga_data  *fpga_ctl = NULL;

struct mutex update_lock;

struct eeprom_bin_private_data {
    u32    base_addr;
    int    port_num;
    int    fpga_type;
    int    pageable;
    int    sfp_support_a2;
    int    i2c_slave_addr;
    int    i2c_mgmt_rtc0_profile;
    int    i2c_contrl_rtc0_config_0;
    int    i2c_contrl_rtc0_config_1;
    int    i2c_contrl_rtc0_stats;
    int    i2c_rtc_read_data;
    int    i2c_rtc_write_data;
};

struct pcie_fpga_dev_platform_data {
    int    port_num;
    char   name[10];      /*ex: port1*/
    char   dev_name[10];  /*ex: optoe1*/
    int    dev_class;
    int    fpga_type;
    struct bin_attribute eeprom_bin;
};

/***********************************************
 *       macro define
 * *********************************************/
#define pcie_err(fmt, args...) \
        printk(KERN_ERR "[accton_pcie_sfp_driver]: " fmt " ", ##args)

#define pcie_info(fmt, args...) \
        printk(KERN_INFO "[accton_pcie_sfp_driver]: " fmt " ", ##args)

/* UDB */
 /*c from 0 to 31*/
#define pcie_udb_qsfp_device_port(c){                                    \
        .name                   = "pcie_udb_fpga_device",                \
        .id                     = c,                                     \
        .dev                    = {                                      \
                    .platform_data = &pcie_udb_dev_platform_data[c],     \
                    .release       = device_release,                     \
        },                                                               \
}
 /*c from 1*/
#define pcie_udb_qsfp_platform_data_init(c){                             \
        .port_num                   = c,                                 \
        .dev_name                   = "optoe1",                          \
        .dev_class                  = 1,                                 \
        .fpga_type = PCIE_FPGA_TYPE_UDB,                                 \
        .eeprom_bin                 = {                                  \
                    .private = &pcie_udb_eeprom_bin_private_data[c-1],   \
        },                                                               \
}
 /*c from 1*/
#define eeprom_udb_private_data_port_init(c){                            \
        .base_addr                   = FPGA_PCIE_UDB_BAR0_ADDRESS,       \
        .port_num                    = c,                                \
        .fpga_type                   = PCIE_FPGA_TYPE_UDB,               \
        .i2c_slave_addr              = 0x50,                             \
        .i2c_mgmt_rtc0_profile       = PCIE_FPGA_I2C_MGMT_RTC0_PROFILE_0   + 0x100*(c-1),     \
        .i2c_contrl_rtc0_config_0    = PCIE_FPGA_I2C_CONTROL_RTC0_CONFIG_0 + 0x100*(c-1),     \
        .i2c_contrl_rtc0_config_1    = PCIE_FPGA_I2C_CONTROL_RTC0_CONFIG_1 + 0x100*(c-1),     \
        .i2c_contrl_rtc0_stats       = PCIE_FPGA_I2C_CONTROL_RTC0_STATUS_0 + 0x100*(c-1),     \
        .i2c_rtc_read_data           = PCIE_FPGA_I2C_RTC_READ_DATA_REG_0   + 0x200*(c-1),     \
        .i2c_rtc_write_data          = PCIE_FPGA_I2C_RTC_WRITE_DATA_REG_0  + 0x200*(c-1),     \
}

/* LDB */
 /*c from 0 to 31*/
#define pcie_ldb_qsfp_device_port(c){                                    \
        .name                   = "pcie_ldb_fpga_device",                \
        .id                     = c,                                     \
        .dev                    = {                                      \
                    .platform_data = &pcie_ldb_dev_platform_data[c],     \
                    .release       = device_release,                     \
        },                                                               \
}
 /*c from 32 to 33*/
#define pcie_ldb_sfp_device_port(c){                                     \
        .name                   = "pcie_ldb_fpga_device",                \
        .id                     = c,                                     \
        .dev                    = {                                      \
                    .platform_data = &pcie_ldb_dev_platform_data[c],     \
                    .release       = device_release,                     \
        },                                                               \
}

/*c from 1 to 32*/
#define pcie_ldb_qsfp_platform_data_init(c){                             \
        .port_num                   = c,                                 \
        .dev_name                   = "optoe1",                          \
        .dev_class                  = 1,                                 \
        .fpga_type = PCIE_FPGA_TYPE_LDB,                                 \
        .eeprom_bin                 = {                                  \
                    .private = &pcie_ldb_eeprom_bin_private_data[c-1],   \
        },                                                               \
}
/*c = 33, 34*/
#define pcie_ldb_sfp_platform_data_init(c){                              \
        .port_num                   = c,                                 \
        .dev_name                   = "optoe2",                          \
        .dev_class                  = 2,                                 \
        .fpga_type = PCIE_FPGA_TYPE_LDB,                                 \
        .eeprom_bin                 = {                                  \
                    .private = &pcie_ldb_eeprom_bin_private_data[c-1],   \
        },                                                               \
}
/*c from 1 to 32, 33, 34*/
#define eeprom_ldb_private_data_port_init(c){                                                 \
        .base_addr                   = FPGA_PCIE_LDB_BAR0_ADDRESS,                            \
        .port_num                    = c + 32,                                                \
        .fpga_type                   = PCIE_FPGA_TYPE_LDB,                                    \
        .i2c_slave_addr              = 0x50,                                                  \
        .i2c_mgmt_rtc0_profile       = PCIE_FPGA_I2C_MGMT_RTC0_PROFILE_0   + 0x100*(c-1),     \
        .i2c_contrl_rtc0_config_0    = PCIE_FPGA_I2C_CONTROL_RTC0_CONFIG_0 + 0x100*(c-1),     \
        .i2c_contrl_rtc0_config_1    = PCIE_FPGA_I2C_CONTROL_RTC0_CONFIG_1 + 0x100*(c-1),     \
        .i2c_contrl_rtc0_stats       = PCIE_FPGA_I2C_CONTROL_RTC0_STATUS_0 + 0x100*(c-1),     \
        .i2c_rtc_read_data           = PCIE_FPGA_I2C_RTC_READ_DATA_REG_0   + 0x200*(c-1),     \
        .i2c_rtc_write_data          = PCIE_FPGA_I2C_RTC_WRITE_DATA_REG_0  + 0x200*(c-1),     \
}

/***********************************************
 *       enum define
 * *********************************************/
enum fpga_type_t {
   PCIE_FPGA_UDB = 0,
   PCIE_FPGA_LDB,
   PCIE_FPGA_SMB
};

enum fpga_set_function_type_t {
   PCIE_FPGA_SET_LPMODE,
   PCIE_FPGA_SET_RESET,
   PCIE_FPGA_SET_TX_DISABLE
};

enum fpga_sysfs_attributes {
    /* transceiver attributes */
    TRANSCEIVER_PRESENT_ATTR_ID(1),
    TRANSCEIVER_PRESENT_ATTR_ID(2),
    TRANSCEIVER_PRESENT_ATTR_ID(3),
    TRANSCEIVER_PRESENT_ATTR_ID(4),
    TRANSCEIVER_PRESENT_ATTR_ID(5),
    TRANSCEIVER_PRESENT_ATTR_ID(6),
    TRANSCEIVER_PRESENT_ATTR_ID(7),
    TRANSCEIVER_PRESENT_ATTR_ID(8),
    TRANSCEIVER_PRESENT_ATTR_ID(9),
    TRANSCEIVER_PRESENT_ATTR_ID(10),
    TRANSCEIVER_PRESENT_ATTR_ID(11),
    TRANSCEIVER_PRESENT_ATTR_ID(12),
    TRANSCEIVER_PRESENT_ATTR_ID(13),
    TRANSCEIVER_PRESENT_ATTR_ID(14),
    TRANSCEIVER_PRESENT_ATTR_ID(15),
    TRANSCEIVER_PRESENT_ATTR_ID(16),
    TRANSCEIVER_PRESENT_ATTR_ID(17),
    TRANSCEIVER_PRESENT_ATTR_ID(18),
    TRANSCEIVER_PRESENT_ATTR_ID(19),
    TRANSCEIVER_PRESENT_ATTR_ID(20),
    TRANSCEIVER_PRESENT_ATTR_ID(21),
    TRANSCEIVER_PRESENT_ATTR_ID(22),
    TRANSCEIVER_PRESENT_ATTR_ID(23),
    TRANSCEIVER_PRESENT_ATTR_ID(24),
    TRANSCEIVER_PRESENT_ATTR_ID(25),
    TRANSCEIVER_PRESENT_ATTR_ID(26),
    TRANSCEIVER_PRESENT_ATTR_ID(27),
    TRANSCEIVER_PRESENT_ATTR_ID(28),
    TRANSCEIVER_PRESENT_ATTR_ID(29),
    TRANSCEIVER_PRESENT_ATTR_ID(30),
    TRANSCEIVER_PRESENT_ATTR_ID(31),
    TRANSCEIVER_PRESENT_ATTR_ID(32),
    TRANSCEIVER_PRESENT_ATTR_ID(33),
    TRANSCEIVER_PRESENT_ATTR_ID(34),
    TRANSCEIVER_PRESENT_ATTR_ID(35),
    TRANSCEIVER_PRESENT_ATTR_ID(36),
    TRANSCEIVER_PRESENT_ATTR_ID(37),
    TRANSCEIVER_PRESENT_ATTR_ID(38),
    TRANSCEIVER_PRESENT_ATTR_ID(39),
    TRANSCEIVER_PRESENT_ATTR_ID(40),
    TRANSCEIVER_PRESENT_ATTR_ID(41),
    TRANSCEIVER_PRESENT_ATTR_ID(42),
    TRANSCEIVER_PRESENT_ATTR_ID(43),
    TRANSCEIVER_PRESENT_ATTR_ID(44),
    TRANSCEIVER_PRESENT_ATTR_ID(45),
    TRANSCEIVER_PRESENT_ATTR_ID(46),
    TRANSCEIVER_PRESENT_ATTR_ID(47),
    TRANSCEIVER_PRESENT_ATTR_ID(48),
    TRANSCEIVER_PRESENT_ATTR_ID(49),
    TRANSCEIVER_PRESENT_ATTR_ID(50),
    TRANSCEIVER_PRESENT_ATTR_ID(51),
    TRANSCEIVER_PRESENT_ATTR_ID(52),
    TRANSCEIVER_PRESENT_ATTR_ID(53),
    TRANSCEIVER_PRESENT_ATTR_ID(54),
    TRANSCEIVER_PRESENT_ATTR_ID(55),
    TRANSCEIVER_PRESENT_ATTR_ID(56),
    TRANSCEIVER_PRESENT_ATTR_ID(57),
    TRANSCEIVER_PRESENT_ATTR_ID(58),
    TRANSCEIVER_PRESENT_ATTR_ID(59),
    TRANSCEIVER_PRESENT_ATTR_ID(60),
    TRANSCEIVER_PRESENT_ATTR_ID(61),
    TRANSCEIVER_PRESENT_ATTR_ID(62),
    TRANSCEIVER_PRESENT_ATTR_ID(63),
    TRANSCEIVER_PRESENT_ATTR_ID(64),
    TRANSCEIVER_PRESENT_ATTR_ID(65),
    TRANSCEIVER_PRESENT_ATTR_ID(66),
    /*Reset*/
    TRANSCEIVER_RESET_ATTR_ID(1),
    TRANSCEIVER_RESET_ATTR_ID(2),
    TRANSCEIVER_RESET_ATTR_ID(3),
    TRANSCEIVER_RESET_ATTR_ID(4),
    TRANSCEIVER_RESET_ATTR_ID(5),
    TRANSCEIVER_RESET_ATTR_ID(6),
    TRANSCEIVER_RESET_ATTR_ID(7),
    TRANSCEIVER_RESET_ATTR_ID(8),
    TRANSCEIVER_RESET_ATTR_ID(9),
    TRANSCEIVER_RESET_ATTR_ID(10),
    TRANSCEIVER_RESET_ATTR_ID(11),
    TRANSCEIVER_RESET_ATTR_ID(12),
    TRANSCEIVER_RESET_ATTR_ID(13),
    TRANSCEIVER_RESET_ATTR_ID(14),
    TRANSCEIVER_RESET_ATTR_ID(15),
    TRANSCEIVER_RESET_ATTR_ID(16),
    TRANSCEIVER_RESET_ATTR_ID(17),
    TRANSCEIVER_RESET_ATTR_ID(18),
    TRANSCEIVER_RESET_ATTR_ID(19),
    TRANSCEIVER_RESET_ATTR_ID(20),
    TRANSCEIVER_RESET_ATTR_ID(21),
    TRANSCEIVER_RESET_ATTR_ID(22),
    TRANSCEIVER_RESET_ATTR_ID(23),
    TRANSCEIVER_RESET_ATTR_ID(24),
    TRANSCEIVER_RESET_ATTR_ID(25),
    TRANSCEIVER_RESET_ATTR_ID(26),
    TRANSCEIVER_RESET_ATTR_ID(27),
    TRANSCEIVER_RESET_ATTR_ID(28),
    TRANSCEIVER_RESET_ATTR_ID(29),
    TRANSCEIVER_RESET_ATTR_ID(30),
    TRANSCEIVER_RESET_ATTR_ID(31),
    TRANSCEIVER_RESET_ATTR_ID(32),
    TRANSCEIVER_RESET_ATTR_ID(33),
    TRANSCEIVER_RESET_ATTR_ID(34),
    TRANSCEIVER_RESET_ATTR_ID(35),
    TRANSCEIVER_RESET_ATTR_ID(36),
    TRANSCEIVER_RESET_ATTR_ID(37),
    TRANSCEIVER_RESET_ATTR_ID(38),
    TRANSCEIVER_RESET_ATTR_ID(39),
    TRANSCEIVER_RESET_ATTR_ID(40),
    TRANSCEIVER_RESET_ATTR_ID(41),
    TRANSCEIVER_RESET_ATTR_ID(42),
    TRANSCEIVER_RESET_ATTR_ID(43),
    TRANSCEIVER_RESET_ATTR_ID(44),
    TRANSCEIVER_RESET_ATTR_ID(45),
    TRANSCEIVER_RESET_ATTR_ID(46),
    TRANSCEIVER_RESET_ATTR_ID(47),
    TRANSCEIVER_RESET_ATTR_ID(48),
    TRANSCEIVER_RESET_ATTR_ID(49),
    TRANSCEIVER_RESET_ATTR_ID(50),
    TRANSCEIVER_RESET_ATTR_ID(51),
    TRANSCEIVER_RESET_ATTR_ID(52),
    TRANSCEIVER_RESET_ATTR_ID(53),
    TRANSCEIVER_RESET_ATTR_ID(54),
    TRANSCEIVER_RESET_ATTR_ID(55),
    TRANSCEIVER_RESET_ATTR_ID(56),
    TRANSCEIVER_RESET_ATTR_ID(57),
    TRANSCEIVER_RESET_ATTR_ID(58),
    TRANSCEIVER_RESET_ATTR_ID(59),
    TRANSCEIVER_RESET_ATTR_ID(60),
    TRANSCEIVER_RESET_ATTR_ID(61),
    TRANSCEIVER_RESET_ATTR_ID(62),
    TRANSCEIVER_RESET_ATTR_ID(63),
    TRANSCEIVER_RESET_ATTR_ID(64),
    TRANSCEIVER_LPMODE_ATTR_ID(1),
    TRANSCEIVER_LPMODE_ATTR_ID(2),
    TRANSCEIVER_LPMODE_ATTR_ID(3),
    TRANSCEIVER_LPMODE_ATTR_ID(4),
    TRANSCEIVER_LPMODE_ATTR_ID(5),
    TRANSCEIVER_LPMODE_ATTR_ID(6),
    TRANSCEIVER_LPMODE_ATTR_ID(7),
    TRANSCEIVER_LPMODE_ATTR_ID(8),
    TRANSCEIVER_LPMODE_ATTR_ID(9),
    TRANSCEIVER_LPMODE_ATTR_ID(10),
    TRANSCEIVER_LPMODE_ATTR_ID(11),
    TRANSCEIVER_LPMODE_ATTR_ID(12),
    TRANSCEIVER_LPMODE_ATTR_ID(13),
    TRANSCEIVER_LPMODE_ATTR_ID(14),
    TRANSCEIVER_LPMODE_ATTR_ID(15),
    TRANSCEIVER_LPMODE_ATTR_ID(16),
    TRANSCEIVER_LPMODE_ATTR_ID(17),
    TRANSCEIVER_LPMODE_ATTR_ID(18),
    TRANSCEIVER_LPMODE_ATTR_ID(19),
    TRANSCEIVER_LPMODE_ATTR_ID(20),
    TRANSCEIVER_LPMODE_ATTR_ID(21),
    TRANSCEIVER_LPMODE_ATTR_ID(22),
    TRANSCEIVER_LPMODE_ATTR_ID(23),
    TRANSCEIVER_LPMODE_ATTR_ID(24),
    TRANSCEIVER_LPMODE_ATTR_ID(25),
    TRANSCEIVER_LPMODE_ATTR_ID(26),
    TRANSCEIVER_LPMODE_ATTR_ID(27),
    TRANSCEIVER_LPMODE_ATTR_ID(28),
    TRANSCEIVER_LPMODE_ATTR_ID(29),
    TRANSCEIVER_LPMODE_ATTR_ID(30),
    TRANSCEIVER_LPMODE_ATTR_ID(31),
    TRANSCEIVER_LPMODE_ATTR_ID(32),
    TRANSCEIVER_LPMODE_ATTR_ID(33),
    TRANSCEIVER_LPMODE_ATTR_ID(34),
    TRANSCEIVER_LPMODE_ATTR_ID(35),
    TRANSCEIVER_LPMODE_ATTR_ID(36),
    TRANSCEIVER_LPMODE_ATTR_ID(37),
    TRANSCEIVER_LPMODE_ATTR_ID(38),
    TRANSCEIVER_LPMODE_ATTR_ID(39),
    TRANSCEIVER_LPMODE_ATTR_ID(40),
    TRANSCEIVER_LPMODE_ATTR_ID(41),
    TRANSCEIVER_LPMODE_ATTR_ID(42),
    TRANSCEIVER_LPMODE_ATTR_ID(43),
    TRANSCEIVER_LPMODE_ATTR_ID(44),
    TRANSCEIVER_LPMODE_ATTR_ID(45),
    TRANSCEIVER_LPMODE_ATTR_ID(46),
    TRANSCEIVER_LPMODE_ATTR_ID(47),
    TRANSCEIVER_LPMODE_ATTR_ID(48),
    TRANSCEIVER_LPMODE_ATTR_ID(49),
    TRANSCEIVER_LPMODE_ATTR_ID(50),
    TRANSCEIVER_LPMODE_ATTR_ID(51),
    TRANSCEIVER_LPMODE_ATTR_ID(52),
    TRANSCEIVER_LPMODE_ATTR_ID(53),
    TRANSCEIVER_LPMODE_ATTR_ID(54),
    TRANSCEIVER_LPMODE_ATTR_ID(55),
    TRANSCEIVER_LPMODE_ATTR_ID(56),
    TRANSCEIVER_LPMODE_ATTR_ID(57),
    TRANSCEIVER_LPMODE_ATTR_ID(58),
    TRANSCEIVER_LPMODE_ATTR_ID(59),
    TRANSCEIVER_LPMODE_ATTR_ID(60),
    TRANSCEIVER_LPMODE_ATTR_ID(61),
    TRANSCEIVER_LPMODE_ATTR_ID(62),
    TRANSCEIVER_LPMODE_ATTR_ID(63),
    TRANSCEIVER_LPMODE_ATTR_ID(64),
    TRANSCEIVER_TX_DISABLE_ATTR_ID(65),
    TRANSCEIVER_TX_DISABLE_ATTR_ID(66),
    TRANSCEIVER_TX_FAULT_ATTR_ID(65),
    TRANSCEIVER_TX_FAULT_ATTR_ID(66),
    TRANSCEIVER_RX_LOS_ATTR_ID(65),
    TRANSCEIVER_RX_LOS_ATTR_ID(66),
    PCIE_FPGA_UDB_VERSION,
    PCIE_FPGA_LDB_VERSION,
    PCIE_FPGA_SMB_VERSION,
};

enum pcie_type_e {
   PCIE_FPGA_TYPE_UDB = 0,
   PCIE_FPGA_TYPE_LDB
};

enum eeprom_page_type_e {
   EEPROM_LOWER_PAGE = -1,
   EEPROM_UPPER_PAGE
};

enum port_sysfs_attributes {
   PORT_SYSFS_NAME_ID = 1,
   PORT_SYSFS_PORT_NAME_ID,
   PORT_SYSFS_DEV_CLASS_ID
};

/***********************************************
 *       function declare
 * *********************************************/
static ssize_t port_status_read(struct device *dev, struct device_attribute *da,
             char *buf);
static ssize_t port_status_write(struct device *dev, struct device_attribute *da,
            const char *buf, size_t count);

static ssize_t port_read(struct device *dev, struct device_attribute *da,
             char *buf);
static ssize_t port_write(struct device *dev, struct device_attribute *da,
            const char *buf, size_t count);

static int fpga_i2c_ready_to_read(struct bin_attribute *attr, int page_type, int i2c_slave_addr);

/* UDB */
/*init eeprom private data*/
static struct eeprom_bin_private_data pcie_udb_eeprom_bin_private_data[] = {
    eeprom_udb_private_data_port_init(1),
    eeprom_udb_private_data_port_init(2),
    eeprom_udb_private_data_port_init(3),
    eeprom_udb_private_data_port_init(4),
    eeprom_udb_private_data_port_init(5),
    eeprom_udb_private_data_port_init(6),
    eeprom_udb_private_data_port_init(7),
    eeprom_udb_private_data_port_init(8),
    eeprom_udb_private_data_port_init(9),
    eeprom_udb_private_data_port_init(10),
    eeprom_udb_private_data_port_init(11),
    eeprom_udb_private_data_port_init(12),
    eeprom_udb_private_data_port_init(13),
    eeprom_udb_private_data_port_init(14),
    eeprom_udb_private_data_port_init(15),
    eeprom_udb_private_data_port_init(16),
    eeprom_udb_private_data_port_init(17),
    eeprom_udb_private_data_port_init(18),
    eeprom_udb_private_data_port_init(19),
    eeprom_udb_private_data_port_init(20),
    eeprom_udb_private_data_port_init(21),
    eeprom_udb_private_data_port_init(22),
    eeprom_udb_private_data_port_init(23),
    eeprom_udb_private_data_port_init(24),
    eeprom_udb_private_data_port_init(25),
    eeprom_udb_private_data_port_init(26),
    eeprom_udb_private_data_port_init(27),
    eeprom_udb_private_data_port_init(28),
    eeprom_udb_private_data_port_init(29),
    eeprom_udb_private_data_port_init(30),
    eeprom_udb_private_data_port_init(31),
    eeprom_udb_private_data_port_init(32),
};

/*init device platform data*/
static struct pcie_fpga_dev_platform_data pcie_udb_dev_platform_data[] = {
    pcie_udb_qsfp_platform_data_init(1),
    pcie_udb_qsfp_platform_data_init(2),
    pcie_udb_qsfp_platform_data_init(3),
    pcie_udb_qsfp_platform_data_init(4),
    pcie_udb_qsfp_platform_data_init(5),
    pcie_udb_qsfp_platform_data_init(6),
    pcie_udb_qsfp_platform_data_init(7),
    pcie_udb_qsfp_platform_data_init(8),
    pcie_udb_qsfp_platform_data_init(9),
    pcie_udb_qsfp_platform_data_init(10),
    pcie_udb_qsfp_platform_data_init(11),
    pcie_udb_qsfp_platform_data_init(12),
    pcie_udb_qsfp_platform_data_init(13),
    pcie_udb_qsfp_platform_data_init(14),
    pcie_udb_qsfp_platform_data_init(15),
    pcie_udb_qsfp_platform_data_init(16),
    pcie_udb_qsfp_platform_data_init(17),
    pcie_udb_qsfp_platform_data_init(18),
    pcie_udb_qsfp_platform_data_init(19),
    pcie_udb_qsfp_platform_data_init(20),
    pcie_udb_qsfp_platform_data_init(21),
    pcie_udb_qsfp_platform_data_init(22),
    pcie_udb_qsfp_platform_data_init(23),
    pcie_udb_qsfp_platform_data_init(24),
    pcie_udb_qsfp_platform_data_init(25),
    pcie_udb_qsfp_platform_data_init(26),
    pcie_udb_qsfp_platform_data_init(27),
    pcie_udb_qsfp_platform_data_init(28),
    pcie_udb_qsfp_platform_data_init(29),
    pcie_udb_qsfp_platform_data_init(30),
    pcie_udb_qsfp_platform_data_init(31),
    pcie_udb_qsfp_platform_data_init(32),
};

/* LDB */
/*init eeprom private data*/
static struct eeprom_bin_private_data pcie_ldb_eeprom_bin_private_data[] = {
    eeprom_ldb_private_data_port_init(1),
    eeprom_ldb_private_data_port_init(2),
    eeprom_ldb_private_data_port_init(3),
    eeprom_ldb_private_data_port_init(4),
    eeprom_ldb_private_data_port_init(5),
    eeprom_ldb_private_data_port_init(6),
    eeprom_ldb_private_data_port_init(7),
    eeprom_ldb_private_data_port_init(8),
    eeprom_ldb_private_data_port_init(9),
    eeprom_ldb_private_data_port_init(10),
    eeprom_ldb_private_data_port_init(11),
    eeprom_ldb_private_data_port_init(12),
    eeprom_ldb_private_data_port_init(13),
    eeprom_ldb_private_data_port_init(14),
    eeprom_ldb_private_data_port_init(15),
    eeprom_ldb_private_data_port_init(16),
    eeprom_ldb_private_data_port_init(17),
    eeprom_ldb_private_data_port_init(18),
    eeprom_ldb_private_data_port_init(19),
    eeprom_ldb_private_data_port_init(20),
    eeprom_ldb_private_data_port_init(21),
    eeprom_ldb_private_data_port_init(22),
    eeprom_ldb_private_data_port_init(23),
    eeprom_ldb_private_data_port_init(24),
    eeprom_ldb_private_data_port_init(25),
    eeprom_ldb_private_data_port_init(26),
    eeprom_ldb_private_data_port_init(27),
    eeprom_ldb_private_data_port_init(28),
    eeprom_ldb_private_data_port_init(29),
    eeprom_ldb_private_data_port_init(30),
    eeprom_ldb_private_data_port_init(31),
    eeprom_ldb_private_data_port_init(32),
    eeprom_ldb_private_data_port_init(33), /*sfp: port65*/
    eeprom_ldb_private_data_port_init(34), /*sfp: port66*/
};

/*init device platform data*/
static struct pcie_fpga_dev_platform_data pcie_ldb_dev_platform_data[] = {
    pcie_ldb_qsfp_platform_data_init(1),
    pcie_ldb_qsfp_platform_data_init(2),
    pcie_ldb_qsfp_platform_data_init(3),
    pcie_ldb_qsfp_platform_data_init(4),
    pcie_ldb_qsfp_platform_data_init(5),
    pcie_ldb_qsfp_platform_data_init(6),
    pcie_ldb_qsfp_platform_data_init(7),
    pcie_ldb_qsfp_platform_data_init(8),
    pcie_ldb_qsfp_platform_data_init(9),
    pcie_ldb_qsfp_platform_data_init(10),
    pcie_ldb_qsfp_platform_data_init(11),
    pcie_ldb_qsfp_platform_data_init(12),
    pcie_ldb_qsfp_platform_data_init(13),
    pcie_ldb_qsfp_platform_data_init(14),
    pcie_ldb_qsfp_platform_data_init(15),
    pcie_ldb_qsfp_platform_data_init(16),
    pcie_ldb_qsfp_platform_data_init(17),
    pcie_ldb_qsfp_platform_data_init(18),
    pcie_ldb_qsfp_platform_data_init(19),
    pcie_ldb_qsfp_platform_data_init(20),
    pcie_ldb_qsfp_platform_data_init(21),
    pcie_ldb_qsfp_platform_data_init(22),
    pcie_ldb_qsfp_platform_data_init(23),
    pcie_ldb_qsfp_platform_data_init(24),
    pcie_ldb_qsfp_platform_data_init(25),
    pcie_ldb_qsfp_platform_data_init(26),
    pcie_ldb_qsfp_platform_data_init(27),
    pcie_ldb_qsfp_platform_data_init(28),
    pcie_ldb_qsfp_platform_data_init(29),
    pcie_ldb_qsfp_platform_data_init(30),
    pcie_ldb_qsfp_platform_data_init(31),
    pcie_ldb_qsfp_platform_data_init(32),
    pcie_ldb_sfp_platform_data_init(33),  /*sfp: port65*/
    pcie_ldb_sfp_platform_data_init(34),  /*sfp: port66*/
};

static void device_release(struct device *dev)
{
    return;
}

/*UDB platform device*/
static struct platform_device pcie_udb_qsfp_device[] = {
    pcie_udb_qsfp_device_port(0),
    pcie_udb_qsfp_device_port(1),
    pcie_udb_qsfp_device_port(2),
    pcie_udb_qsfp_device_port(3),
    pcie_udb_qsfp_device_port(4),
    pcie_udb_qsfp_device_port(5),
    pcie_udb_qsfp_device_port(6),
    pcie_udb_qsfp_device_port(7),
    pcie_udb_qsfp_device_port(8),
    pcie_udb_qsfp_device_port(9),
    pcie_udb_qsfp_device_port(10),
    pcie_udb_qsfp_device_port(11),
    pcie_udb_qsfp_device_port(12),
    pcie_udb_qsfp_device_port(13),
    pcie_udb_qsfp_device_port(14),
    pcie_udb_qsfp_device_port(15),
    pcie_udb_qsfp_device_port(16),
    pcie_udb_qsfp_device_port(17),
    pcie_udb_qsfp_device_port(18),
    pcie_udb_qsfp_device_port(19),
    pcie_udb_qsfp_device_port(20),
    pcie_udb_qsfp_device_port(21),
    pcie_udb_qsfp_device_port(22),
    pcie_udb_qsfp_device_port(23),
    pcie_udb_qsfp_device_port(24),
    pcie_udb_qsfp_device_port(25),
    pcie_udb_qsfp_device_port(26),
    pcie_udb_qsfp_device_port(27),
    pcie_udb_qsfp_device_port(28),
    pcie_udb_qsfp_device_port(29),
    pcie_udb_qsfp_device_port(30),
    pcie_udb_qsfp_device_port(31),
};

/*LDB platform device*/
static struct platform_device pcie_ldb_qsfp_device[] = {
    pcie_ldb_qsfp_device_port(0),
    pcie_ldb_qsfp_device_port(1),
    pcie_ldb_qsfp_device_port(2),
    pcie_ldb_qsfp_device_port(3),
    pcie_ldb_qsfp_device_port(4),
    pcie_ldb_qsfp_device_port(5),
    pcie_ldb_qsfp_device_port(6),
    pcie_ldb_qsfp_device_port(7),
    pcie_ldb_qsfp_device_port(8),
    pcie_ldb_qsfp_device_port(9),
    pcie_ldb_qsfp_device_port(10),
    pcie_ldb_qsfp_device_port(11),
    pcie_ldb_qsfp_device_port(12),
    pcie_ldb_qsfp_device_port(13),
    pcie_ldb_qsfp_device_port(14),
    pcie_ldb_qsfp_device_port(15),
    pcie_ldb_qsfp_device_port(16),
    pcie_ldb_qsfp_device_port(17),
    pcie_ldb_qsfp_device_port(18),
    pcie_ldb_qsfp_device_port(19),
    pcie_ldb_qsfp_device_port(20),
    pcie_ldb_qsfp_device_port(21),
    pcie_ldb_qsfp_device_port(22),
    pcie_ldb_qsfp_device_port(23),
    pcie_ldb_qsfp_device_port(24),
    pcie_ldb_qsfp_device_port(25),
    pcie_ldb_qsfp_device_port(26),
    pcie_ldb_qsfp_device_port(27),
    pcie_ldb_qsfp_device_port(28),
    pcie_ldb_qsfp_device_port(29),
    pcie_ldb_qsfp_device_port(30),
    pcie_ldb_qsfp_device_port(31),
    pcie_ldb_sfp_device_port(32), /*sfp port65*/
    pcie_ldb_sfp_device_port(33), /*sfp port66*/
};

#define DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(module_present_##index, S_IRUGO, port_status_read, NULL, MODULE_PRESENT_##index); \
    static SENSOR_DEVICE_ATTR(module_reset_##index, S_IWUSR|S_IRUGO, port_status_read, port_status_write, MODULE_RESET_##index); \
    static SENSOR_DEVICE_ATTR(module_lp_mode_##index, S_IRUGO | S_IWUSR, port_status_read, port_status_write, MODULE_LPMODE_##index)
#define DECLARE_TRANSCEIVER_ATTR(index) \
    &sensor_dev_attr_module_present_##index.dev_attr.attr, \
    &sensor_dev_attr_module_reset_##index.dev_attr.attr, \
    &sensor_dev_attr_module_lp_mode_##index.dev_attr.attr

/* transceiver attributes */
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(1);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(2);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(3);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(4);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(5);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(6);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(7);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(8);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(9);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(10);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(11);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(12);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(13);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(14);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(15);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(16);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(17);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(18);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(19);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(20);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(21);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(22);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(23);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(24);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(25);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(26);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(27);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(28);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(29);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(30);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(31);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(32);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(33);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(34);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(35);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(36);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(37);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(38);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(39);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(40);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(41);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(42);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(43);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(44);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(45);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(46);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(47);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(48);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(49);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(50);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(51);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(52);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(53);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(54);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(55);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(56);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(57);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(58);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(59);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(60);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(61);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(62);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(63);
DECLARE_TRANSCEIVER_SENSOR_DEVICE_ATTR(64);
static SENSOR_DEVICE_ATTR(module_present_65, S_IRUGO, port_status_read, NULL, MODULE_PRESENT_65);
static SENSOR_DEVICE_ATTR(module_present_66, S_IRUGO, port_status_read, NULL, MODULE_PRESENT_66);
static SENSOR_DEVICE_ATTR(module_tx_disable_65, S_IRUGO | S_IWUSR, port_status_read, port_status_write, MODULE_TX_DISABLE_65);
static SENSOR_DEVICE_ATTR(module_tx_disable_66, S_IRUGO | S_IWUSR, port_status_read, port_status_write, MODULE_TX_DISABLE_66);
static SENSOR_DEVICE_ATTR(module_tx_fault_65, S_IRUGO, port_status_read, NULL, MODULE_TX_FAULT_65);
static SENSOR_DEVICE_ATTR(module_tx_fault_66, S_IRUGO, port_status_read, NULL, MODULE_TX_FAULT_66);
static SENSOR_DEVICE_ATTR(module_rx_los_65, S_IRUGO, port_status_read, NULL, MODULE_RX_LOS_65);
static SENSOR_DEVICE_ATTR(module_rx_los_66, S_IRUGO, port_status_read, NULL, MODULE_RX_LOS_66);
static SENSOR_DEVICE_ATTR(udb_version, S_IRUGO, port_status_read, NULL, PCIE_FPGA_UDB_VERSION);
static SENSOR_DEVICE_ATTR(ldb_version, S_IRUGO, port_status_read, NULL, PCIE_FPGA_LDB_VERSION);
static SENSOR_DEVICE_ATTR(smb_version, S_IRUGO, port_status_read, NULL, PCIE_FPGA_SMB_VERSION);

static struct attribute *fpga_transceiver_attributes[] = {
    DECLARE_TRANSCEIVER_ATTR(1),
    DECLARE_TRANSCEIVER_ATTR(2),
    DECLARE_TRANSCEIVER_ATTR(3),
    DECLARE_TRANSCEIVER_ATTR(4),
    DECLARE_TRANSCEIVER_ATTR(5),
    DECLARE_TRANSCEIVER_ATTR(6),
    DECLARE_TRANSCEIVER_ATTR(7),
    DECLARE_TRANSCEIVER_ATTR(8),
    DECLARE_TRANSCEIVER_ATTR(9),
    DECLARE_TRANSCEIVER_ATTR(10),
    DECLARE_TRANSCEIVER_ATTR(11),
    DECLARE_TRANSCEIVER_ATTR(12),
    DECLARE_TRANSCEIVER_ATTR(13),
    DECLARE_TRANSCEIVER_ATTR(14),
    DECLARE_TRANSCEIVER_ATTR(15),
    DECLARE_TRANSCEIVER_ATTR(16),
    DECLARE_TRANSCEIVER_ATTR(17),
    DECLARE_TRANSCEIVER_ATTR(18),
    DECLARE_TRANSCEIVER_ATTR(19),
    DECLARE_TRANSCEIVER_ATTR(20),
    DECLARE_TRANSCEIVER_ATTR(21),
    DECLARE_TRANSCEIVER_ATTR(22),
    DECLARE_TRANSCEIVER_ATTR(23),
    DECLARE_TRANSCEIVER_ATTR(24),
    DECLARE_TRANSCEIVER_ATTR(25),
    DECLARE_TRANSCEIVER_ATTR(26),
    DECLARE_TRANSCEIVER_ATTR(27),
    DECLARE_TRANSCEIVER_ATTR(28),
    DECLARE_TRANSCEIVER_ATTR(29),
    DECLARE_TRANSCEIVER_ATTR(30),
    DECLARE_TRANSCEIVER_ATTR(31),
    DECLARE_TRANSCEIVER_ATTR(32),
    DECLARE_TRANSCEIVER_ATTR(33),
    DECLARE_TRANSCEIVER_ATTR(34),
    DECLARE_TRANSCEIVER_ATTR(35),
    DECLARE_TRANSCEIVER_ATTR(36),
    DECLARE_TRANSCEIVER_ATTR(37),
    DECLARE_TRANSCEIVER_ATTR(38),
    DECLARE_TRANSCEIVER_ATTR(39),
    DECLARE_TRANSCEIVER_ATTR(40),
    DECLARE_TRANSCEIVER_ATTR(41),
    DECLARE_TRANSCEIVER_ATTR(42),
    DECLARE_TRANSCEIVER_ATTR(43),
    DECLARE_TRANSCEIVER_ATTR(44),
    DECLARE_TRANSCEIVER_ATTR(45),
    DECLARE_TRANSCEIVER_ATTR(46),
    DECLARE_TRANSCEIVER_ATTR(47),
    DECLARE_TRANSCEIVER_ATTR(48),
    DECLARE_TRANSCEIVER_ATTR(49),
    DECLARE_TRANSCEIVER_ATTR(50),
    DECLARE_TRANSCEIVER_ATTR(51),
    DECLARE_TRANSCEIVER_ATTR(52),
    DECLARE_TRANSCEIVER_ATTR(53),
    DECLARE_TRANSCEIVER_ATTR(54),
    DECLARE_TRANSCEIVER_ATTR(55),
    DECLARE_TRANSCEIVER_ATTR(56),
    DECLARE_TRANSCEIVER_ATTR(57),
    DECLARE_TRANSCEIVER_ATTR(58),
    DECLARE_TRANSCEIVER_ATTR(59),
    DECLARE_TRANSCEIVER_ATTR(60),
    DECLARE_TRANSCEIVER_ATTR(61),
    DECLARE_TRANSCEIVER_ATTR(62),
    DECLARE_TRANSCEIVER_ATTR(63),
    DECLARE_TRANSCEIVER_ATTR(64),
    &sensor_dev_attr_module_present_65.dev_attr.attr,
    &sensor_dev_attr_module_present_66.dev_attr.attr,
    &sensor_dev_attr_module_tx_disable_65.dev_attr.attr,
    &sensor_dev_attr_module_tx_disable_66.dev_attr.attr,
    &sensor_dev_attr_module_tx_fault_65.dev_attr.attr,
    &sensor_dev_attr_module_tx_fault_66.dev_attr.attr,
    &sensor_dev_attr_module_rx_los_65.dev_attr.attr,
    &sensor_dev_attr_module_rx_los_66.dev_attr.attr,
    &sensor_dev_attr_udb_version.dev_attr.attr,
    &sensor_dev_attr_ldb_version.dev_attr.attr,
    &sensor_dev_attr_smb_version.dev_attr.attr,
    NULL
};

/* eeprom attribute */
static SENSOR_DEVICE_ATTR(name, S_IRUGO, port_read, NULL, PORT_SYSFS_NAME_ID);            /* optoe{1, 2, 3} */
static SENSOR_DEVICE_ATTR(port_name, S_IRUGO, port_read, NULL, PORT_SYSFS_PORT_NAME_ID);  /* port{1~64} */
static SENSOR_DEVICE_ATTR(dev_class, S_IRUGO|S_IWUSR, port_read, port_write, PORT_SYSFS_DEV_CLASS_ID); /* 1 or 2 or 3 */

static struct attribute *fpga_eeprom_attributes[] = {
    &sensor_dev_attr_name.dev_attr.attr,
    &sensor_dev_attr_port_name.dev_attr.attr,
    &sensor_dev_attr_dev_class.dev_attr.attr,
    NULL
};

static const struct attribute_group fpga_port_stat_group = {
    .attrs = fpga_transceiver_attributes,
};

static const struct attribute_group fpga_eeprom_group = {
    .attrs = fpga_eeprom_attributes,
};

static ssize_t fpga_read_sfp_ddm_status_value(struct bin_attribute *eeprom)
{
    u32 reg_val = 0;
    u16 pageable = 0;
    u16 ddm_support = 0;
    void __iomem *addr = NULL;
    struct eeprom_bin_private_data *pdata = NULL;

    if(eeprom == NULL) {
        return -1;
    }

    pdata = eeprom->private; /*assign private sturct value*/

    if(pdata->port_num > FPGA_QSFP_PORT_NUM)
    {
        /*get sfp pagable status*/
        if( fpga_i2c_ready_to_read(eeprom, EEPROM_LOWER_PAGE, pdata->i2c_slave_addr) != 1) {
            return 0;
        }

        addr = ioremap(pdata->base_addr + (pdata->i2c_rtc_read_data + TWO_ADDR_PAGEABLE_REG), 32);
        reg_val = ioread32(addr);
        iounmap(addr);
        pageable = (reg_val) & 0xff;  /*check on bit4*/

        /*get sfp support a2 status*/
        if( fpga_i2c_ready_to_read(eeprom, EEPROM_LOWER_PAGE, pdata->i2c_slave_addr) != 1) {
            return 0;
        }

        addr = ioremap(pdata->base_addr + (pdata->i2c_rtc_read_data + TWO_ADDR_0X51_REG ), 32);
        reg_val = ioread32(addr);
        iounmap(addr);
        ddm_support = (reg_val) & 0xff;  /*check on bit6*/

        pdata->pageable = (pageable & TWO_ADDR_PAGEABLE ) ? 1 : 0;
        pdata->sfp_support_a2 = (ddm_support & TWO_ADDR_0X51_SUPP) ? 1 : 0;
    }

    return 0;
}

static ssize_t fpga_read_port_status_value(struct bin_attribute *eeprom)
{
    int i = 0;
    void __iomem *addr = NULL;

    if (time_before(jiffies, fpga_ctl->last_updated + HZ / 2) && fpga_ctl->valid) {
        return 0;
    }

    for (i = 0; i < ARRAY_SIZE(fpga_ctl->pci_fpga_dev) -1; i++) {
        /*Update present*/
        addr = ioremap(fpga_ctl->pci_fpga_dev[i].base_addr + QSFP_PRESENT_REG_OFFSET, 32);
        fpga_ctl->pci_fpga_dev[i].qsfp_present = ioread32(addr);
        iounmap(addr);
        if(i==PCI_SUBSYSTEM_ID_LDB)
        {
            /*Read output data*/
            addr = ioremap(fpga_ctl->pci_fpga_dev[i].base_addr + SFP_LDB_GPIO1_DATA_OUT, 32);
            fpga_ctl->pci_fpga_dev[i].sfp_output_data = ioread32(addr);
            iounmap(addr);
            /*Read input data*/
            addr = ioremap(fpga_ctl->pci_fpga_dev[i].base_addr + SFP_LDB_GPIO1_DATA_IN, 32);
            fpga_ctl->pci_fpga_dev[i].sfp_input_data = ioread32(addr);
            iounmap(addr);
        }
        /*Update lpmode*/
        addr = ioremap(fpga_ctl->pci_fpga_dev[i].base_addr + QSFP_LPMODE_REG_OFFSET, 32);
        fpga_ctl->pci_fpga_dev[i].qsfp_lpmode = ioread32(addr);
        iounmap(addr);
        /*Update reset*/
        addr = ioremap(fpga_ctl->pci_fpga_dev[i].base_addr + QSFP_RESET_REG_OFFSET, 32);
        fpga_ctl->pci_fpga_dev[i].qsfp_reset = ioread32(addr);
        iounmap(addr);
    }

    /*get version*/
    addr = ioremap(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].base_addr, 32);
    fpga_ctl->udb_version = ioread32(addr);
    iounmap(addr);

    addr = ioremap(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].base_addr, 32);
    fpga_ctl->ldb_version = ioread32(addr);
    iounmap(addr);

    addr = ioremap(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_SMB].base_addr, 32);
    fpga_ctl->smb_version = ioread32(addr);
    iounmap(addr);

    fpga_ctl->valid = 1;
    fpga_ctl->last_updated = jiffies;

    return 0;
}

static ssize_t fpga_write_port_value(int fpga_type, int set_type, int bit_num, long val)
{
    long val_set = 0;
    u32  reg_val = 0;
    void __iomem *addr = NULL;

    if(set_type == PCIE_FPGA_SET_LPMODE) {
        reg_val = fpga_ctl->pci_fpga_dev[fpga_type].qsfp_lpmode;
    } else if(set_type == PCIE_FPGA_SET_RESET) {
        reg_val = fpga_ctl->pci_fpga_dev[fpga_type].qsfp_reset;
    } else {
        reg_val = fpga_ctl->pci_fpga_dev[fpga_type].sfp_output_data;
    }

    if(val){
        val_set = reg_val | (1<<bit_num);
    } else {
        val_set = reg_val & ~(1<<bit_num);
    }

    switch(set_type)
    {
        case PCIE_FPGA_SET_LPMODE:
            addr = ioremap(fpga_ctl->pci_fpga_dev[fpga_type].base_addr + QSFP_LPMODE_REG_OFFSET, 32);
            iowrite32(val_set, addr);
            iounmap(addr);
            break;
        case PCIE_FPGA_SET_RESET:
            addr = ioremap(fpga_ctl->pci_fpga_dev[fpga_type].base_addr + QSFP_RESET_REG_OFFSET, 32);
            iowrite32(val_set, addr);
            iounmap(addr);
            break;
        case PCIE_FPGA_SET_TX_DISABLE:
            addr = ioremap(fpga_ctl->pci_fpga_dev[fpga_type].base_addr + SFP_LDB_GPIO1_DATA_OUT, 32);
            iowrite32(val_set, addr);
            iounmap(addr);
            break;
        default:
            break;
    }

    return 0;
}

static int get_present_by_attr_index(int attr_index)
{
    int present = 0;
    int index_mapping = 0;

    switch(attr_index)
    {
        case MODULE_PRESENT_1 ... MODULE_PRESENT_32:
        case MODULE_PRESENT_33 ... MODULE_PRESENT_64:
        case MODULE_PRESENT_65: /*sfp port*/
        case MODULE_PRESENT_66: /*sfp port*/
            index_mapping = attr_index;
            break;
        case MODULE_LPMODE_1 ... MODULE_LPMODE_32:
            index_mapping = attr_index - MODULE_LPMODE_1;
            break;
        case MODULE_LPMODE_33 ... MODULE_LPMODE_64:
            index_mapping = attr_index - MODULE_LPMODE_33;
            break;
        case MODULE_RESET_1 ... MODULE_RESET_32:
            index_mapping = attr_index - MODULE_RESET_1;
            break;
        case MODULE_RESET_33 ... MODULE_RESET_64:
            index_mapping = attr_index - MODULE_RESET_33;
            break;
        case MODULE_TX_DISABLE_65:
        case MODULE_TX_FAULT_65:
        case MODULE_RX_LOS_65:
            index_mapping = MODULE_PRESENT_65;
            break;
        case MODULE_TX_DISABLE_66:
        case MODULE_TX_FAULT_66:
        case MODULE_RX_LOS_66:
            index_mapping = MODULE_PRESENT_66;
            break;
        default:
            index_mapping = -EINVAL;
            break;
    }

    if( (index_mapping >= MODULE_PRESENT_1) && (index_mapping <= MODULE_PRESENT_32) )
    {
        present =((fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].qsfp_present>>(index_mapping - MODULE_PRESENT_1)) & 0x1)?0:1;
    }
    else if( (index_mapping >= MODULE_PRESENT_33) && (index_mapping <= MODULE_PRESENT_64) )
    {
        present = ((fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].qsfp_present>>(index_mapping - MODULE_PRESENT_33)) & 0x1)?0:1;
    }
    else if( index_mapping == MODULE_PRESENT_65 )
    {
        present = ((SFP_PORT0_ABS(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1)?0:1;
    }
    else if( index_mapping == MODULE_PRESENT_66 )
    {
        present = ((SFP_PORT1_ABS(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1)?0:1;
    }
    else {
        present = 0; /*unpresent*/
    }

    return present;
}

static ssize_t port_status_read(struct device *dev, struct device_attribute *da, char *buf)
{
    int  present = 0;
    ssize_t  ret = 0;
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct bin_attribute *eeprom = NULL;

    mutex_lock(&update_lock);
    fpga_read_port_status_value(eeprom);

    present = get_present_by_attr_index(attr->index);

    switch(attr->index)
    {
        case MODULE_PRESENT_1 ... MODULE_PRESENT_32:
        case MODULE_PRESENT_33 ... MODULE_PRESENT_64:
        case MODULE_PRESENT_65: /*sfp port*/
        case MODULE_PRESENT_66: /*sfp port*/
            ret = sprintf(buf, "%d\n", present);
            break;
        case MODULE_LPMODE_1 ... MODULE_LPMODE_32:
            if(present){
                ret = sprintf(buf, "%d\n", ((fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].qsfp_lpmode >> (attr->index - MODULE_LPMODE_1)) & 0x1));
            } else {
                ret = sprintf(buf, "%d\n", 0); /*unpresent: default value*/
            }
            break;
        case MODULE_LPMODE_33 ... MODULE_LPMODE_64:
            if(present){
                ret = sprintf(buf, "%d\n", ((fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].qsfp_lpmode >> (attr->index - MODULE_LPMODE_33)) & 0x1));
            } else {
                ret = sprintf(buf, "%d\n", 0); /*unpresent: default value*/
            }
            break;
        case MODULE_RESET_1 ... MODULE_RESET_32:
            if(present){
                ret = sprintf(buf, "%d\n", ((fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].qsfp_reset >>(attr->index - MODULE_RESET_1)) & 0x1)?0:1);
            } else {
                ret = sprintf(buf, "%d\n", 0); /*unpresent: default value*/
            }
            break;
        case MODULE_RESET_33 ... MODULE_RESET_64:
            if(present){
                ret = sprintf(buf, "%d\n", ((fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].qsfp_reset >> (attr->index - MODULE_RESET_33)) & 0x1)?0:1);
            } else {
                ret = sprintf(buf, "%d\n", 0); /*unpresent: default value*/
            }
            break;
        case MODULE_TX_DISABLE_65:
            if(present){
                ret = sprintf(buf, "%d\n", (SFP_PORT0_TXDIS(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1);
            } else {
                ret = sprintf(buf, "%d\n", 0); /*unpresent: default value*/
            }
            break;
        case MODULE_TX_DISABLE_66:
            if(present){
                ret = sprintf(buf, "%d\n", (SFP_PORT1_TXDIS(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1);
            } else {
                ret = sprintf(buf, "%d\n", 0); /*unpresent: default value*/
            }
            break;
        case MODULE_TX_FAULT_65:
            if(present){
                ret = sprintf(buf, "%d\n", (SFP_PORT0_TXFLT(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1);
            } else {
                ret = sprintf(buf, "%d\n", 1); /*unpresent: tx_fault is true*/
            }
            break;
        case MODULE_TX_FAULT_66:
            if(present){
                ret = sprintf(buf, "%d\n", (SFP_PORT1_TXFLT(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1);
            } else {
                ret = sprintf(buf, "%d\n", 1); /*unpresent: tx_fault is true*/
            }
            break;
        case MODULE_RX_LOS_65:
            if(present){
                ret = sprintf(buf, "%d\n", (SFP_PORT0_RXLOS(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1);
            } else {
                ret = sprintf(buf, "%d\n", 1); /*unpresent: rx_los is true*/
            }
            break;
        case MODULE_RX_LOS_66:
            if(present){
                ret = sprintf(buf, "%d\n", (SFP_PORT1_RXLOS(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1);
            } else {
                ret = sprintf(buf, "%d\n", 1); /*unpresent: rx_los is true*/
            }
            break;
        case PCIE_FPGA_UDB_VERSION:
            ret = sprintf(buf, "%d.%d\n", (fpga_ctl->udb_version>>8) & 0x7f, fpga_ctl->udb_version & 0xff);
            break;
        case PCIE_FPGA_LDB_VERSION:
            ret = sprintf(buf, "%d.%d\n", (fpga_ctl->ldb_version>>8) & 0x7f, fpga_ctl->ldb_version & 0xff);
            break;
        case PCIE_FPGA_SMB_VERSION:
            ret = sprintf(buf, "%d.%d\n", (fpga_ctl->smb_version>>8) & 0x7f, fpga_ctl->smb_version & 0xff);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    mutex_unlock(&update_lock);

    return ret;
}

static ssize_t port_status_write(struct device *dev, struct device_attribute *da,
            const char *buf, size_t count)
{
    long value;
    int status;
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct bin_attribute *eeprom = NULL;

    status = kstrtol(buf, 16, &value);
    if (status) {
        return status;
    }

    mutex_lock(&update_lock);
    fpga_read_port_status_value(eeprom);

    switch(attr->index)
    {
        case MODULE_LPMODE_1 ... MODULE_LPMODE_32:
            fpga_write_port_value(PCIE_FPGA_UDB, PCIE_FPGA_SET_LPMODE, (attr->index - MODULE_LPMODE_1), !!value);
            break;
        case MODULE_LPMODE_33 ... MODULE_LPMODE_64:
            fpga_write_port_value(PCIE_FPGA_LDB, PCIE_FPGA_SET_LPMODE, (attr->index - MODULE_LPMODE_1), !!value);
            break;
        case MODULE_RESET_1 ... MODULE_RESET_32:
            fpga_write_port_value(PCIE_FPGA_UDB, PCIE_FPGA_SET_RESET, (attr->index - MODULE_RESET_1), !value);
            break;
        case MODULE_RESET_33 ... MODULE_RESET_64:
            fpga_write_port_value(PCIE_FPGA_LDB, PCIE_FPGA_SET_RESET, (attr->index - MODULE_RESET_1), !value);
            break;
        case MODULE_TX_DISABLE_65 ... MODULE_TX_DISABLE_66:
            fpga_write_port_value(PCIE_FPGA_LDB, PCIE_FPGA_SET_TX_DISABLE, ((attr->index - MODULE_TX_DISABLE_65) ? BIT(3) : BIT(11) ), !!value); /*bit3 and bit11*/
            break;
        default:
            mutex_unlock(&update_lock);
            return  -EINVAL;
    }

    mutex_unlock(&update_lock);

    return count;
}

static ssize_t port_read(struct device *dev, struct device_attribute *da, char *buf)
{
    ssize_t  ret = 0;
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct platform_device *pdev = to_platform_device(dev);
    struct pcie_fpga_dev_platform_data *pdata = NULL;

    pdata = pdev->dev.platform_data;

    mutex_lock(&update_lock);
    switch(attr->index)
    {
        case PORT_SYSFS_PORT_NAME_ID:
            ret = sprintf(buf, "%s\n", pdata->name);
            break;
        case PORT_SYSFS_NAME_ID:
            ret = sprintf(buf, "%s\n", pdata->dev_name);
            break;
        case PORT_SYSFS_DEV_CLASS_ID:
            ret = sprintf(buf, "%d\n", pdata->dev_class);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    mutex_unlock(&update_lock);

    return ret;
}

static ssize_t port_write(struct device *dev, struct device_attribute *da,
            const char *buf, size_t count)
{
    int value;
    int status;
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct platform_device *pdev = to_platform_device(dev);
    struct pcie_fpga_dev_platform_data *pdata = NULL;

    pdata = pdev->dev.platform_data;

    status = kstrtoint(buf, 10, &value);
    if (status) {
        return status;
    }

    mutex_lock(&update_lock);
    switch(attr->index)
    {
        case PORT_SYSFS_DEV_CLASS_ID:
            pdata->dev_class = value;
            break;
        default:
            mutex_unlock(&update_lock);
            return  -EINVAL;
    }
    mutex_unlock(&update_lock);
    return count;
}

/*
 * eeprom read fumction
 */
static int fpga_i2c_ready_to_read(struct bin_attribute *attr, int page_type, int i2c_slave_addr)
{
    int  cnt = 0;
    u32  i2c_new_trigger_val = 0;
    u32  flag = 0;
    void __iomem *addr = NULL;
    struct eeprom_bin_private_data *pdata = NULL;

    pdata = attr->private;

    /*Select i2c protocol profile*/
    addr = ioremap(pdata->base_addr + pdata->i2c_mgmt_rtc0_profile, 32);
    iowrite32(0x0, addr);
    iounmap(addr);

    /*clean read data*/
    for(cnt = 0 ; cnt < 32; cnt++)
    {
        addr = ioremap(pdata->base_addr + ( pdata->i2c_rtc_read_data + (4 * cnt) ), 32);
        iowrite32(0x0, addr);
        iounmap(addr);
    }

    /*clean done status*/
    addr = ioremap(pdata->base_addr + pdata->i2c_contrl_rtc0_stats, 32);
    iowrite32(0x3, addr);
    iounmap(addr);

    /*set read slave addr*/
    addr = ioremap(pdata->base_addr + pdata->i2c_contrl_rtc0_config_0, 32);
    iowrite32( 0x10000080|(i2c_slave_addr << 8), addr);
    iounmap(addr);

    /*triger*/
    if(page_type == EEPROM_LOWER_PAGE) {
        i2c_new_trigger_val = PCIE_FPGA_I2C_NEW_TRIGGER_VALUE;
    } else {
        i2c_new_trigger_val = PCIE_FPGA_I2C_NEW_TRIGGER_VALUE + 0x80;
    }
    addr = ioremap(pdata->base_addr + pdata->i2c_contrl_rtc0_config_1, 32);
    iowrite32(i2c_new_trigger_val, addr);
    iounmap(addr);

    /*read done status*/
    while( 1 ) {
        addr = ioremap(pdata->base_addr + pdata->i2c_contrl_rtc0_stats, 32);
        flag = ioread32(addr);
        iounmap(addr);
        if(flag == 0) {
            usleep_range(10, 20);
            continue;
        }
        else {
            break;
        }
    }
    msleep(1);

    return flag;
}

static int fpga_i2c_set_data(struct bin_attribute *attr, loff_t offset, char *data, int i2c_slave_addr)
{
    int cnt = 0;
    struct eeprom_bin_private_data *pdata = NULL;
    void __iomem *addr = NULL;
    u32  flag = 0;
    u32  i2c_new_trigger_val = 0;

    pdata = attr->private;

    /*Select i2c protocol profile*/
    addr = ioremap(pdata->base_addr + pdata->i2c_mgmt_rtc0_profile, 32);
    iowrite32(0x0, addr);
    iounmap(addr);

    /*clean read data*/
    for( cnt=0 ; cnt < (PCIE_FPGA_I2C_MAX_LEN/4); cnt++)
    {
        addr = ioremap(pdata->base_addr + ( pdata->i2c_rtc_write_data + (4 * cnt) ), 32);
        iowrite32(0x0, addr);
        iounmap(addr);
    }

    /* Prepare date to set into data registor*/
    addr = ioremap(pdata->base_addr + pdata->i2c_rtc_write_data, 32);
    iowrite32(data[0], addr);
    iounmap(addr);

    /*clean done status*/
    addr = ioremap(pdata->base_addr + pdata->i2c_contrl_rtc0_stats, 32);
    iowrite32(0x3, addr);
    iounmap(addr);

    /*set write slave addr*/
    addr = ioremap(pdata->base_addr + pdata->i2c_contrl_rtc0_config_0, 32);
    iowrite32( EEPROM_ALLOW_SET_LEN | (i2c_slave_addr << 8), addr);
    iounmap(addr);

    /*triger*/
    i2c_new_trigger_val = PCIE_FPGA_I2C_NEW_TRIGGER_VALUE + offset;
    addr = ioremap(pdata->base_addr + pdata->i2c_contrl_rtc0_config_1, 32);
    iowrite32(i2c_new_trigger_val, addr);
    iounmap(addr);

    /*read done status*/
    addr = ioremap(pdata->base_addr + pdata->i2c_contrl_rtc0_stats, 32);

    while( 1 ) {
        flag = ioread32(addr);
        if(flag == 0) {
            usleep_range(10, 20);
            continue;
        } else {
            break;
        }
    }
    iounmap(addr);
    msleep(1);

    return flag;
}

static ssize_t fpga_i2c_read_data(struct bin_attribute *attr, u8 *data)
{
    int cnt = 0;
    u32  read_status = 0;
    ssize_t byte_size = 0;
    struct eeprom_bin_private_data *pdata = NULL;
    void __iomem *addr = NULL;

    pdata = attr->private;

    for( cnt=0 ; cnt < (PCIE_FPGA_I2C_MAX_LEN/4); cnt++)
    {
        addr = ioremap(pdata->base_addr + (pdata->i2c_rtc_read_data + cnt*4), 32);
        read_status = ioread32(addr);
        iounmap(addr);
        *(data + cnt*4) = read_status & 0xff;
        *(data + cnt*4 + 1) = (read_status >> 8) & 0xff;
        *(data + cnt*4 + 2) =  (read_status >> 16) & 0xff;
        *(data + cnt*4 + 3) =  (read_status >> 24) & 0xff;

        byte_size = cnt*4 + 3;
    }

    return byte_size + 1;
}

static int get_port_present_status(struct bin_attribute *attr)
{
    int present = 0;
    struct eeprom_bin_private_data *pdata = NULL;

    fpga_read_port_status_value(attr);

    pdata = attr->private;
    /*
     * get present status:
     * regval:0 is   present, convert
     * regval:1 is unpresent, convert
     */
    if(pdata->port_num == FPGA_LDB_SFP_PORT1_NO) /*sfp:65*/
    {
        present = !((SFP_PORT0_ABS(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1);
    }
    else if(pdata->port_num == FPGA_LDB_SFP_PORT2_NO) /*sfp:66*/
    {
        present = !((SFP_PORT1_ABS(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].sfp_input_data)) & 0x1);
    }
    else
    {
        if(pdata->port_num <= FPGA_LDB_QSFP_PORT_NUM) { /*qsfp:1~32*/
            present = !((fpga_ctl->pci_fpga_dev[pdata->fpga_type].qsfp_present>>(pdata->port_num - 1)) & 0x1);
        } else { /*qsfp:33~64*/
            present = !((fpga_ctl->pci_fpga_dev[pdata->fpga_type].qsfp_present>>(pdata->port_num - 33)) & 0x1);
        }
    }

    return present;
}

static ssize_t
sfp_eeprom_read(struct file *filp, struct kobject *kobj,
             struct bin_attribute *attr,
             char *buf, loff_t off, size_t count, int *page)
{
    int page_num, slice;
    char set_page_num[1] = {0};
    struct eeprom_bin_private_data *pdata = NULL;
    pdata = attr->private;

    ssize_t byte_cnt;

    u8 data[128] = {0};

    slice = off / OPTOE_PAGE_SIZE;
    /*Cross page case, calculate number of count in current page*/
    if ((off + count) > (slice * OPTOE_PAGE_SIZE + OPTOE_PAGE_SIZE)) {
        count = slice * OPTOE_PAGE_SIZE + OPTOE_PAGE_SIZE - off;
    }

    if( slice == 0 )
    {
        if( fpga_i2c_ready_to_read(attr, EEPROM_LOWER_PAGE, pdata->i2c_slave_addr) != 1) {
            goto exit_emtpy;
        }
        byte_cnt = fpga_i2c_read_data(attr, &data[0]);
    }
    else if( slice == 1 )
    {
        if( fpga_i2c_ready_to_read(attr, EEPROM_UPPER_PAGE, pdata->i2c_slave_addr) != 1) {
            goto exit_emtpy;
        }
        byte_cnt = fpga_i2c_read_data(attr, &data[0]);
    }
    else
    {
        page_num = slice - 1;
        if( pdata->port_num <= FPGA_QSFP_PORT_NUM) /*qsfp page1~0xff*/
        {
            set_page_num[0] = page_num;
            if( fpga_i2c_set_data(attr, OPTOE_PAGE_SELECT_REG, set_page_num, pdata->i2c_slave_addr) != 1) {
                goto exit_emtpy;
            }

            if( fpga_i2c_ready_to_read(attr, EEPROM_UPPER_PAGE, pdata->i2c_slave_addr) != 1 ) {
                goto exit_emtpy;
            }
            byte_cnt = fpga_i2c_read_data(attr, &data[byte_cnt]);
            *page = page_num;
        }
        else /*sfp support a2(0x51), cat behind a0(0x50)*/
        {
            if(page_num == 1) /*a2 lower page*/
            {
                if( fpga_i2c_ready_to_read(attr, EEPROM_LOWER_PAGE, TWO_ADDR_0X51) != 1) {
                    goto exit_emtpy;
                }
                byte_cnt = fpga_i2c_read_data(attr, &data[0]);
            }
            else if (page_num == 2) /*a2 page0*/
            {
                set_page_num[0] = 0;
                if( fpga_i2c_set_data(attr, OPTOE_PAGE_SELECT_REG, set_page_num, TWO_ADDR_0X51)!=1) {
                        goto exit_emtpy;
                }

                if( fpga_i2c_ready_to_read(attr, EEPROM_UPPER_PAGE, TWO_ADDR_0X51) != 1) {
                    goto exit_emtpy;
                }
                byte_cnt = fpga_i2c_read_data(attr, &data[0]);
            }
            else
            {
                set_page_num[0] = page_num - 2;
                if( fpga_i2c_set_data(attr, OPTOE_PAGE_SELECT_REG, set_page_num, TWO_ADDR_0X51) != 1) { /*set page from 1*/
                    goto exit_emtpy;
                }

                if( fpga_i2c_ready_to_read(attr, EEPROM_UPPER_PAGE, TWO_ADDR_0X51) != 1 ) {
                    goto exit_emtpy;
                }
                byte_cnt = fpga_i2c_read_data(attr, &data[byte_cnt]);
                *page = page_num-2;
            }
        }
    }
    memcpy(buf, &data[off%128], count);

    return count;

exit_emtpy:
    pcie_err("ERROR: Port%d pcie get failed!!", pdata->port_num);
    return -EBUSY;
}

static ssize_t sfp_bin_read(struct file *filp, struct kobject *kobj,
        struct bin_attribute *attr,
        char *buf, loff_t off, size_t count)
{
    int present;
    int page = 0;
    int i2c_slave_addr;
    char set_page_num[1] ={0};
    ssize_t retval = 0;
    struct eeprom_bin_private_data *pdata = NULL;
    pdata = attr->private;

    if (unlikely(!count)) {
        return count;
    }

    mutex_lock(&update_lock);

    present = get_port_present_status(attr);
    if( !present ) { /*unpresent*/
        mutex_unlock(&update_lock);
        return -ENODEV;
    }
    mutex_unlock(&update_lock);

    /*
     * Read data from chip, protecting against concurrent updates
     * from this host
     */
    mutex_lock(&update_lock);
    while (count) {
        ssize_t status;

        status = sfp_eeprom_read(filp, kobj, attr, buf, off, count, &page);
        if (status <= 0) {
            if (retval == 0) {
                retval = status;
            }
            break;
        }

        buf += status;
        off += status;
        count -= status;
        retval += status;
    }
    /*
     * return the page register to page 0 - why?
     * We either have to set the page register to 0 on every access
     * to it, or restore it to 0 whenever we change it.  Otherwise,
     * accesses to page 0 would actually go to whatever the last page
     * was.  Assume more accesses to page 0 than all other pages
     * combined, so less total accesses if we always leave it at page 0
     */
    if( (page > 0) && (pdata->pageable))
    {
        i2c_slave_addr =
            ( pdata->port_num > FPGA_QSFP_PORT_NUM) ? TWO_ADDR_0X51 : pdata->i2c_slave_addr;

        if( fpga_i2c_set_data(attr, OPTOE_PAGE_SELECT_REG, set_page_num, i2c_slave_addr)!=1) { /*set page to 0*/
            pcie_err("ERROR: Port%d pcie get failed!!", pdata->port_num);
            mutex_unlock(&update_lock);
            return -EBUSY;
        }
    }
    mutex_unlock(&update_lock);

    return retval;
}

static ssize_t
sfp_eeprom_write(struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct eeprom_bin_private_data *pdata = NULL;
    pdata = attr->private;

    if( fpga_i2c_set_data(attr, off, buf, pdata->i2c_slave_addr) != 1) {
        goto exit_emtpy;
    }

    return count;

exit_emtpy:
    pcie_err("ERROR: Port%d pcie set failed!!", pdata->port_num);
    return -EBUSY;
}

static ssize_t sfp_bin_write(struct file *filp, struct kobject *kobj,
                             struct bin_attribute *attr,
                             char *buf, loff_t off, size_t count)
{
    int present;
    ssize_t status = 0;

    if (unlikely(!count) ||
        likely(count > EEPROM_ALLOW_SET_LEN)) { //only allow count = 1
        return count;
    }

    mutex_lock(&update_lock);

    present = get_port_present_status(attr);
    if( !present ) { /*unpresent*/
        mutex_unlock(&update_lock);
        return -ENODEV;
    }
    mutex_unlock(&update_lock);

    /*
     * Write data to chip, protecting against concurrent updates
     * from this host.
     */
    mutex_lock(&update_lock);

    status = sfp_eeprom_write(attr, buf, off, count);

    mutex_unlock(&update_lock);

    return status;
}

static int check_qsfp_eeprom_pageable(struct bin_attribute *eeprom)
{
    int ret = 0;
    int not_pageable;
    u8 identifier_reg;
    u8 pageable_reg;
    u32  read_status = 0;

    void __iomem *addr = NULL;
    struct eeprom_bin_private_data *pdata = NULL;

    pdata = eeprom->private;

    if( fpga_i2c_ready_to_read(eeprom, EEPROM_LOWER_PAGE, pdata->i2c_slave_addr) != 1) {
        return ret;
    }

    addr = ioremap(pdata->base_addr + (pdata->i2c_rtc_read_data), 32);
    read_status = ioread32(addr);
    iounmap(addr);

    identifier_reg = read_status & 0xff;
    pageable_reg = (read_status >> 16) & 0xff;  /*check on bit2*/

    if(identifier_reg == QSFPDD_TYPE) {
        not_pageable = CMIS_NOT_PAGEABLE;
    } else {
        not_pageable = QSFP_NOT_PAGEABLE;
    }

    if(pageable_reg & not_pageable) { /*not support*/
        pdata->pageable = 0;
    } else {
        pdata->pageable = 1;
    }

    return pdata->pageable;
}

static int sfp_sysfs_eeprom_init(struct kobject *kobj, struct bin_attribute *eeprom)
{
    int err;
    int ret;
    struct eeprom_bin_private_data *pdata = NULL;

    pdata = eeprom->private;

    sysfs_bin_attr_init(eeprom);
    eeprom->attr.name   = EEPROM_SYSFS_NAME;
    eeprom->attr.mode   = S_IWUSR | S_IRUGO;
    eeprom->read        = sfp_bin_read;
    eeprom->write       = sfp_bin_write;

    mutex_lock(&update_lock);

    ret = fpga_read_sfp_ddm_status_value(eeprom);
    if(ret < 0) {
        pcie_err("Err: PCIE device port eeprom is empty");
        mutex_unlock(&update_lock);
        return ret;
    }

    if(pdata->port_num > FPGA_QSFP_PORT_NUM) /*sfp*/
    {
        if( !(pdata->sfp_support_a2) ) { /*no A2(0x51)*/
            eeprom->size = TWO_ADDR_NO_0X51_SIZE;
        }
        else {
            eeprom->size = ( (pdata->sfp_support_a2) && (!pdata->pageable) ) ? TWO_ADDR_EEPROM_UNPAGED_SIZE : TWO_ADDR_EEPROM_SIZE;
        }
    }
    else /*qsfp*/
    {
        if( fpga_i2c_ready_to_read(eeprom, EEPROM_LOWER_PAGE, pdata->i2c_slave_addr) != 1) { /*unpresent*/
            eeprom->size = OPTOE_ARCH_PAGES;
        } else {
            eeprom->size = ( check_qsfp_eeprom_pageable(eeprom) ) ? ONE_ADDR_EEPROM_SIZE : ONE_ADDR_EEPROM_UNPAGED_SIZE;
        }
    }

    mutex_unlock(&update_lock);

    /* Create eeprom file */
    err = sysfs_create_bin_file(kobj, eeprom);
    if (err) {
        return err;
    }

    return 0;
}

static int as9736_64d_pcie_fpga_stat_probe (struct platform_device *pdev)
{
    int cnt = 0, status = 0;
    int find_flag = 0; /*UDB and LDB*/
    struct pci_dev *pcidev, *pcidev_from;
    int err = 0;
    int fpga_no = 0;

    u16 id16 = 0;
    u32 val32;
    void __iomem *addr = NULL;

    /* Find Accton register memory space */
    for(cnt = 0 ; cnt < FPGA_NUM ; cnt++)
    {
        pcidev = pci_get_device(PCI_VENDOR_ID_ACCTON, PCI_DEVICE_ID_ACCTON, (cnt == 0) ? NULL : pcidev_from);

        if (!pcidev && !cnt ) { /*Failed at first time*/
            return -ENODEV;
        }

        err = pci_enable_device(pcidev);
        if (err != 0) {
            pcie_err("Cannot enable PCI device\n" );
            status = -ENODEV;
            goto exit_pci;
        }

        if (pci_read_config_word(pcidev, PCI_SUBSYSTEM_ID, &id16)) {
            status = -ENODEV;
            goto exit_pci;
        }
        pcie_info("Found PCI Device: %s", FPGA_NAME[id16]);

        if (pci_read_config_dword(pcidev, PCI_BASE_ADDRESS_0, &val32)) {
            status = -ENODEV;
            goto exit_pci;
        }

        switch(id16)
        {
            case PCI_SUBSYSTEM_ID_UDB:
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].base_addr = val32;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].size = 32;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].id = PCI_SUBSYSTEM_ID_UDB;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].aslpc_cpld1_offset = ASLPC_DEV_UDB_CPLD1_PCIE_START_OFFST;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_UDB].aslpc_cpld2_offset = ASLPC_DEV_UDB_CPLD2_PCIE_START_OFFST;
                find_flag++;
                break;
            case PCI_SUBSYSTEM_ID_LDB:
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].base_addr = val32;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].size = 32;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].id = PCI_SUBSYSTEM_ID_LDB;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].aslpc_cpld1_offset = ASLPC_DEV_LDB_CPLD1_PCIE_START_OFFST;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].aslpc_cpld2_offset = ASLPC_DEV_LDB_CPLD2_PCIE_START_OFFST;
                find_flag++;
                break;
            case PCI_SUBSYSTEM_ID_SMB:
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_SMB].base_addr = val32;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_SMB].size = 32;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_SMB].id = PCI_SUBSYSTEM_ID_SMB;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_SMB].aslpc_cpld1_offset = ASLPC_DEV_SMB_CPLD_PCIE_START_OFFST;
                fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_SMB].aslpc_cpld2_offset = 0;
                break;
            default:
                status = -ENODEV;
                break;
        }
        pcidev_from = pcidev;
    }

    if ( find_flag != (FPGA_NUM-1) ) {
        dev_err(&pdev->dev, "Failed found UDB/LDB FPAG device!!\n");
        return -ENODEV;
    }

    status = sysfs_create_group(&pdev->dev.kobj, &fpga_port_stat_group);
    if (status) {
        goto exit_pci;
    }

    addr = ioremap(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].base_addr + SFP_LDB_GPIO1_DATA_EN, 32);
    iowrite32(0x707, addr);
    iounmap(addr);

    /* Init port Enable >> LDB/UDB (0 >> 1) */
    for(fpga_no = PCI_SUBSYSTEM_ID_LDB; fpga_no >= PCI_SUBSYSTEM_ID_UDB; fpga_no--)
    {
        for(cnt = 0; cnt <= 1; cnt++) {
            addr = ioremap(fpga_ctl->pci_fpga_dev[fpga_no].base_addr +
                           fpga_ctl->pci_fpga_dev[fpga_no].aslpc_cpld1_offset + 0xb0 + cnt, 32);
            iowrite8(0xff, addr);
            iounmap(addr);
        }
        for(cnt = 0; cnt <= 1; cnt++) {
            addr = ioremap(fpga_ctl->pci_fpga_dev[fpga_no].base_addr +
                           fpga_ctl->pci_fpga_dev[fpga_no].aslpc_cpld2_offset + 0xb0 + cnt, 32);
            iowrite8(0xff, addr);
            iounmap(addr);
        }
    }
    /* Init present >> LDB/UDB (1 >> 0) */
    for(fpga_no = PCI_SUBSYSTEM_ID_LDB; fpga_no >= PCI_SUBSYSTEM_ID_UDB; fpga_no--)
    {
        for(cnt = 0; cnt <= 1; cnt++) {
            addr = ioremap(fpga_ctl->pci_fpga_dev[fpga_no].base_addr +
                           fpga_ctl->pci_fpga_dev[fpga_no].aslpc_cpld1_offset + 0xb8 + cnt, 32);
            iowrite8(0x0, addr);
            iounmap(addr);
        }
        for(cnt = 0; cnt <= 1; cnt++) {
            addr = ioremap(fpga_ctl->pci_fpga_dev[fpga_no].base_addr +
                           fpga_ctl->pci_fpga_dev[fpga_no].aslpc_cpld2_offset + 0xb8 + cnt, 32);
            iowrite8(0x0, addr);
            iounmap(addr);
        }
    }
    /* Init 2XSFP Port Eanble & Present */
    addr = ioremap(fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].base_addr +
                   fpga_ctl->pci_fpga_dev[PCI_SUBSYSTEM_ID_LDB].aslpc_cpld1_offset + 0xbd, 32);
    iowrite8(0x3, addr);
    iounmap(addr);

    return 0;

exit_pci:
    pci_dev_put(pcidev);
    return status;
}

static int as9736_64d_pcie_fpga_stat_remove(struct platform_device *pdev)
{
    sysfs_remove_group(&pdev->dev.kobj, &fpga_port_stat_group);

    return 0;
}

static int as9736_64d_pcie_fpga_sfp_probe (struct platform_device *pdev)
{
    int status = 0;

    struct pcie_fpga_dev_platform_data *pdata = NULL;

    pdata = pdev->dev.platform_data;

    if (!pdata) {
        status = -ENOMEM;
        pcie_err("kzalloc failed\n");
        goto exit;
    }

    /*assign port num*/
    if(pdata->fpga_type==PCIE_FPGA_TYPE_LDB) {
        sprintf(pdata->name, "port%d", pdata->port_num + 32);
    } else {
        sprintf(pdata->name, "port%d", pdata->port_num);
    }

    status = sysfs_create_group(&pdev->dev.kobj, &fpga_eeprom_group);
    if (status) {
        pcie_err("sysfs_create_group failed\n");
        goto exit;
    }

    /* init eeprom */
    status = sfp_sysfs_eeprom_init(&pdev->dev.kobj, &pdata->eeprom_bin);
    if (status) {
        pcie_err("sfp_sysfs_eeprom_init failed\n");
        goto exit_remove;
    }

    return 0;

exit_remove:
    sysfs_remove_group(&pdev->dev.kobj, &fpga_eeprom_group);
exit:
    return status;
}

static int __exit as9736_64d_pcie_fpga_sfp_remove(struct platform_device *pdev)
{
    struct pcie_fpga_dev_platform_data *pdata = NULL;

    pdata = pdev->dev.platform_data;

    sysfs_remove_bin_file(&pdev->dev.kobj, &pdata->eeprom_bin);
    sysfs_remove_group(&pdev->dev.kobj, &fpga_eeprom_group);

    return 0;
}

static struct platform_driver pcie_fpga_port_stat_driver = {
    .probe      = as9736_64d_pcie_fpga_stat_probe,
    .remove     = as9736_64d_pcie_fpga_stat_remove,
    .driver     = {
        .owner = THIS_MODULE,
        .name  = DRVNAME,
    },
};

static struct platform_driver pcie_udb_fpga_driver = {
    .probe = as9736_64d_pcie_fpga_sfp_probe,
    .remove = __exit_p(as9736_64d_pcie_fpga_sfp_remove),
    .driver = {
        .owner = THIS_MODULE,
        .name = "pcie_udb_fpga_device",
    }
};

static struct platform_driver pcie_ldb_fpga_driver = {
    .probe = as9736_64d_pcie_fpga_sfp_probe,
    .remove = __exit_p(as9736_64d_pcie_fpga_sfp_remove),
    .driver = {
        .owner = THIS_MODULE,
        .name = "pcie_ldb_fpga_device",
    }
};

static int __init as9736_64d_pcie_sfp_init(void)
{
    int status = 0;
    int err_cnt;

    int udb_fpga_cnt = 0, ldb_fpga_cnt = 0, ldb_fpga_sfp_ddm_cnt = 0;

    /*Step1.
     *Init UDB, LDB port status driver*/
    status = platform_driver_register(&pcie_fpga_port_stat_driver);
    if (status < 0) {
        goto exit;
    }

    fpga_ctl = kzalloc(sizeof(struct as9736_64d_fpga_data), GFP_KERNEL);
    if (!fpga_ctl) {
        status = -ENOMEM;
        platform_driver_unregister(&pcie_fpga_port_stat_driver);
        goto exit;
    }

    mutex_init(&fpga_ctl->driver_lock);

    fpga_ctl->pdev = platform_device_register_simple(DRVNAME, -1, NULL, 0);
    if (IS_ERR(fpga_ctl->pdev)) {
        status = PTR_ERR(fpga_ctl->pdev);
        goto exit_pci;
    }

    /*Step2. Init port device driver*/
    mutex_init(&update_lock);

    /*UDB driver*/
    status = platform_driver_register(&pcie_udb_fpga_driver);
    if (status < 0) {
        pcie_err("Fail to register udb_fpga driver\n");
        goto exit_pci;
    }

    /*UDB port1-32 qsfp device*/
    for (udb_fpga_cnt = 0; udb_fpga_cnt < ARRAY_SIZE(pcie_udb_qsfp_device); udb_fpga_cnt++)
    {
        status = platform_device_register(&pcie_udb_qsfp_device[udb_fpga_cnt]);
        if (status) {
            pcie_err("Fail to register (UDB)port%d device.\n", (udb_fpga_cnt + 1) );
            goto exit_udb_fpga;
        }
    }
    pcie_info("Init UDB_FPGA driver and device.");

    /*LDB driver*/
    status = platform_driver_register(&pcie_ldb_fpga_driver);
    if (status < 0) {
        pcie_err("Fail to register ldb_fpga driver.\n");
        goto exit_udb_fpga;
    }
    /*LDB port33-64, 65-66 qsfp and sfp device*/
    for (ldb_fpga_cnt = 0; ldb_fpga_cnt < ARRAY_SIZE(pcie_ldb_qsfp_device); ldb_fpga_cnt++)
    {
        status = platform_device_register(&pcie_ldb_qsfp_device[ldb_fpga_cnt]);
        if (status) {
            pcie_err("Fail to register (LDB)port%d device.\n", (ldb_fpga_cnt + 33) );
            goto exit_ldb_fpga;
        }
    }
    pcie_info("Init LDB_FPGA driver and device.");

    return 0;

exit_ldb_fpga:
    for(err_cnt=(ldb_fpga_cnt-1);err_cnt>=0;err_cnt--){
        platform_device_unregister(&pcie_ldb_qsfp_device[err_cnt]);
    }
    platform_driver_unregister(&pcie_ldb_fpga_driver);
exit_udb_fpga:
    for(err_cnt=(udb_fpga_cnt-1);err_cnt>=0;err_cnt--){
        platform_device_unregister(&pcie_udb_qsfp_device[err_cnt]);
    }
    platform_driver_unregister(&pcie_udb_fpga_driver);
exit_pci:
    platform_driver_unregister(&pcie_fpga_port_stat_driver);
    kfree(fpga_ctl);
exit:
    return status;
}

static void __exit as9736_64d_pcie_sfp_exit(void)
{
    int i = 0;

    /*LDB qsfp port33-64, sfp port65-66*/
    for ( i = 0; i < ARRAY_SIZE(pcie_ldb_qsfp_device); i++ ) {
        platform_device_unregister(&pcie_ldb_qsfp_device[i]);
    }
    platform_driver_unregister(&pcie_ldb_fpga_driver);
    pcie_info("Remove LDB_FPGA driver and device.\n");

    /*UDB qsfp port1-32 */
    for ( i = 0; i < ARRAY_SIZE(pcie_udb_qsfp_device); i++ ) {
        platform_device_unregister(&pcie_udb_qsfp_device[i]);
    }
    platform_driver_unregister(&pcie_udb_fpga_driver);
    pcie_info("Remove UDB_FPGA driver and device.\n");

    /*UDB and LDB get port status*/
    platform_device_unregister(fpga_ctl->pdev);
    platform_driver_unregister(&pcie_fpga_port_stat_driver);
    pcie_info("Remove FPGA status driver.\n");
    kfree(fpga_ctl);
}


module_init(as9736_64d_pcie_sfp_init);
module_exit(as9736_64d_pcie_sfp_exit);

MODULE_AUTHOR("Michael Shih <michael_shih@edge-core.com>");
MODULE_DESCRIPTION("AS9734-64D READ EEPROM From FPGA via PCIE");
MODULE_LICENSE("GPL");

