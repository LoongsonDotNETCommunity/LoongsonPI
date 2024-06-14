/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2022 - 2023 Mucse Corporation. */

#ifndef _RNPGBE_PHY_H_
#define _RNPGBE_PHY_H_

#include "rnpgbe_type.h"
#define RNP_I2C_EEPROM_DEV_ADDR 0xA0
#define RNP_I2C_EEPROM_DEV_ADDR2 0xA2


#define RNP_YT8531_PHY_SPEC_CTRL      0x10
#define RNP_YT8531_PHY_SPEC_CTRL_FORCE_MDIX            0x0020
#define RNP_YT8531_PHY_SPEC_CTRL_AUTO_MDI_MDIX         0x0060
#define RNP_YT8531_PHY_SPEC_CTRL_MDIX_CFG_MASK         0x0060

/* EEPROM byte offsets */
#define SFF_MODULE_ID_OFFSET	0x00
#define SFF_DIAG_SUPPORT_OFFSET 0x5c
#define SFF_MODULE_ID_SFP		0x3
#define SFF_MODULE_ID_QSFP		0xc
#define SFF_MODULE_ID_QSFP_PLUS 0xd
#define SFF_MODULE_ID_QSFP28	0x11

/* Bitmasks */
#define RNP_SFF_DA_PASSIVE_CABLE 0x4
#define RNP_SFF_DA_ACTIVE_CABLE 0x8
#define RNP_SFF_DA_SPEC_ACTIVE_LIMITING 0x4
#define RNP_SFF_1GBASESX_CAPABLE 0x1
#define RNP_SFF_1GBASELX_CAPABLE 0x2
#define RNP_SFF_1GBASET_CAPABLE 0x8
#define RNP_SFF_10GBASESR_CAPABLE 0x10
#define RNP_SFF_10GBASELR_CAPABLE 0x20
#define RNP_SFF_ADDRESSING_MODE 0x4
#define RNP_I2C_EEPROM_READ_MASK 0x100
#define RNP_I2C_EEPROM_STATUS_MASK 0x3
#define RNP_I2C_EEPROM_STATUS_NO_OPERATION 0x0
#define RNP_I2C_EEPROM_STATUS_PASS 0x1
#define RNP_I2C_EEPROM_STATUS_FAIL 0x2
#define RNP_I2C_EEPROM_STATUS_IN_PROGRESS 0x3

/* Flow control defines */
#define RNP_TAF_SYM_PAUSE 0x400
#define RNP_TAF_ASM_PAUSE 0x800

/* Bit-shift macros */
#define RNP_SFF_VENDOR_OUI_BYTE0_SHIFT 24
#define RNP_SFF_VENDOR_OUI_BYTE1_SHIFT 16
#define RNP_SFF_VENDOR_OUI_BYTE2_SHIFT 8

/* Vendor OUIs: format of OUI is 0x[byte0][byte1][byte2][00] */
#define RNP_SFF_VENDOR_OUI_TYCO 0x00407600
#define RNP_SFF_VENDOR_OUI_FTL 0x00906500
#define RNP_SFF_VENDOR_OUI_AVAGO 0x00176A00
#define RNP_SFF_VENDOR_OUI_INTEL 0x001B2100

/* I2C SDA and SCL timing parameters for standard mode */
#define RNP_I2C_T_HD_STA 4
#define RNP_I2C_T_LOW 5
#define RNP_I2C_T_HIGH 4
#define RNP_I2C_T_SU_STA 5
#define RNP_I2C_T_HD_DATA 5
#define RNP_I2C_T_SU_DATA 1
#define RNP_I2C_T_RISE 1
#define RNP_I2C_T_FALL 1
#define RNP_I2C_T_SU_STO 4
#define RNP_I2C_T_BUF 5

#define RNP_TN_LASI_STATUS_REG 0x9005
#define RNP_TN_LASI_STATUS_TEMP_ALARM 0x0008

/* SFP+ SFF-8472 Compliance code */
#define RNP_SFF_SFF_8472_UNSUP 0x00

#endif /* _RNP_PHY_H_ */
