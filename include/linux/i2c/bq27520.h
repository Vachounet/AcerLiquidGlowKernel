/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_BQ27520_H
#define __LINUX_BQ27520_H


#define	BQ275250_DFI_SUPPORT	1




#define BQ27520_BATT_LOW		19
#define BQ27520_GAUGE_INT		27


#define BQ27520_DRIVER_NAME				"bq27520-battery"



#define BQ27520_REG_CNTL		0x00
#define BQ27520_REG_AR			0x02
#define BQ27520_REG_ARTTE		0x04
#define BQ27520_REG_TEMP		0x06
#define BQ27520_REG_VOLT		0x08
#define BQ27520_REG_FLAGS		0x0A
#define BQ27520_REG_NAC			0x0C
#define BQ27520_REG_FAC			0x0e
#define BQ27520_REG_RM			0x10
#define BQ27520_REG_FCC			0x12
#define BQ27520_REG_AI			0x14
#define BQ27520_REG_TTE			0x16
#define BQ27520_REG_TTF			0x18
#define BQ27520_REG_SI			0x1a
#define BQ27520_REG_STTE		0x1c
#define BQ27520_REG_MLI			0x1e
#define BQ27520_REG_MLTTE		0x20
#define BQ27520_REG_AE			0x22
#define BQ27520_REG_AP			0x24
#define BQ27520_REG_TTECP		0x26
#define BQ27520_REG_SOH			0x28
#define BQ27520_REG_SOC			0x2c
#define BQ27520_REG_NIC			0x2e
#define BQ27520_REG_ICR			0x30
#define BQ27520_REG_LOGIDX		0x32
#define BQ27520_REG_LOGBUF		0x34

#define BQ27520_FLAG_DSC		BIT(0)
#define BQ27520_FLAG_SYSDOWN	BIT(1)
#define BQ27520_FLAG_SOC1		BIT(2)
#define BQ27520_FLAG_BAT_DET	BIT(3)
#define BQ27520_FLAG_WAIT_ID	BIT(4)
#define BQ27520_FLAG_OCV_GD		BIT(5)
#define BQ27520_FLAG_CHG		BIT(8)
#define BQ27520_FLAG_FC			BIT(9)
#define BQ27520_FLAG_XCHG		BIT(10)
#define BQ27520_FLAG_CHG_INH	BIT(11)
#define BQ27520_FLAG_OTD		BIT(14)
#define BQ27520_FLAG_OTC		BIT(15)

#define BQ27520_CS_QEN			BIT(0)
#define BQ27520_CS_SS		    BIT(13)
#define BQ27520_CS_DLOGEN		BIT(15)

#define BQ27520_SUBCMD_CTNL_STATUS	0x0000
#define BQ27520_SUBCMD_DEVCIE_TYPE	0x0001
#define BQ27520_SUBCMD_FW_VER		0x0002
#define BQ27520_SUBCMD_HW_VER		0x0003
#define BQ27520_SUBCMD_DF_CSUM		0x0004
#define BQ27520_SUBCMD_PREV_MACW	0x0007
#define BQ27520_SUBCMD_CHEM_ID		0x0008
#define BQ27520_SUBCMD_BD_OFFSET	0x0009
#define BQ27520_SUBCMD_INT_OFFSET	0x000a
#define BQ27520_SUBCMD_CC_VER		0x000b
#define BQ27520_SUBCMD_OCV			0x000c
#define BQ27520_SUBCMD_BAT_INS		0x000d
#define BQ27520_SUBCMD_BAT_REM		0x000e
#define BQ27520_SUBCMD_SET_HIB		0x0011
#define BQ27520_SUBCMD_CLR_HIB		0x0012
#define BQ27520_SUBCMD_SET_SLP		0x0013
#define BQ27520_SUBCMD_CLR_SLP		0x0014
#define BQ27520_SUBCMD_FCT_RES		0x0015
#define BQ27520_SUBCMD_ENABLE_DLOG	0x0018
#define BQ27520_SUBCMD_DISABLE_DLOG	0x0019
#define BQ27520_SUBCMD_SEALED		0x0020
#define BQ27520_SUBCMD_ENABLE_IT	0x0021
#define BQ27520_SUBCMD_DISABLE_IT	0x0023
#define BQ27520_SUBCMD_CAL_MODE		0x0040
#define BQ27520_SUBCMD_RESET		0x0041


#define BQ27520_EXTCMD_OPERATOR_CONFIG	0x3a
#define BQ27520_EXTCMD_APP_STATUS		0x6a



#define BQ27520_COULOMB_POLL				30	
#define BQ27520_INIT_DELAY					500	
#define BQ27520_POLLING_DETECT				300	
#define BQ27520_POLLING_CHARGING			5000
#define BQ27520_POLLING_DISCHARGING			30000
#define BQ27520_POLLING_CHARGINGDONE		5000
#define BQ27520_POLLING_FAULT				1000	
#define BQ27520_POLLING_BAT_LOW				3000
#define BQ27520_POLLING_BAT_CRITICAL_LOW	1000
#define BQ27520_POLLING_RECHG_MON			30000



struct bq27520_platform_data {
	const char *name;
	unsigned int soc_int;
	unsigned int bi_tout;
	unsigned int chip_en; /* CE */

	unsigned int bat_low;

	const char *vreg_name; /* regulater used by bq27520 */
	int vreg_value; /* its value */
	int enable_dlog; 
	int enable_it; 
};


int engineer_gauge_remain_capacity(void);
int engineer_gauge_full_charge_capacity(void);
int engineer_gauge_fw_version(void);
int engineer_gauge_flags(void);
int engineer_gauge_cntl_status(void);


void dump_gauge_log(void);


#endif /* __LINUX_BQ27520_H */
