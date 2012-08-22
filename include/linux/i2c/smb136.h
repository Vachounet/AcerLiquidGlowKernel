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
#ifndef __SMB136_H__
#define __SMB136_H__

#include <mach/msm_hsusb.h>


#define SMB136_CHG_STAT_INT		90
#define SMB136_CHG_EN			91
#define OVP_FLAG_INT			41
#define OVP_EXT_EN				84
#define SMB136_CHARGER_ADDR		(0x80 >> 1)


#define SMB136_DRIVER_NAME				"smb136-charger"



#define SMB136_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))
#define SMB136_REG_SET(REG, MASK, VALUE) (REG = (REG & ~MASK) | (VALUE & MASK))


#define CHG_CURRENT_REG						0x00
#define FAST_CHG_CURRENT_MASK				SMB136_MASK(3, 5)
#define PRE_CHG_CURRENT_MASK				SMB136_MASK(2, 3)
#define TERM_CHG_CURRENT_MASK				SMB136_MASK(2, 1)

#define FAST_CHG_CURRENT_950				(0x4 << 5)
#define PRE_CHG_CURRENT_100					(0x1 << 3)
#define TERM_CHG_CURRENT_50					(0x0 << 1)


#define INPUT_CURRENT_LIMIT_REG				0x01
#define IN_CURRENT_MASK						SMB136_MASK(3, 5)
#define IN_CURRENT_LIMIT_EN_BIT				BIT(2)
#define IN_CURRENT_DET_THRESH_MASK			SMB136_MASK(2, 0)

#define IN_CURRENT_1100						(0x4 << 5)
#define IN_CURRENT_DET_THRESH_AUTO			(0x0)


#define FLOAT_VOLTAGE_REG					0x02
#define STAT_OUT_POLARITY_BIT				BIT(7)
#define FLOAT_VOLTAGE_MASK					SMB136_MASK(7, 0)

#define FLOAT_VOLTAGE_4V22					(0x4C)


#define CONTROL_A_REG						0x03
#define AUTO_RECHARGE_DIS_BIT				BIT(7)
#define CURR_CYCLE_TERM_BIT					BIT(6)
#define PRE_TO_FAST_V_MASK					SMB136_MASK(3, 3)
#define TEMP_BEHAV_BIT						BIT(2)
#define THERM_NTC_CURR_MODE_BIT				BIT(1)
#define THERM_NTC_47KOHM_BIT				BIT(0)

#define PRE_TO_FAST_3V1						(0x7 << 3)


#define CONTROL_B_REG						0x04
#define STAT_OUTPUT_MODE_MASK				SMB136_MASK(2, 6)
#define BATT_OV_ENDS_CYCLE_BIT				BIT(5)
#define AUTO_PRE_TO_FAST_DIS_BIT			BIT(4)
#define SAFETY_TIMER_EN_BIT					BIT(3)
#define OTG_LBR_WD_EN_BIT					BIT(2)
#define CHG_WD_TIMER_EN_BIT					BIT(1)
#define IRQ_OP_MASK							BIT(0)

#define STAT_OUTPUT_MODE_CHARGER				(0x0 << 6)


#define PIN_CTRL_REG						0x05
#define AUTO_CHG_EN_BIT						BIT(7)
#define AUTO_LBR_EN_BIT						BIT(6)
#define OTG_LBR_BIT							BIT(5)
#define I2C_PIN_BIT							BIT(4)
#define PIN_EN_CTRL_MASK					SMB136_MASK(2, 2)
#define OTG_LBR_PIN_CTRL_MASK				SMB136_MASK(2, 0)

#define PIN_EN_CTRL_ACT_LOW					(0x2 << 2)
#define I2C_CTRL_0_DISABLE					(0x2 << 1)
#define OTG_LBR_PIN_CTRL_ACT_HIGH			(0x3)


#define OTG_LBR_CTRL_REG					0x06
#define BATT_MISSING_DET_EN_BIT				BIT(7)
#define AUTO_RECHARGE_THRESH_MASK			BIT(6)
#define USB_DP_DN_DET_EN_MASK				BIT(5)
#define OTG_LBR_BATT_CURRENT_LIMIT_MASK		SMB136_MASK(2, 3)
#define OTG_LBR_UVLO_THRESH_MASK			SMB136_MASK(3, 0)

#define OTG_LBR_BATT_CURRENT_LIMIT_500		(0x1 << 3)
#define OTG_LBR_UVLO_THRESH_2V75			(0x1)


#define FAULT_INTR_REG						0x07
#define SAFETY_TIMER_EXP_MASK				BIT(7)
#define BATT_TEMP_UNSAFE_MASK				BIT(6)
#define INPUT_OVLO_IVLO_MASK				BIT(5)
#define BATT_OVLO_MASK						BIT(4)
#define INTERNAL_OVER_TEMP_MASK				BIT(2)
#define ENTER_TAPER_CHG_MASK				BIT(1)
#define CHG_MASK							BIT(0)


#define CELL_TEMP_MON_REG					0x08
#define THERMISTOR_CURR_MASK				SMB136_MASK(2, 6)
#define LOW_TEMP_CHG_INHIBIT_MASK			SMB136_MASK(3, 3)
#define HIGH_TEMP_CHG_INHIBIT_MASK			SMB136_MASK(3, 0)

#define THERMISTOR_CURR_100uA				(0x0 << 6)
#define LOW_TEMP_CHG_INHIBIT_0				(0x4 << 3)
#define HIGH_TEMP_CHG_INHIBIT_45			(0x3)


#define	SAFETY_TIMER_THERMAL_SHUTDOWN_REG	0x09
#define DCIN_OVLO_SEL_MASK					BIT(7)
#define RELOAD_EN_INPUT_VOLTAGE_MASK		BIT(6)
#define THERM_SHUTDN_EN_MASK				BIT(5)
#define STANDBY_WD_TIMER_EN_MASK			BIT(4)
#define COMPLETE_CHG_TMOUT_MASK				SMB136_MASK(2, 2)
#define PRE_CHG_TMOUT_MASK					SMB136_MASK(2, 0)

#define COMPLETE_CHG_TMOUT_382				(0x0 << 2)
#define PRE_CHG_TMOUT_48					(0x0)


#define	VSYS_REG							0x0A
#define	VSYS_MASK							SMB136_MASK(3, 4)

#define	VSYS_3V38							(0x4 << 4)


#define	I2C_ADDR_REG						0x0B
#define	VOLATILE_WRITE_PRESSIOM				BIT(0)


#define IRQ_RESET_REG						0x30


#define COMMAND_A_REG						0x31
#define	VOLATILE_REGS_WRITE_PERM_BIT		BIT(7)
#define	POR_BIT								BIT(6)
#define	FAST_CHG_SETTINGS_BIT				BIT(5)
#define	BATT_CHG_EN_BIT						BIT(4)
#define	USBIN_MODE_500_BIT					BIT(3)
#define	USBIN_MODE_HCMODE_BIT				BIT(2)
#define	OTG_LBR_EN_BIT						BIT(1)
#define	STAT_OE_BIT							BIT(0)


#define STATUS_A_REG						0x32
#define INTERNAL_TEMP_IRQ_STAT				BIT(4)
#define DCIN_OV_IRQ_STAT					BIT(3)
#define DCIN_UV_IRQ_STAT					BIT(2)
#define USBIN_OV_IRQ_STAT					BIT(1)
#define USBIN_UV_IRQ_STAT					BIT(0)


#define STATUS_B_REG						0x33
#define USB_PIN_STAT						BIT(7)
#define USB51_MODE_STAT						BIT(6)
#define USB51_HC_MODE_STAT					BIT(5)
#define INTERNAL_TEMP_LIMIT_B_STAT			BIT(4)
#define DC_IN_OV_STAT						BIT(3)
#define DC_IN_UV_STAT						BIT(2)
#define USB_IN_OV_STAT						BIT(1)
#define USB_IN_UV_STAT						BIT(0)


#define	STATUS_C_REG						0x34
#define AUTO_IN_CURR_LIMIT_MASK				SMB136_MASK(4, 4)
#define AUTO_IN_CURR_LIMIT_STAT				BIT(3)
#define AUTO_SOURCE_DET_COMP_STAT_MASK		SMB136_MASK(2, 1)
#define AUTO_SOURCE_DET_RESULT_STAT			BIT(0)


#define	STATUS_D_REG						0x35
#define	VBATT_VSYS_STAT						BIT(7)
#define	USBIN_FAIL_STAT						BIT(6)
#define	BATT_TEMP_STAT_MASK					SMB136_MASK(2, 4)
#define	INTERNAL_TEMP_LIMIT_STAT			BIT(2)
#define	OTG_LBR_MODE_EN_STAT				BIT(1)
#define	OTG_LBR_VBATT_UVLO_STAT				BIT(0)

#define	BATT_TEMP_STAT_OK					(0x0 << 4)
#define	VBATT_LESS_THAN_VSYS				(0x1 << 7)
#define	USBIN_LESS_THAN_VBUSFAIL			(0x1 << 6)


#define	STATUS_E_REG						0x36
#define	CHARGE_CYCLE_COUNT_STAT				BIT(7)
#define	CHARGER_TERM_STAT					BIT(6)
#define	SAFETY_TIMER_STAT_MASK				SMB136_MASK(2, 4)
#define	CHARGER_ERROR_STAT					BIT(3)
#define	CHARGING_STAT_E						SMB136_MASK(2, 1)
#define	CHARGING_EN							BIT(0)

#define	NOT_CHARGING_STATUS						0x0
#define PRE_CHARGE_STATUS						0x1
#define FAST_CHARGE_STATUS						0x2
#define TAPER_CHARGE_STATUS						0x3


#define	STATUS_F_REG						0x37
#define	WD_IRQ_ACTIVE_STAT					BIT(7)
#define	OTG_OVERCURRENT_STAT				BIT(6)
#define	BATT_PRESENT_STAT					BIT(4)
#define	BATT_OV_LATCHED_STAT				BIT(3)
#define	CHARGER_OVLO_STAT					BIT(2)
#define	CHARGER_UVLO_STAT					BIT(1)
#define	BATT_LOW_STAT						BIT(0)


#define	STATUS_G_REG						0x38
#define	CHARGE_TIMEOUT_IRQ_STAT				BIT(7)
#define	PRECHARGE_TIMEOUT_IRQ_STAT			BIT(6)
#define	BATT_HOT_IRQ_STAT					BIT(5)
#define	BATT_COLD_IRQ_STAT					BIT(4)
#define	BATT_OV_IRQ_STAT					BIT(3)
#define	TAPER_CHG_IRQ_STAT					BIT(2)
#define	FAST_CHG_IRQ_STAT					BIT(1)
#define	CHARGING_IRQ_STAT					BIT(0)


#define	STATUS_H_REG						0x39
#define	CHARGE_TIMEOUT_STAT					BIT(7)
#define	PRECHARGE_TIMEOUT_STAT				BIT(6)
#define	BATT_HOT_STAT						BIT(5)
#define	BATT_COLD_STAT						BIT(4)
#define	BATT_OV_STAT						BIT(3)
#define	TAPER_CHG_STAT						BIT(2)
#define	FAST_CHG_STAT						BIT(1)
#define	CHARGING_STAT_H						BIT(0)


#define DEV_ID_REG							0x3B


#define COMMAND_B_REG						0x3C
#define	THERM_NTC_CURR_VERRIDE				BIT(7)




struct smb136_platform_data {
	int chg_stat_gpio;
	int chg_en_gpio;
	int ext_ovp_en_gpio;
	int ovp_flag_n_gpio;
	int batt_mah_rating;
#ifndef CONFIG_EXTERNAL_CHARGER
	int valid_n_gpio;
	int (*chg_detection_config) (void);
#endif
};

void smb136_otg_power(int on);

#endif
