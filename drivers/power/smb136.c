/* Copyright (c) 2010-2011 Code Aurora Forum. All rights reserved.
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

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c/smb136.h>
#include <linux/power_supply.h>
#include <linux/msm-charger.h>
#include <linux/device.h>

#ifdef CONFIG_EXTERNAL_CHARGER
#include <mach/msm_hsusb.h>
#endif


#include <mach/chicago_hwid.h>


#include <linux/reboot.h>	


#include "../../arch/arm/mach-msm/proc_comm.h"
#include <oem/smem_pc_oem_cmd.h>


#define SMB136_DISCHG_PERIOD				(150 * 100)	
#define SMB136_CHG_PERIOD					(5 * 100)	
#define SMB136_CHG_DETECT					(3 * 10)	
#define SMB136_CHG_FAULT					(3 * 10)	



#define VCHG_VOLT_THRESHOLD					4400


#define INPUT_CURRENT_REG_DEFAULT			0xE1
#define INPUT_CURRENT_REG_MIN				0x01
#define	COMMAND_A_REG_DEFAULT				0xA0
#define	COMMAND_A_REG_OTG_MODE				0xA2

#define	PIN_CTRL_REG_DEFAULT				0x00
#define	PIN_CTRL_REG_CHG_OFF				0x04

#define SMB136_DEFAULT_BATT_RATING   		950
#define I2C_RETRY_MAX						5

struct smb136_data {
	struct i2c_client *client;
	struct delayed_work charge_work;

	bool charging;
	bool stop_charging;
	int chgcurrent;
	int cur_charging_mode;
	int max_system_voltage;
	int min_system_voltage;

	int chg_stat_gpio;
	int chg_en_gpio;
	int ext_ovp_en_gpio;
	int ovp_flag_n_gpio;
#ifndef CONFIG_EXTERNAL_CHARGER
	int valid_n_gpio;
#endif
	int batt_status;
	int batt_chg_type;
	int batt_present;

	int batt_health;

	int min_design;
	int max_design;
	int batt_mah_rating;

	int usb_status;

	u8 dev_id_reg;
	int chg_work_period;
	struct msm_hardware_charger adapter_hw_chg;

#ifdef CONFIG_EXTERNAL_CHARGER
	struct msm_otg_chg_notify otg_chg_notify;
#endif


	int vchg_switch;

};

static unsigned int disabled;
static DEFINE_MUTEX(init_lock);
static unsigned int init_otg_power;
static int valid_charger = -1;

enum charger_stat {
	SMB136_ABSENT,
	SMB136_PRESENT,
	SMB136_ENUMERATED,
};

static struct smb136_data *usb_smb136_chg = NULL;

static int smb136_read_reg(struct i2c_client *client, int reg,
	u8 *val)
{
	s32 ret;
	struct smb136_data *smb136_chg;

	smb136_chg = i2c_get_clientdata(client);
	ret = i2c_smbus_read_byte_data(smb136_chg->client, reg);
	if (ret < 0) {
		dev_err(&smb136_chg->client->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else
		*val = ret;

	return 0;
}

static int smb136_read_i2c(struct i2c_adapter *adap, u8 addr, int reg, u8* buf, u8 len)
{
	struct i2c_msg msgs[] = {
		[0] = {
			.addr   = addr,
			.flags  = 0,
			.buf    = (void *)&reg,
			.len    = 1
		},
		[1] = {
			.addr   = addr,
			.flags  = I2C_M_RD,
			.buf    = (void *)buf,
			.len    = len
		}
	};

	if(!usb_smb136_chg)
		return -ENODEV;

	return i2c_transfer(adap, msgs, 2);
}

#if 0
static int smb136_read_i2c_retry(struct i2c_adapter *adap, u8 addr, int reg, u8* buf, u8 len)
{
	int i, ret;

	for(i = 0; i < I2C_RETRY_MAX; i++) {
		ret = smb136_read_i2c(adap, addr, reg, buf, len);

		if(ret == 2)
 			return ret;
		else
			msleep(10);
	}

	return ret;
}
#endif

static int smb136_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;
	struct smb136_data *smb136_chg;

	smb136_chg = i2c_get_clientdata(client);
	ret = i2c_smbus_write_byte_data(smb136_chg->client, reg, val);
	if (ret < 0) {
		dev_err(&smb136_chg->client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb136_write_i2c(struct i2c_adapter *adap, u8 addr, int reg, u8* buf, u8 len)
{
	int i;
	u8 buf_w[64];
	struct i2c_msg msgs[] = {
		[0] = {
			.addr   = addr,
			.flags  = 0,
			.buf    = (void *)&buf_w,
			.len    = len + 1
		},
	};

	if (len >= sizeof(buf_w))
		return -ENOMEM;

	if(!usb_smb136_chg)
		return -ENODEV;

	buf_w[0] = reg;
	for(i = 0; i < len; i++)
		buf_w[i + 1] = buf[i];

	return i2c_transfer(adap, msgs, 1);
}

#if 0
static int smb136_wirte_i2c_retry(struct i2c_adapter *adap, u8 addr, int reg, u8* buf, u8 len)
{
	int i, ret;

	for(i = 0; i < I2C_RETRY_MAX; i++) {
		ret = smb136_write_i2c(adap, addr, reg, buf, len);

		if(ret == 1)
 			return ret;
		else
			msleep(10);
	}

	return ret;
}
#endif


static int get_vchg_voltage(int *volt)
{
	unsigned arg1 = SMEM_PC_OEM_AC_WALL_READ_VOLTAGE, arg2 = 0, ret;

	ret = msm_proc_comm(PCOM_CUSTOMER_CMD1, &arg1, &arg2);
	if(ret != 0) {
		pr_err("%s, msm_proc_comm result:%d\n", __func__, ret);
		*volt = 0;
	} else { 
		*volt = arg2;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: vbus volt(%d)--\n", __func__, arg2);
	}

	return ret; 
}

static int get_vbus_voltage(int *volt)
{
	unsigned arg1 = SMEM_PC_OEM_USB_WALL_READ_VOLTAGE, arg2 = 0, ret;

	ret = msm_proc_comm(PCOM_CUSTOMER_CMD1, &arg1, &arg2);
	if(ret != 0) {
		pr_err("%s, msm_proc_comm result:%d\n", __func__, ret);
		*volt = 0;
	} else { 
		*volt = arg2;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: vbus volt(%d)--\n", __func__, arg2);
	}

	return ret; 
}


static ssize_t id_reg_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct smb136_data *smb136_chg;

	smb136_chg = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%02x\n", smb136_chg->dev_id_reg);
}
static DEVICE_ATTR(id_reg, S_IRUGO | S_IWUSR, id_reg_show, NULL);


static void smb136_charger_register_log(struct i2c_client *client, bool smbus)
{
	u8 reg00_0b[12];
	u8 reg30_3c[13];
	int i, ret;

	memset(reg00_0b, 0, sizeof(reg00_0b));
	memset(reg30_3c, 0, sizeof(reg00_0b));

	if (smbus == 1) {
		for (i = 0; i < sizeof(reg00_0b); i++) {
			ret = smb136_read_reg(client, (CHG_CURRENT_REG + i), &reg00_0b[i]);
			if(ret) {
				dev_err(&client->dev, "%s read register(%d) failed(%d)\n",
					__func__, (CHG_CURRENT_REG + i), ret);
			}
		}

		for (i = 0; i < sizeof(reg30_3c); i++) {
			ret = smb136_read_reg(client, (IRQ_RESET_REG + i), &reg30_3c[i]);
			if(ret) {
				dev_err(&client->dev, "%s read register(%d) failed(%d)\n",
					__func__, (IRQ_RESET_REG + i), ret);
			}
		}
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "[SMB136]-smbus:(00-0B= %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X  %02X %02X)\n",
			reg00_0b[0], reg00_0b[1], reg00_0b[2], reg00_0b[3], reg00_0b[4],
			reg00_0b[5], reg00_0b[6], reg00_0b[7], reg00_0b[8], reg00_0b[9],
			reg00_0b[10], reg00_0b[11]);

		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "[SMB136]-smbus:(31~39= %02X %02X %02X %02X %02X  %02X %02X %02X %02X) (%02X %02X %02X)\n",
			reg30_3c[1], reg30_3c[2], reg30_3c[3], reg30_3c[4], reg30_3c[5],
			reg30_3c[6], reg30_3c[7], reg30_3c[8], reg30_3c[9],	reg30_3c[10],
			reg30_3c[11], reg30_3c[12]);
	} else {
		smb136_read_i2c(client->adapter, SMB136_CHARGER_ADDR, CHG_CURRENT_REG, &reg00_0b[0], sizeof(reg00_0b));
		smb136_read_i2c(client->adapter, SMB136_CHARGER_ADDR, IRQ_RESET_REG, &reg30_3c[0], sizeof(reg30_3c));

		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "[SMB136]-xfer:(00-0B= %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X  %02X %02X)\n",
			reg00_0b[0], reg00_0b[1], reg00_0b[2], reg00_0b[3], reg00_0b[4],
			reg00_0b[5], reg00_0b[6], reg00_0b[7], reg00_0b[8], reg00_0b[9],
			reg00_0b[10], reg00_0b[11]);

		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "[SMB136]-xfer:(31~39= %02X %02X %02X %02X %02X  %02X %02X %02X %02X) (%02X %02X %02X)\n",
			reg30_3c[1], reg30_3c[2], reg30_3c[3], reg30_3c[4], reg30_3c[5],
			reg30_3c[6], reg30_3c[7], reg30_3c[8], reg30_3c[9], reg30_3c[10],
			reg30_3c[11], reg30_3c[12]);
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS,"[SMB136]:CHG_INT=%d\n", gpio_get_value(SMB136_CHG_STAT_INT));
}

static void smb136_status_register_log(struct i2c_client *client, bool smbus)
{
 	u8 reg30_3c[13];
	int i, ret;

 	memset(reg30_3c, 0, sizeof(reg30_3c));

	if (smbus == 1) {
		for (i = 0; i < sizeof(reg30_3c); i++) {
			ret = smb136_read_reg(client, (IRQ_RESET_REG + i), &reg30_3c[i]);
			if(ret) {
				dev_err(&client->dev, "%s read register(%d) failed(%d)\n",
					__func__, (IRQ_RESET_REG + i), ret);
			}
		}
		msm_chg_printk(DBG_MSM_CHG_PROBE, "[SMB136]-smbus: Command: A(%02X), ", reg30_3c[1]);
		msm_chg_printk(DBG_MSM_CHG_PROBE, "Status: A(%02X) B(%02X) C(%02X) D(%02X) E(%02X) F(%02X) \
			G(%02X) H(%02X)\n", reg30_3c[2], reg30_3c[3], reg30_3c[4], reg30_3c[5], reg30_3c[6],
			reg30_3c[7], reg30_3c[8], reg30_3c[9]);

	} else {
 		smb136_read_i2c(client->adapter, SMB136_CHARGER_ADDR, IRQ_RESET_REG, &reg30_3c[0], sizeof(reg30_3c));

		msm_chg_printk(DBG_MSM_CHG_PROBE, "[SMB136]-smbus: Command: A(%02X), ", reg30_3c[1]);
		msm_chg_printk(DBG_MSM_CHG_PROBE, "Status: A(%02X) B(%02X) C(%02X) D(%02X) E(%02X) F(%02X) \
			G(%02X) H(%02X)\n", reg30_3c[2], reg30_3c[3], reg30_3c[4], reg30_3c[5], reg30_3c[6],
			reg30_3c[7], reg30_3c[8], reg30_3c[9]);
 	}

	msm_chg_printk(DBG_MSM_CHG_PROBE, "[SMB136]:CHG_INT=%d\n", gpio_get_value(SMB136_CHG_STAT_INT));
}


static void smb136_register_init(struct i2c_client *client, int smbus, int inited)
{
	int i, ret;
	u8 value, reg00_0a[11];

	reg00_0a[0] = (FAST_CHG_CURRENT_950 | PRE_CHG_CURRENT_100 | TERM_CHG_CURRENT_50);
	reg00_0a[1] = (IN_CURRENT_1100 | IN_CURRENT_DET_THRESH_AUTO);
	reg00_0a[2] = (FLOAT_VOLTAGE_4V22);
	reg00_0a[3] = (PRE_TO_FAST_3V1 | THERM_NTC_CURR_MODE_BIT);
	reg00_0a[4] = (STAT_OUTPUT_MODE_CHARGER | BATT_OV_ENDS_CYCLE_BIT | OTG_LBR_WD_EN_BIT | IRQ_OP_MASK);

	reg00_0a[5] = (I2C_PIN_BIT | PIN_EN_CTRL_ACT_LOW | OTG_LBR_PIN_CTRL_ACT_HIGH);

	reg00_0a[6] = (USB_DP_DN_DET_EN_MASK | OTG_LBR_BATT_CURRENT_LIMIT_500 | OTG_LBR_UVLO_THRESH_2V75);

	reg00_0a[7] = (SAFETY_TIMER_EXP_MASK | BATT_OVLO_MASK | INTERNAL_OVER_TEMP_MASK | CHG_MASK);

	reg00_0a[8] = (THERMISTOR_CURR_100uA | LOW_TEMP_CHG_INHIBIT_0 | HIGH_TEMP_CHG_INHIBIT_45);
	reg00_0a[9] = (THERM_SHUTDN_EN_MASK | COMPLETE_CHG_TMOUT_382 | PRE_CHG_TMOUT_48);
	reg00_0a[10] = (VSYS_3V38);

	
	ret = smb136_read_reg(client, COMMAND_A_REG, &value);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: COMMAND_A_REG=0x%x--\n", __func__, value);
	if(ret) {
		dev_err(&client->dev, "%s read register(%d) failed(%d)\n",
			__func__, COMMAND_A_REG, ret);
	}
	value |= VOLATILE_REGS_WRITE_PERM_BIT;
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: value=0x%x--\n", __func__, value);
	ret = smb136_write_reg(client, COMMAND_A_REG, value);
	if(ret) {
		dev_err(&client->dev, "%s write register(%d) failed(%d)\n",
			__func__, COMMAND_A_REG, ret);
	}

	ret = smb136_read_reg(client, STATUS_D_REG, &value);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: STATUS_D_REG=0x%x--\n", __func__, value);
	if(ret) {
		dev_err(&client->dev, "%s read register(%d) failed(%d)\n",
			__func__, STATUS_D_REG, ret);
	}

	if (smbus == 1) {
		for (i = 0; i < sizeof(reg00_0a); i++) {
			ret = smb136_write_reg(client, CHG_CURRENT_REG + i, reg00_0a[i]);
			if(ret) {
				dev_err(&client->dev, "%s write register(%d) failed(%d)\n",
					__func__, CHG_CURRENT_REG + i, ret);
			}
		}
	} else {
		ret = smb136_write_i2c(client->adapter, SMB136_CHARGER_ADDR, CHG_CURRENT_REG,
				reg00_0a, sizeof(reg00_0a));
	#if 0
		if (ret < 0) {
			dev_err(&client->dev, "%s write register failed(%d)\n",
				__func__, ret);
		}
	#endif
	}
}


static int smb136_start_charging(struct msm_hardware_charger *hw_chg,
		int chg_voltage, int chg_current)
{
	int cmd_val = COMMAND_A_REG_DEFAULT;
	u8 temp = 0;
	int ret = 0;
	struct smb136_data *smb136_chg;

	smb136_chg = container_of(hw_chg, struct smb136_data, adapter_hw_chg);

	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s--\n", __func__);

	if (disabled || smb136_chg->stop_charging) {
		dev_err(&smb136_chg->client->dev,
			"%s called when disabled\n", __func__);
		goto out;
	}

	if (smb136_chg->charging == true &&
		smb136_chg->chgcurrent == chg_current) {
		
		 dev_err(&smb136_chg->client->dev,
			 "%s charge with same current %d called again\n",
			  __func__, chg_current);
		goto out;
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);
	cancel_delayed_work_sync(&smb136_chg->charge_work);


	if (chg_current < 500) {
		cmd_val &= ~USBIN_MODE_500_BIT;
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: USBIN_MODE_100--\n",
			__func__, __LINE__);
	}
	else if (chg_current == 500) {
		cmd_val |= USBIN_MODE_500_BIT;
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: USBIN_MODE_500--\n",
			__func__, __LINE__);
	}
	else {
		cmd_val |= USBIN_MODE_HCMODE_BIT;
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: USBIN_MODE_HCMODE--\n",
			__func__, __LINE__);
	}

	smb136_chg->chgcurrent = chg_current;
	smb136_chg->cur_charging_mode = cmd_val;

	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: chgcurrent(%d)--\n",
		__func__, __LINE__, chg_current);

	
	ret = smb136_write_reg(smb136_chg->client,
			COMMAND_A_REG, cmd_val);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't write to command_reg\n", __func__);
		goto out;
	}


	ret = smb136_read_reg(smb136_chg->client, STATUS_D_REG, &temp);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: STATUS_D_REG=0x%x--\n", __func__, temp);
	if(ret) {
		dev_err(&smb136_chg->client->dev, "%s read register(%d) failed(%d)\n",
			__func__, STATUS_D_REG, ret);
		goto out;
	}

	
	if ((temp & BATT_TEMP_STAT_MASK) == BATT_TEMP_STAT_OK) {

	
		ret = smb136_read_reg(smb136_chg->client, PIN_CTRL_REG, &temp);
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: PIN_CTRL_REG=0x%x--\n", __func__, temp);
		if(ret) {
			dev_err(&smb136_chg->client->dev, "%s read register(%d) failed(%d)\n",
				__func__, PIN_CTRL_REG, ret);
			goto out;
		}

		temp &= ~I2C_PIN_BIT;

		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s: battery ok, change R05[4]--\n", __func__);
		ret = smb136_write_reg(smb136_chg->client, PIN_CTRL_REG, temp);
		if(ret) {
			dev_err(&smb136_chg->client->dev, "%s write register(%d) failed(%d)\n",
				__func__, PIN_CTRL_REG, ret);
			goto out;
		}
	}



#if 0
	ret = smb136_write_reg(smb136_chg->client,
			PIN_CTRL_REG, PIN_CTRL_REG_DEFAULT);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't write to pin ctrl reg\n", __func__);
		goto out;
	}
#else
	gpio_set_value(smb136_chg->chg_en_gpio, 0);
#endif

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);

	smb136_chg->charging = true;
	smb136_chg->batt_status = POWER_SUPPLY_STATUS_CHARGING;
	smb136_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	smb136_chg->chg_work_period = SMB136_CHG_DETECT;

	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: batt_chg_type is Trickle Charge--\n",
		__func__, __LINE__);

	ret = smb136_read_reg(smb136_chg->client, STATUS_E_REG, &temp);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d: STATUS_E_REG=%d--\n", __func__, __LINE__, temp);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't read status e reg %d\n", __func__, ret);
	} else {
		if (temp & CHARGER_ERROR_STAT) {
			dev_err(&smb136_chg->client->dev,
				"%s chg error E=0x%x\n", __func__, temp);
			smb136_status_register_log(smb136_chg->client, 0);
		}

		if (((temp & CHARGING_STAT_E) >> 1) >= FAST_CHARGE_STATUS) {
			smb136_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
			smb136_chg->chg_work_period = SMB136_CHG_PERIOD;
			
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_BEGIN_FAST_CHARGING--\n",
				__func__);
			msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_BATT_BEGIN_FAST_CHARGING);
			
		}
	}

	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s: chg_work_period=%d--\n",
		__func__, smb136_chg->chg_work_period);

	
	schedule_delayed_work(&smb136_chg->charge_work, smb136_chg->chg_work_period);

	smb136_charger_register_log(smb136_chg->client, 0);
out:
	return ret;
}

static int smb136_stop_charging(struct msm_hardware_charger *hw_chg)
{
	int ret = 0;
	struct smb136_data *smb136_chg;

	smb136_chg = container_of(hw_chg, struct smb136_data, adapter_hw_chg);

	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s--\n", __func__);

	if (smb136_chg->charging == false) {
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s Not charging~ Exit!!--\n", __func__);
		return 0;
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);

	smb136_chg->charging = false;
	smb136_chg->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	smb136_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
	smb136_chg->chg_work_period = SMB136_DISCHG_PERIOD;

	ret = smb136_write_reg(smb136_chg->client, COMMAND_A_REG,
				smb136_chg->cur_charging_mode);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't write to command_reg\n", __func__);
		goto out;
	}


#if 0
	ret = smb136_write_reg(smb136_chg->client,
			PIN_CTRL_REG, PIN_CTRL_REG_CHG_OFF);
	if (ret)
		dev_err(&smb136_chg->client->dev,
			"%s couldn't write to pin ctrl reg\n", __func__);
#else
	gpio_set_value(smb136_chg->chg_en_gpio, 1);
#endif
	
	smb136_register_init(smb136_chg->client, 0, 1);
	

	smb136_charger_register_log(smb136_chg->client, 0);

	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d--\n", __func__, __LINE__);

out:
	return ret;
}


static int smb136_charger_switch(struct msm_hardware_charger *hw_chg,
				int chg_voltage, int chg_current)
{
	int ret = 0;
	u8 cmd_val = 0;
	struct smb136_data *smb136_chg;

	smb136_chg = container_of(hw_chg, struct smb136_data, adapter_hw_chg);

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (disabled || smb136_chg->stop_charging) {
		dev_err(&smb136_chg->client->dev,
			"%s called when disabled\n", __func__);
		goto out;
	}

	if (smb136_chg->charging == true && smb136_chg->chgcurrent == chg_current) {
		
		 dev_err(&smb136_chg->client->dev,
			 "%s charge with same current %d called again\n",
			  __func__, chg_current);
		goto out;
	}

	ret = smb136_read_reg(smb136_chg->client, COMMAND_A_REG, &cmd_val);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d: COMMAND_A_REG=%d--\n",
		__func__, __LINE__, cmd_val);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't read command a reg %d\n", __func__, ret);
		goto out;
	}

	if (chg_current < 500) {
		cmd_val &= ~USBIN_MODE_500_BIT;
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: USBIN_MODE_100--\n",
			__func__, __LINE__);
	}
	else if (chg_current == 500) {
		cmd_val |= USBIN_MODE_500_BIT;
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: USBIN_MODE_500--\n",
			__func__, __LINE__);
	}
	else {
		cmd_val |= USBIN_MODE_HCMODE_BIT;
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: USBIN_MODE_HCMODE--\n",
			__func__, __LINE__);
	}

	smb136_chg->chgcurrent = chg_current;
	smb136_chg->cur_charging_mode = cmd_val;

	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s,%d: chgcurrent(%d)--\n",
		__func__, __LINE__, chg_current);

	ret = smb136_write_reg(smb136_chg->client,
			COMMAND_A_REG, cmd_val);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't write to command_reg(%d)\n", __func__, COMMAND_A_REG);
		goto out;
	}

out:
	return ret;
}



static int smb136_charger_state(void)
{
	if (usb_smb136_chg) {
		if (usb_smb136_chg->usb_status == SMB136_ABSENT) {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: Charger absent--\n", __func__);
			return 0;
		} else {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: Charger present--\n", __func__);
			return 1;
		}
	} else {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: No charger exit--\n", __func__);
		return -1;
	}
}



static int smb136_charger_register_status(int reg, u8 value)
{
	int ret = 0;

	if (usb_smb136_chg) {
		ret = smb136_read_reg(usb_smb136_chg->client, reg, &value);
		if(ret) {
			dev_err(&usb_smb136_chg->client->dev, "%s read register(%d) failed(%d)\n",
				__func__, reg, ret);
		}
	} else {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: usb_smb136_chg not ready!!--\n", __func__);
		ret = -1;
	}
	return ret;
}


#ifndef CONFIG_EXTERNAL_CHARGER
static irqreturn_t smb136_valid_handler(int irq, void *dev_id)
{
	struct smb136_data *smb136_chg = (struct smb136_data*) dev_id;
	int val;

	msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "%s: Cable Detected USB inserted.\n", __func__);

	
	usleep_range(1000, 1200);
	val = gpio_get_value_cansleep(smb136_chg->valid_n_gpio);
	if (val < 0) {
		dev_err(&smb136_chg->client->dev,
			"%s gpio_get_value failed for %d ret=%d\n", __func__,
			smb136_chg->valid_n_gpio, val);
		goto err;
	}
	msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s,%d: val=%d--\n", __func__, __LINE__, val);

	if (val) {
		if (smb136_chg->usb_status != SMB136_ABSENT) {
			smb136_chg->usb_status = SMB136_ABSENT;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_REMOVED_EVENT--\n",
				__func__);
			msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_REMOVED_EVENT);
		}
	} else {
		if (smb136_chg->usb_status == SMB136_ABSENT) {
			smb136_chg->usb_status = SMB136_PRESENT;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_INSERTED_EVENT--\n",
				__func__);
			msm_charger_notify_event(&smb136_chg->adapter_hw_chg,
					CHG_INSERTED_EVENT);
		}
	}

err:
	return IRQ_HANDLED;
}
#endif


static irqreturn_t smb136_charger_fault_handler(int irq, void *dev_id)
{
	struct smb136_data *smb136_chg = (struct smb136_data*) dev_id;

	int ret;
	u8 reg_irq = 0, status = 0;

	msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s--\n", __func__);

	if(!smb136_chg->client)
		goto err;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);

	
	ret = smb136_write_reg(smb136_chg->client, IRQ_RESET_REG, 0X00);
	if(ret){
		dev_err(&smb136_chg->client->dev,
			"%s write IRQ reset register fail %d\n", __func__, ret);
		goto err;
	}

	
	ret = smb136_read_reg(smb136_chg->client, STATUS_A_REG, &reg_irq);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't read status A reg %d\n", __func__, ret);
		goto err;
	} else
		msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s: STATUS_A_REG=0x%x--\n", __func__, reg_irq);

	
	ret = smb136_read_reg(smb136_chg->client, STATUS_B_REG, &status);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't read status B reg %d\n", __func__, ret);
		goto err;
	} else
		msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s: STATUS_B_REG=0x%x--\n", __func__, status);
#if 0 
	if (reg_irq & USBIN_OV_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: USBIN OV IRQ.\n", __func__);

		if(status & USBIN_OV_IRQ_STAT) {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: USBIN OV.\n", __func__);
		}
	}

	if (reg_irq & USBIN_UV_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: USBIN UV IRQ.\n", __func__);

		if(status & USBIN_UV_IRQ_STAT){
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: USBIN UV.\n", __func__);
		}
	}
#endif

	if (reg_irq & INTERNAL_TEMP_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: INTERNAL TEMP IRQ.\n", __func__);

		if(status & INTERNAL_TEMP_IRQ_STAT){
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: INTERNAL TEMP LIMIT.\n", __func__);
		}
	}
	
	ret = smb136_read_reg(smb136_chg->client, STATUS_G_REG, &reg_irq);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't read status G reg %d\n", __func__, ret);
		goto err;
	} else
		msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s: STATUS_G_REG=0x%x--\n", __func__, reg_irq);

	
	ret = smb136_read_reg(smb136_chg->client, STATUS_H_REG, &status);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't read status H reg %d\n", __func__, ret);
		goto err;
	} else
		msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s: STATUS_B_REG=0x%x--\n", __func__, status);

	
	if (reg_irq & BATT_OV_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT OVER VOLTAGE IRQ.\n", __func__);

		if(status & BATT_OV_STAT){
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT_IS_OVER_VOLTAGE.\n", __func__);
			smb136_chg->chg_work_period = SMB136_CHG_FAULT;
			smb136_chg->batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			msm_charger_notify_event(&smb136_chg->adapter_hw_chg, OVP_FAULT_EVENT);
		}
	}
	

	if (reg_irq & CHARGE_TIMEOUT_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: CHARGE TIMEOUT IRQ.\n", __func__);

		if(status & CHARGE_TIMEOUT_STAT) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_COMPLETE_CHG_TIME_OUT--\n",
				__func__);
			smb136_chg->chg_work_period = SMB136_DISCHG_PERIOD;
			msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_COMPLETE_CHG_TIME_OUT);
		}
	}

	if (reg_irq & PRECHARGE_TIMEOUT_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: PRECHARGE TIMEOUT IRQ.\n", __func__);

		if(status & PRECHARGE_TIMEOUT_IRQ_STAT){
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_PRE_CHG_TIME_OUT--\n",
				__func__);
			smb136_chg->chg_work_period = SMB136_DISCHG_PERIOD;
			msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_PRE_CHG_TIME_OUT);
		}
	}
#if 0 
	if (reg_irq & BATT_HOT_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT HOT IRQ.\n", __func__);

		if(status & BATT_HOT_IRQ_STAT){
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT TOO HOT.\n", __func__);
		}
	}

	if (reg_irq & BATT_COLD_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT COLD IRQ.\n", __func__);

		if(status & BATT_COLD_IRQ_STAT){
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT TOO COLD.\n", __func__);
		}
	}

	if (reg_irq & BATT_OV_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT OV IRQ.\n", __func__);

		if(status & BATT_OV_IRQ_STAT){
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT OV.\n", __func__);
		}
	}

	if (reg_irq & TAPER_CHG_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: TAPER CHG IRQ.\n", __func__);

		if(status & TAPER_CHG_IRQ_STAT){
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: TAPER CHG.\n", __func__);
		}
	}

	if (reg_irq & FAST_CHG_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: FAST CHG IRQ.\n", __func__);

		if(status & FAST_CHG_IRQ_STAT){
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: FAST CHG.\n", __func__);
		}
	}
#endif
	if (reg_irq & CHARGING_IRQ_STAT) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: CHARGING IRQ.\n", __func__);

		if(!(status & CHARGING_IRQ_STAT)){
			if (!smb136_chg->batt_present && smb136_chg->charging) {
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: CHARGING.\n", __func__);
			}
		}
	}

err:
	return IRQ_HANDLED;
}


static int ovp_fault_gpio_state(void)
{
	return gpio_get_value(usb_smb136_chg->ovp_flag_n_gpio);
}

static irqreturn_t ovp_fault_handler(int irq, void *dev_id)
{
	struct smb136_data *smb136_chg = (struct smb136_data*) dev_id;

	msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s--\n", __func__);

	
	msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: ovp_flag_n_gpio=%d--\n",
		__func__, gpio_get_value(smb136_chg->ovp_flag_n_gpio));

	msleep(50);

	
	if (!gpio_get_value(smb136_chg->ovp_flag_n_gpio)) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: OVP_FAULT_EVENT--\n", __func__);
		smb136_chg->chg_work_period = SMB136_CHG_FAULT;
		msm_charger_notify_event(&smb136_chg->adapter_hw_chg, OVP_FAULT_EVENT);
	} else {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: OVP_RECOVERY_EVENT--\n", __func__);
		smb136_chg->chg_work_period = SMB136_CHG_DETECT;
		msm_charger_notify_event(&smb136_chg->adapter_hw_chg, OVP_RECOVERY_EVENT);
	}
	

	return IRQ_HANDLED;
}



#ifdef CONFIG_DEBUG_FS
static struct dentry *dent;
static int debug_fs_otg;

static int otg_get(void *data, u64 *value)
{
	*value = debug_fs_otg;
	return 0;
}

static int otg_set(void *data, u64 value)
{
	smb136_otg_power(debug_fs_otg);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smb136_otg_fops, otg_get, otg_set, "%llu\n");

static int chg_id_get(void *data, u64 *value)
{
	struct smb136_data *smb136_chg = (struct smb136_data *)data;
	u8 id_reg = 0;

	if (smb136_chg->dev_id_reg != 0xA) {
		smb136_read_reg(smb136_chg->client, DEV_ID_REG, &id_reg);
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "--%s: chip id is %d--\n",
			__func__, id_reg);

		if (id_reg != 0xA)
			*value = 0;
	}
	else
		*value = smb136_chg->dev_id_reg;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smb136_chg_id_fops, chg_id_get, NULL, "%llu\n");

static int chg_en_get(void *data, u64 *value)
{
	struct smb136_data *smb136_chg = (struct smb136_data *)data;
	int gpio;

	gpio = gpio_get_value(smb136_chg->chg_en_gpio);

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "--%s: chg_en_gpio(%d)=%d, charger is %s--\n",
		__func__, smb136_chg->chg_en_gpio, gpio, gpio ? "disable":"enable");

	*value = !gpio;

	return 0;
}

static int chg_en_set(void *data, u64 value)
{
	struct smb136_data *smb136_chg = (struct smb136_data *)data;

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "--%s: set chg_en_gpio(%d) as %s--\n",
		__func__, smb136_chg->chg_en_gpio, value? "enable":"disable");

	gpio_set_value(smb136_chg->chg_en_gpio, !value);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smb136_chg_en_fops, chg_en_get, chg_en_set, "%llu\n");

static int smb136_register_get(void *data, u64 *value)
{
	struct smb136_data *smb136_chg = (struct smb136_data *)data;

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "--%s: get smb136 charger log--\n", __func__);

	smb136_charger_register_log(smb136_chg->client, 0);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smb136_register_fops, smb136_register_get, NULL, "%llu\n");


static int ext_ovp_en_get(void *data, u64 *value)
{
	struct smb136_data *smb136_chg = (struct smb136_data *)data;
	int gpio;

	gpio = gpio_get_value(smb136_chg->ext_ovp_en_gpio);

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "--%s: chg_en_gpio(%d)=%d, charger is %s--\n",
		__func__, smb136_chg->ext_ovp_en_gpio, gpio, gpio ? "disable":"enable");

	*value = !gpio;

	return 0;
}

static int ext_ovp_en_set(void *data, u64 value)
{
	struct smb136_data *smb136_chg = (struct smb136_data *)data;

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "--%s: set chg_en_gpio(%d) as %s--\n",
		__func__, smb136_chg->ext_ovp_en_gpio, value? "enable":"disable");

	gpio_set_value(smb136_chg->ext_ovp_en_gpio, !value);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smb136_ext_ovp_en_fops, ext_ovp_en_get, ext_ovp_en_set, "%llu\n");



static int vchg_volt_get(void *data, u64 *value)
{

	int volt = 0, ret;

	ret = get_vchg_voltage(&volt);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: vchg voltage(%d)--\n", __func__, volt);
	if (ret < 0) {
		pr_err("%s, read vchg voltage failed\n", __func__);
		*value = 0;
	} else {
		*value = volt;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smb136_vchg_volt_fops, vchg_volt_get, NULL, "%llu\n");

static int vbus_volt_get(void *data, u64 *value)
{

	int volt = 0, ret;

	ret = get_vbus_voltage(&volt);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: vbus voltage(%d)--\n", __func__, volt);
	if (ret < 0) {
		pr_err("%s, read vbus voltage failed\n", __func__);
		*value = 0;
	} else {
		*value = volt;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smb136_vbus_volt_fops, vbus_volt_get, NULL, "%llu\n");


static void smb136_create_debugfs_entries(struct smb136_data *smb136_chg)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	dent = debugfs_create_dir(SMB136_DRIVER_NAME, NULL);
	if (dent) {
		debugfs_create_file("smb136_otg", S_IRUGO | S_IWUGO, dent, smb136_chg, &smb136_otg_fops);
		debugfs_create_file("chg_en", S_IRUGO | S_IWUGO, dent, smb136_chg, &smb136_chg_en_fops);
		debugfs_create_file("chip_id", S_IRUGO, dent, smb136_chg, &smb136_chg_id_fops);
		debugfs_create_file("reg_log", S_IRUGO, dent, smb136_chg, &smb136_register_fops);
		
		debugfs_create_file("ext_ovp_en", S_IRUGO | S_IWUGO, dent, smb136_chg, &smb136_ext_ovp_en_fops);
		
		
		debugfs_create_file("vchg_volt", S_IRUGO, dent, smb136_chg, &smb136_vchg_volt_fops);
		debugfs_create_file("vbus_volt", S_IRUGO, dent, smb136_chg, &smb136_vbus_volt_fops);
		
	}
}

static void smb136_destroy_debugfs_entries(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (dent)
		debugfs_remove_recursive(dent);
}
#else
static void smb136_create_debugfs_entries(struct smb136_data *smb136_chg)
{
}
static void smb136_destroy_debugfs_entries(void)
{
}
#endif

static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	if (usb_smb136_chg && disabled) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_DONE_EVENT--\n", __func__);
		msm_charger_notify_event(&usb_smb136_chg->adapter_hw_chg, CHG_DONE_EVENT);
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: disable=%d--\n", __func__, disabled);

	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&disabled, 0644);

static void smb136_charge_sm(struct work_struct *smb136_work)
{
	int ret;
	struct smb136_data *smb136_chg;
	u8 temp = 0;
	int notify_batt_changed = 0, volt;

	msm_chg_printk(DBG_MSM_CHG_WROKQUEUE, "--%s--\n", __func__);

	smb136_chg = container_of(smb136_work, struct smb136_data, charge_work.work);

	
	if (!smb136_chg->charging) {
		msm_chg_printk(DBG_MSM_CHG_WROKQUEUE, "--%s Not charging~ Exit!!--\n", __func__);
		return;
	}

	ret = smb136_read_reg(smb136_chg->client, STATUS_F_REG, &temp);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: STATUS_F_REG=0x%x --\n", __func__, temp);
	if (ret) {
		dev_err(&smb136_chg->client->dev,
			"%s couldn't read status f reg %d\n", __func__, ret);
		goto out;
	}

	if (smb136_chg->batt_present != !(temp & BATT_PRESENT_STAT)) {
		smb136_chg->batt_present = !(temp & BATT_PRESENT_STAT);
		notify_batt_changed = 1;
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s: notify_batt_changed--\n", __func__);
	}

	if (!smb136_chg->batt_present) {
		smb136_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s: no batt present, batt_chg_type is None--\n", __func__);
	}

	
	if (!smb136_chg->batt_present && smb136_chg->charging) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_REMOVED--\n", __func__);
		smb136_chg->chg_work_period = SMB136_CHG_FAULT;
		msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_BATT_REMOVED);
	}
	

	if (smb136_chg->batt_present && smb136_chg->charging &&
		smb136_chg->batt_chg_type != POWER_SUPPLY_CHARGE_TYPE_FAST) {
	
		ret = smb136_read_reg(smb136_chg->client, STATUS_D_REG, &temp);
		if(ret) {
			dev_err(&smb136_chg->client->dev, "%s read register(%d) failed(%d)\n",
				__func__, STATUS_D_REG, ret);
			goto out;
		} else {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: STATUS_D_REG=0x%x--\n", __func__, temp);

			
			if ((temp & BATT_TEMP_STAT_MASK) == BATT_TEMP_STAT_OK) {
			
			
				ret = smb136_read_reg(smb136_chg->client, PIN_CTRL_REG, &temp);
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: PIN_CTRL_REG=0x%x--\n", __func__, temp);
				if(ret) {
					dev_err(&smb136_chg->client->dev, "%s read register(%d) failed(%d)\n",
						__func__, PIN_CTRL_REG, ret);
					goto out;
				}

				temp &= ~I2C_PIN_BIT;
				msm_chg_printk(DBG_MSM_CHG_INFO, "--%s: battery ok, change R05[4]--\n", __func__);
				ret = smb136_write_reg(smb136_chg->client, PIN_CTRL_REG, temp);
				if(ret) {
					dev_err(&smb136_chg->client->dev, "%s couldn't write register(%d) failed(%d)\n",
						__func__, PIN_CTRL_REG, ret);
					goto out;
				}
			}
		}
	

		ret = smb136_read_reg(smb136_chg->client, STATUS_E_REG, &temp);
		if (ret) {
			dev_err(&smb136_chg->client->dev,
				"%s couldn't read cntrl reg\n", __func__);
			goto out;

		} else {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: STATUS_E_REG=0x%x --\n", __func__, temp);

			if (temp & CHARGER_ERROR_STAT) {
				dev_err(&smb136_chg->client->dev,
					"%s error E=0x%x\n", __func__, temp);
				smb136_status_register_log(smb136_chg->client, 0);
			}

			if (((temp & CHARGING_STAT_E) >> 1) >= FAST_CHARGE_STATUS) {
				smb136_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
				smb136_chg->chg_work_period = SMB136_CHG_PERIOD;
				notify_batt_changed = 1;
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_BEGIN_FAST_CHARGING--\n", __func__);
				smb136_chg->chg_work_period = SMB136_CHG_PERIOD;
				msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_BATT_BEGIN_FAST_CHARGING);

			} else {
				smb136_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
				msm_chg_printk(DBG_MSM_CHG_INFO, "--%s: batt_chg_type is Trickle Charge--\n",
					__func__);
			}

			
			if (temp & CHARGER_TERM_STAT) {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_DONE_EVENT--\n", __func__);
			
			}
			
		}
	}

	if (smb136_chg->batt_present && smb136_chg->charging) {
		
		
		ret = smb136_read_reg(smb136_chg->client, STATUS_H_REG, &temp);
		if (ret) {
			dev_err(&smb136_chg->client->dev,
				"%s couldn't read status H reg %d\n", __func__, ret);
			goto out;
		} else
			msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s: STATUS_H_REG=0x%x--\n", __func__, temp);

		if (smb136_chg->batt_health == POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
			if ((temp & BATT_OV_STAT) != BATT_OV_STAT){
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: BATT_VOLTAGE_IS_RECOVERY.\n", __func__);
				smb136_chg->chg_work_period = SMB136_CHG_FAULT;
				smb136_chg->batt_health = POWER_SUPPLY_HEALTH_GOOD;
				msm_charger_notify_event(&smb136_chg->adapter_hw_chg, OVP_RECOVERY_EVENT);
			}
		}
		

		
		if (smb136_chg->vchg_switch < 2 &&
			smb136_chg->adapter_hw_chg.charger_type == USB_CHG_TYPE__OTHERWALLCHARGER) {
			ret = get_vchg_voltage(&volt);
			if (ret < 0) {
				dev_err(&smb136_chg->client->dev, "%s, read vchg voltage failed\n", __func__);
			} else {
				if (volt < VCHG_VOLT_THRESHOLD){
					msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: vbus(%d) drop below %dV--\n",
						__func__, volt, VCHG_VOLT_THRESHOLD);

				
				#if 0
					if (smb136_chg->vchg_switch == 0) {
						smb136_chg->vchg_switch++;
						msm_chg_printk(DBG_MSM_CRITICAL, "--%s: switch to 500mA--\n", __func__);
						smb136_charger_switch(&smb136_chg->adapter_hw_chg, smb136_chg->max_design, 500);
					} else if (smb136_chg->vchg_switch == 1) {
						smb136_chg->vchg_switch++;
						msm_chg_printk(DBG_MSM_CRITICAL, "--%s: detect as invalid charger--\n", __func__);
						msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_INVALID_CHARGER);
					}
				#else
					smb136_chg->vchg_switch++;
					if (smb136_chg->vchg_switch > 1) {
						msm_chg_printk(DBG_MSM_CRITICAL, "--%s: detect as invalid charger--\n", __func__);
						msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_INVALID_CHARGER);
					}
				#endif
				} else {
					smb136_chg->vchg_switch = 0;
					msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: vchg(%d) is stable--\n",
						__func__, volt);
				}
				
			}
		}
		
	}

out:
	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s: chg_work_period=%d--\n",
		__func__, smb136_chg->chg_work_period);
	schedule_delayed_work(&smb136_chg->charge_work, smb136_chg->chg_work_period);
}

static void __smb136_otg_power(int on)
{
	int ret;

	if (on) {

		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);

		
		#if 0
			ret = smb136_write_reg(usb_smb136_chg->client,
						PIN_CTRL_REG, PIN_CTRL_REG_CHG_OFF);
			if (ret) {
				pr_err("%s turning off charging in pin_ctrl err=%d\n",
									__func__, ret);
				
				return;
			}
		#else
			gpio_set_value(usb_smb136_chg->chg_en_gpio, 1);
		#endif

		ret = smb136_write_reg(usb_smb136_chg->client,
			COMMAND_A_REG, COMMAND_A_REG_OTG_MODE);
		if (ret)
			pr_err("%s failed turning on OTG mode ret=%d\n",
								__func__, ret);
	} else {

		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);

		ret = smb136_write_reg(usb_smb136_chg->client,
			COMMAND_A_REG, COMMAND_A_REG_DEFAULT);
		if (ret)
			pr_err("%s failed turning off OTG mode ret=%d\n",
								__func__, ret);

		
		#if 0
			ret = smb136_write_reg(usb_smb136_chg->client,
					PIN_CTRL_REG, PIN_CTRL_REG_DEFAULT);
			if (ret)
				pr_err("%s failed writing to pn_ctrl ret=%d\n",
									__func__, ret);
		#else
			gpio_set_value(usb_smb136_chg->chg_en_gpio, 0);
		#endif
	}
}


#ifdef CONFIG_EXTERNAL_CHARGER
void smb136_hsusb_chg_connected(enum chg_type chgtype)
{
	
	char *chg_types[] = {
			"STD DOWNSTREAM PORT",
			"CARKIT",
			"DEDICATED CHARGER",
			"INVALID",
			"OTHERWALLCHARGER",
			"FASTERCHARGER"};

	msm_chg_printk(DBG_MSM_CHG_INFO, "--%s: Charger Type:%s--\n",
		__func__, chg_types[chgtype]);

	if(usb_smb136_chg)
		usb_smb136_chg->adapter_hw_chg.charger_type = chgtype;

	if (chgtype == USB_CHG_TYPE__INVALID) {

		valid_charger = 0;

		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s, %d--\n", __func__, __LINE__);

		if (usb_smb136_chg && usb_smb136_chg->usb_status != SMB136_ABSENT) {
			usb_smb136_chg->usb_status = SMB136_ABSENT;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_REMOVED_EVENT--\n", __func__);
			msm_charger_notify_event(&usb_smb136_chg->adapter_hw_chg, CHG_REMOVED_EVENT);
			usb_smb136_chg->vchg_switch = 0;
		}
		return;
	}
	

	valid_charger = 1;

	if (usb_smb136_chg && usb_smb136_chg->usb_status == SMB136_ABSENT) {
		usb_smb136_chg->usb_status = SMB136_PRESENT;
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_INSERTED_EVENT--\n", __func__);
		msm_charger_notify_event(&usb_smb136_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
	}
}
EXPORT_SYMBOL(smb136_hsusb_chg_connected);
#endif



static int __devinit smb136_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	const struct smb136_platform_data *pdata;
	struct smb136_data *smb136_chg;

	int chg_fault_int, ovp_fault_int, ret = 0;


	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		dev_err(&client->dev, "%s no platform data\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto out;
	}

	smb136_chg = kzalloc(sizeof(*smb136_chg), GFP_KERNEL);
	if (!smb136_chg) {
		ret = -ENOMEM;
		goto out;
	}

	INIT_DELAYED_WORK(&smb136_chg->charge_work, smb136_charge_sm);

	smb136_chg->client = client;
#ifndef CONFIG_EXTERNAL_CHARGER
	smb136_chg->valid_n_gpio = pdata->valid_n_gpio;
#endif
	smb136_chg->chg_stat_gpio = pdata->chg_stat_gpio;
	smb136_chg->chg_en_gpio = pdata->chg_en_gpio;
	smb136_chg->ovp_flag_n_gpio = pdata->ovp_flag_n_gpio;
	smb136_chg->ext_ovp_en_gpio = pdata->ext_ovp_en_gpio;

	smb136_chg->batt_mah_rating = pdata->batt_mah_rating;
	if (smb136_chg->batt_mah_rating == 0)
		smb136_chg->batt_mah_rating = SMB136_DEFAULT_BATT_RATING;

	
	smb136_chg->adapter_hw_chg.type = CHG_TYPE_USB;
	smb136_chg->adapter_hw_chg.rating = pdata->batt_mah_rating;
	smb136_chg->adapter_hw_chg.name = "Smb136-Charger";
	smb136_chg->adapter_hw_chg.charger_type = USB_CHG_TYPE__INVALID;
	smb136_chg->adapter_hw_chg.start_charging = smb136_start_charging;
	smb136_chg->adapter_hw_chg.stop_charging = smb136_stop_charging;
	smb136_chg->adapter_hw_chg.charging_switched = smb136_charger_switch;

	smb136_chg->adapter_hw_chg.get_charger_state = smb136_charger_state;


	smb136_chg->adapter_hw_chg.get_charger_register_status = smb136_charger_register_status;



	smb136_chg->adapter_hw_chg.ovp_fault_gpio_state = ovp_fault_gpio_state;


#ifdef CONFIG_EXTERNAL_CHARGER
	smb136_chg->otg_chg_notify.msm7x27a_chg_connected = smb136_hsusb_chg_connected;
	smb136_chg->otg_chg_notify.msm7x27a_chg_vbus_draw = msm_charger_vbus_draw;

	smb136_chg->otg_chg_notify.msm7x27a_get_batt_capacity = msm_chg_get_batt_capacity_percent;

#endif


	i2c_set_clientdata(client, smb136_chg);

#ifndef CONFIG_EXTERNAL_CHARGER
	if (pdata->chg_detection_config) {
		ret = pdata->chg_detection_config();
		if (ret) {
			dev_err(&client->dev, "%s valid config failed ret=%d\n",
				__func__, ret);
			goto free_smb136_chg;
		}
	}

	ret = gpio_request(pdata->valid_n_gpio, "smb136_charger_valid");
	if (ret) {
		dev_err(&client->dev, "%s gpio_request failed for %d ret=%d\n",
			__func__, pdata->valid_n_gpio, ret);
		goto free_smb136_chg;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret) {
		dev_err(&client->dev, "%s failed for irq_set_irq_wake %d ret =%d\n",
			 __func__, client->irq, ret);
		goto disable_irq_wake;
	}

	ret = request_threaded_irq(client->irq, NULL,
				   smb136_valid_handler,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   "smb136_charger_valid", smb136_chg);
	if (ret) {
		dev_err(&client->dev,
			"%s request_threaded_irq failed for %d ret =%d\n",
			__func__, client->irq, ret);
		goto disable_irq_wake;
	
	} else {
		disable_irq_nosync(client->irq);
	}
	
	ret = gpio_get_value_cansleep(smb136_chg->valid_n_gpio);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s gpio_get_value failed for %d ret=%d\n", __func__,
			pdata->valid_n_gpio, ret);
		
		ret = 1;
	}
	if (!ret) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_INSERTED_EVENT--\n", __func__);
		msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
		smb136_chg->usb_status = SMB136_PRESENT;
	}
#endif

	if(smb136_chg->chg_en_gpio) {
		ret = gpio_request(smb136_chg->chg_en_gpio, "smb136_charger_enable");
		if (ret) {
			dev_err(&client->dev, "%s gpio_request failed for %d ret=%d\n",
				__func__, smb136_chg->chg_en_gpio, ret);
		#if (defined CONFIG_EXTERNAL_CHARGER)
			goto free_smb136_chg;
		#else
			goto disable_irq_wake;
		#endif
		}
	}
	
	gpio_direction_output(smb136_chg->chg_en_gpio, 1);
	

	if(smb136_chg->chg_stat_gpio) {
		ret = gpio_request(smb136_chg->chg_stat_gpio, "smb136_charger_stat");
		if (ret) {
			dev_err(&client->dev, "%s gpio_request failed for %d ret=%d\n",
				__func__, smb136_chg->chg_stat_gpio, ret);
			goto free_en_gpio;
		}
	}
	gpio_direction_input(smb136_chg->chg_stat_gpio);

	chg_fault_int = gpio_to_irq(smb136_chg->chg_stat_gpio);
	ret = request_threaded_irq(chg_fault_int, NULL,
				   smb136_charger_fault_handler,
				   IRQF_TRIGGER_FALLING,
				   "smb136_charger_stat", smb136_chg);
	if (ret) {
		dev_err(&client->dev,
			"%s request_threaded_irq failed for %d ret =%d\n",
			__func__, chg_fault_int, ret);
		goto free_stat_gpio;
	
	} else {
		disable_irq_nosync(smb136_chg->chg_stat_gpio);
	}
	

	ret = irq_set_irq_wake(chg_fault_int, 1);
	if (ret) {
		dev_err(&client->dev, "%s failed for irq_set_irq_wake %d ret =%d\n",
			 __func__, chg_fault_int, ret);
		goto free_stat_gpio;
	}


	if(smb136_chg->ext_ovp_en_gpio) {
		ret = gpio_request(smb136_chg->ext_ovp_en_gpio, "external_ovp_enable");
		if (ret) {
			dev_err(&client->dev, "%s gpio_request failed for %d ret=%d\n",
				__func__, smb136_chg->ext_ovp_en_gpio, ret);
			goto disable_fault_int_wake;;
		}
	}
	gpio_direction_output(smb136_chg->ext_ovp_en_gpio, 0);

	if(smb136_chg->ovp_flag_n_gpio) {
		ret = gpio_request(smb136_chg->ovp_flag_n_gpio, "ovp_falut_int");
		if (ret) {
			dev_err(&client->dev, "%s gpio_request failed for %d ret=%d\n",
				__func__, smb136_chg->ovp_flag_n_gpio, ret);
			goto free_ext_ovp_en_gpio;
		}
	}
	gpio_direction_input(smb136_chg->ovp_flag_n_gpio);

	ovp_fault_int = gpio_to_irq(smb136_chg->ovp_flag_n_gpio);
	ret = request_threaded_irq(ovp_fault_int, NULL,
				   ovp_fault_handler,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   "ovp_flag_int", smb136_chg);
	if (ret) {
		dev_err(&client->dev,
			"%s request_threaded_irq failed for %d ret =%d\n",
			__func__, ovp_fault_int, ret);
		goto free_ovp_flags_gpio;
	
	} else {
		disable_irq_nosync(smb136_chg->ovp_flag_n_gpio);
	}
	

	ret = irq_set_irq_wake(ovp_fault_int, 1);
	if (ret) {
		dev_err(&client->dev, "%s failed for irq_set_irq_wake %d ret =%d\n",
			 __func__, ovp_fault_int, ret);
		goto free_ovp_flags_gpio;
	}


	
	smb136_register_init(client, 1, 0);
	

	smb136_charger_register_log(client, 1);

	ret = smb136_read_reg(smb136_chg->client, DEV_ID_REG,
			&smb136_chg->dev_id_reg);

	ret = device_create_file(&smb136_chg->client->dev, &dev_attr_id_reg);

	
	smb136_chg->min_design = 3200;
	smb136_chg->max_design = 4200;

	smb136_chg->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	smb136_chg->batt_chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;

	smb136_chg->batt_health = POWER_SUPPLY_HEALTH_GOOD;

	smb136_chg->chg_work_period = SMB136_CHG_DETECT;

	smb136_chg->vchg_switch = 0;

	device_init_wakeup(&client->dev, 1);

	mutex_lock(&init_lock);
	usb_smb136_chg = smb136_chg;
	if (init_otg_power)
		__smb136_otg_power(init_otg_power);
	mutex_unlock(&init_lock);


#ifdef CONFIG_EXTERNAL_CHARGER
	msm_otg_chg_connected_register(&smb136_chg->otg_chg_notify);
#endif


	ret = msm_charger_register(&smb136_chg->adapter_hw_chg);
	if (ret) {
		dev_err(&client->dev, "%s msm_charger_register failed for ret=%d\n",
			__func__, ret);
#ifdef CONFIG_EXTERNAL_CHARGER
		goto unregister_otg_chg;
#endif
	}

	smb136_create_debugfs_entries(smb136_chg);

	
	if (valid_charger == 1) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_INSERTED_EVENT--\n", __func__);
		msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
		smb136_chg->usb_status = SMB136_PRESENT;
	}
	
	enable_irq(smb136_chg->chg_stat_gpio);
	enable_irq(smb136_chg->ovp_flag_n_gpio);
	

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s OK device_id=%x, usb_status=%d\n",
		__func__, smb136_chg->dev_id_reg, smb136_chg->usb_status);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s probe success!\n", __func__);

	return 0;



#ifdef CONFIG_EXTERNAL_CHARGER
unregister_otg_chg:
	msm_otg_chg_connected_unregister(&smb136_chg->otg_chg_notify);
#endif

free_ovp_flags_gpio:
	gpio_free(pdata->ovp_flag_n_gpio);
free_ext_ovp_en_gpio:
	gpio_free(pdata->ext_ovp_en_gpio);
disable_fault_int_wake:
	irq_set_irq_wake(chg_fault_int, 0);
free_stat_gpio:
	gpio_free(pdata->chg_stat_gpio);
free_en_gpio:
	gpio_free(pdata->chg_en_gpio);
#ifndef CONFIG_EXTERNAL_CHARGER
disable_irq_wake:
	irq_set_irq_wake(client->irq, 0);
free_valid_gpio:
	gpio_free(pdata->valid_n_gpio);
#endif
free_smb136_chg:
	kfree(smb136_chg);
out:
	return ret;
}


void smb136_otg_power(int on)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: Enter on=%d--\n", __func__, on);

	mutex_lock(&init_lock);
	if (!usb_smb136_chg) {
		init_otg_power = !!on;
		pr_warning("%s called when not initialized\n", __func__);
		mutex_unlock(&init_lock);
		return;
	}
	__smb136_otg_power(on);
	mutex_unlock(&init_lock);
}

static int __devexit smb136_remove(struct i2c_client *client)
{
	const struct smb136_platform_data *pdata;
	struct smb136_data *smb136_chg = i2c_get_clientdata(client);

	pdata = client->dev.platform_data;
	device_init_wakeup(&client->dev, 0);
	irq_set_irq_wake(client->irq, 0);
	free_irq(client->irq, client);
#ifndef CONFIG_EXTERNAL_CHARGER
	gpio_free(pdata->valid_n_gpio);
#endif
	gpio_free(pdata->chg_en_gpio);
	gpio_free(pdata->chg_stat_gpio);

	cancel_delayed_work_sync(&smb136_chg->charge_work);

	smb136_destroy_debugfs_entries();

	msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_REMOVED_EVENT--\n", __func__);
	msm_charger_notify_event(&smb136_chg->adapter_hw_chg, CHG_REMOVED_EVENT);

#ifdef CONFIG_EXTERNAL_CHARGER
	msm_otg_chg_connected_unregister(&smb136_chg->otg_chg_notify);
#endif


	msm_charger_unregister(&smb136_chg->adapter_hw_chg);

	kfree(smb136_chg);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s remove success!\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int smb136_suspend(struct device *dev)
{
	struct smb136_data *smb136_chg = dev_get_drvdata(dev);

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (smb136_chg->charging) {
		msm_chg_printk(DBG_MSM_CHG_INFO, "--%s--\n", __func__);
		return -EBUSY;
	}
	return 0;
}

static int smb136_resume(struct device *dev)
{


	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);
	return 0;
}

static const struct dev_pm_ops smb136_pm_ops = {
	.suspend = smb136_suspend,
	.resume = smb136_resume,
};
#endif

static const struct i2c_device_id smb136_id[] = {
	{SMB136_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb136_id);

static struct i2c_driver smb136_driver = {
	.driver = {
		   .name = SMB136_DRIVER_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM
		   .pm = &smb136_pm_ops,
#endif
	},
	.probe = smb136_probe,
	.remove = __devexit_p(smb136_remove),
	.id_table = smb136_id,
};

static int __init smb136_init(void)
{
	int ret;

	ret = i2c_add_driver(&smb136_driver);
	if (ret)
		printk(KERN_ERR "Unable to register driver ret = %d\n", ret);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s initializing success!\n", __func__);

	return ret;

}

device_initcall_sync(smb136_init);

static void __exit smb136_exit(void)
{
	i2c_del_driver(&smb136_driver);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s exit success!\n", __func__);
}
module_exit(smb136_exit);

MODULE_AUTHOR("Abhijeet Dharmapurikar <adharmap@codeaurora.org>");
MODULE_DESCRIPTION("Driver for SMB136 Charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb136");
