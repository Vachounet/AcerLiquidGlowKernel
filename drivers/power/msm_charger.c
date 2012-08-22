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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mfd/pmic8058.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/msm-charger.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <asm/atomic.h>

#include <mach/msm_hsusb.h>

#include <linux/i2c/bq27520.h>
#include <linux/i2c/smb136.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/seq_file.h> 
#include <asm/uaccess.h>
#include <mach/chicago_hwid.h> 
#include <linux/reboot.h>	
extern int QnooffchargeVariable;

extern struct dentry *kernel_debuglevel_dir;
union MSM_CHG_DEBUGLEVEL MSM_CHG_DEBUG_DLL;
uint MSM_GAUGE_LOG;

#define DEFAULT_BATT_MAX_V		4200
#define DEFAULT_BATT_MIN_V		3200

#define NOCHG_BATT_MAX_V		4250
#define CHG_BATT_MAX_V			4350

#define MSM_CHARGER_GAUGE_MISSING_VOLTS 3500
#define MSM_CHARGER_GAUGE_MISSING_TEMP  35

#define MSM_CHARGER_GAUGE_MISSING_CURRS  	100
#define MSM_CHARGER_GAUGE_MISSING_CAPACITY	15
#define MSM_CHARGER_GAUGE_MISSING_TIME_TO_EMPTY_NOW	5
#define MSM_CHARGER_GAUGE_MISSING_TIME_TO_EMPTY_AVG	5
#define MSM_CHARGER_GAUGE_MISSING_TIME_TO_FULL_NOW	5


/**
 * enum msm_battery_status
 * @BATT_STATUS_ABSENT: battery not present
 * @BATT_STATUS_ID_INVALID: battery present but the id is invalid
 * @BATT_STATUS_DISCHARGING: battery is present and is discharging
 * @BATT_STATUS_TRKL_CHARGING: battery is being trickle charged
 * @BATT_STATUS_FAST_CHARGING: battery is being fast charged
 * @BATT_STATUS_JUST_FINISHED_CHARGING: just finished charging,
 *		battery is fully charged. Do not begin charging untill the
 *		voltage falls below a threshold to avoid overcharging
 * @BATT_STATUS_TEMPERATURE_OUT_OF_RANGE: battery present,
					no charging, temp is hot/cold
 */
enum msm_battery_status {
	BATT_STATUS_ABSENT,						
	BATT_STATUS_ID_INVALID,					
	BATT_STATUS_DISCHARGING,				
	BATT_STATUS_TRKL_CHARGING,				
	BATT_STATUS_FAST_CHARGING,				
	BATT_STATUS_JUST_FINISHED_CHARGING,		
	BATT_STATUS_TEMPERATURE_OUT_OF_RANGE,	

	BATT_STATUS_TIME_OUT,					
	BATT_STATUS_VOLTAGE_OUT_OF_RANGE,		
	BATT_STATUS_OVP_FAULT,					


	BATT_STATUS_INVALID_CHARGING,			

};


char *msm_batt_status_string[] = {
		"ABSENT",
		"ID_INVALID",
		"DISCHARGING",
		"TRKL_CHARGING",
		"FAST_CHARGING",
		"JUST_FINISHED_CHARGING",
		"TEMP_OUT_OF_RANGE",
		"TIME_OUT",
		"VOLT_OUT_OF_RANGE",
		"OVP_FAULT",
		"INVALID_CHARGING",
};


struct msm_hardware_charger_priv {
	struct list_head list;
	struct msm_hardware_charger *hw_chg;
	enum msm_hardware_charger_state hw_chg_state;
	unsigned int max_source_current;

	struct power_supply ac_psy;
	struct power_supply usb_psy;

};

struct msm_charger_event {
	enum msm_hardware_charger_event event;
	struct msm_hardware_charger *hw_chg;
};

struct msm_charger_mux {
	int inited;

	int bat_inited;
	int chg_inited;


	int chg_present;

	struct list_head msm_hardware_chargers;
	int count_chargers;
	struct mutex msm_hardware_chargers_lock;

	struct device *dev;

	unsigned int max_voltage;
	unsigned int min_voltage;

	unsigned int safety_time;
	struct delayed_work teoc_work;

	unsigned int update_time;
	int stop_update;
	struct delayed_work update_heartbeat_work;

	struct mutex status_lock;
	enum msm_battery_status batt_status;
	struct msm_hardware_charger_priv *current_chg_priv;
	struct msm_hardware_charger_priv *current_mon_priv;

	unsigned int (*get_batt_capacity_percent) (void);

	struct msm_charger_event *queue;
	int tail;
	int head;
	spinlock_t queue_lock;
	int queue_count;
	struct work_struct queue_work;
	struct workqueue_struct *event_wq_thread;
	struct wake_lock wl;
};

static struct msm_charger_mux msm_chg;

static struct msm_battery_gauge *msm_batt_gauge;


static struct msm_hardware_charger *msm_hw_chg;


static bool power_supply_prop_test = 0;
struct fake_power_supply_info {
	int fake_present;
	int fake_status;
	int fake_health;
	int fake_capacity;
	int fake_temp;
	int fake_voltage;
	int fake_curr;
	int fake_tte;
	int fake_ttf;
	int fake_ttecp;
};
static struct fake_power_supply_info fake_batt;

static int is_chg_capable_of_charging(struct msm_hardware_charger_priv *priv)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (priv->hw_chg_state == CHG_READY_STATE
	    || priv->hw_chg_state == CHG_CHARGING_STATE)
		return 1;

	return 0;
}

static int is_batt_status_capable_of_charging(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);


	if (msm_chg.batt_status == BATT_STATUS_ABSENT
	    || msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE
		|| msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE
		|| msm_chg.batt_status == BATT_STATUS_OVP_FAULT
		|| msm_chg.batt_status == BATT_STATUS_TIME_OUT
	    || msm_chg.batt_status == BATT_STATUS_ID_INVALID
	    || msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING
	    || msm_chg.batt_status == BATT_STATUS_INVALID_CHARGING)
		return 0;
	return 1;

}

static int is_batt_status_charging(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING
	    || msm_chg.batt_status == BATT_STATUS_FAST_CHARGING)
		return 1;
	return 0;
}

static int is_battery_present(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (msm_batt_gauge && msm_batt_gauge->is_battery_present)
		return msm_batt_gauge->is_battery_present();
	else {
		pr_err("msm-charger: no batt gauge batt=absent\n");
		return 0;
	}
}

static int is_battery_temp_within_range(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (msm_batt_gauge && msm_batt_gauge->is_battery_temp_within_range)
		return msm_batt_gauge->is_battery_temp_within_range();
	else {
		pr_err("msm-charger no batt gauge batt=out_of_temperatur\n");
		return 0;
	}
}

static int is_battery_id_valid(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (msm_batt_gauge && msm_batt_gauge->is_battery_id_valid)
		return msm_batt_gauge->is_battery_id_valid();
	else {
		pr_err("msm-charger no batt gauge batt=id_invalid\n");
		return 0;
	}
}


static int msm_get_charger_state(void)
{
	int ret = -1;

	if (msm_hw_chg && msm_hw_chg->get_charger_state)
		ret = msm_hw_chg->get_charger_state();

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s(%d)--\n", __func__, ret);

	return ret;
}



static int msm_get_charger_register_status(int reg, u8 value)
{
	int ret = -1;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (msm_hw_chg && msm_hw_chg->get_charger_register_status)
		ret = msm_hw_chg->get_charger_register_status(reg, value);

	return ret;
}


static int get_prop_batt_mvolts(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_battery_mvolts)
		return msm_batt_gauge->get_battery_mvolts();
	else {
		pr_err("msm-charger no batt gauge assuming %d mV\n",
			MSM_CHARGER_GAUGE_MISSING_VOLTS);
		return MSM_CHARGER_GAUGE_MISSING_VOLTS;
	}
}


static int get_prop_batt_temperature(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_battery_temperature)
		return msm_batt_gauge->get_battery_temperature();
	else {
		pr_err("msm-charger no batt gauge assuming %d deg C\n",
			MSM_CHARGER_GAUGE_MISSING_TEMP);
		return MSM_CHARGER_GAUGE_MISSING_TEMP * 10;
	}
}


static int get_prop_batt_capacity(void)
{
	int ret;

	if (msm_batt_gauge && msm_batt_gauge->get_batt_remaining_capacity) {
		ret = msm_batt_gauge->get_batt_remaining_capacity();
	} else {
		pr_err("msm-charger no batt gauge assuming %d%%\n",
			MSM_CHARGER_GAUGE_MISSING_CAPACITY);
		ret = MSM_CHARGER_GAUGE_MISSING_CAPACITY;
	}

	
	if (ret == 0) {
		if (system_rev <= CHICAGO_EVB2 || get_prop_batt_mvolts() >= 3750) {
			ret = 1;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: Assign capacity to 1 to avoid system shutdown--\n",
				__func__);
		}
	}

	return ret;
	
}


static int get_prop_batt_curr(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_average_current)
		return msm_batt_gauge->get_average_current();
	else {
		pr_err("msm-charger no batt gauge assuming %dmA\n",
			MSM_CHARGER_GAUGE_MISSING_CURRS);
		return MSM_CHARGER_GAUGE_MISSING_CURRS;
	}
}

static int get_prop_batt_time_to_empty_now(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_time_to_empty_now)
		return msm_batt_gauge->get_time_to_empty_now();
	else {
		pr_err("msm-charger no batt gauge assuming %d mins\n",
			MSM_CHARGER_GAUGE_MISSING_TIME_TO_EMPTY_NOW);
		return MSM_CHARGER_GAUGE_MISSING_TIME_TO_EMPTY_NOW;
	}
}

static int get_prop_batt_time_to_empty_avg(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_time_to_empty_avg)
		return msm_batt_gauge->get_time_to_empty_avg();
	else {
		pr_err("msm-charger no batt gauge assuming %d mins\n",
			MSM_CHARGER_GAUGE_MISSING_TIME_TO_EMPTY_AVG);
		return MSM_CHARGER_GAUGE_MISSING_TIME_TO_EMPTY_AVG;
	}
}

static int get_prop_batt_time_to_full_now(void)
{
	if (msm_batt_gauge && msm_batt_gauge->get_time_to_full_now)
		return msm_batt_gauge->get_time_to_full_now();
	else {
		pr_err("msm-charger no batt gauge assuming %d mins\n",
			MSM_CHARGER_GAUGE_MISSING_TIME_TO_FULL_NOW);
		return MSM_CHARGER_GAUGE_MISSING_TIME_TO_FULL_NOW;
	}
}


static void updating_prop_batt_info(int polling_time)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: polling_time=%d--\n", __func__, polling_time);

	cancel_delayed_work(&msm_chg.update_heartbeat_work);

	msm_chg.update_time = polling_time;

	if (msm_batt_gauge && msm_batt_gauge->updating_batt_info)
		msm_batt_gauge->updating_batt_info(polling_time);

	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));
}



static int get_prop_batt_health(void)
{

	static int status = 0;
	int temp = 0;

	
	if (msm_chg.batt_status == BATT_STATUS_ABSENT) {
		temp = get_prop_batt_mvolts();
		if (temp > BATT_INVALID_VOLT_THRESHOLD) {
			status = POWER_SUPPLY_HEALTH_DEAD;
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_HEALTH_DEAD--\n",
				__func__);
		} else {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: battery volt(%d)--\n",
				__func__, temp);
		}
	
	} else if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE) {
		temp = get_prop_batt_temperature();
		if (temp < DEFAULT_BATT_LOW_TEMP) {
			status = POWER_SUPPLY_HEALTH_COLD;
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_HEALTH_COLD(%d)--\n",
				__func__, temp);
		} else if (temp > DEFAULT_BATT_HIGH_TEMP) {
			status = POWER_SUPPLY_HEALTH_OVERHEAT;
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_HEALTH_OVERHEAT(%d)--\n",
				__func__, temp);
		} else {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: Temperature(%d) in Hysteresis--\n",
				__func__, temp);
		}
	} else if (msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
		temp = get_prop_batt_mvolts();
		status = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_HEALTH_OVERVOLTAGE(%d)--\n",
			__func__, temp);
	} else if (msm_chg.batt_status == BATT_STATUS_OVP_FAULT) {
		temp = get_prop_batt_mvolts();
		
		if (temp > CHG_BATT_MAX_V) {
			status = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_HEALTH_OVERVOLTAGE(%d)--\n",
				__func__, temp);
		} else {
			status = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_HEALTH_UNSPEC_FAILURE(%d)--\n",
				__func__, temp);
		}
		
	
	} else if (msm_chg.batt_status == BATT_STATUS_INVALID_CHARGING) {
		status = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_HEALTH_UNSPEC_FAILURE--\n",
				__func__);
	
	} else {
		status = POWER_SUPPLY_HEALTH_GOOD;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_HEALTH_GOOD--\n", __func__);
	}

	return status;

}

static int get_prop_charge_type(void)
{
	int status = 0;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: msm_batt_status is %s--\n",
		__func__, msm_batt_status_string[msm_chg.batt_status]);

	if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING) {
		status = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_TRKL_CHARGING--\n", __func__);
	}
	else if (msm_chg.batt_status == BATT_STATUS_FAST_CHARGING) {
		status = POWER_SUPPLY_CHARGE_TYPE_FAST;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_FAST_CHARGING--\n", __func__);
	}
	else {
		status = POWER_SUPPLY_CHARGE_TYPE_NONE;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_CHARGE_TYPE_NONE--\n", __func__);
	}

	return status;
}

static int get_prop_batt_status(void)
{
	u8 value = 0;
	int status = 0, ret;

	char *batt_status_string[] = {
			"UNKNOWN",
			"CHARGING",
			"DISCHARGING",
			"NOT_CHARGING",
			"FULL"};

	if (msm_batt_gauge && msm_batt_gauge->get_battery_status) {
		status = msm_batt_gauge->get_battery_status();
		#if 0
		if (status == POWER_SUPPLY_STATUS_CHARGING ||
			status == POWER_SUPPLY_STATUS_FULL ||
			status == POWER_SUPPLY_STATUS_DISCHARGING ||
			status == POWER_SUPPLY_STATUS_NOT_CHARGING) {

			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: battery_status is %s--\n",
				__func__, batt_status_string[status]);

			return status;
		}
		#endif
		
		if (msm_chg.batt_status == BATT_STATUS_INVALID_CHARGING) {
			if (msm_get_charger_state() == 1) {
				status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: invalid charger--\n",
					__func__);
			}
		} else {
		
		
			if (status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: recheck battery_status--\n",
					__func__);

				if (msm_get_charger_state() == 1) {
					ret = msm_get_charger_register_status(STATUS_E_REG, value);
					if (ret) {
						msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: read charger reg(%d) failed--\n",
							__func__, STATUS_E_REG);
					} else {
						if (((value & CHARGING_STAT_E) >> 1) >= PRE_CHARGE_STATUS) {
							status = POWER_SUPPLY_STATUS_CHARGING;
						} else {
							status = POWER_SUPPLY_STATUS_NOT_CHARGING;
						}
					}
				} else {
					status = POWER_SUPPLY_STATUS_DISCHARGING;
				}
			} else if (status == POWER_SUPPLY_STATUS_CHARGING)  {
			
				if (msm_get_charger_state() == 0) {
					status = POWER_SUPPLY_STATUS_DISCHARGING;
				}
			}
			
		}

		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: battery_status is %s--\n",
				__func__, batt_status_string[status]);
		return status;
	}

	if (is_batt_status_charging()) {
		status = POWER_SUPPLY_STATUS_CHARGING;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_STATUS_CHARGING--\n", __func__);
	}
	else if (msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING &&
		msm_chg.current_chg_priv != NULL) {
		status = POWER_SUPPLY_STATUS_FULL;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_JUST_FINISHED_CHARGING--\n", __func__);
	}
	else {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: POWER_SUPPLY_STATUS_DISCHARGING--\n", __func__);
	}

	return status;
}


static void update_batt_status(void)
{
	if (is_battery_present()) {
		if (is_battery_id_valid()) {
			
			if (is_battery_temp_within_range()) {
				
				if(get_prop_batt_mvolts() < NOCHG_BATT_MAX_V) {
					if (msm_chg.batt_status == BATT_STATUS_ABSENT
						|| msm_chg.batt_status == BATT_STATUS_ID_INVALID
						|| msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE
						|| msm_chg.batt_status == BATT_STATUS_TIME_OUT
						|| msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE
						|| msm_chg.batt_status == BATT_STATUS_INVALID_CHARGING) {

						if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE) {
							msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_TEMP_INRANGE--\n", __func__);
							msm_charger_notify_event(NULL, CHG_BATT_TEMP_INRANGE);
						} else if (msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
							msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_VOLTAGE_INRANGE--\n", __func__);
							msm_charger_notify_event(NULL, CHG_BATT_VOLT_INRANGE);
						} else if (msm_chg.batt_status == BATT_STATUS_INVALID_CHARGING) {
							if (msm_get_charger_state() == 0) {
								msm_chg.batt_status = BATT_STATUS_DISCHARGING;
								msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: BATT_STATUS_DISCHARGING--\n", __func__);
								msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);
							}
						} else {
							msm_chg.batt_status = BATT_STATUS_DISCHARGING;
							msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: BATT_STATUS_DISCHARGING--\n", __func__);
							msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);
						}
					} else {
						msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: msm_batt_status: %s--\n",
							__func__, msm_batt_status_string[msm_chg.batt_status]);
					}
				
				} else {
					if (!msm_get_charger_state()) {
						if (msm_chg.batt_status != BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
							msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_VOLTAGE_OUTOFRANGE--\n", __func__);
							msm_charger_notify_event(NULL, CHG_BATT_VOLT_OUTOFRANGE);
						}
					} else {
						if (msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
							msm_charger_notify_event(NULL, CHG_BATT_VOLT_INRANGE);
							msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: CHARGING WITH VOLTAGE_OUTOFRANGE--\n", __func__);
						}
					}
				}
			} else {
				if (msm_chg.batt_status != BATT_STATUS_TEMPERATURE_OUT_OF_RANGE) {
					msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_TEMP_OUTOFRANGE--\n", __func__);
					msm_charger_notify_event(NULL, CHG_BATT_TEMP_OUTOFRANGE);
				}
			
			}
		} else {
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
			msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_ID_INVALID--\n", __func__);
		}
	} else {
		msm_chg.batt_status = BATT_STATUS_ABSENT;
		msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_ABSENT--\n", __func__);
	}
}

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct msm_hardware_charger_priv *priv;
	int ret;


	if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		priv = container_of(psy, struct msm_hardware_charger_priv, ac_psy);
	else if (psy->type == POWER_SUPPLY_TYPE_USB)
		priv = container_of(psy, struct msm_hardware_charger_priv, usb_psy);
	else {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: No such power_supply\n", __func__);
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = !(priv->hw_chg_state == CHG_ABSENT_STATE);
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d: ret=%d--\n", __func__, __LINE__, ret);
		
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (priv->hw_chg->charger_type == USB_CHG_TYPE__CARKIT ||
				priv->hw_chg->charger_type == USB_CHG_TYPE__WALLCHARGER ||
				priv->hw_chg->charger_type == USB_CHG_TYPE__OTHERWALLCHARGER ||
				priv->hw_chg->charger_type == USB_CHG_TYPE__FASTERCHARGER)
				val->intval = ret;
			else
				val->intval = 0;

			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: AC POWER_SUPPLY_PROP_PRESENT=%d--\n",
				__func__, val->intval);
		}
		

		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			if (priv->hw_chg->charger_type == USB_CHG_TYPE__SDP)
				val->intval = ret;
			else
				val->intval = 0;

			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: USB POWER_SUPPLY_PROP_PRESENT=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		ret = (priv->hw_chg_state == CHG_READY_STATE)
			|| (priv->hw_chg_state == CHG_CHARGING_STATE);
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d: ret=%d--\n", __func__, __LINE__, ret);
		
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (priv->hw_chg->charger_type == USB_CHG_TYPE__CARKIT ||
				priv->hw_chg->charger_type == USB_CHG_TYPE__WALLCHARGER ||
				priv->hw_chg->charger_type == USB_CHG_TYPE__OTHERWALLCHARGER ||
				priv->hw_chg->charger_type == USB_CHG_TYPE__FASTERCHARGER)
				val->intval = ret;
			else
				val->intval = 0;

			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: AC POWER_SUPPLY_PROP_ONLINE=%d--\n",
				__func__, val->intval);
		}
		

		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			if (priv->hw_chg->charger_type == USB_CHG_TYPE__SDP)
				val->intval = ret;
			else
				val->intval = 0;

			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: USB POWER_SUPPLY_PROP_ONLINE=%d--\n",
				__func__, val->intval);
		}
		break;


	default:
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: default!!--\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	
};

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (power_supply_prop_test && fake_batt.fake_status != -1) {
			val->intval = fake_batt.fake_status;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_STATUS=%d--\n",
				__func__, val->intval);
		} else {
			val->intval = get_prop_batt_status();
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_STATUS=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type();
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_CHARGE_TYPE=%d--\n",
			__func__, val->intval);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (power_supply_prop_test && fake_batt.fake_health != -1) {
			val->intval = fake_batt.fake_health;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_HEALTH=%d--\n",
				__func__, val->intval);
		} else {
		val->intval = get_prop_batt_health();
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_HEALTH=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		if (power_supply_prop_test && fake_batt.fake_present != -1) {
			val->intval = fake_batt.fake_present;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_PRESENT=%d--\n",
				__func__, val->intval);
		} else {
		val->intval = !(msm_chg.batt_status == BATT_STATUS_ABSENT);
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_PRESENT=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_TECHNOLOGY=%d--\n",
			__func__, val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_chg.max_voltage * 1000;
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN=%d--\n",
			__func__, val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_chg.min_voltage * 1000;
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN=%d--\n",
			__func__, val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (power_supply_prop_test && fake_batt.fake_voltage != -1) {
			val->intval = fake_batt.fake_voltage;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_VOLTAGE_NOW=%d--\n",
				__func__, val->intval);
		} else {
			
			val->intval = get_prop_batt_mvolts() * 1000;
			
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_VOLTAGE_NOW=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (power_supply_prop_test && fake_batt.fake_capacity != -1) {
			val->intval = fake_batt.fake_capacity;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_CAPACITY=%d--\n",
				__func__, val->intval);
		} else {
			val->intval = get_prop_batt_capacity();
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_CAPACITY=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (power_supply_prop_test && fake_batt.fake_curr != -1) {
			val->intval = fake_batt.fake_curr;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_CURRENT_NOW=%d--\n",
				__func__, val->intval);
		} else {
			val->intval = get_prop_batt_curr();
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_CURRENT_NOW=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_TEMP:
		if (power_supply_prop_test && fake_batt.fake_temp != -1) {
			val->intval = fake_batt.fake_temp;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_TEMP=%d--\n",
				__func__, val->intval);
		} else {
			val->intval = get_prop_batt_temperature();
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_TEMP=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		if (power_supply_prop_test && fake_batt.fake_tte != -1) {
			val->intval = fake_batt.fake_tte;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW=%d--\n",
				__func__, val->intval);
		} else {
			val->intval = get_prop_batt_time_to_empty_now();
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		if (power_supply_prop_test && fake_batt.fake_ttecp != -1) {
			val->intval = fake_batt.fake_ttecp;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG=%d--\n",
				__func__, val->intval);
		} else {
			val->intval = get_prop_batt_time_to_empty_avg();
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG=%d--\n",
				__func__, val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		if (power_supply_prop_test && fake_batt.fake_ttf != -1) {
			val->intval = fake_batt.fake_ttf;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: FAKE-POWER_SUPPLY_PROP_TIME_TO_FULL_NOW=%d--\n",
				__func__, val->intval);
		} else {
			val->intval = get_prop_batt_time_to_full_now();
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: POWER_SUPPLY_PROP_TIME_TO_FULL_NOW=%d--\n",
				__func__, val->intval);
		}


	default:
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: default!!--\n", __func__);
		return -EINVAL;
	}

	return 0;
}


static struct power_supply msm_psy_batt = {
	.name = "Battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
};


static int usb_chg_current;
static struct msm_hardware_charger_priv *usb_hw_chg_priv;
#ifndef CONFIG_EXTERNAL_CHARGER
static void (*notify_vbus_state_func_ptr)(int);
static int usb_notified_of_insertion;

/* this is passed to the hsusb via platform_data msm_otg_pdata */
int msm_charger_register_vbus_sn(void (*callback)(int))
{
	notify_vbus_state_func_ptr = callback;
	return 0;
}

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void msm_charger_unregister_vbus_sn(void (*callback)(int))
{
	notify_vbus_state_func_ptr = NULL;
}

static void notify_usb_of_the_plugin_event(struct msm_hardware_charger_priv
					   *hw_chg, int plugin)
{
	plugin = !!plugin;
	if (plugin == 1 && usb_notified_of_insertion == 0) {
		usb_notified_of_insertion = 1;
		if (notify_vbus_state_func_ptr)
			(*notify_vbus_state_func_ptr) (plugin);

		usb_hw_chg_priv = hw_chg;
	}
	if (plugin == 0 && usb_notified_of_insertion == 1) {
		if (notify_vbus_state_func_ptr)
			(*notify_vbus_state_func_ptr) (plugin);

		usb_notified_of_insertion = 0;
		usb_hw_chg_priv = NULL;
	}
}
#endif


unsigned int msm_chg_get_batt_capacity_percent(void)

{

#ifdef CONFIG_BATTERY_BQ27520
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	return get_prop_batt_capacity();
#else
	unsigned int current_voltage = get_prop_batt_mvolts();
	unsigned int low_voltage = msm_chg.min_voltage;
	unsigned int high_voltage = msm_chg.max_voltage;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
		    / (high_voltage - low_voltage);
#endif

}


#ifdef CONFIG_DEBUG_FS
static struct dentry *dent;


static int msm_chg_id_get(void *data, u64 *value)
{


	*value = 0;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(msm_chg_id_fops, msm_chg_id_get, NULL, "%llu\n");

static int power_supply_test_get(void *data, u64 *value)
{

	*value = power_supply_prop_test;

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: get power_supply_prop_test=%d\n",
		__func__, power_supply_prop_test);

	return 0;
}

static int power_supply_test_set(void *data, u64 value)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set power_supply_prop_test as %llu\n",
		__func__, value);

	if (value == 0) {
		power_supply_prop_test = 0;
	} else if (value == 1) {
		power_supply_prop_test = 1;
		memset(&fake_batt, -1, sizeof(fake_batt));
	} else {
		dev_err(msm_chg.dev, "%s: unsupport value for power_supply_test!!\n",
			__func__);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(msm_power_supply_test_fops, power_supply_test_get, power_supply_test_set, "%llu\n");

static int fake_present_get(void *data, u64 *value)
{
	if (power_supply_prop_test == 1) {
		*value = fake_batt.fake_present;
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: fake_present=%d--\n",
			__func__, fake_batt.fake_present);
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}

static int fake_present_set(void *data, u64 value)
{
	if (power_supply_prop_test == 1) {
		if (value > 1) {
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: unsupport value(%llu) for present--\n",
				__func__, value);
		} else {
			fake_batt.fake_present = value;
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set present as %d--\n",
				__func__, fake_batt.fake_present);
		}
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fake_present_fops, fake_present_get, fake_present_set, "%llu\n");

static int fake_status_get(void *data, u64 *value)
{
	if (power_supply_prop_test == 1) {
		*value = fake_batt.fake_status;
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: fake_status=%d--\n",
			__func__, fake_batt.fake_status);
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}

static int fake_status_set(void *data, u64 value)
{
	if (power_supply_prop_test == 1) {
		if (value > POWER_SUPPLY_STATUS_FULL) {
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: unsupport value(%llu) for status--\n",
				__func__, value);
		} else {
			fake_batt.fake_status = value;
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set status as %d--\n",
				__func__, fake_batt.fake_status);
		}
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fake_status_fops, fake_status_get, fake_status_set, "%llu\n");

static int fake_health_get(void *data, u64 *value)
{
	if (power_supply_prop_test == 1) {
		*value = fake_batt.fake_health;
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: fake_health=%d--\n",
			__func__, fake_batt.fake_health);
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}

static int fake_health_set(void *data, u64 value)
{
	if (power_supply_prop_test == 1) {
		if (value > POWER_SUPPLY_HEALTH_COLD) {
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: unsupport value(%llu) for health--\n",
				__func__, value);
		} else {
			fake_batt.fake_health = value;
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set health as %d--\n",
				__func__, fake_batt.fake_health);
		}
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fake_health_fops, fake_health_get, fake_health_set, "%llu\n");

static int fake_capacity_get(void *data, u64 *value)
{
	if (power_supply_prop_test == 1) {
		*value = fake_batt.fake_capacity;
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: fake_capacity=%d--\n",
			__func__, fake_batt.fake_capacity);
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}

static int fake_capacity_set(void *data, u64 value)
{
	if (power_supply_prop_test == 1) {
		if (value > 100) {
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: unsupport value(%llu) for capacity--\n",
				__func__, value);
		} else {
			fake_batt.fake_capacity = value;
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set capacity as %d--\n",
				__func__, fake_batt.fake_capacity);
		}
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fake_capacity_fops, fake_capacity_get, fake_capacity_set, "%llu\n");

static int fake_vlot_get(void *data, u64 *value)
{
	if (power_supply_prop_test == 1) {
		*value = fake_batt.fake_voltage;
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: fake_voltage=%d--\n",
			__func__, fake_batt.fake_voltage);
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}

static int fake_volt_set(void *data, u64 value)
{
	if (power_supply_prop_test == 1) {
		if (value > DEFAULT_BATT_MAX_V) {
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: unsupport value(%llu) for voltage--\n",
				__func__, value);
		} else {
			fake_batt.fake_voltage = value;
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set voltage as %d--\n",
				__func__, fake_batt.fake_voltage);
		}
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fake_volt_fops, fake_vlot_get, fake_volt_set, "%llu\n");

static int fake_curr_get(void *data, u64 *value)
{
	if (power_supply_prop_test == 1) {
		*value = fake_batt.fake_curr;
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: fake_curr=%d--\n",
			__func__, fake_batt.fake_curr);
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}

static int fake_curr_set(void *data, u64 value)
{
	if (power_supply_prop_test == 1) {
		if (value > 2000) {
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: unsupport value(%llu) for current--\n",
				__func__, value);
		} else {
			fake_batt.fake_curr = value;
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set current as %d--\n",
				__func__, fake_batt.fake_curr);
		}
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fake_curr_fops, fake_curr_get, fake_curr_set, "%llu\n");

static int fake_temp_get(void *data, u64 *value)
{
	if (power_supply_prop_test == 1) {
		*value = fake_batt.fake_temp;
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: fake_temp=%d--\n",
			__func__, fake_batt.fake_temp);
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}

static int fake_temp_set(void *data, u64 value)
{
	if (power_supply_prop_test == 1) {
		if (value > 1000) {
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: unsupport value(%llu) for temperature--\n",
				__func__, value);
		} else {
			fake_batt.fake_temp = value;
			msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set temperature as %d--\n",
				__func__, fake_batt.fake_temp);
		}
	} else {
		dev_err(msm_chg.dev, "%s: unsupport state of power_supply_prop_test!!\n",
			__func__);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fake_temp_fops, fake_temp_get, fake_temp_set, "%llu\n");

#define MSM_BATT_THERM_MAX_CELCIUS	450
#define MSM_BATT_THERM_MIN_CELCIUS	0


static int msm_engineer_data_get(struct seq_file *s, void *unused)
{


	int temp = 0, flags = 0;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	
	seq_printf(s, "%d,",get_prop_batt_capacity());
	seq_printf(s, "%d,", get_prop_batt_mvolts());
	seq_printf(s, "%d,", get_prop_batt_curr());
	seq_printf(s, "%d,", get_prop_batt_temperature());
	seq_printf(s, "0x%x,", engineer_gauge_fw_version());

	
	seq_printf(s, "%d,", engineer_gauge_remain_capacity());
	seq_printf(s, "%d,", engineer_gauge_full_charge_capacity());
	flags = engineer_gauge_flags();
	seq_printf(s, "0x%x,", flags);

	temp = engineer_gauge_cntl_status();
	seq_printf(s, "%s,", (temp & BIT(5)) ? "true":"false");

	seq_printf(s, "%s,", (flags & BIT(5)) ? "true":"false");
	seq_printf(s, "%s,", (flags & BIT(4)) ? "true":"false");
	seq_printf(s, "%s,", (flags & BIT(3)) ? "true":"false");
	seq_printf(s, "%s,", (flags & BIT(2)) ? "true":"false");
	seq_printf(s, "%s,", (flags & BIT(9)) ? "true":"false");

	
	
	if(msm_hw_chg){
		if (msm_hw_chg->charger_type == USB_CHG_TYPE__CARKIT ||
			msm_hw_chg->charger_type == USB_CHG_TYPE__WALLCHARGER ||
			msm_hw_chg->charger_type == USB_CHG_TYPE__OTHERWALLCHARGER ||
			msm_hw_chg->charger_type == USB_CHG_TYPE__FASTERCHARGER) {
				seq_printf(s, "%s,", "detected");
				seq_printf(s, "%s,", "no detected");
		} else if (msm_hw_chg->charger_type == USB_CHG_TYPE__SDP) {
				seq_printf(s, "%s,", "no detected");
				seq_printf(s, "%s,", "detected");
		} else {
			seq_printf(s, "%s,", "no detected");
			seq_printf(s, "%s,", "no detected");
		}
	} else {
		seq_printf(s, "%s,", "NULL");
		seq_printf(s, "%s,", "NULL");
	}
	

	
	temp = get_prop_batt_temperature();
	seq_printf(s, "%s,", (temp > MSM_BATT_THERM_MAX_CELCIUS) ? "too high":"not too high");
	seq_printf(s, "%s,", (temp < MSM_BATT_THERM_MIN_CELCIUS) ? "too low":"not too low");

	temp = is_battery_temp_within_range(); 
		seq_printf(s, "%s,", temp ? "good":"over heat");

	temp = is_batt_status_charging();
	seq_printf(s, "%s,", temp ? "yes":"no");

	seq_printf(s, "%s,", "NULL"); 
	seq_printf(s, "%s,", "NULL"); 
	seq_printf(s, "%s,", "NULL"); 
	seq_printf(s, "%s,", "NULL"); 
	seq_printf(s, "%s,", "NULL"); 
	seq_printf(s, "%s,", "NULL"); 
	seq_printf(s, "%s,", "NULL"); 


	temp = msm_hw_chg->ovp_fault_gpio_state();
	seq_printf(s, "%s,", temp ? "Normal":"Abnormal");


	seq_printf(s, "\n"); 

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Finish engineer updated!!\n", __func__);

	return 0;
}

static int msm_engineer_mode_get_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_engineer_data_get, NULL);
}

static const struct file_operations msm_engineer_mode_operations = {
	.open		= msm_engineer_mode_get_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};



#ifdef BQ275250_DFI_SUPPORT
static int msm_update_gauge_dfi(struct seq_file *s, void *data)
{


	msm_chg_printk(DBG_MSM_CHG_DFI, "--%s--\n", __func__);

	return 0;
}

static int msm_update_gauge_dfi_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_update_gauge_dfi, NULL);
}

static ssize_t msm_update_gauge_dfi_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int ret, i;
	char *buf = (char *) __get_free_page(GFP_USER);

	msm_chg_printk(DBG_MSM_CHG_DFI, "--%s--\n", __func__);

	if (!buf)
		return -ENOMEM;

	ret = -EINVAL;
	if (count >= PAGE_SIZE)
		goto out;

	ret = -EFAULT;
	if (copy_from_user(buf, buffer, count))
		goto out;

	buf[count] = '\0';
	
	cancel_delayed_work(&msm_chg.update_heartbeat_work);
	
	for (i = 0; i < count; i++) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%02x ", buf[i]);
		if (((i + 1) % 16) == 0)
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "\n");
	}

	if (msm_batt_gauge && msm_batt_gauge->copy_dfi_from_user_to_kernel) {
		ret = msm_batt_gauge->copy_dfi_from_user_to_kernel((const char *)buf, count);
		if (ret)
			ret = count;
		else {
			ret = -EFAULT;
			dev_err(msm_chg.dev, "%s: copy dfi from user to kernel failed\n", __func__);
		}
	}

	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (4000))); 

 out:
	free_page((unsigned long)buf);
	return ret;
}

static const struct file_operations msm_update_gauge_dfi_operations = {
	.open		= msm_update_gauge_dfi_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= msm_update_gauge_dfi_write,
};
#endif

static void msm_chg_create_debugfs_entries(struct msm_charger_mux *msm_chg)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	dent = debugfs_create_dir(MSM_CHG_DRIVER_NAME, NULL);
	if (dent) {
		debugfs_create_file("msm_chg", S_IRUGO | S_IWUGO, dent, msm_chg, &msm_chg_id_fops);
		debugfs_create_file("power_supply_test", S_IRUGO | S_IWUGO, dent, msm_chg, &msm_power_supply_test_fops);
		debugfs_create_file("fake_present", S_IRUGO | S_IWUGO, dent, msm_chg, &fake_present_fops);
		debugfs_create_file("fake_status", S_IRUGO | S_IWUGO, dent, msm_chg, &fake_status_fops);
		debugfs_create_file("fake_health", S_IRUGO | S_IWUGO, dent, msm_chg, &fake_health_fops);
		debugfs_create_file("fake_capacity", S_IRUGO | S_IWUGO, dent, msm_chg, &fake_capacity_fops);
		debugfs_create_file("fake_volt", S_IRUGO | S_IWUGO, dent, msm_chg, &fake_volt_fops);
		debugfs_create_file("fake_curr", S_IRUGO | S_IWUGO, dent, msm_chg, &fake_curr_fops);
		debugfs_create_file("fake_temp", S_IRUGO | S_IWUGO, dent, msm_chg, &fake_temp_fops);

		debugfs_create_file("engineer_mode", S_IRUGO, dent, NULL, &msm_engineer_mode_operations);
#ifdef BQ275250_DFI_SUPPORT
		debugfs_create_file("update_gauge_dfi", S_IWUGO, dent, NULL, &msm_update_gauge_dfi_operations);
#endif
	}
}

static void msm_chg_destroy_debugfs_entries(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (dent)
		debugfs_remove_recursive(dent);
}


static void msm_chg_create_kernel_debuglevel_entries(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	memset(&MSM_CHG_DEBUG_DLL, 0, sizeof(MSM_CHG_DEBUG_DLL));
	MSM_GAUGE_LOG = 0; 

	if (kernel_debuglevel_dir != NULL) {
		debugfs_create_u8("msm_info_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&MSM_CHG_DEBUG_DLL.debug_byte[0]));
		debugfs_create_u8("msm_charger_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&MSM_CHG_DEBUG_DLL.debug_byte[1]));
		debugfs_create_u8("msm_gauge_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&MSM_CHG_DEBUG_DLL.debug_byte[2]));
		debugfs_create_u8("msm_debug_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&MSM_CHG_DEBUG_DLL.debug_byte[3]));
		debugfs_create_u8("msm_gauge_log", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&MSM_GAUGE_LOG));
	} else {
		msm_chg_printk(DBG_MSM_CRITICAL, "msm chg kernel debuglevel dir falied\n");
	}
}


static void msm_chg_destroy_kernel_debuglevel_entries(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}
#else
static void msm_chg_create_debugfs_entries(struct msm_charger_mux *msm_chg)
{
}
static void msm_chg_destroy_debugfs_entries(void)
{
}
static void msm_chg_create_kernel_debuglevel_entries(void)
{
}

static void msm_chg_destroy_kernel_debuglevel_entries(void)
{
}
#endif


static struct msm_hardware_charger_priv *find_best_charger(void)
{
	struct msm_hardware_charger_priv *hw_chg_priv;
	struct msm_hardware_charger_priv *better;
	int rating;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	better = NULL;
	rating = 0;

	list_for_each_entry(hw_chg_priv, &msm_chg.msm_hardware_chargers, list) {
		if (is_chg_capable_of_charging(hw_chg_priv)) {
			if (hw_chg_priv->hw_chg->rating > rating) {
				rating = hw_chg_priv->hw_chg->rating;
				better = hw_chg_priv;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: rating=%d, better=%s--\n",
					__func__, rating, hw_chg_priv->hw_chg->name);
			}

			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d----\n",
				__func__, __LINE__);
		}
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d----\n",
		__func__, __LINE__);

	return better;
}

static int msm_charging_switched(struct msm_hardware_charger_priv *priv)
{
	int ret = 0;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (priv->hw_chg->charging_switched)
		
		ret = priv->hw_chg->charging_switched(priv->hw_chg,
				msm_chg.max_voltage, priv->max_source_current);
		

	return ret;
}

static int msm_stop_charging(struct msm_hardware_charger_priv *priv)
{
	int ret;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	ret = priv->hw_chg->stop_charging(priv->hw_chg);
	if (!ret)
		wake_unlock(&msm_chg.wl);
	return ret;
}

static void msm_enable_system_current(struct msm_hardware_charger_priv *priv)
{
	if (priv->hw_chg->start_system_current) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);
		priv->hw_chg->start_system_current(priv->hw_chg,
					 priv->max_source_current);
	}
}

static void msm_disable_system_current(struct msm_hardware_charger_priv *priv)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (priv->hw_chg->stop_system_current)
		priv->hw_chg->stop_system_current(priv->hw_chg);
}

/* the best charger has been selected -start charging from current_chg_priv */
static int msm_start_charging(void)
{
	int ret;
	struct msm_hardware_charger_priv *priv;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	priv = msm_chg.current_chg_priv;
	wake_lock(&msm_chg.wl);
	ret = priv->hw_chg->start_charging(priv->hw_chg, msm_chg.max_voltage,
					 priv->max_source_current);
	if (ret) {
		wake_unlock(&msm_chg.wl);
		dev_err(msm_chg.dev, "%s couldnt start chg, error = %d\n",
			priv->hw_chg->name, ret);
	} else
		priv->hw_chg_state = CHG_CHARGING_STATE;

	return ret;
}

static void handle_charging_done(struct msm_hardware_charger_priv *priv)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (msm_chg.current_chg_priv == priv) {
		if (msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE)
			if (msm_stop_charging(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
			}

		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;
		msm_chg.batt_status = BATT_STATUS_JUST_FINISHED_CHARGING;
		msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);

		cancel_delayed_work(&msm_chg.teoc_work);

		if (msm_batt_gauge && msm_batt_gauge->monitor_for_recharging)
			msm_batt_gauge->monitor_for_recharging();
		else
			dev_err(msm_chg.dev, "%s: no batt gauge recharge monitor\n", __func__);
	}
}

static void teoc(struct work_struct *work)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: safety timer work expired--\n", __func__);


#if 0
	
	mutex_lock(&msm_chg.status_lock);
	if (msm_chg.current_chg_priv != NULL
	    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
		handle_charging_done(msm_chg.current_chg_priv);
	
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_TIME_OUT--\n", __func__);
		msm_charger_notify_event(msm_chg.current_chg_priv->hw_chg, CHG_BATT_TIME_OUT);
	
	}
	mutex_unlock(&msm_chg.status_lock);
#endif

}

static void handle_battery_inserted(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	/* if a charger is already present start charging */
	if (msm_chg.current_chg_priv != NULL &&
	    is_batt_status_capable_of_charging() &&
	    !is_batt_status_charging()) {
		if (msm_start_charging()) {
			dev_err(msm_chg.dev, "%s couldnt start chg\n",
				msm_chg.current_chg_priv->hw_chg->name);
			return;
		}
		msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;

		queue_delayed_work(msm_chg.event_wq_thread, &msm_chg.teoc_work,
			round_jiffies_relative(msecs_to_jiffies(msm_chg.safety_time)));
	}
}

static void handle_battery_removed(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	
	if (msm_chg.current_chg_priv != NULL
	    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
		if (msm_stop_charging(msm_chg.current_chg_priv)) {
			dev_err(msm_chg.dev, "%s couldnt stop chg\n",
				msm_chg.current_chg_priv->hw_chg->name);
		}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

		cancel_delayed_work(&msm_chg.teoc_work);
	}
}


static void handle_charger_fault(int event)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: event(%d)--\n", __func__, event);

	switch (event) {
		case CHG_PRE_CHG_TIME_OUT:
		case CHG_COMPLETE_CHG_TIME_OUT:
		case CHG_BATT_TIME_OUT:
		case CHG_BATT_TEMP_OUTOFRANGE:
		case CHG_BATT_VOLT_OUTOFRANGE:
		case OVP_FAULT_EVENT:
		case CHG_INVALID_CHARGER:
			
			if (msm_chg.current_chg_priv != NULL
			    && msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
				if (msm_stop_charging(msm_chg.current_chg_priv)) {
					dev_err(msm_chg.dev, "%s couldnt stop chg",
						msm_chg.current_chg_priv->hw_chg->name);
				}
				msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;

				cancel_delayed_work(&msm_chg.teoc_work);
			}
			break;

		default:
			break;
	}
}


static void update_heartbeat(struct work_struct *work)
{
	uint update = 0;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);
	
	
	if (is_battery_present() && !is_battery_temp_within_range()) {
	
		if (msm_chg.batt_status != BATT_STATUS_TEMPERATURE_OUT_OF_RANGE)
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_TEMP_OUTOFRANGE--\n", __func__);
			msm_charger_notify_event(NULL, CHG_BATT_TEMP_OUTOFRANGE);
	} else if (get_prop_batt_mvolts() > NOCHG_BATT_MAX_V) {
		if (!msm_get_charger_state()) {
			if (msm_chg.batt_status != BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_VOLT_OUTOFRANGE--\n", __func__);
				msm_charger_notify_event(NULL, CHG_BATT_VOLT_OUTOFRANGE);
			}
		} else {
			if (msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
				msm_charger_notify_event(NULL, CHG_BATT_VOLT_INRANGE);
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: CHARGING WITH VOLTAGE_OUTOFRANGE--\n", __func__);
			}
		}
	} else {
		if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_TEMP_INRANGE--\n", __func__);
			msm_charger_notify_event(NULL, CHG_BATT_TEMP_INRANGE);
		} else if (msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_VOLT_INRANGE--\n", __func__);
			msm_charger_notify_event(NULL, CHG_BATT_VOLT_INRANGE);
		} else {
	
			if (msm_chg.batt_status == BATT_STATUS_ABSENT
				|| msm_chg.batt_status == BATT_STATUS_ID_INVALID) {
				if (is_battery_present()) {
					if (is_battery_id_valid()) {
						msm_chg.batt_status = BATT_STATUS_DISCHARGING;
						msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: BATT_STATUS_DISCHARGING--\n", __func__);
						handle_battery_inserted();
					}
				}
				update = 1;
			} else {
				if (!is_battery_present()) {
					msm_chg.batt_status = BATT_STATUS_ABSENT;
					msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: BATT_STATUS_ABSENT--\n", __func__);
					handle_battery_removed();
					update = 1;
				}
				
				if (!is_battery_id_valid()) {
					msm_chg.batt_status = BATT_STATUS_ID_INVALID;
					msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: BATT_STATUS_ID_INVALID--\n", __func__);
					handle_battery_removed();
					update = 1;
				}
			}
		}
	}
	msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: msm_batt_status is %s--\n",
		__func__, msm_batt_status_string[msm_chg.batt_status]);

	if (msm_chg.current_chg_priv
		&& msm_chg.current_chg_priv->hw_chg_state
			== CHG_CHARGING_STATE) {
		/* TODO implement JEITA SPEC*/
	}

	/* notify that the voltage has changed
	 * the read of the capacity will trigger a
	 * voltage read*/
	
	if (msm_chg.bat_inited == 1 && update == 1)
		power_supply_changed(&msm_psy_batt);
	
	if (msm_chg.stop_update) {
		msm_chg.stop_update = 0;
		return;
	}

	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));
}

/* set the charger state to READY before calling this */
static void handle_charger_ready(struct msm_hardware_charger_priv *hw_chg_priv)
{
	struct msm_hardware_charger_priv *old_chg_priv = NULL;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);


	if (msm_chg.current_chg_priv != NULL
	    && hw_chg_priv->hw_chg->rating >=
	    msm_chg.current_chg_priv->hw_chg->rating) {

		/*
		 * a better charger was found, ask the current charger
		 * to stop charging if it was charging
		 */
		if (msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
			if (msm_stop_charging(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
				return;
			}

			if (msm_charging_switched(msm_chg.current_chg_priv)) {
				dev_err(msm_chg.dev, "%s couldnt switch chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
				return;
			}
		}
		msm_chg.current_chg_priv->hw_chg_state = CHG_READY_STATE;
		old_chg_priv = msm_chg.current_chg_priv;
		msm_chg.current_chg_priv = NULL;
	}

	if (msm_chg.current_chg_priv == NULL) {
		msm_chg.current_chg_priv = hw_chg_priv;
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: best charger = %s--\n",
			 __func__,  msm_chg.current_chg_priv->hw_chg->name);

		msm_enable_system_current(msm_chg.current_chg_priv);
		/*
		 * since a better charger was chosen, ask the old
		 * charger to stop providing system current
		 */
		if (old_chg_priv != NULL)
			msm_disable_system_current(old_chg_priv);

		if (!is_batt_status_capable_of_charging())
		{
			dev_err(msm_chg.dev, "%s isn't capable of charging, batt_status is %s\n",
				__func__, msm_batt_status_string[msm_chg.batt_status]);
			return;
		}

		/* start charging from the new charger */
		if (!msm_start_charging()) {
			/* if we simply switched chg continue with teoc timer
			 * else we update the batt state and set the teoc
			 * timer */
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);

			if (!is_batt_status_charging()) {
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);
				queue_delayed_work(msm_chg.event_wq_thread,
							&msm_chg.teoc_work,
						      round_jiffies_relative
						      (msecs_to_jiffies
						       (msm_chg.safety_time)));
				msm_chg.batt_status = BATT_STATUS_TRKL_CHARGING;
			}
		} else {
			/* we couldnt start charging from the new readied
			 * charger */
			if (is_batt_status_charging()) {
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);
			}
		}
	}
}

static void handle_charger_removed(struct msm_hardware_charger_priv
				   *hw_chg_removed, int new_state)
{
	struct msm_hardware_charger_priv *hw_chg_priv;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (msm_chg.current_chg_priv == hw_chg_removed) {
		msm_disable_system_current(hw_chg_removed);
		if (msm_chg.current_chg_priv->hw_chg_state == CHG_CHARGING_STATE) {
			if (msm_stop_charging(hw_chg_removed)) {
				dev_err(msm_chg.dev, "%s couldnt stop chg\n",
					msm_chg.current_chg_priv->hw_chg->name);
			}
		}
		msm_chg.current_chg_priv = NULL;
	}

	hw_chg_removed->hw_chg_state = new_state;

	if (msm_chg.current_chg_priv == NULL) {
		hw_chg_priv = find_best_charger();
		if (hw_chg_priv == NULL) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: no chargers--\n", __func__);

			/* if the battery was Just finished charging
			 * we keep that state as is so that we dont rush
			 * in to charging the battery when a charger is
			 * plugged in shortly. */
			if (is_batt_status_charging()) {
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);
			}
		} else {
			msm_chg.current_chg_priv = hw_chg_priv;
			msm_enable_system_current(hw_chg_priv);
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: best charger = %s--\n",
				 __func__,  msm_chg.current_chg_priv->hw_chg->name);

			if (!is_batt_status_capable_of_charging()) {
				dev_err(msm_chg.dev, "%s isn't capable of charging, batt_status is %s\n",
					__func__, msm_batt_status_string[msm_chg.batt_status]);
				return;
			}

			if (msm_start_charging()) {
				/* we couldnt start charging for some reason */
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);
			}
		}
	}

	/* if we arent charging stop the safety timer */
	if (!is_batt_status_charging()) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);
		cancel_delayed_work(&msm_chg.teoc_work);
	}

	if (get_prop_batt_status() == POWER_SUPPLY_STATUS_FULL)
		updating_prop_batt_info(BQ27520_POLLING_CHARGING);
}


static void update_power_supply_event(struct msm_hardware_charger_priv *hw_chg_priv)
{
	static int chg_type = USB_CHG_TYPE__INVALID;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);

	
	if (hw_chg_priv->hw_chg->charger_type == USB_CHG_TYPE__CARKIT ||
		hw_chg_priv->hw_chg->charger_type == USB_CHG_TYPE__WALLCHARGER ||
		hw_chg_priv->hw_chg->charger_type == USB_CHG_TYPE__OTHERWALLCHARGER ||
		hw_chg_priv->hw_chg->charger_type == USB_CHG_TYPE__FASTERCHARGER) {
		power_supply_changed(&hw_chg_priv->ac_psy);
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: update ac_psy uevent--\n",
			__func__, __LINE__);
		chg_type = hw_chg_priv->hw_chg->charger_type;
	
	} else if (hw_chg_priv->hw_chg->charger_type == USB_CHG_TYPE__SDP) {
		power_supply_changed(&hw_chg_priv->usb_psy);
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: update usb_psy uevent--\n",
			__func__, __LINE__);
		chg_type = hw_chg_priv->hw_chg->charger_type;
	} else {
		if (chg_type == USB_CHG_TYPE__CARKIT ||
			chg_type == USB_CHG_TYPE__WALLCHARGER ||
			chg_type == USB_CHG_TYPE__OTHERWALLCHARGER ||
			chg_type == USB_CHG_TYPE__FASTERCHARGER) {
			power_supply_changed(&hw_chg_priv->ac_psy);
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: update ac_psy uevent--\n",
				__func__, __LINE__);
		} else if (chg_type == USB_CHG_TYPE__SDP) {
			power_supply_changed(&hw_chg_priv->usb_psy);
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: update usb_psy uevent--\n",
				__func__, __LINE__);
		} else {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: unknow uevent--\n",
				__func__, __LINE__);
		}
		chg_type = USB_CHG_TYPE__INVALID;
	}
	
}


static void dump_changed_log(int event)
{
	int ac_online = 0, usb_online = 0, chg_curr = 0, flags =0;
	char *event_types[] = {
		"CHG_INSERTED_EVENT",
		"CHG_ENUMERATED_EVENT",
		"CHG_REMOVED_EVENT",
		"CHG_DONE_EVENT",
		"CHG_PRE_CHG_TIME_OUT",
		"CHG_COMPLETE_CHG_TIME_OUT",
		"CHG_INVALID_CHARGER",
		"CHG_BATT_BEGIN_FAST_CHARGING",
		"CHG_BATT_TEMP_OUTOFRANGE",
		"CHG_BATT_TEMP_INRANGE",
		"CHG_BATT_INSERTED",
		"CHG_BATT_REMOVED",
		"CHG_BATT_STATUS_CHANGE",
		"CHG_BATT_CAPACITY_CHANGE",
		"CHG_BATT_NEEDS_RECHARGING",
		"CHG_BATT_TIME_OUT",
		"CHG_BATT_LOW",
		"CHG_BATT_CRITICAL_LOW",
		"CHG_BATT_VOLT_OUTOFRANGE",
		"CHG_BATT_VOLT_INRANGE",
		"OVP_FAULT_EVENT",
		"OVP_RECOVERY_EVENT"};

	msm_chg_printk(DBG_MSM_CHG_PROBE, "[BATT]%s \n", event_types[event]);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "[BATT]Status(%d), ",
		get_prop_batt_status());

	msm_chg_printk(DBG_MSM_CHG_PROBE, "Temperature(%d), ",
		get_prop_batt_temperature());

	msm_chg_printk(DBG_MSM_CHG_PROBE, "Capacity(%d), ",
		get_prop_batt_capacity());

	msm_chg_printk(DBG_MSM_CHG_PROBE, "Voltage(%d), ",
		get_prop_batt_mvolts());

	msm_chg_printk(DBG_MSM_CHG_PROBE, "Current(%d), ",
		get_prop_batt_curr());

	msm_chg_printk(DBG_MSM_CHG_PROBE, "Heahth(%d)\n",
		get_prop_batt_health());

	if (msm_hw_chg) {
		if (msm_hw_chg->charger_type == USB_CHG_TYPE__CARKIT ||
			msm_hw_chg->charger_type == USB_CHG_TYPE__WALLCHARGER ||
			msm_hw_chg->charger_type == USB_CHG_TYPE__OTHERWALLCHARGER ||
			msm_hw_chg->charger_type == USB_CHG_TYPE__FASTERCHARGER) {
				ac_online = 1;
				usb_online = 0;
		} else if (msm_hw_chg->charger_type == USB_CHG_TYPE__SDP) {
				ac_online = 0;
				usb_online = 1;
		} else {
				ac_online = 0;
				usb_online = 0;
		}
	}

	if (usb_hw_chg_priv) {
		chg_curr = usb_hw_chg_priv->max_source_current;
	}

	flags = engineer_gauge_flags();

	msm_chg_printk(DBG_MSM_CHG_PROBE, "[BATT]AC(%d), USB(%d), chg_curr(%d), ",
		ac_online, usb_online, chg_curr);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "Flags(%x):FC(%d), CHG(%d), OCV_GD(%d), ",
		flags, TST_BIT(flags, 9), TST_BIT(flags, 8), TST_BIT(flags, 5));

	msm_chg_printk(DBG_MSM_CHG_PROBE, "SOC1(%d), SYSDOWN(%d), DSG(%d)\n",
		TST_BIT(flags, 2), TST_BIT(flags, 1), TST_BIT(flags, 0));

	msm_chg_printk(DBG_MSM_CHG_PROBE, "[BATT]batt_status(%d), update_time(%d), ",
		msm_chg.batt_status, msm_chg.update_time);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "CHG_STAT_INT(%d), OVP_FLAG_INT(%d), ",
		gpio_get_value(SMB136_CHG_STAT_INT), gpio_get_value(OVP_FLAG_INT));

	msm_chg_printk(DBG_MSM_CHG_PROBE, "BATT_LOW(%d), GAUGE_INT(%d)\n",
		gpio_get_value(BQ27520_BATT_LOW), gpio_get_value(BQ27520_GAUGE_INT));
}

static void handle_event(struct msm_hardware_charger *hw_chg, int event)
{
	struct msm_hardware_charger_priv *priv = NULL;
	int polling_time, capacity;

	msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: event(%d)--\n", __func__, event);

	/*
	 * if hw_chg is NULL then this event comes from non-charger
	 * parties like battery gauge
	 */
	 
	if (hw_chg) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: event(%d) from %s--\n",
			__func__, event, hw_chg->name);
		priv = hw_chg->charger_private;
	}
	

	mutex_lock(&msm_chg.status_lock);

	switch (event) {
	case CHG_INSERTED_EVENT:
		if (priv->hw_chg_state != CHG_ABSENT_STATE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: hw_chg_state(%d)--\n",
				__func__, __LINE__, priv->hw_chg_state);
			break;
		}
		updating_prop_batt_info(BQ27520_POLLING_DETECT);
		update_batt_status();
	
		if (hw_chg->type == CHG_TYPE_USB) {
			priv->hw_chg_state = CHG_PRESENT_STATE;
			#ifndef CONFIG_EXTERNAL_CHARGER
			notify_usb_of_the_plugin_event(priv, 1);
			#else
			usb_hw_chg_priv = priv;
			#endif
			if (usb_chg_current) {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: usb_chg_current=%d--\n",
					__func__, usb_chg_current);
				priv->max_source_current = usb_chg_current;
				usb_chg_current = 0;
				/* usb has already indicated us to charge */
				priv->hw_chg_state = CHG_READY_STATE;
				handle_charger_ready(priv);
			} else {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: max_source_current=100--\n",
					__func__);
				
				priv->max_source_current = 100;
				
				priv->hw_chg_state = CHG_READY_STATE;
				handle_charger_ready(priv);
	
			}
		} else {
			priv->hw_chg_state = CHG_READY_STATE;
			handle_charger_ready(priv);
		}
		
		if (msm_chg.chg_inited == 1)
			update_power_supply_event(priv);
		
		break;
	case CHG_ENUMERATED_EVENT:	/* only in USB types */
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: cable is absent--\n",
				__func__, __LINE__);
			break;
		}
		updating_prop_batt_info(BQ27520_POLLING_CHARGING);
		update_batt_status();
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: %s enum with %dmA to draw--\n",
			__func__, hw_chg->name, priv->max_source_current);

		if (priv->max_source_current == 0) {
			/* usb subsystem doesnt want us to draw
			 * charging current */
			/* act as if the charge is removed */
			if (priv->hw_chg_state != CHG_PRESENT_STATE) {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: act as if charger is removed, hw_chg_state=%d--\n",
					__func__, priv->hw_chg_state);
				handle_charger_removed(priv, CHG_PRESENT_STATE);
			}
		} else {
	
			if (priv->hw_chg_state == CHG_CHARGING_STATE) {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: switch charger to %dmA--\n",
					__func__, priv->max_source_current);
				handle_charger_ready(priv);
			} else {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: Charger is not exit, hw_chg_state=%d--\n",
					__func__, priv->hw_chg_state);
			}
	
		}
		break;
	case CHG_REMOVED_EVENT:
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: cable already removed--\n",
				__func__, __LINE__);
			break;
		}
		updating_prop_batt_info(BQ27520_POLLING_DETECT);
		
		update_batt_status();
		

		if (hw_chg->type == CHG_TYPE_USB) {
			usb_chg_current = 0;
			#ifndef CONFIG_EXTERNAL_CHARGER
			notify_usb_of_the_plugin_event(priv, 0);
			#else
			usb_hw_chg_priv = NULL;
			#endif
		}
		handle_charger_removed(priv, CHG_ABSENT_STATE);
		
		if (msm_chg.chg_inited == 1)
			update_power_supply_event(priv);
		
		break;
	case CHG_DONE_EVENT:
		if (priv->hw_chg_state == CHG_CHARGING_STATE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHARGER DONE--\n",
				__func__);
			updating_prop_batt_info(BQ27520_POLLING_CHARGINGDONE);
			handle_charging_done(priv);
		}
		break;
	case CHG_BATT_BEGIN_FAST_CHARGING:
		/* only update if we are TRKL charging */
		if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING) {
			msm_chg.batt_status = BATT_STATUS_FAST_CHARGING;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: BATT_STATUS_FAST_CHARGING--\n",
				__func__);
		}
		if (get_prop_batt_status() == POWER_SUPPLY_STATUS_FULL)
			updating_prop_batt_info(BQ27520_POLLING_CHARGING);
		break;
	case CHG_BATT_NEEDS_RECHARGING:
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		handle_battery_inserted();
		priv = msm_chg.current_chg_priv;
		break;
	case CHG_BATT_TEMP_OUTOFRANGE:
		/* the batt_temp out of range can trigger
		 * when the battery is absent */
		if (!is_battery_present()
		    && msm_chg.batt_status != BATT_STATUS_ABSENT) {
			msm_chg.batt_status = BATT_STATUS_ABSENT;
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: battery is absent--\n",
				__func__, __LINE__);
			handle_battery_removed();
			break;
		}
		if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: batt_status already out of temp range--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
	
		updating_prop_batt_info(BQ27520_POLLING_FAULT);
	

		handle_charger_fault(CHG_BATT_TEMP_OUTOFRANGE);
		break;
	case CHG_BATT_TEMP_INRANGE:
		if (msm_chg.batt_status != BATT_STATUS_TEMPERATURE_OUT_OF_RANGE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: batt_status already temp recovery--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_ID_INVALID;
		/* check id */
		if (!is_battery_id_valid()) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: batt id isn't valid--\n",
				__func__, __LINE__);
			break;
		}
		/* assume that we are discharging from the battery
		 * and act as if the battery was inserted
		 * if a charger is present charging will be resumed */
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		handle_battery_inserted();
	
		if (msm_chg.batt_status == BATT_STATUS_DISCHARGING)
			updating_prop_batt_info(BQ27520_POLLING_DETECT);
	
		break;
	case CHG_BATT_INSERTED:
		if (msm_chg.batt_status != BATT_STATUS_ABSENT) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: battery already present--\n",
				__func__);
			break;
		}
		/* debounce */
		if (!is_battery_present()) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: battery is absent--\n",
				__func__, __LINE__);
			break;
		}

		msm_chg.batt_status = BATT_STATUS_ID_INVALID;
		if (!is_battery_id_valid()) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: batt id isn't valid--\n",
				__func__, __LINE__);
			break;
		}
		/* assume that we are discharging from the battery */
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		/* check if a charger is present */
		handle_battery_inserted();
		break;
	case CHG_BATT_REMOVED:
		if (msm_chg.batt_status == BATT_STATUS_ABSENT) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: battery already absent--\n",
				__func__);
			break;
		}
		/* debounce */
		if (is_battery_present()) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: battery is present--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_ABSENT;
		handle_battery_removed();
		break;

	case CHG_BATT_TIME_OUT:
	case CHG_PRE_CHG_TIME_OUT:
	case CHG_COMPLETE_CHG_TIME_OUT:
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: cable already removed--\n",
				__func__, __LINE__);
			break;
		}
		if (msm_chg.batt_status == BATT_STATUS_TIME_OUT) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: batt_status already time out--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_TIME_OUT;
		updating_prop_batt_info(BQ27520_POLLING_FAULT);

		if (hw_chg->type == CHG_TYPE_USB) {
			usb_chg_current = 0;
			#ifndef CONFIG_EXTERNAL_CHARGER
			notify_usb_of_the_plugin_event(priv, 0);
			#else
			usb_hw_chg_priv = NULL;
			#endif
		}

		if (event == CHG_BATT_TIME_OUT) {
			handle_charger_fault(CHG_BATT_TIME_OUT);
		} else if (event == CHG_PRE_CHG_TIME_OUT) {
			handle_charger_fault(CHG_PRE_CHG_TIME_OUT);
		} else {
			handle_charger_fault(CHG_COMPLETE_CHG_TIME_OUT);
		}
		break;

	
	case CHG_INVALID_CHARGER:
		if (priv->hw_chg_state == CHG_ABSENT_STATE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: cable already removed--\n",
				__func__, __LINE__);
			break;
		}
		if (msm_chg.batt_status == BATT_STATUS_INVALID_CHARGING) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: invalid charger detected--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_INVALID_CHARGING;
		updating_prop_batt_info(BQ27520_POLLING_FAULT);
		handle_charger_fault(CHG_INVALID_CHARGER);
		break;
	


	case CHG_BATT_VOLT_OUTOFRANGE:
		if (msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: batt_status already volt out of range--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_VOLTAGE_OUT_OF_RANGE;
		updating_prop_batt_info(BQ27520_POLLING_FAULT);
		handle_charger_fault(CHG_BATT_VOLT_OUTOFRANGE);
		break;

	case CHG_BATT_VOLT_INRANGE:
		if (msm_chg.batt_status != BATT_STATUS_VOLTAGE_OUT_OF_RANGE) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: batt_status already volt recovery--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		updating_prop_batt_info(BQ27520_POLLING_DETECT);
		handle_battery_inserted();
		break;


	case OVP_FAULT_EVENT:
		if (msm_chg.batt_status == BATT_STATUS_OVP_FAULT) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: batt_status already ovp fault--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_OVP_FAULT;
		handle_charger_fault(OVP_FAULT_EVENT);
		break;

	case OVP_RECOVERY_EVENT:
		if (msm_chg.batt_status != BATT_STATUS_OVP_FAULT) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: batt_status already ovp recovery--\n",
				__func__);
			break;
		}
		msm_chg.batt_status = BATT_STATUS_DISCHARGING;
		updating_prop_batt_info(BQ27520_POLLING_DETECT);
		handle_battery_inserted();
		break;


	case CHG_BATT_LOW:
		if (msm_chg.batt_status == BATT_STATUS_FAST_CHARGING ||
			msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING ||
			msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: batt_status is on charging--\n",
				__func__, __LINE__);
			break;
		}

		if (msm_chg.update_time != BQ27520_POLLING_BAT_LOW) {
			updating_prop_batt_info(BQ27520_POLLING_BAT_LOW);
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: updating polling time as %d secs--\n",
				__func__, __LINE__, BQ27520_POLLING_BAT_LOW);
		}
		break;

	case CHG_BATT_CRITICAL_LOW:
		if (msm_chg.batt_status == BATT_STATUS_FAST_CHARGING ||
			msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING ||
			msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: batt_status is on charging--\n",
				__func__, __LINE__);
			break;
		}

		if (msm_chg.update_time != BQ27520_POLLING_BAT_CRITICAL_LOW) {
			updating_prop_batt_info(BQ27520_POLLING_BAT_CRITICAL_LOW);
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: updating polling time as %d secs--\n",
				__func__, __LINE__, BQ27520_POLLING_BAT_CRITICAL_LOW);
		}
		break;


	case CHG_BATT_STATUS_CHANGE:
	
		if (msm_chg.batt_status == BATT_STATUS_TEMPERATURE_OUT_OF_RANGE ||
			msm_chg.batt_status == BATT_STATUS_TIME_OUT ||
			msm_chg.batt_status == BATT_STATUS_OVP_FAULT ||
			msm_chg.batt_status == BATT_STATUS_VOLTAGE_OUT_OF_RANGE ||
			msm_chg.batt_status == BATT_STATUS_INVALID_CHARGING) {
			polling_time = BQ27520_POLLING_FAULT;
		} else if (msm_chg.batt_status == BATT_STATUS_TRKL_CHARGING ||
			msm_chg.batt_status == BATT_STATUS_FAST_CHARGING) {
			polling_time = BQ27520_POLLING_CHARGING;
		} else if (msm_chg.batt_status == BATT_STATUS_JUST_FINISHED_CHARGING) {
			polling_time = BQ27520_POLLING_CHARGINGDONE;
		} else if (msm_chg.batt_status == BATT_STATUS_ABSENT ||
			msm_chg.batt_status == BATT_STATUS_ID_INVALID) {
			polling_time = BQ27520_POLLING_DETECT;
		} else {
			capacity = get_prop_batt_capacity();
			if (capacity <= 5) {
				polling_time = BQ27520_POLLING_BAT_CRITICAL_LOW;
			} else if (capacity <= 15) {
				polling_time = BQ27520_POLLING_BAT_LOW;
			} else {
				polling_time = BQ27520_POLLING_DISCHARGING;
			}
		}

		if (msm_chg.update_time != polling_time) {
			updating_prop_batt_info(polling_time);
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: updating polling time as %d secs--\n",
				__func__, __LINE__, polling_time);
		}
	
		/* TODO  battery SOC like battery-alarm/charging-full features
		can be added here for future improvement */
		break;
	
	case CHG_BATT_CAPACITY_CHANGE:
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: battery capacity change--\n",
			__func__);

		if (msm_chg.batt_status == BATT_STATUS_DISCHARGING &&
			msm_chg.update_time != BQ27520_POLLING_DISCHARGING) {
			capacity = get_prop_batt_capacity();
			if (capacity <= 5) {
				polling_time = BQ27520_POLLING_BAT_CRITICAL_LOW;
			} else if (capacity <= 15) {
				polling_time = BQ27520_POLLING_BAT_LOW;
			} else {
				polling_time = BQ27520_POLLING_DISCHARGING;
			}

			if (msm_chg.update_time != polling_time) {
				updating_prop_batt_info(polling_time);
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: updating polling time as %d secs--\n",
					__func__, __LINE__, polling_time);
			}
		}
		break;
	}
	
	msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s %d done batt_status is %s--\n",
				__func__, event, msm_batt_status_string[msm_chg.batt_status]);

	dump_changed_log(event);
	
	if (MSM_GAUGE_LOG == 1)
		dump_gauge_log();
	

	
	/* update userspace */
	if (msm_batt_gauge && msm_chg.bat_inited == 1)
		power_supply_changed(&msm_psy_batt);
	

	mutex_unlock(&msm_chg.status_lock);
}

static int msm_chg_dequeue_event(struct msm_charger_event **event)
{
	unsigned long flags;

	spin_lock_irqsave(&msm_chg.queue_lock, flags);
	if (msm_chg.queue_count == 0) {
		spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
		return -EINVAL;
	}
	*event = &msm_chg.queue[msm_chg.head];
	msm_chg.head = (msm_chg.head + 1) % MSM_CHG_MAX_EVENTS;
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: dequeueing %d--\n",
		__func__, (*event)->event);

	msm_chg.queue_count--;
	spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
	return 0;
}

static int msm_chg_enqueue_event(struct msm_hardware_charger *hw_chg,
			enum msm_hardware_charger_event event)
{
	unsigned long flags;

	spin_lock_irqsave(&msm_chg.queue_lock, flags);
	if (msm_chg.queue_count == MSM_CHG_MAX_EVENTS) {
		spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
		pr_err("%s: queue full cannot enqueue %d\n",
				__func__, event);
		return -EAGAIN;
	}
 	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: queueing event=%d--\n", __func__, event);

	msm_chg.queue[msm_chg.tail].event = event;
	msm_chg.queue[msm_chg.tail].hw_chg = hw_chg;
	msm_chg.tail = (msm_chg.tail + 1)%MSM_CHG_MAX_EVENTS;
	msm_chg.queue_count++;
	spin_unlock_irqrestore(&msm_chg.queue_lock, flags);
	return 0;
}

static void process_events(struct work_struct *work)
{
	struct msm_charger_event *event;
	int rc;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	do {
		rc = msm_chg_dequeue_event(&event);
		if (!rc)
			handle_event(event->hw_chg, event->event);
	} while (!rc);
}

/* USB calls these to tell us how much charging current we should draw */
void msm_charger_vbus_draw(unsigned int mA)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (usb_hw_chg_priv) {
	
		if (mA != usb_hw_chg_priv->max_source_current) {
			if(mA != 0) {
				usb_hw_chg_priv->max_source_current = mA;

				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: usb current(%d)--\n",
					__func__, __LINE__, mA);
				msm_charger_notify_event(usb_hw_chg_priv->hw_chg, CHG_ENUMERATED_EVENT);
			} else {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: usb current(%d)--\n",
					__func__, __LINE__, mA);
			}
		} else {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: charger have set the current(%d) before--\n",
				__func__, mA);
		}
	
	} else {
		/* remember the current, to be used when charger is ready */
		usb_chg_current = mA;

		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s,%d: usb current(%d)--\n",
			__func__, __LINE__, mA);
	}
}

static int __init determine_initial_batt_status(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (is_battery_present()) {
		if (is_battery_id_valid()) {
			if (is_battery_temp_within_range()) {
				msm_chg.batt_status = BATT_STATUS_DISCHARGING;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_DISCHARGING--\n", __func__);
			} else {
				msm_chg.batt_status = BATT_STATUS_TEMPERATURE_OUT_OF_RANGE;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_TEMPERATURE_OUT_OF_RANGE--\n", __func__);
				updating_prop_batt_info(BQ27520_POLLING_FAULT);
			}
		} else {
			msm_chg.batt_status = BATT_STATUS_ID_INVALID;
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_ID_INVALID--\n", __func__);
		}
	} else {
		msm_chg.batt_status = BATT_STATUS_ABSENT;
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: BATT_STATUS_ABSENT--\n", __func__);
	}


	if (is_batt_status_capable_of_charging()) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: handle batt insert--\n", __func__);
		handle_battery_inserted();
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);


	/* start updaing the battery powersupply every msm_chg.update_time
	 * milliseconds */
	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (msm_chg.update_time)));

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: batt OK, msm_batt_status is %s--\n",
		__func__, msm_batt_status_string[msm_chg.batt_status]);

	return 0;
}

static int __devinit msm_charger_probe(struct platform_device *pdev)
{
	msm_chg.dev = &pdev->dev;

	if (pdev->dev.platform_data) {
		unsigned int milli_secs;

		struct msm_charger_platform_data *pdata
		    = (struct msm_charger_platform_data *)pdev->dev.platform_data;

		milli_secs = pdata->safety_time * 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);

			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		msm_chg.safety_time = milli_secs;

		milli_secs = pdata->update_time * 60 * MSEC_PER_SEC;
		if (milli_secs > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
			dev_warn(&pdev->dev, "%s: safety time too large"
				 "%dms\n", __func__, milli_secs);

			milli_secs = jiffies_to_msecs(MAX_JIFFY_OFFSET);
		}
		msm_chg.update_time = milli_secs;

		msm_chg.max_voltage = pdata->max_voltage;
		msm_chg.min_voltage = pdata->min_voltage;
		msm_chg.get_batt_capacity_percent = pdata->get_batt_capacity_percent;
	}
	if (msm_chg.safety_time == 0)
		msm_chg.safety_time = CHARGING_TEOC_MS;
	if (msm_chg.update_time == 0)
		msm_chg.update_time = UPDATE_TIME_MS;
	if (msm_chg.max_voltage == 0)
		msm_chg.max_voltage = DEFAULT_BATT_MAX_V;
	if (msm_chg.min_voltage == 0)
		msm_chg.min_voltage = DEFAULT_BATT_MIN_V;
	if (msm_chg.get_batt_capacity_percent == NULL)
		msm_chg.get_batt_capacity_percent = msm_chg_get_batt_capacity_percent;

	mutex_init(&msm_chg.status_lock);
	INIT_DELAYED_WORK(&msm_chg.teoc_work, teoc);
	INIT_DELAYED_WORK(&msm_chg.update_heartbeat_work, update_heartbeat);

	msm_chg_create_debugfs_entries(&msm_chg);
	msm_chg_create_kernel_debuglevel_entries();

	wake_lock_init(&msm_chg.wl, WAKE_LOCK_SUSPEND, "msm_charger");

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s probe success!\n", __func__);

	return 0;
}

static int __devexit msm_charger_remove(struct platform_device *pdev)
{

	msm_chg_destroy_debugfs_entries();
	msm_chg_destroy_kernel_debuglevel_entries();

	wake_lock_destroy(&msm_chg.wl);
	mutex_destroy(&msm_chg.status_lock);
	power_supply_unregister(&msm_psy_batt);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s remove success!\n", __func__);
	return 0;
}

int msm_charger_notify_event(struct msm_hardware_charger *hw_chg,
			     enum msm_hardware_charger_event event)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: event=%d--\n", __func__, event);

	msm_chg_enqueue_event(hw_chg, event);
	queue_work(msm_chg.event_wq_thread, &msm_chg.queue_work);
	return 0;
}
EXPORT_SYMBOL(msm_charger_notify_event);

int msm_charger_register(struct msm_hardware_charger *hw_chg)
{
	struct msm_hardware_charger_priv *priv;
	int rc = 0;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (!msm_chg.inited) {
		pr_err("%s: msm_chg is NULL,Too early to register\n", __func__);
		return -EAGAIN;
	}

	if (hw_chg->start_charging == NULL
		|| hw_chg->stop_charging == NULL
		|| hw_chg->name == NULL
		|| hw_chg->rating == 0) {
		pr_err("%s: invalid hw_chg\n", __func__);
		return -EINVAL;
	}

	priv = kzalloc(sizeof *priv, GFP_KERNEL);
	if (priv == NULL) {
		dev_err(msm_chg.dev, "%s kzalloc failed\n", __func__);
		return -ENOMEM;
	}


	msm_hw_chg = hw_chg;


	priv->hw_chg = hw_chg;
	priv->hw_chg_state = CHG_ABSENT_STATE;


	priv->ac_psy.name = "Mains",
	priv->ac_psy.type = POWER_SUPPLY_TYPE_MAINS,
	priv->ac_psy.supplied_to = msm_power_supplied_to,
	priv->ac_psy.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	priv->ac_psy.properties = msm_power_props,
	priv->ac_psy.num_properties = ARRAY_SIZE(msm_power_props),
	priv->ac_psy.get_property = msm_power_get_property,

	priv->usb_psy.name = "USB",
	priv->usb_psy.type = POWER_SUPPLY_TYPE_USB,
	priv->usb_psy.supplied_to = msm_power_supplied_to,
	priv->usb_psy.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	priv->usb_psy.properties = msm_power_props,
	priv->usb_psy.num_properties = ARRAY_SIZE(msm_power_props),
	priv->usb_psy.get_property = msm_power_get_property,

	rc = power_supply_register(msm_chg.dev, &priv->ac_psy);
	if (rc) {
		dev_err(msm_chg.dev, "%s ac power_supply_register failed\n",
			__func__);
		goto out;
	}

	rc = power_supply_register(msm_chg.dev, &priv->usb_psy);
	if (rc) {
		dev_err(msm_chg.dev, "%s usb power_supply_register failed\n",
			__func__);
		goto unreg_power_supply;
	}

	msm_chg.chg_inited = 1;



	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: finish!!--\n", __func__);

	INIT_LIST_HEAD(&priv->list);
	mutex_lock(&msm_chg.msm_hardware_chargers_lock);
	list_add_tail(&priv->list, &msm_chg.msm_hardware_chargers);
	mutex_unlock(&msm_chg.msm_hardware_chargers_lock);
	hw_chg->charger_private = (void *)priv;

	return 0;


unreg_power_supply:
	power_supply_unregister(&priv->ac_psy);

out:
	kfree(priv);
	return rc;
}
EXPORT_SYMBOL(msm_charger_register);

void msm_battery_gauge_register(struct msm_battery_gauge *batt_gauge)
{
	int rc ;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (!msm_chg.inited) {
		pr_err("%s: msm_chg is NULL,Too early to register\n", __func__);
	} else {
		if (msm_batt_gauge) {
			msm_batt_gauge = batt_gauge;
			pr_err("msm-charger %s multiple battery gauge called\n",
								__func__);
		} else {
			msm_batt_gauge = batt_gauge;

			rc = power_supply_register(msm_chg.dev, &msm_psy_batt);
			if (rc < 0) {
				dev_err(msm_chg.dev, "%s: power_supply_register failed"
					" rc=%d\n", __func__, rc);
				return;
			}

			
			msm_chg.bat_inited = 1;
			
			determine_initial_batt_status();
		}
	}
}

EXPORT_SYMBOL(msm_battery_gauge_register);

void msm_battery_gauge_unregister(struct msm_battery_gauge *batt_gauge)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	msm_batt_gauge = NULL;

	msm_chg.bat_inited = 0;

}
EXPORT_SYMBOL(msm_battery_gauge_unregister);

int msm_charger_unregister(struct msm_hardware_charger *hw_chg)
{
	struct msm_hardware_charger_priv *priv;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	priv = (struct msm_hardware_charger_priv *)(hw_chg->charger_private);
	mutex_lock(&msm_chg.msm_hardware_chargers_lock);
	list_del(&priv->list);
	mutex_unlock(&msm_chg.msm_hardware_chargers_lock);

	if (priv) {
		power_supply_unregister(&priv->ac_psy);
		power_supply_unregister(&priv->usb_psy);
	}

	kfree(priv);

	msm_chg.chg_inited = 0;


	return 0;
}
EXPORT_SYMBOL(msm_charger_unregister);

static int msm_charger_suspend(struct device *dev)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	msm_chg.stop_update = 1;
	cancel_delayed_work(&msm_chg.update_heartbeat_work);
	mutex_lock(&msm_chg.status_lock);
	handle_battery_removed();
	mutex_unlock(&msm_chg.status_lock);
	return 0;
}

static int msm_charger_resume(struct device *dev)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	msm_chg.stop_update = 0;
	/* start updaing the battery powersupply every msm_chg.update_time
	 * milliseconds */

	queue_delayed_work(msm_chg.event_wq_thread,
				&msm_chg.update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (10))); 

	mutex_lock(&msm_chg.status_lock);
	handle_battery_inserted();
	mutex_unlock(&msm_chg.status_lock);

	return 0;
}

static SIMPLE_DEV_PM_OPS(msm_charger_pm_ops,
		msm_charger_suspend, msm_charger_resume);

static struct platform_driver msm_charger_driver = {
	.probe = msm_charger_probe,
	.remove = __devexit_p(msm_charger_remove),
	.driver = {
		   .name = MSM_CHG_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .pm = &msm_charger_pm_ops,
	},
};

static int __init msm_charger_init(void)
{
	int rc;

	INIT_LIST_HEAD(&msm_chg.msm_hardware_chargers);
	msm_chg.count_chargers = 0;
	mutex_init(&msm_chg.msm_hardware_chargers_lock);

	msm_chg.queue = kzalloc(sizeof(struct msm_charger_event)
				* MSM_CHG_MAX_EVENTS,
				GFP_KERNEL);
	if (!msm_chg.queue) {
		rc = -ENOMEM;
		goto out;
	}

	msm_chg.bat_inited = 0;
	msm_chg.chg_inited = 0;

	msm_chg.tail = 0;
	msm_chg.head = 0;
	spin_lock_init(&msm_chg.queue_lock);
	msm_chg.queue_count = 0;
	INIT_WORK(&msm_chg.queue_work, process_events);
	msm_chg.event_wq_thread = create_workqueue("msm_charger_eventd");
	if (!msm_chg.event_wq_thread) {
		rc = -ENOMEM;
		goto free_queue;
	}
	rc = platform_driver_register(&msm_charger_driver);
	if (rc < 0) {
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);
		goto destroy_wq_thread;
	}
	msm_chg.inited = 1;

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s initialining success!\n", __func__);

	return 0;

destroy_wq_thread:
	destroy_workqueue(msm_chg.event_wq_thread);
free_queue:
	kfree(msm_chg.queue);
out:
	return rc;
}

static void __exit msm_charger_exit(void)
{
	flush_workqueue(msm_chg.event_wq_thread);
	destroy_workqueue(msm_chg.event_wq_thread);
	kfree(msm_chg.queue);
	platform_driver_unregister(&msm_charger_driver);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s exit success!\n", __func__);
}

module_init(msm_charger_init);
module_exit(msm_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Abhijeet Dharmapurikar <adharmap@codeaurora.org>");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
