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
#ifndef __MSM_CHARGER_H__
#define __MSM_CHARGER_H__

#include <linux/power_supply.h>

#define MSM_CHG_DRIVER_NAME				"msm-charger"




#define MYBIT(b)        (1<<b)
#define TST_BIT(x,b)    ((x & (1<<b)) ? 1 : 0)
#define CLR_BIT(x,b)    (x &= (~(1<<b)))
#define SET_BIT(x,b)    (x |= (1<<b))

#define MSM_CHG_MAX_EVENTS		16
#define CHARGING_TEOC_MS		9000000
#define RESUME_CHECK_PERIOD_MS	60000
#define UPDATE_TIME_MS			60000


#define BATT_INVALID_VOLT_THRESHOLD		2000 
#define BATT_INVALID_TEMP_THRESHOLD		-400 



#define DEFAULT_BATT_HIGH_TEMP		450 
#define DEFAULT_BATT_LOW_TEMP		0	



enum msm_chg__debug_level {
	DBG_MSM_CRITICAL = 0,
	DBG_MSM_CHG_PROBE,
	DBG_MSM_CHG_DEBUGFS,
	DBG_MSM_CHG_DFI,
	DBG_MSM_CHG_EVENT,
	DBG_MSM_CHG_INFO,
	DBG_MSM_CHG_UPDATE,
	DBG_MSM_CHG_INTERRUPT,
	DBG_MSM_CHG_WROKQUEUE,
	DBG_MSM_CHG_DEBUG,
};

union MSM_CHG_DEBUGLEVEL {
	uint debug_level;
	char debug_byte[4];
};

extern union MSM_CHG_DEBUGLEVEL MSM_CHG_DEBUG_DLL;

#define	MSM_INFO_DLL_ON		(1 << 0)
#define	MSM_CHARGER_DLL_ON	(1 << 7)
#define	MSM_GAUGE_DLL_ON	(1 << 15)
#define	MSM_DEBUG_DLL_ON	(1 << 23)

#define msm_chg_printk(level, fmt, args...) \
	do { \
		if (MSM_CHG_DEBUG_DLL.debug_level >= MSM_DEBUG_DLL_ON) { \
			if (level <= DBG_MSM_CHG_DEBUG) printk(fmt, ##args); \
		} else if (MSM_CHG_DEBUG_DLL.debug_level >= MSM_GAUGE_DLL_ON) { \
			if (level <= DBG_MSM_CHG_UPDATE) printk(fmt, ##args); \
		} else if (MSM_CHG_DEBUG_DLL.debug_level >= MSM_CHARGER_DLL_ON) { \
			if (level <= DBG_MSM_CHG_INFO) printk(fmt, ##args); \
		} else if (MSM_CHG_DEBUG_DLL.debug_level >= MSM_INFO_DLL_ON) { \
			if (level <= DBG_MSM_CHG_EVENT) printk(fmt, ##args); \
		} else { \
			if (level <= DBG_MSM_CHG_DFI) printk(fmt, ##args); } \
	} while(0)



extern uint MSM_GAUGE_LOG;


enum {	CHG_TYPE_USB,
	CHG_TYPE_AC
};

enum msm_hardware_charger_event {
	CHG_INSERTED_EVENT,
	CHG_ENUMERATED_EVENT,
	CHG_REMOVED_EVENT,
	CHG_DONE_EVENT,

	CHG_PRE_CHG_TIME_OUT,
	CHG_COMPLETE_CHG_TIME_OUT,


	CHG_INVALID_CHARGER,

	CHG_BATT_BEGIN_FAST_CHARGING,
	CHG_BATT_TEMP_OUTOFRANGE,
	CHG_BATT_TEMP_INRANGE,
	CHG_BATT_INSERTED,
	CHG_BATT_REMOVED,
	CHG_BATT_STATUS_CHANGE,

	CHG_BATT_CAPACITY_CHANGE,

	CHG_BATT_NEEDS_RECHARGING,

	CHG_BATT_TIME_OUT,


	CHG_BATT_LOW,
	CHG_BATT_CRITICAL_LOW,


	CHG_BATT_VOLT_OUTOFRANGE,
	CHG_BATT_VOLT_INRANGE,


	OVP_FAULT_EVENT,
	OVP_RECOVERY_EVENT,

};

/**
 * enum hardware_charger_state
 * @CHG_ABSENT_STATE: charger cable is unplugged
 * @CHG_PRESENT_STATE: charger cable is plugged but charge current isnt drawn
 * @CHG_READY_STATE: charger cable is plugged and kernel knows how much current
 *			it can draw
 * @CHG_CHARGING_STATE: charger cable is plugged and current is drawn for
 *			charging
 */
enum msm_hardware_charger_state {
	CHG_ABSENT_STATE,
	CHG_PRESENT_STATE,
	CHG_READY_STATE,
	CHG_CHARGING_STATE,
};

struct msm_hardware_charger {
	int type;
	int rating;
	int charger_type;
	const char *name;
	int (*start_charging) (struct msm_hardware_charger *hw_chg,
			       int chg_voltage, int chg_current);
	int (*stop_charging) (struct msm_hardware_charger *hw_chg);

	int (*charging_switched) (struct msm_hardware_charger *hw_chg,
							int chg_voltage, int chg_current);


	int (*get_charger_state) (void);


	int (*get_charger_register_status) (int reg, u8 value);

	void (*start_system_current) (struct msm_hardware_charger *hw_chg,
							int chg_current);
	void (*stop_system_current) (struct msm_hardware_charger *hw_chg);

	void *charger_private;	/* used by the msm_charger.c */


	int (*ovp_fault_gpio_state) (void);

};

struct msm_battery_gauge {
	int (*get_battery_mvolts) (void);
	int (*get_battery_temperature) (void);
	int (*is_battery_present) (void);
	int (*is_battery_temp_within_range) (void);
	int (*is_battery_id_valid) (void);
	int (*get_battery_status)(void);
	int (*get_batt_remaining_capacity) (void);
	int (*monitor_for_recharging) (void);

	int (*get_average_current) (void);
	int (*get_time_to_empty_now) (void);
	int (*get_time_to_empty_avg) (void);
	int (*get_time_to_full_now) (void);
	void (*updating_batt_info) (int);
	int (*copy_dfi_from_user_to_kernel) (const char *val, u16 size);

};
/**
 * struct msm_charger_platform_data
 * @safety_time: max charging time in minutes
 * @update_time: how often the userland be updated of the charging progress
 * @max_voltage: the max voltage the battery should be charged upto
 * @min_voltage: the voltage where charging method switches from trickle to fast
 * @get_batt_capacity_percent: a board specific function to return battery
 *			capacity. Can be null - a default one will be used
 */
struct msm_charger_platform_data {
	unsigned int safety_time;
	unsigned int update_time;
	unsigned int max_voltage;
	unsigned int min_voltage;
	unsigned int (*get_batt_capacity_percent) (void);
};


#if 0
struct msm_power_monitor{
 	
 	int battery_capacity;
	int battery_voltage;
	int battery_current;
	int battery_temperature;
	int gauge_firmware_version;

	
	int battery_remaining_capacity;
	int battery_full_capacity;
	int gauge_flags;

	
	char snooze_state[8];
	
	char good_ocv[8];
	
	char wait_battery_id[8];
	
	char battery_detected[8];
	
	char charge_threshold1_reached[8];
	
	char charge_threshold_final_reached[8];

	
	char usb[15];
	char wall_charger[15];

	

	char battery_temp_too_high[15];
	char battery_temp_too_low[15];
	char battery_health[10];
	char is_charging[5];
	
	
 	char usb_current_limit[6];
	
	char usb_suspend[15];
	
	char current_limit_mode[15];
	
	char charging_enable[8];
	
	char chg_fault[10];
	
	char dc_pwr_ok[20];
	
	char chg_status[15];
	
};
#endif


typedef void (*notify_vbus_state) (int);


#if defined(CONFIG_CHARGER_MSM7x27A)

void msm_battery_gauge_register(struct msm_battery_gauge *batt_gauge);
void msm_battery_gauge_unregister(struct msm_battery_gauge *batt_gauge);
int msm_charger_register(struct msm_hardware_charger *hw_chg);
int msm_charger_unregister(struct msm_hardware_charger *hw_chg);
int msm_charger_notify_event(struct msm_hardware_charger *hw_chg,
			     enum msm_hardware_charger_event event);
void msm_charger_vbus_draw(unsigned int mA);

unsigned int msm_chg_get_batt_capacity_percent(void);


int msm_charger_register_vbus_sn(void (*callback)(int));
void msm_charger_unregister_vbus_sn(void (*callback)(int));
#else
static inline void msm_battery_gauge_register(struct msm_battery_gauge *gauge)
{
}
static inline void msm_battery_gauge_unregister(struct msm_battery_gauge *gauge)
{
}
static inline int msm_charger_register(struct msm_hardware_charger *hw_chg)
{
	return -ENXIO;
}
static inline int msm_charger_unregister(struct msm_hardware_charger *hw_chg)
{
	return -ENXIO;
}
static inline int msm_charger_notify_event(struct msm_hardware_charger *hw_chg,
			     enum msm_hardware_charger_event event)
{
	return -ENXIO;
}
static inline void msm_charger_vbus_draw(unsigned int mA)
{
}
static inline int msm_charger_register_vbus_sn(void (*callback)(int))
{
	return -ENXIO;
}
static inline void msm_charger_unregister_vbus_sn(void (*callback)(int))
{
}
#endif
#endif /* __MSM_CHARGER_H__ */
