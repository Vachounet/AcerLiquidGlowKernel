/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <asm/mach-types.h>
#include <mach/rpc_pmapp.h>
#include <mach/pm_log.h>
#include "board-msm7627a.h"
#include "devices-msm7x2xa.h"
#include "timer.h"

#define GPIO_WLAN_3V3_EN 119
static const char *id = "WLAN";


struct proc_dir_entry *q_wlan_entry;
unsigned char q_wlan_flag = 0;
EXPORT_SYMBOL(q_wlan_flag);


enum {
	WLAN_VREG_S3 = 0,
	WLAN_VREG_L17,
	WLAN_VREG_L19
};

struct wlan_vreg_info {
	const char *vreg_id;
	unsigned int level_min;
	unsigned int level_max;
	unsigned int pmapp_id;
	unsigned int is_vreg_pin_controlled;
	struct regulator *reg;
};

static struct wlan_vreg_info vreg_info[] = {
	{"msme1",     1800000, 1800000, 2,  0, NULL},
	{"bt",        3300000, 3300000, 21, 0, NULL},
	{"wlan4",     1800000, 1800000, 23, 0, NULL}
};

int gpio_wlan_sys_rest_en = 33;
static void gpio_wlan_config(void)
{
	if (machine_is_msm7627a_qrd1() || machine_is_msm7627a_evb())
		gpio_wlan_sys_rest_en = 124;
}

static unsigned int qrf6285_init_regs(void)
{
	struct regulator_bulk_data regs[ARRAY_SIZE(vreg_info)];
	int i = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		regs[i].supply = vreg_info[i].vreg_id;
		regs[i].min_uV = vreg_info[i].level_min;
		regs[i].max_uV = vreg_info[i].level_max;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);
	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		vreg_info[i].reg = regs[i].consumer;

out:
	return rc;
}

static unsigned int setup_wlan_gpio(bool on)
{
	int rc = 0;
	
	pr_info("+%s: wlan_rest_en:%d+\n", __func__, gpio_wlan_sys_rest_en);

	if (on) {
		rc = gpio_direction_output(gpio_wlan_sys_rest_en, 1);
		msleep(100);
	} else {
		gpio_set_value_cansleep(gpio_wlan_sys_rest_en, 0);
		rc = gpio_direction_input(gpio_wlan_sys_rest_en);
		msleep(100);
	}

	if (rc)
		pr_err("%s: WLAN sys_reset_en GPIO: Error", __func__);

	return rc;
}

#if 0
static unsigned int setup_wlan_clock(bool on)
{
	int rc = 0;

	if (on) {
		
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
					PMAPP_CLOCK_VOTE_ON);
	} else {
		
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
					 PMAPP_CLOCK_VOTE_OFF);
	}

	if (rc)
		pr_err("%s: Configuring A0 clock for WLAN: Error", __func__);

	return rc;
}
#endif
static unsigned int wlan_switch_regulators(int on)
{
	int rc = 0, index = 0;

	if (machine_is_msm7627a_qrd1())
		index = 2;

	for ( ; index < ARRAY_SIZE(vreg_info); index++) {
		if (on) {
			rc = regulator_set_voltage(vreg_info[index].reg,
						vreg_info[index].level_min,
						vreg_info[index].level_max);
			if (rc) {
				pr_err("%s:%s set voltage failed %d\n",
					__func__, vreg_info[index].vreg_id, rc);
				goto reg_disable;
			}

			rc = regulator_enable(vreg_info[index].reg);
			if (rc) {
				pr_err("%s:%s vreg enable failed %d\n",
					__func__, vreg_info[index].vreg_id, rc);
				goto reg_disable;
			}

			if (vreg_info[index].is_vreg_pin_controlled) {
				rc = pmapp_vreg_lpm_pincntrl_vote(id,
						vreg_info[index].pmapp_id,
						PMAPP_CLOCK_ID_A0, 1);
				if (rc) {
					pr_err("%s:%s pincntrl failed %d\n",
						__func__,
						vreg_info[index].vreg_id, rc);
					goto pin_cnt_fail;
				}
			}
		} else {
			if (vreg_info[index].is_vreg_pin_controlled) {
				rc = pmapp_vreg_lpm_pincntrl_vote(id,
						vreg_info[index].pmapp_id,
						PMAPP_CLOCK_ID_A0, 0);
				if (rc) {
					pr_err("%s:%s pincntrl failed %d\n",
						__func__,
						vreg_info[index].vreg_id, rc);
					goto pin_cnt_fail;
				}
			}

			rc = regulator_disable(vreg_info[index].reg);
			if (rc) {
				pr_err("%s:%s vreg disable failed %d\n",
					__func__,
					vreg_info[index].vreg_id, rc);
				goto reg_disable;
			}
		}
	}
	return 0;
pin_cnt_fail:
	if (on)
		regulator_disable(vreg_info[index].reg);
reg_disable:
	if (!machine_is_msm7627a_qrd1()) {
		while (index) {
			if (on) {
				index--;
				regulator_disable(vreg_info[index].reg);
				regulator_put(vreg_info[index].reg);
			}
		}
	}
	return rc;
}

static unsigned int msm_AR600X_setup_power(bool on)
{
	int rc = 0;
	static bool init_done;

	if (unlikely(!init_done)) {
		gpio_wlan_config();
		rc = qrf6285_init_regs();
		if (rc) {
			pr_err("%s: qrf6285 init failed = %d\n", __func__, rc);
			return rc;
		} else {
			init_done = true;
		}
	}

	rc = wlan_switch_regulators(on);
	if (rc) {
		pr_err("%s: wlan_switch_regulators error = %d\n", __func__, rc);
		goto out;
	}

	
	if (machine_is_msm7627a_qrd1()) {
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_3V3_EN, 0,
					GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s gpio_tlmm_config %d failed,error = %d\n",
				__func__, rc, GPIO_WLAN_3V3_EN);
			goto reg_disable;
		}
		gpio_set_value(GPIO_WLAN_3V3_EN, 1);
	}

	
	if (machine_is_msm7627a_qrd1() || machine_is_msm7627a_evb()) {
		rc = gpio_tlmm_config(GPIO_CFG(gpio_wlan_sys_rest_en, 0,
					GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s gpio_tlmm_config %d failed,error = %d\n",
				__func__, gpio_wlan_sys_rest_en, rc);
			goto qrd_gpio_fail;
		}
		gpio_set_value(gpio_wlan_sys_rest_en, 1);
	} else {
		rc = gpio_request(gpio_wlan_sys_rest_en, "WLAN_DEEP_SLEEP_N");
		if (rc) {
			pr_err("%s: WLAN sys_rest_en GPIO %d request failed %d\n",
				__func__,
				gpio_wlan_sys_rest_en, rc);
			goto qrd_gpio_fail;
		}
		rc = setup_wlan_gpio(on);
		if (rc) {
			pr_err("%s: wlan_set_gpio = %d\n", __func__, rc);
			goto gpio_fail;
		}
	}
#if 0
	
	rc = setup_wlan_clock(on);
	if (rc) {
		pr_err("%s: setup_wlan_clock = %d\n", __func__, rc);
		goto set_gpio_fail;
	}

	
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
				 PMAPP_CLOCK_VOTE_PIN_CTRL);
	if (rc) {
		pr_err("%s: Configuring A0 to Pin controllable failed %d\n",
				__func__, rc);
		goto set_clock_fail;
	}
#endif
	pr_info("WLAN power-up success\n");
	return 0;
#if 0
set_clock_fail:
	setup_wlan_clock(0);
set_gpio_fail:
	setup_wlan_gpio(0);
#endif
gpio_fail:
	gpio_free(gpio_wlan_sys_rest_en);
qrd_gpio_fail:
	gpio_free(GPIO_WLAN_3V3_EN);
reg_disable:
	wlan_switch_regulators(0);
out:
	pr_info("WLAN power-up failed\n");
	return rc;
}

static unsigned int msm_AR600X_shutdown_power(bool on)
{
	int rc = 0;

#if 0
	
	rc = setup_wlan_clock(on);
	if (rc) {
		pr_err("%s: setup_wlan_clock = %d\n", __func__, rc);
		goto set_clock_fail;
	}
#endif
	
	if (machine_is_msm7627a_qrd1() || machine_is_msm7627a_evb()) {
		rc = gpio_tlmm_config(GPIO_CFG(gpio_wlan_sys_rest_en, 0,
					GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s gpio_tlmm_config %d failed,error = %d\n",
			__func__, GPIO_WLAN_3V3_EN, rc);
			goto gpio_fail;
		}
		gpio_set_value(gpio_wlan_sys_rest_en, 0);
	} else {
		rc = setup_wlan_gpio(on);
		if (rc) {
			pr_err("%s: wlan_set_gpio = %d\n", __func__, rc);
			goto set_gpio_fail;
		}
		gpio_free(gpio_wlan_sys_rest_en);
	}

	
	if (machine_is_msm7627a_qrd1()) {
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_3V3_EN, 0,
					GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s gpio_tlmm_config %d failed,error = %d\n",
				__func__, GPIO_WLAN_3V3_EN, rc);
			goto qrd_gpio_fail;
		}
		gpio_set_value(GPIO_WLAN_3V3_EN, 0);
	}

	rc = wlan_switch_regulators(on);
	if (rc) {
		pr_err("%s: wlan_switch_regulators error = %d\n",
			__func__, rc);
		goto reg_disable;
	}

	pr_info("WLAN power-down success\n");
	return 0;
#if 0
set_clock_fail:
	setup_wlan_clock(0);
#endif
set_gpio_fail:
	setup_wlan_gpio(0);
gpio_fail:
	gpio_free(gpio_wlan_sys_rest_en);
qrd_gpio_fail:
	gpio_free(GPIO_WLAN_3V3_EN);
reg_disable:
	wlan_switch_regulators(0);
	pr_info("WLAN power-down failed\n");
	return rc;
}

static int q_proc_call(char *buf, char **start, off_t off,
        int count, int *eof, void *data)
{
    int len = 0;
    if (q_wlan_flag == 0)                                                                                                                                                   
        len = sprintf(buf + len, "%s", "0\n");
    else if (q_wlan_flag == 1)
        len = sprintf(buf + len, "%s", "1\n");
    else
        len = sprintf(buf + len, "%s", "0\n");
    
    return len;
}

int  ar600x_wlan_power(bool on)
{
	int rec = 0;
	if (on) {
		rec = msm_AR600X_setup_power(on);
		if (!rec) {
			q_wlan_entry = create_proc_read_entry("q_wlan", 0, NULL, q_proc_call, NULL);
			if (!q_wlan_entry)
				pr_info("cl: unable to create proc file.\n");
			else 
				pr_info("cl: created proc file.\n");
		}
		PM_LOG_EVENT(PM_LOG_ON, PM_LOG_WIFI);
	} else {
		rec = msm_AR600X_shutdown_power(on);
		if (!rec) {
			pr_info("cl: remove q_wlan proc file.\n");
			remove_proc_entry("q_wlan", NULL);
			q_wlan_flag = 0;
		}
		PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_WIFI);
	}
	return 0;
}

