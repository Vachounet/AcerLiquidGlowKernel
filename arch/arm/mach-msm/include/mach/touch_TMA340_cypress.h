/* Header file for:
 * Cypress TrueTouch(TM) Standard Product touchscreen drivers.
 * arch/arm/mach-msm/include/mash/cyttsp_touch.h
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */


#ifndef __TOUCH_TMA340_CYPRESS_H__
#define __TOUCH_TMA340_CYPRESS_H__

#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <asm/mach-types.h>

#define CYPRESS_TS_NAME	"touch_cypress"
#define CYPRESS_KEY_NAME "capkey_cypress"
#define CY_I2C_NAME		"cyttsp-i2c-gpio"




#define CY_USE_I2C_DRIVER


#define CYTTSP_X_MAX                              480
#define CYTTSP_Y_MAX                              960


#define DISPLAY_PANEL_X_MAX                     	480

#define DISPLAY_PANEL_Y_MAX                     	830
#define CYTTSP_X_ORIGIN                           0
#define CYTTSP_Y_ORIGIN                           0
#define ECHOSTR_SIZE	20

#define NUM_OF_OP_MODE_SIZE	31


#define NUM_OF_SYS_INFO_SIZE	32



#define CY_USE_TIMER_DEBUG


#ifdef CY_USE_TIMER_DEBUG
	#define	TOUCHSCREEN_TIMEOUT	(msecs_to_jiffies(1000))
#else
	#define	TOUCHSCREEN_TIMEOUT	(msecs_to_jiffies(28))
#endif


#define CY_USE_DEEP_SLEEP_SEL		0x80
#define CY_USE_LOW_POWER_SEL		0x01

#ifdef CY_USE_TEST_DATA
	#define cyttsp_testdat(ray1, ray2, sizeofray) \
		{ \
			int i; \
			u8 *up1 = (u8 *)ray1; \
			u8 *up2 = (u8 *)ray2; \
			for (i = 0; i < sizeofray; i++) { \
				up1[i] = up2[i]; \
			} \
		}
#else
	#define cyttsp_testdat(xy, test_xy, sizeofray)
#endif 


#define GET_NUM_TOUCHES(x)		((x) & 0x0F)
#define GET_TOUCH1_ID(x)		(((x) & 0xF0) >> 4)
#define GET_TOUCH2_ID(x)		((x) & 0x0F)
#define GET_TOUCH3_ID(x)		(((x) & 0xF0) >> 4)
#define GET_TOUCH4_ID(x)		((x) & 0x0F)
#define IS_LARGE_AREA(x)		(((x) & 0x10) >> 4)
		


#define CY_NUM_MT_TCH_ID		4


#define CY_NUM_OF_CAP_KEY		1



#define CY_MAX_NUM_GPIO		2
#define CY_MAX_REPORT_ID	4	
#define CY_NUM_OF_VREG	    3

#define CY_NTCH				0	
#define CY_TCH				1	
#define CY_ST_FNGR1_IDX			0
#define CY_ST_FNGR2_IDX			1
#define CY_MT_TCH1_IDX			0
#define CY_MT_TCH2_IDX			1
#define CY_MT_TCH3_IDX			2
#define CY_MT_TCH4_IDX			3
#define CY_XPOS				0
#define CY_YPOS				1
#define CY_IGNR_TCH			(-1)
#define CY_SMALL_TOOL_WIDTH		10
#define CY_LARGE_TOOL_WIDTH		255
#define CY_REG_BASE				0x00
#define CY_REG_BL_VER			0x0F
#define CY_REG_cap_key			0x1B
#define CY_REG_FW_VER			0x1C
#define CY_REG_GEST_SET			0x1E
#define CY_REG_ACT_INTRVL		0x1D
#define CY_REG_TCH_TMOUT		(CY_REG_ACT_INTRVL+1)
#define CY_REG_LP_INTRVL		(CY_REG_TCH_TMOUT+1)
#define CY_SOFT_RESET			((1 << 0))
#define CY_DEEP_SLEEP			((1 << 1))
#define CY_LOW_POWER			((1 << 2))
#define CY_MAXZ				255
#define	CY_DLY_DFLT			10	
#define CY_DLY_SYSINFO			20	
#define CY_DLY_BL			300
#define CY_DLY_DNLOAD			100	
#define CY_NUM_RETRY			4	


#define CY_IDLE_STATE		0
#define CY_ACTIVE_STATE		1
#define CY_LOW_PWR_STATE		2
#define CY_SLEEP_STATE		3


#define CY_OP_MODE		0x00
#define CY_SYSINFO_MODE		0x10


#define CY_SOFT_RESET_MODE		0x01	
#define CY_DEEP_SLEEP_MODE		0x02
#define CY_LOW_PWR_MODE		0x04

enum cyttsp_pwr_state_t {
    IDLE_STATE = 0,
    ACTIVE_STATE,
    SLEEP_STATE,
	LOW_PWR_STATE
};

enum cyttsp_ts_state_t {
	TS_RELEASE = 0,
    TS_PRESS,
    TS_MOVE
};

enum cyttsp_key_state_t {
	KEY_RELEASE = 0,
    KEY_PRESS,
};

struct cyttsp_point_t {
    uint	x;
    uint    y;
	uint8_t	z;
	uint8_t	tch_id;
};

struct cyttsp_key_point_t {
    uint    x;
    uint    y;
};


struct cyttsp_touch_point_status_t 
{
    struct cyttsp_point_t  coord;
    enum cyttsp_ts_state_t state;
    uint8_t w;
};


struct cyttsp_cap_key_point_status_t 
{
    struct cyttsp_key_point_t  key_coord;
    enum cyttsp_key_state_t key_state;
};

struct cyttsp_platform_data {
	u8 use_sleep;
	bool wakeup;
	int rst_gpio;
	int irq_gpio;
	int sclk_gpio; 
	int sdata_gpio; 
};


struct cyttsp_op_mode_data_t {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	u16 x[4];
	u16 y[4];
	u8 z[4];
	u8 tch_id[4];
	u8 gest_cnt;
	u8 gest_id;
	u8 tt_undef[3];
	u8 gest_set;
	u8 tt_reserved;
};



struct cyttsp_sysinfo_data_t {
	u8 hst_mode;
	u8 mfg_cmd;
	u8 mfg_stat;
	u8 cid[3];
	u8 tt_undef1;
	u8 uid[8];
	u16 bl_ver;
	u16 tts_ver;
    u16 app_id;
	u16 app_ver;
	u8 tt_undef[6];
	u8 act_intrvl;
	u8 tch_tmout;
	u8 lp_intrvl;
};


#endif 
