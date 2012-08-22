/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include "mt9p017.h"
#include <linux/debugfs.h>

extern struct dentry *kernel_debuglevel_dir;
static unsigned int CAM_MT9P017_DLL=0;  

#define CAM_PRINTK(level, fmt, args...) if(level <= CAM_MT9P017_DLL) printk("%s:" fmt, __func__, ##args);  

#define WOLF_AF 


#if 1
#ifdef CDBG
#undef CDBG
#endif
#define CDBG(fmt, args...) printk(KERN_INFO "[CAM]" fmt, ##args);
#endif




#define REG_GROUPED_PARAMETER_HOLD		0x0104
#define GROUPED_PARAMETER_HOLD_OFF		0x00
#define GROUPED_PARAMETER_HOLD			0x01

#define REG_COARSE_INTEGRATION_TIME		0x3012

#define REG_GLOBAL_GAIN	0x305E

#define REG_FRAME_LENGTH_LINES		0x300A

#define REG_TEST_PATTERN_MODE			0x0601
#define REG_VCM_NEW_CODE			0x30F2




#define Q8	0x00000100
#define Q10	0x00000400
#define MT9P017_MASTER_CLK_RATE 24000000

#ifndef WOLF_AF

#define MT9P017_TOTAL_STEPS_NEAR_TO_FAR    32
uint16_t mt9p017_nl_region_code_per_step1;
uint16_t mt9p017_l_region_code_per_step = 4;
#else
#define MT9P017_TOTAL_STEPS_NEAR_TO_FAR    49
uint16_t mt9p017_l_region_boundary1 = 41;
uint16_t mt9p017_nl_region_code_per_step1 = 0;
uint16_t mt9p017_l_region_code_per_step = 4;
#endif

uint16_t mt9p017_step_position_table[MT9P017_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t mt9p017_nl_region_boundary1;

uint16_t mt9p017_damping_threshold = 10;
uint16_t mt9p017_sw_damping_time_wait = 1;

struct mt9p017_work_t {
	struct work_struct work;
};

static struct mt9p017_work_t *mt9p017_sensorw;
static struct i2c_client *mt9p017_client;

struct mt9p017_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;
	uint32_t pict_fps_divider;
	uint16_t fps;

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum mt9p017_resolution_t prev_res;
	enum mt9p017_resolution_t pict_res;
	enum mt9p017_resolution_t curr_res;
	enum mt9p017_test_mode_t  set_test;
};


static bool CSI_CONFIG;

#if 0
#ifdef LENS_SHADING
static bool readOTP = false;
#define OTP_LENGTH 106

#define OTP_INDEX 0
#define LSC_INDEX 1

uint16_t otp_data0[OTP_LENGTH] = {0};
uint16_t otp_data1[OTP_LENGTH] = {0};
uint16_t otp_data2[OTP_LENGTH] = {0};



static uint16_t otp_lsc_map[OTP_LENGTH][2]=
{
    { 0x3800,0x3600 }, { 0x3802,0x3602 }, { 0x3804,0x3604 }, { 0x3806,0x3606 }, 
    { 0x3808,0x3608 }, { 0x380A,0x360A }, { 0x380C,0x360C }, { 0x380E,0x360E },
    { 0x3810,0x3610 }, { 0x3812,0x3612 }, { 0x3814,0x3614 }, { 0x3816,0x3616 },
    { 0x3818,0x3618 }, { 0x381A,0x361A }, { 0x381C,0x361C }, { 0x381E,0x361E },
    { 0x3820,0x3620 }, { 0x3822,0x3622 }, { 0x3824,0x3624 }, { 0x3826,0x3626 },
    { 0x3828,0x3640 }, { 0x382A,0x3642 }, { 0x382C,0x3644 }, { 0x382E,0x3646 },
    { 0x3830,0x3648 }, { 0x3832,0x364A }, { 0x3834,0x364C }, { 0x3836,0x364E },
    { 0x3838,0x3650 }, { 0x383A,0x3652 }, { 0x383C,0x3654 }, { 0x383E,0x3656 },
    { 0x3840,0x3658 }, { 0x3842,0x365A }, { 0x3844,0x365C }, { 0x3846,0x365E },
    { 0x3848,0x3660 }, { 0x384A,0x3662 }, { 0x384C,0x3664 }, { 0x384E,0x3666 },
    { 0x3850,0x3680 }, { 0x3852,0x3682 }, { 0x3854,0x3684 }, { 0x3856,0x3686 },
    { 0x3858,0x3688 }, { 0x385A,0x368A }, { 0x385C,0x368C }, { 0x385E,0x368E },
    { 0x3860,0x3690 }, { 0x3862,0x3692 }, { 0x3864,0x3694 }, { 0x3866,0x3696 },
    { 0x3868,0x3698 }, { 0x386A,0x369A }, { 0x386C,0x369C }, { 0x386E,0x369E },
    { 0x3870,0x36A0 }, { 0x3872,0x36A2 }, { 0x3874,0x36A4 }, { 0x3876,0x36A6 },
    { 0x3878,0x36C0 }, { 0x387A,0x36C2 }, { 0x387C,0x36C4 }, { 0x387E,0x36C6 },
    { 0x3880,0x36C8 }, { 0x3882,0x36CA }, { 0x3884,0x36CC }, { 0x3886,0x36CE },
    { 0x3888,0x36D0 }, { 0x388A,0x36D2 }, { 0x388C,0x36D4 }, { 0x388E,0x36D6 },
    { 0x3890,0x36D8 }, { 0x3892,0x36DA }, { 0x3894,0x36DC }, { 0x3896,0x36DE },
    { 0x3898,0x36E0 }, { 0x389A,0x36E2 }, { 0x389C,0x36E4 }, { 0x389E,0x36E6 },
    { 0x38A0,0x3700 }, { 0x38A2,0x3702 }, { 0x38A4,0x3704 }, { 0x38A6,0x3706 },
    { 0x38A8,0x3708 }, { 0x38AA,0x370A }, { 0x38AC,0x370C }, { 0x38AE,0x370E },
    { 0x38B0,0x3710 }, { 0x38B2,0x3712 }, { 0x38B4,0x3714 }, { 0x38B6,0x3716 },
    { 0x38B8,0x3718 }, { 0x38BA,0x371A }, { 0x38BC,0x371C }, { 0x38BE,0x371E },
    { 0x38C0,0x3720 }, { 0x38C2,0x3722 }, { 0x38C4,0x3724 }, { 0x38C6,0x3726 },
    { 0x38C8,0x3782 }, { 0x38CA,0x3784 }, { 0x38CC,0x37C0 }, { 0x38CE,0x37C2 },
    { 0x38D0,0x37C4 }, { 0x38D2,0x37C6 },
};
#endif
#endif

static void cam_create_kernel_debuglevel(void)
{
	CAM_PRINTK(1, "create kernel debuglevel!!!\n");

	
	if (kernel_debuglevel_dir!=NULL)
	{
	  debugfs_create_u32("cam_mt9p017_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&CAM_MT9P017_DLL));
	}
	else
	{
		printk(KERN_ERR "kernel_debuglevel_dir ts_dl Fail\n");
	}

}


static void cam_destroy_kernel_debuglevel(void)
{
	CAM_PRINTK(1, "destroy kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
} 


static struct mt9p017_ctrl_t *mt9p017_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9p017_wait_queue);
DEFINE_MUTEX(mt9p017_mut);

static int cam_debug_init(void);
static struct dentry *debugfs_base;


static int mt9p017_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(mt9p017_client->adapter, msgs, 2) < 0) {
		CDBG("mt9p017_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t mt9p017_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(mt9p017_client->adapter, msg, 1) < 0) {
		CDBG("mt9p017_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t mt9p017_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = mt9p017_i2c_rxdata(mt9p017_client->addr, buf, rlen);
	if (rc < 0) {
		CDBG("mt9p017_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	
	return rc;
}

static int32_t mt9p017_i2c_write_w_sensor(unsigned short waddr, uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	
	rc = mt9p017_i2c_txdata(mt9p017_client->addr, buf, 4);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t mt9p017_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	
	rc = mt9p017_i2c_txdata(mt9p017_client->addr, buf, 3);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t mt9p017_i2c_write_w_table(struct mt9p017_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = mt9p017_i2c_write_w_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static void mt9p017_group_hold_on(void)
{
	mt9p017_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
}

static void mt9p017_group_hold_off(void)
{
	mt9p017_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
}

static void mt9p017_start_stream(void)
{



	mt9p017_i2c_write_b_sensor(0x0104, 0x00);
	mt9p017_i2c_write_w_sensor(0x301A, 0x065C|0x2);
}

static void mt9p017_stop_stream(void)
{
	
	
	mt9p017_i2c_write_b_sensor(0x0104, 0x01);
}

static void mt9p017_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	
	uint32_t divider, d1, d2;

	d1 = mt9p017_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata
		* 0x00000400/
		mt9p017_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	d2 = mt9p017_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata
		* 0x00000400/
		mt9p017_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	divider = d1 * d2 / 0x400;

	
	*pfps = (uint16_t) (fps * divider / 0x400);
	
}

static uint16_t mt9p017_get_prev_lines_pf(void)
{
	if (mt9p017_ctrl->prev_res == QTR_SIZE)
		return mt9p017_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->prev_res == FULL_SIZE)
		return mt9p017_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->prev_res == HFR_60FPS)
		return mt9p017_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->prev_res == HFR_90FPS)
		return mt9p017_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		return mt9p017_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9p017_get_prev_pixels_pl(void)
{
	if (mt9p017_ctrl->prev_res == QTR_SIZE)
		return mt9p017_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9p017_ctrl->prev_res == FULL_SIZE)
		return mt9p017_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9p017_ctrl->prev_res == HFR_60FPS)
		return mt9p017_regs.reg_60fps[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9p017_ctrl->prev_res == HFR_90FPS)
		return mt9p017_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
	else
		return mt9p017_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
}

static uint16_t mt9p017_get_pict_lines_pf(void)
{
	if (mt9p017_ctrl->pict_res == QTR_SIZE)
		return mt9p017_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->pict_res == FULL_SIZE)
		return mt9p017_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->pict_res == HFR_60FPS)
		return mt9p017_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->pict_res == HFR_90FPS)
		return mt9p017_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		return mt9p017_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9p017_get_pict_pixels_pl(void)
{
	if (mt9p017_ctrl->pict_res == QTR_SIZE)
		return mt9p017_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9p017_ctrl->pict_res == FULL_SIZE)
		return mt9p017_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9p017_ctrl->pict_res == HFR_60FPS)
		return mt9p017_regs.reg_60fps[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9p017_ctrl->pict_res == HFR_90FPS)
		return mt9p017_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
	else
		return mt9p017_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
}

static uint32_t mt9p017_get_pict_max_exp_lc(void)
{
	if (mt9p017_ctrl->pict_res == QTR_SIZE)
		return mt9p017_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9p017_ctrl->pict_res == FULL_SIZE)
		return mt9p017_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9p017_ctrl->pict_res == HFR_60FPS)
		return mt9p017_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9p017_ctrl->pict_res == HFR_90FPS)
		return mt9p017_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else
		return mt9p017_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
}

static int32_t mt9p017_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	if (mt9p017_ctrl->curr_res == QTR_SIZE)
		total_lines_per_frame =
		mt9p017_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->curr_res == FULL_SIZE)
		total_lines_per_frame =
		mt9p017_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->curr_res == HFR_60FPS)
		total_lines_per_frame =
		mt9p017_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9p017_ctrl->curr_res == HFR_90FPS)
		total_lines_per_frame =
		mt9p017_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		total_lines_per_frame =
		mt9p017_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;

	mt9p017_ctrl->fps_divider = fps->fps_div;
	mt9p017_ctrl->pict_fps_divider = fps->pict_fps_div;

	if (mt9p017_ctrl->curr_res == FULL_SIZE) {
		total_lines_per_frame = (uint16_t)
		(total_lines_per_frame * mt9p017_ctrl->pict_fps_divider/0x400);
	} else {
		total_lines_per_frame = (uint16_t)
		(total_lines_per_frame * mt9p017_ctrl->fps_divider/0x400);
	}

	mt9p017_group_hold_on();
	rc = mt9p017_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
							total_lines_per_frame);
	mt9p017_group_hold_off();
	return rc;
}

static int32_t mt9p017_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0xE7F;
	int32_t rc = 0;
	if (gain > max_legal_gain) {
		CDBG("Max legal gain Line:%d\n", __LINE__);
		gain = max_legal_gain;
	}

	if (mt9p017_ctrl->curr_res != FULL_SIZE) {
		mt9p017_ctrl->my_reg_gain = gain;
		mt9p017_ctrl->my_reg_line_count = (uint16_t) line;
		line = (uint32_t) (line * mt9p017_ctrl->fps_divider /
						   0x00000400);
	} else {
		line = (uint32_t) (line * mt9p017_ctrl->pict_fps_divider /
						   0x00000400);
	}

	gain |= 0x1000;

	mt9p017_group_hold_on();
	rc = mt9p017_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain);
	rc = mt9p017_i2c_write_w_sensor(REG_COARSE_INTEGRATION_TIME, line);
	mt9p017_group_hold_off();
	return rc;
}

static int32_t mt9p017_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = mt9p017_write_exp_gain(gain, line);
	mt9p017_i2c_write_w_sensor(0x301A, 0x065C|0x2);
	return rc;
}

#define DIV_CEIL(x, y) (x/y + ((x%y) ? 1 : 0))

static int32_t mt9p017_move_focus(int direction,
	int32_t num_steps)
{
      
	int16_t step_direction, dest_lens_position, dest_step_position;
	int16_t target_dist, small_step, next_lens_position;
	
       CDBG("%s++, step_direction:%d, num_steps:%d", __func__, direction, num_steps);
	if (direction == MOVE_NEAR)
		step_direction = 1;
	else
		step_direction = -1;

	dest_step_position = mt9p017_ctrl->curr_step_pos
						+ (step_direction * num_steps);

	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > MT9P017_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_position = MT9P017_TOTAL_STEPS_NEAR_TO_FAR;

	if (dest_step_position == mt9p017_ctrl->curr_step_pos)
		return 0;

	dest_lens_position = mt9p017_step_position_table[dest_step_position];
	target_dist = step_direction *
		(dest_lens_position - mt9p017_ctrl->curr_lens_pos);

	if (step_direction < 0 && (target_dist >=
		mt9p017_step_position_table[mt9p017_damping_threshold])) {
		small_step = DIV_CEIL(target_dist, 10);
		mt9p017_sw_damping_time_wait = 10;
	} else {
		small_step = DIV_CEIL(target_dist, 4);
		mt9p017_sw_damping_time_wait = 4;
	}

      CDBG("%s, dest_lens_position:%d, target_dist:%d, small_step:%d, sw_damping_time_wait:%d", __func__, dest_step_position,target_dist, small_step, mt9p017_sw_damping_time_wait );
      if(small_step ==0 )
      	{
            small_step = 1;
      }
      
	for (next_lens_position = mt9p017_ctrl->curr_lens_pos
		+ (step_direction * small_step);
		(step_direction * next_lens_position) <=
		(step_direction * dest_lens_position);
		next_lens_position += (step_direction * small_step)) {
		mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		next_lens_position);
		mt9p017_ctrl->curr_lens_pos = next_lens_position;
		usleep(mt9p017_sw_damping_time_wait*1000);
	}

	if (mt9p017_ctrl->curr_lens_pos != dest_lens_position) {
		mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		dest_lens_position);
		usleep(mt9p017_sw_damping_time_wait*1000);
	}
	mt9p017_ctrl->curr_lens_pos = dest_lens_position;
	mt9p017_ctrl->curr_step_pos = dest_step_position;

	CDBG("%s--", __func__);
	return 0;
}

static int32_t mt9p017_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;
	if (mt9p017_ctrl->curr_step_pos != 0) {
		rc = mt9p017_move_focus(MOVE_FAR,
		mt9p017_ctrl->curr_step_pos);
	} else {
		mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE, 0x00);
	}

	mt9p017_ctrl->curr_lens_pos = 0;
	mt9p017_ctrl->curr_step_pos = 0;

	return rc;
}
static void mt9p017_enable_focus(int on)
{
    int rc;
    static struct vreg *vreg_vcm_2p8;

    if(on)
    {
	if (vreg_vcm_2p8 == NULL) {
		vreg_vcm_2p8 = vreg_get(NULL, "usim2"); 
		if (IS_ERR(vreg_vcm_2p8)) {
			printk("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "usim2", PTR_ERR(vreg_vcm_2p8));
			return;
		}
		rc = vreg_set_level(vreg_vcm_2p8, 2800);
		if (rc) {
			printk("%s: vreg_vcm_2p8 set_level failed (%d)\n",
				__func__, rc);
		}

		vreg_enable(vreg_vcm_2p8);
	}

    }
    else
    {
             rc = vreg_disable(vreg_vcm_2p8);
             if(rc){
			pr_err("%s: rfrx1 disable failed (%d)\n",
				__func__, rc);
    
             }
             vreg_vcm_2p8 = NULL;

    }

}

static int mt9p017_enable_flash(int on)
{
    int rc;

    printk("%s, mode: %d\n", __func__, on);

    if(on)
    {
	rc = gpio_request(13, "FL_EN");
	if (rc < 0) {
		printk("%s, request gpio failed!\n", __func__);
		return -rc;
	}
	gpio_direction_output(13, 1);	
		
    }
    else
    {
       gpio_direction_output(13, 0);
       gpio_free(13);
    }

    return 0;
}



static void mt9p017_init_focus(void)
{
#ifndef WOLF_AF
	uint8_t i;
	mt9p017_step_position_table[0] = 0;
	for (i = 1; i <= MT9P017_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		if (i <= mt9p017_nl_region_boundary1) {
			mt9p017_step_position_table[i] =
				mt9p017_step_position_table[i-1]
				+ mt9p017_nl_region_code_per_step1;
		} else {
			mt9p017_step_position_table[i] =
				mt9p017_step_position_table[i-1]
				+ mt9p017_l_region_code_per_step;
		}

		if (mt9p017_step_position_table[i] > 255)
			mt9p017_step_position_table[i] = 255;
	}
#else
      uint8_t i;
	mt9p017_step_position_table[0] = 0;
       mt9p017_step_position_table[1] = (uint16_t)(32*(1.0/3.0)); 
       mt9p017_step_position_table[2] = (uint16_t)(32*(2.0/3.0)); 
       mt9p017_step_position_table[3] = (uint16_t) 32; 

	for (i = 4; i <= MT9P017_TOTAL_STEPS_NEAR_TO_FAR; i++) {

		
		if (i <= mt9p017_nl_region_boundary1) {
			mt9p017_step_position_table[i] =
				mt9p017_step_position_table[i-1]
				+ mt9p017_nl_region_code_per_step1;
		} 
		else if( i<= mt9p017_l_region_boundary1)
		{
			mt9p017_step_position_table[i] =
				mt9p017_step_position_table[i-1]
				+ mt9p017_l_region_code_per_step;
		}
		else
		
			mt9p017_step_position_table[i] = 255;
	}
       
#endif
}

static int32_t mt9p017_test(enum mt9p017_test_mode_t mo)
{
	int32_t rc = 0;
	if (mo == TEST_OFF)
		return rc;
	else {
		
		if (mt9p017_i2c_write_b_sensor(REG_TEST_PATTERN_MODE,
			(uint8_t) mo) < 0) {
			return rc;
		}
	}
	return rc;
}


#if 0
static void mt9p017_read_otp(uint16_t type)
{
  uint16_t bitRead = 0;
  uint16_t *otp = 0;
  uint16_t i = 0, j = 0;

  
  
  

    switch(type){
        case 0x3000:  
            otp = otp_data0;
            break;
        case 0x3100:  
            otp = otp_data1;
            break;
        case 0x3200:  
            otp = otp_data2;
            break;
        default:
            otp = otp_data0;
    }

  mt9p017_i2c_write_w_sensor(0x301A, 0x0020);
  mt9p017_i2c_write_w_sensor(0x301A, 0x0610);
  mt9p017_i2c_write_w_sensor(0x3134,0xCD95);

#if 0
  mt9p017_i2c_write_w_sensor(0x0103, 0x0001);
  mt9p017_i2c_read(0x0103, &bitRead, 2);
  printk("register:0x0103, value is 0x%x\n", bitRead);

  mt9p017_i2c_write_w_sensor(0x3052, 0x2704);
  mt9p017_i2c_read(0x3052, &bitRead, 2);
  printk("register:0x3052, value is 0x%x\n", bitRead);

  mt9p017_i2c_read(0x3054, &bitRead, 2);
  mt9p017_i2c_write_w_sensor(0x3054, bitRead|0x0100);
  mt9p017_i2c_read(0x3054, &bitRead, 2);
  printk("register:0x3050, value is 0x%x\n", bitRead);
#endif

  mt9p017_i2c_write_w_sensor(0x304c, type);
  mt9p017_i2c_write_w_sensor(0x304A, 0x0010);

  for(i = 0; i<10; i++)
  {
       mt9p017_i2c_read(0x304A, &bitRead, 2);
        printk("%s, bitRead:0x%x\n", __func__, bitRead);
       if( 0x60 & bitRead)
       {
           CDBG("%s, bitRead == 0x60", __func__);
           for (j = 0; j < OTP_LENGTH; j++)
           {
               mt9p017_i2c_read(otp_lsc_map[j][OTP_INDEX],&otp[j], 2);
               printk(" register: 0x%x, Value : 0x%x\n ", otp_lsc_map[j][OTP_INDEX], otp[j] );
           }
           break;
      }

     usleep(10000);


  }
}

static void mt9p017_set_lensRollOff( int type)
{
    uint16_t* pOTP = 0;
    
   uint16_t i = 0;
   return;
    
    if(!readOTP)
    {
       
       mt9p017_read_otp(0x3100);
       

        readOTP = true;
    }
    
    switch(type)
    {
    	case 1: 
       	pOTP = otp_data0;
    		break;

    	case 2: 
      	 	pOTP = otp_data1;
    		break;

    	case 3: 
      		 pOTP = otp_data2;
    		break;

    	default:
       	 pOTP = otp_data1;
    }

    mt9p017_i2c_write_w_sensor(0x3780, 0x0000); 

    for(i =0; i<(OTP_LENGTH -6); i++)
    {
	mt9p017_i2c_write_w_sensor((0x3600+i*2) , pOTP[i]);
    }
    mt9p017_i2c_write_w_sensor(0x3782, pOTP[100]);
    mt9p017_i2c_write_w_sensor(0x3784, pOTP[101]);
    mt9p017_i2c_write_w_sensor(0x37C0, pOTP[102]);
    mt9p017_i2c_write_w_sensor(0x37C2, pOTP[103]);
    mt9p017_i2c_write_w_sensor(0x37C4, pOTP[104]);
    mt9p017_i2c_write_w_sensor(0x37C6, pOTP[105]);

   mt9p017_i2c_write_w_sensor(0x3780, 0x8000); 
}
#endif

static int32_t mt9p017_sensor_setting(int update_type, int rt)
{

	int32_t rc = 0;
	uint16_t bitRead = 0;
	struct msm_camera_csi_params mt9p017_csi_params;
	uint8_t stored_af_step = 0;
		CDBG("%s+, update_type = %d, rt = %d\n",__func__,update_type,rt);  
	stored_af_step = mt9p017_ctrl->curr_step_pos;

	 mt9p017_i2c_read(0x304A, &bitRead, 2);
        printk("%s, bitRead:0x%x\n", __func__, bitRead);
	mt9p017_set_default_focus(0);
	mt9p017_stop_stream();
	msleep(15);
	if (update_type == REG_INIT) {
		mt9p017_i2c_write_w_table(mt9p017_regs.reg_mipi,
			mt9p017_regs.reg_mipi_size);
		mt9p017_i2c_write_w_table(mt9p017_regs.rec_settings,
			mt9p017_regs.rec_size);

	
		cam_debug_init();
		CSI_CONFIG = 0;
	} else if (update_type == UPDATE_PERIODIC) {
		if (rt == QTR_SIZE) {
			mt9p017_i2c_write_w_table(mt9p017_regs.reg_pll,
				mt9p017_regs.reg_pll_size);

		       mt9p017_i2c_write_w_table(mt9p017_regs.reg_prev,
				mt9p017_regs.reg_prev_size);
						
		} else if (rt == FULL_SIZE) {
			mt9p017_i2c_write_w_table(mt9p017_regs.reg_pll,
				mt9p017_regs.reg_pll_size);

			mt9p017_i2c_write_w_table(mt9p017_regs.reg_snap,
				mt9p017_regs.reg_snap_size);
		} else if (rt == HFR_60FPS) {
			mt9p017_i2c_write_w_table(mt9p017_regs.reg_pll_120fps,
				mt9p017_regs.reg_pll_120fps_size);
			mt9p017_i2c_write_w_sensor(0x0306, 0x0029);
			mt9p017_i2c_write_w_table(mt9p017_regs.reg_120fps,
				mt9p017_regs.reg_120fps_size);
		} else if (rt == HFR_90FPS) {
			mt9p017_i2c_write_w_table(mt9p017_regs.reg_pll_120fps,
				mt9p017_regs.reg_pll_120fps_size);
			mt9p017_i2c_write_w_sensor(0x0306, 0x003D);
			mt9p017_i2c_write_w_table(mt9p017_regs.reg_120fps,
				mt9p017_regs.reg_120fps_size);
		} else if (rt == HFR_120FPS) {
			msm_camio_vfe_clk_rate_set(266667000);
			mt9p017_i2c_write_w_table(mt9p017_regs.reg_pll_120fps,
				mt9p017_regs.reg_pll_120fps_size);
			mt9p017_i2c_write_w_table(mt9p017_regs.reg_120fps,
				mt9p017_regs.reg_120fps_size);
		}
		if (!CSI_CONFIG) {
			msm_camio_vfe_clk_rate_set(192000000);
			mt9p017_csi_params.data_format = CSI_10BIT;
			mt9p017_csi_params.lane_cnt = 2;
			mt9p017_csi_params.lane_assign = 0xe4;
			mt9p017_csi_params.dpcm_scheme = 0;
			mt9p017_csi_params.settle_cnt = 0x14;
			rc = msm_camio_csi_config(&mt9p017_csi_params);
			msleep(10);
			CSI_CONFIG = 1;
		}
		mt9p017_move_focus(MOVE_NEAR, stored_af_step);
		mt9p017_start_stream();

		#ifdef LENS_SHADING
		
		#endif
	}
	return rc;
}

static int32_t mt9p017_video_config(int mode)
{

	int32_t rc = 0;

	CDBG("video config\n");
	
	if (mt9p017_sensor_setting(UPDATE_PERIODIC,
			mt9p017_ctrl->prev_res) < 0)
		return rc;
	if (mt9p017_ctrl->set_test) {
		if (mt9p017_test(mt9p017_ctrl->set_test) < 0)
			return  rc;
	}

	mt9p017_ctrl->curr_res = mt9p017_ctrl->prev_res;
	mt9p017_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9p017_snapshot_config(int mode)
{
	int32_t rc = 0;
	
	if (mt9p017_ctrl->curr_res != mt9p017_ctrl->pict_res) {
		if (mt9p017_sensor_setting(UPDATE_PERIODIC,
				mt9p017_ctrl->pict_res) < 0)
			return rc;
	}

	mt9p017_ctrl->curr_res = mt9p017_ctrl->pict_res;
	mt9p017_ctrl->sensormode = mode;
	return rc;
} 

static int32_t mt9p017_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	
	if (mt9p017_ctrl->curr_res != mt9p017_ctrl->pict_res) {
		if (mt9p017_sensor_setting(UPDATE_PERIODIC,
				mt9p017_ctrl->pict_res) < 0)
			return rc;
	}

	mt9p017_ctrl->curr_res = mt9p017_ctrl->pict_res;
	mt9p017_ctrl->sensormode = mode;
	return rc;
} 

static int32_t mt9p017_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;

	CDBG("%s+, mode = %d\n",__func__,mode);  
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
	case SENSOR_HFR_60FPS_MODE:
	case SENSOR_HFR_90FPS_MODE:
	case SENSOR_HFR_120FPS_MODE:
		mt9p017_ctrl->prev_res = res;
		rc = mt9p017_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		mt9p017_ctrl->pict_res = res;
		rc = mt9p017_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		mt9p017_ctrl->pict_res = res;
		rc = mt9p017_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	CDBG("%s-, rc = %d\n",__func__,rc); 
	return rc;
}

static int32_t mt9p017_power_down(void)
{
      	mt9p017_enable_flash(false);
       mt9p017_enable_focus(false);

       CDBG("%s\n",__func__);
	return 0;
}

static int mt9p017_probe_init_done(const struct msm_camera_sensor_info *data)
{
	CDBG("probe done\n");
#if 1
	gpio_set_value_cansleep(data->sensor_reset, 0);
	gpio_direction_input(data->sensor_reset);
	gpio_free(data->sensor_reset);
#endif
	return 0;
}

static int mt9p017_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	uint16_t revid = 0;
	CDBG("%s: %d\n", __func__, __LINE__);

       rc = gpio_request(data->sensor_reset, "mt9p017");
	CDBG(" mt9p017_probe_init_sensor\n");
	if (!rc) {
		
		gpio_tlmm_config(GPIO_CFG(37, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
		CDBG("sensor_reset = %d\n", rc);
		gpio_direction_output(data->sensor_reset, 0);
		msleep(10);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(10);
      	} else {
		goto init_probe_done;
	}
    
	CDBG(" mt9p017_probe_init_sensor is called\n");
	rc = mt9p017_i2c_read(0x0000, &chipid, 2);
	CDBG("ID: %d\n", chipid);
	
	if (chipid != 0x4800) {
		rc = -ENODEV;
		CDBG("mt9p017_probe_init_sensor fail chip id doesnot match\n");
		goto init_probe_fail;
	}

       rc = mt9p017_i2c_read(0x31fe, &revid, 2);
       printk(KERN_INFO "[CAM]%s, RevID = 0x%X\n",__func__,revid);


	mt9p017_ctrl = kzalloc(sizeof(struct mt9p017_ctrl_t), GFP_KERNEL);
	if (!mt9p017_ctrl) {
		CDBG("mt9p017_init failed!\n");
		rc = -ENOMEM;
	}
	mt9p017_ctrl->fps_divider = 1 * 0x00000400;
	mt9p017_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9p017_ctrl->set_test = TEST_OFF;
	mt9p017_ctrl->prev_res = QTR_SIZE;
	mt9p017_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9p017_ctrl->sensordata = data;
       
	goto init_probe_done;
init_probe_fail:
	CDBG(" mt9p017_probe_init_sensor fails\n");
	
	mt9p017_probe_init_done(data);
init_probe_done:
	CDBG(" mt9p017_probe_init_sensor finishes\n");
	return rc;
}


int mt9p017_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling mt9p017_sensor_open_init\n");
	
	mt9p017_ctrl = kzalloc(sizeof(struct mt9p017_ctrl_t), GFP_KERNEL);
	if (!mt9p017_ctrl) {
		CDBG("mt9p017_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	mt9p017_ctrl->fps_divider = 1 * 0x00000400;
	mt9p017_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9p017_ctrl->set_test = TEST_OFF;
	mt9p017_ctrl->prev_res = QTR_SIZE;
	mt9p017_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9p017_ctrl->sensordata = data;
	if (rc < 0) {
		CDBG("Calling mt9p017_sensor_open_init fail1\n");
		return rc;
	}
	

      mt9p017_enable_flash(true);
	
	
	msm_camio_clk_rate_set(MT9P017_MASTER_CLK_RATE);
	usleep_range(1000, 2000); 
	msleep(10);
	rc = mt9p017_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;

	CDBG("init settings\n");
	rc = mt9p017_sensor_setting(REG_INIT, mt9p017_ctrl->prev_res);
	mt9p017_ctrl->fps = 30*Q8;
	mt9p017_enable_focus(true);
	mt9p017_init_focus();
	if (rc < 0) {
		gpio_set_value_cansleep(data->sensor_reset, 0);
		goto init_fail;
	} else
		goto init_done;
init_fail:
	CDBG("init_fail\n");
	mt9p017_probe_init_done(data);
init_done:
	CDBG("init_done\n");
	return rc;
} 

static int mt9p017_init_client(struct i2c_client *client)
{
	
	init_waitqueue_head(&mt9p017_wait_queue);
	return 0;
}

static const struct i2c_device_id mt9p017_i2c_id[] = {
	{"mt9p017", 0},
	{ }
};

static int mt9p017_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("%s+\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
			CDBG("%s, i2c_check_functionality failed\n",__func__); 
		goto probe_failure;
	}
	
	mt9p017_sensorw = kzalloc(sizeof(struct mt9p017_work_t), GFP_KERNEL);
	if (!mt9p017_sensorw) {
			CDBG("%s, kzalloc failed\n",__func__);
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9p017_sensorw);
	mt9p017_init_client(client);
	mt9p017_client = client;


	CDBG("%s-\n",__func__);
	return 0;

probe_failure:
	CDBG("mt9p017_probe failed! rc = %d\n", rc);
	return rc;
}

static int mt9p017_send_wb_info(struct wb_info_cfg *wb)
{
	return 0;

} 

static int __exit mt9p017_remove(struct i2c_client *client)
{
	struct mt9p017_work_t_t *sensorw = i2c_get_clientdata(client);	
	CDBG("%s+\n",__func__);  
	free_irq(client->irq, sensorw);
	mt9p017_client = NULL;
	kfree(sensorw);
	CDBG("%s-\n",__func__);  
	return 0;
}

static struct i2c_driver mt9p017_i2c_driver = {
	.id_table = mt9p017_i2c_id,
	.probe  = mt9p017_i2c_probe,
	.remove = __exit_p(mt9p017_i2c_remove),
	.driver = {
		.name = "mt9p017",
	},
};

int mt9p017_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&mt9p017_mut);
	CDBG("mt9p017_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
			mt9p017_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			mt9p017_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				mt9p017_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				mt9p017_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				mt9p017_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				mt9p017_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = mt9p017_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				mt9p017_write_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				mt9p017_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			rc = mt9p017_set_sensor_mode(cdata.mode,
					cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = mt9p017_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				mt9p017_move_focus(
				cdata.cfg.focus.dir,
				cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				mt9p017_set_default_focus(
				cdata.cfg.focus.steps);
			break;

		case CFG_GET_AF_MAX_STEPS:
			cdata.max_steps = MT9P017_TOTAL_STEPS_NEAR_TO_FAR;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_EFFECT:
			rc = mt9p017_set_default_focus(
				cdata.cfg.effect);
			break;


		case CFG_SEND_WB_INFO:
			rc = mt9p017_send_wb_info(
				&(cdata.cfg.wb_info));
			break;

		default:
			rc = -EFAULT;
			break;
		}

	mutex_unlock(&mt9p017_mut);
 CDBG("%s-, rc=%ld\n",__func__,rc); 
	return rc;
}

static int mt9p017_sensor_release(void)
{
#ifndef TEST
	int rc = -EBADF;
	CDBG("%s\n",__func__);
	mutex_lock(&mt9p017_mut);
	mt9p017_power_down();
	gpio_set_value_cansleep(mt9p017_ctrl->sensordata->sensor_reset, 0);
	msleep(5);
	gpio_free(mt9p017_ctrl->sensordata->sensor_reset);
	kfree(mt9p017_ctrl);
	mt9p017_ctrl = NULL;
	CDBG("mt9p017_release completed\n");
	mutex_unlock(&mt9p017_mut);
      return rc;
#endif
	return 0;
}

static int mt9p017_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0; 
       printk("%s+\n",__func__);
	rc = i2c_add_driver(&mt9p017_i2c_driver);
	if (rc < 0 || mt9p017_client == NULL) {
		rc = -ENOTSUPP;
	       printk("I2C add driver failed\n");
		goto probe_fail;
	}
	msm_camio_clk_rate_set(MT9P017_MASTER_CLK_RATE);
	msleep(10);
	rc = mt9p017_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = mt9p017_sensor_open_init;
	s->s_release = mt9p017_sensor_release;
	s->s_config  = mt9p017_sensor_config;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;
	
	mt9p017_probe_init_done(info);
       printk("%s-",__func__);

	return rc;

probe_fail:
	CDBG("mt9p017_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static int __mt9p017_probe(struct platform_device *pdev)
{
       printk("%s+",__func__);
	return msm_camera_drv_start(pdev, mt9p017_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9p017_probe,
	.driver = {
		.name = "msm_camera_mt9p017",
		.owner = THIS_MODULE,
	},
};

static int __init mt9p017_init(void)
{
       printk("%s+",__func__);
	return platform_driver_register(&msm_camera_driver);
       printk("%s-",__func__);
       cam_create_kernel_debuglevel();

}

module_init(mt9p017_init);
void mt9p017_exit(void)
{
	i2c_del_driver(&mt9p017_i2c_driver);
	cam_destroy_kernel_debuglevel();
}
MODULE_DESCRIPTION("Aptina 8 MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

static bool streaming = 1;

static int mt9p017_focus_test(void *data, u64 *val)
{
	int i = 0;
	mt9p017_set_default_focus(0);

	for (i = 90; i < 256; i++) {
		mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	msleep(5000);
	for (i = 255; i > 90; i--) {
		mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_focus, mt9p017_focus_test,
			NULL, "%lld\n");

static int mt9p017_step_test(void *data, u64 *val)
{
	int i = 0;
	mt9p017_set_default_focus(0);

	for (i = 0; i < MT9P017_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		mt9p017_move_focus(MOVE_NEAR, 1);
		msleep(5000);
	}

	mt9p017_move_focus(MOVE_FAR, MT9P017_TOTAL_STEPS_NEAR_TO_FAR);
	msleep(5000);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_step, mt9p017_step_test,
			NULL, "%lld\n");

static int cam_debug_stream_set(void *data, u64 val)
{
	int rc = 0;

	if (val) {
		mt9p017_start_stream();
		streaming = 1;
	} else {
		mt9p017_stop_stream();
		streaming = 0;
	}

	return rc;
}

static int cam_debug_stream_get(void *data, u64 *val)
{
	*val = streaming;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cam_stream, cam_debug_stream_get,
			cam_debug_stream_set, "%llu\n");


static int cam_debug_init(void)
{
	struct dentry *cam_dir;
	debugfs_base = debugfs_create_dir("sensor", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	cam_dir = debugfs_create_dir("mt9p017", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;

	if (!debugfs_create_file("focus", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_focus))
		return -ENOMEM;
	if (!debugfs_create_file("step", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_step))
		return -ENOMEM;
	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_stream))
		return -ENOMEM;

	return 0;
}

