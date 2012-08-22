/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
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
#include <linux/debugfs.h>
#include "q_mt9p017.h"

static int cam_chipID = 0;

#define C7_AF  

#define MT9P017_REG_MODEL_ID		0x0000
#define MT9P017_MODEL_ID			0x4800
#define REG_GROUPED_PARAMETER_HOLD	0x0104
#define GROUPED_PARAMETER_HOLD_OFF	0x00
#define GROUPED_PARAMETER_HOLD		0x01
#define REG_MODE_SELECT				0x0100
#define MODE_SELECT_STANDBY_MODE	0x00
#define MODE_SELECT_STREAM			0x01


#define REG_FRAME_LENGTH_LINES		0x300A
#define REG_COARSE_INT_TIME			0x3012
#define REG_GLOBAL_GAIN				0x305E
#define REG_LENS_SHADING			0x3780
#define REG_NOISE_MODEL_COEFF		0x3102

#define REG_RESET_REGISTER			0x301A
#define RESET_STREAM_READY			0x8652
#define RESET_STREAM_STOP			0x0050

#define REG_VCM_NEW_CODE			0x30F2
#define REG_VCM_CONTROL				0x30F0
#define REG_VCM_ANALOG_POWER		0x317A

#define  REG_VCM_CONTROL_POWER_EN	0x8010
#define  REG_VCM_ANALOG_POWER_EN	0x2000




#define Q8		0x00000100
#define MT9P017_MASTER_CLK_RATE	24000000


#define MT9P017_FULL_SIZE_WIDTH			2608
#define MT9P017_FULL_SIZE_HEIGHT		1960
#define MT9P017_FULL_SIZE_DUMMY_PIXELS	0
#define MT9P017_FULL_SIZE_DUMMY_LINES	0

#define MT9P017_QTR_SIZE_WIDTH			1296
#define MT9P017_QTR_SIZE_HEIGHT			972
#define MT9P017_QTR_SIZE_DUMMY_PIXELS	0
#define MT9P017_QTR_SIZE_DUMMY_LINES	0


#define MT9P017_HRZ_FULL_BLK_PIXELS		1102
#define MT9P017_VER_FULL_BLK_LINES		77

#define MT9P017_HRZ_QTR_BLK_PIXELS		2134
#define MT9P017_VER_QTR_BLK_LINES		73

#ifndef C7_AF

#define MT9P017_TOTAL_STEPS_NEAR_TO_FAR		32
#define MT9P017_STEPS_NEAR_TO_CLOSEST_INF	32
uint16_t mt9p017_nl_region_code_per_step1 = 32;
#else  
#define MT9P017_TOTAL_STEPS_NEAR_TO_FAR    49
#define MT9P017_STEPS_NEAR_TO_CLOSEST_INF	49
uint16_t mt9p017_l_region_boundary1 = 49;
uint16_t mt9p017_nl_region_code_per_step1 = 0;
#endif





uint16_t mt9p017_step_position_table[MT9P017_TOTAL_STEPS_NEAR_TO_FAR + 1];
uint16_t mt9p017_nl_region_boundary1 = 2;
uint16_t mt9p017_l_region_code_per_step = 4;
uint16_t mt9p017_damping_threshold = 10;
uint16_t mt9p017_sw_damping_time_wait = 1;

enum mt9p017_move_focus_dir {
	CAMSENSOR_MOVE_FOCUS_NEAR,
	CAMSENSOR_MOVE_FOCUS_FAR
};

struct mt9p017_work_t {
	struct work_struct work;
};

static struct mt9p017_work_t *mt9p017_sensorw;
static struct i2c_client *mt9p017_client;
static int32_t config_csi;


struct mt9p017_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;		
	uint32_t pict_fps_divider;	
	uint16_t fps;

	int16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum mt9p017_resolution_t prev_res;
	enum mt9p017_resolution_t pict_res;
	enum mt9p017_resolution_t curr_res;
	enum mt9p017_test_mode_t  set_test;

	unsigned short imgaddr;
};


static uint8_t mt9p017_delay_msecs_stdby = 20;


static struct mt9p017_ctrl_t *mt9p017_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9p017_wait_queue);
DEFINE_MUTEX(mt9p017_mut);


extern struct dentry *kernel_debuglevel_dir;
static unsigned int CAM_MT9P017_DLL=0;
#define CAM_PRINTK(level, fmt, args...) if(level <= CAM_MT9P017_DLL) printk("%s:" fmt, __func__, ##args);
static int otpm=0;


static void cam_create_kernel_debuglevel(void)
{
	CAM_PRINTK(1, "create kernel debuglevel!\n");


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
	CAM_PRINTK(1, "destroy kernel debuglevel!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}




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
		CAM_PRINTK(1, "mt9p017_i2c_rxdata failed!\n");
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
		CAM_PRINTK(1, "mt9p017_i2c_txdata faild 0x%x\n", mt9p017_client->addr);
		return -EIO;
	}

	return 0;
}

static int32_t mt9p017_i2c_read_w(unsigned short saddr,
	unsigned short raddr, unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);
	
		
		
		
	
	rc = mt9p017_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		CAM_PRINTK(1, "mt9p017_i2c_read failed!\n");

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
		CAM_PRINTK(1, " mt9p017 i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t mt9p017_i2c_write_w_sensor
	(unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00)>>8;
	buf[3] = (wdata & 0x00FF);
	rc = mt9p017_i2c_txdata(mt9p017_client->addr, buf, 4);
	if (rc < 0) {
		CAM_PRINTK(1, "i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}
static int32_t mt9p017_i2c_write_w(unsigned short saddr,
	unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00)>>8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9p017_i2c_txdata(saddr, buf, 4);
	if (rc < 0)
		pr_err("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);
	return rc;
}

static int32_t mt9p017_i2c_write_w_table(
	struct mt9p017_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = mt9p017_i2c_write_w(mt9p017_client->addr,
			reg_conf_tbl->waddr, reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}

	return rc;
} 

static void mt9p017_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	
	uint16_t preview_frame_length_lines, snapshot_frame_length_lines;
	uint16_t preview_line_length_pck, snapshot_line_length_pck;
	uint32_t divider, d1, d2;
	
	preview_frame_length_lines = MT9P017_QTR_SIZE_HEIGHT +
		MT9P017_VER_QTR_BLK_LINES;
	preview_line_length_pck = MT9P017_QTR_SIZE_WIDTH +
		MT9P017_HRZ_QTR_BLK_PIXELS;
	
	snapshot_frame_length_lines = MT9P017_FULL_SIZE_HEIGHT +
		MT9P017_VER_FULL_BLK_LINES;
	snapshot_line_length_pck = MT9P017_FULL_SIZE_WIDTH +
		MT9P017_HRZ_FULL_BLK_PIXELS;

	d1 = preview_frame_length_lines * 0x00000400/
		snapshot_frame_length_lines;
	d2 = preview_line_length_pck * 0x00000400/
		snapshot_line_length_pck;
	divider = d1 * d2 / 0x400;
	
	*pfps = (uint16_t) (fps * divider / 0x400);
	

}

static int32_t mt9p017_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;

	CAM_PRINTK(1, "%s: entered. enable = %d\n", __func__, is_enable);

	rc = mt9p017_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	rc = mt9p017_i2c_write_w_sensor(REG_LENS_SHADING,
			((uint16_t) is_enable) << 15);
	if (rc < 0)
		return rc;

	rc = mt9p017_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD_OFF);

	CAM_PRINTK(1, "%s: exiting. rc = %d\n", __func__, rc);
	return rc;
}

static uint16_t mt9p017_get_prev_lines_pf(void)
{
	if (mt9p017_ctrl->prev_res == QTR_SIZE)
		return MT9P017_QTR_SIZE_HEIGHT + MT9P017_VER_QTR_BLK_LINES;
	else
		return MT9P017_FULL_SIZE_HEIGHT + MT9P017_VER_FULL_BLK_LINES;
} 

static uint16_t mt9p017_get_prev_pixels_pl(void)
{
	if (mt9p017_ctrl->prev_res == QTR_SIZE)
		return MT9P017_QTR_SIZE_WIDTH + MT9P017_HRZ_QTR_BLK_PIXELS;
	else
		return MT9P017_FULL_SIZE_WIDTH + MT9P017_HRZ_FULL_BLK_PIXELS;
} 

static uint16_t mt9p017_get_pict_lines_pf(void)
{
		if (mt9p017_ctrl->pict_res == QTR_SIZE)
			return MT9P017_QTR_SIZE_HEIGHT +
				MT9P017_VER_QTR_BLK_LINES;
		else
			return MT9P017_FULL_SIZE_HEIGHT +
				MT9P017_VER_FULL_BLK_LINES;
} 

static uint16_t mt9p017_get_pict_pixels_pl(void)
{
	if (mt9p017_ctrl->pict_res == QTR_SIZE)
		return MT9P017_QTR_SIZE_WIDTH +
			MT9P017_HRZ_QTR_BLK_PIXELS;
	else
		return MT9P017_FULL_SIZE_WIDTH +
			MT9P017_HRZ_FULL_BLK_PIXELS;
} 

static uint32_t mt9p017_get_pict_max_exp_lc(void)
{
	if (mt9p017_ctrl->pict_res == QTR_SIZE)
		return (MT9P017_QTR_SIZE_HEIGHT +
			MT9P017_VER_QTR_BLK_LINES)*24;
	else
		return (MT9P017_FULL_SIZE_HEIGHT +
			MT9P017_VER_FULL_BLK_LINES)*24;
} 

static int32_t mt9p017_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;

	
	
	
	total_lines_per_frame = (uint16_t)(((MT9P017_QTR_SIZE_HEIGHT +
		MT9P017_VER_QTR_BLK_LINES) * mt9p017_ctrl->fps_divider)/0x400);

	if (mt9p017_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
		total_lines_per_frame) < 0)
		return rc;

	return rc;
} 

static int32_t mt9p017_write_exp_gain(uint16_t gain, uint32_t line)
{

	uint16_t max_legal_gain = 0x0E7F;
	int32_t rc = 0;
	CDBG("mt9p017_write_exp_gain entering....\n");
	if (mt9p017_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		mt9p017_ctrl->my_reg_gain = gain;
		mt9p017_ctrl->my_reg_line_count = (uint16_t) line;
	}

	if (gain > max_legal_gain)
		gain = max_legal_gain;

	if (mt9p017_ctrl->sensormode != SENSOR_SNAPSHOT_MODE)
		line = (uint32_t) (line * mt9p017_ctrl->fps_divider /
				   0x00000400);
	else
		line = (uint32_t) (line * mt9p017_ctrl->pict_fps_divider /
				   0x00000400);

	
	gain |= 0x1000;
	rc = mt9p017_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;
	CDBG("mt9p017_write_exp_gain REG_GLOBAL_GAIN....\n");

	rc = mt9p017_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain);
	if (rc < 0)
		return rc;
	CDBG("mt9p017_write_exp_gain REG_COARSE_INT_TIME ....\n");

	rc = mt9p017_i2c_write_w_sensor(REG_COARSE_INT_TIME, line);
	if (rc < 0)
		return rc;
	rc = mt9p017_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD_OFF);
	if (rc < 0)
		return rc;
	CDBG("mt9p017_write_exp_gain exit....\n");
	return rc;
}

static int32_t mt9p017_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	rc = mt9p017_write_exp_gain(gain, line);
	if (rc < 0)
	{
	       CAM_PRINTK(1, "mt9p017_write_exp_gain failed\n");
		return rc;
	}
	msleep(5);

	return rc;
} 


static int read_otpm(struct mt9p017_i2c_reg_conf *regs)
{
	int i;
	unsigned short result, reg_addr;
	mt9p017_i2c_write_w(mt9p017_client->addr, 0x301A, 0x0610);
	mt9p017_i2c_write_w(mt9p017_client->addr, 0x3134, 0xCD95);
	mt9p017_i2c_write_w(mt9p017_client->addr, 0x304C, 0x3100);
	mt9p017_i2c_write_w(mt9p017_client->addr, 0x304A, 0x0010);

	msleep(20);

	mt9p017_i2c_read_w(mt9p017_client->addr, 0x304A, &result);
	CAM_PRINTK(1, "[CAM]:%s, %d, Read OTPM : the value is %x!\n", __func__, __LINE__, result);

	if ((result & 0x0060) != 0x0060) {
		CAM_PRINTK(1, "[CAM]:%s, %d, Failed reading OTPM!\n", __func__, __LINE__);
		return -1;
	}

	
	CAM_PRINTK(1, "starting to read OTPM data:\n" );
	for (i = 0, reg_addr = 0x3800; i < (mt9p017_regs.reg_lensroff_size-1); i++, regs++, reg_addr += 2) {
		mt9p017_i2c_read_w(mt9p017_client->addr, reg_addr, &result);
       
		if (i/20 < 5) {
			regs->waddr = 0x3600 + (i/20)*0x040 + (i%20)*2;
		} else if (i > 99 && i < 102)
		{
			regs->waddr = 0x3780 + (i - 99)*2;
		}
		else
		{
			regs->waddr = 0x37C0 + (i - 102)*2;
		}
		CAM_PRINTK(1, "\t dest reg:0x%x: source reg:0x%x \t value:0x%x\n",  regs->waddr, reg_addr, result);
        
		regs->wdata = result;

	}
       CAM_PRINTK(1, "End to  read OTPM data: total data: %d\n", mt9p017_regs.reg_lensroff_size);
	CAM_PRINTK(1, "[CAM]:%s, %d, Read OTPM successfully,!\n", __func__, __LINE__);

	return 0;
}


static int32_t mt9p017_set_lc(void)
{
	int32_t rc = 0;

      
	if (!otpm) {
		struct mt9p017_i2c_reg_conf *reg_lensroff;
		reg_lensroff = kmalloc(mt9p017_regs.reg_lensroff_size * sizeof( struct mt9p017_i2c_reg_conf), GFP_KERNEL);
		read_otpm(reg_lensroff);
		rc = mt9p017_i2c_write_w_table(reg_lensroff, mt9p017_regs.reg_lensroff_size);
		kfree(reg_lensroff);
	} else
		rc = mt9p017_i2c_write_w_table(mt9p017_regs.reg_lensroff,
			mt9p017_regs.reg_lensroff_size);
	

	
	CAM_PRINTK(1, "[CAM]%s, %d sett the reg_lensroff.\n", __func__, __LINE__);
	

	return rc;
} 

static int32_t mt9p017_move_focus(int direction, int32_t num_steps)
{
	int32_t rc = 0;
	int16_t step_direction, dest_lens_position, dest_step_position;
	int16_t target_dist, small_step, next_lens_position;
       CAM_PRINTK(1, "mt9p017_move_focus entering....\n");
	if (direction == CAMSENSOR_MOVE_FOCUS_NEAR) {
		step_direction = 1;
	} else if (direction == CAMSENSOR_MOVE_FOCUS_FAR)
		step_direction = -1;
	else{
		pr_err("Illegal focus direction\n");
		return -EINVAL;
	}
	CAM_PRINTK(1, "mt9p017_move_focus calculating dest_step_position\n");
	dest_step_position = mt9p017_ctrl->curr_step_pos +
		(step_direction * num_steps);
	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > MT9P017_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_position = MT9P017_TOTAL_STEPS_NEAR_TO_FAR;

	if (dest_step_position == mt9p017_ctrl->curr_step_pos)
		return rc;

	dest_lens_position = mt9p017_step_position_table[dest_step_position];
	target_dist = step_direction *
		(dest_lens_position - mt9p017_ctrl->curr_lens_pos);
	CAM_PRINTK(1, "Target Dist: %hd\n", target_dist);

	if (step_direction < 0 && (target_dist >=
		mt9p017_step_position_table[mt9p017_damping_threshold])){
		small_step = (uint16_t)((target_dist/10));
		if (small_step == 0)
			small_step = 1;
		mt9p017_sw_damping_time_wait = 1;
	} else {
		small_step = (uint16_t)(target_dist/4);
		if (small_step == 0)
			small_step = 1;
		mt9p017_sw_damping_time_wait = 4;
	}
	CAM_PRINTK(1, "mt9p017_move_focus small_step %d ...\n", small_step);

	for (next_lens_position = mt9p017_ctrl->curr_lens_pos +
		(step_direction * small_step);
	(step_direction * next_lens_position) <=
		(step_direction * dest_lens_position);
	next_lens_position += (step_direction * small_step)) {
		rc = mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE,
			next_lens_position);
		if (rc < 0)
			return rc;

		mt9p017_ctrl->curr_lens_pos = next_lens_position;
		usleep(mt9p017_sw_damping_time_wait*10);
	}
	if (mt9p017_ctrl->curr_lens_pos != dest_lens_position) {
		rc = mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE,
			 dest_lens_position);
		if (rc < 0)
			return rc;
		usleep(mt9p017_sw_damping_time_wait*10);
	}
	mt9p017_ctrl->curr_lens_pos = dest_lens_position;
	mt9p017_ctrl->curr_step_pos = dest_step_position;
	CAM_PRINTK(1, "mt9p017_move_focus exit....\n");
	return rc;
}


static int mt9p017_enable_flash(int on)
{
    int rc;

    CAM_PRINTK(1, "%s, mode: %d\n", __func__, on);

    if(on)
    {
	rc = gpio_request(13, "FL_EN");
	if (rc < 0) {
		CAM_PRINTK(1, "%s, request gpio failed!\n", __func__);
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


static void mt9p017_enable_focus(int on)
{
    int rc;
    static struct vreg *vreg_vcm_2p8;

    if(on)
    {
	if (vreg_vcm_2p8 == NULL) {
		vreg_vcm_2p8 = vreg_get(NULL, "usim2"); 
		if (IS_ERR(vreg_vcm_2p8)) {
			CAM_PRINTK(1, "%s: vreg_get(%s) failed (%ld)\n",
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
			CAM_PRINTK(1, "%s: rfrx1 disable failed (%d)\n",
				__func__, rc);

             }
             vreg_vcm_2p8 = NULL;

    }

}


static void mt9p017_af_init(void)
{
#ifndef C7_AF
	uint8_t i;
	mt9p017_step_position_table[0] = 0;
	mt9p017_step_position_table[1] = (uint16_t)(32*(1.0/3.0)); 
       mt9p017_step_position_table[2] = (uint16_t)(32*(2.0/3.0)); 
       mt9p017_step_position_table[3] = (uint16_t) 32;

	for (i = 1; i <= MT9P017_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		if (i <= mt9p017_nl_region_boundary1)
			mt9p017_step_position_table[i] =
			mt9p017_step_position_table[i-1]
			+ mt9p017_nl_region_code_per_step1;
		else
			mt9p017_step_position_table[i] =
			mt9p017_step_position_table[i-1]
			+ mt9p017_l_region_code_per_step;

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

static int32_t mt9p017_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;
	CAM_PRINTK(1, "mt9p017_set_default_focus entering....\n");
	if (mt9p017_ctrl->curr_step_pos != 0) {
		rc = mt9p017_move_focus(CAMSENSOR_MOVE_FOCUS_FAR,
			mt9p017_ctrl->curr_step_pos);
		if (rc < 0)
			return rc;
	} else {
		rc = mt9p017_i2c_write_w_sensor(REG_VCM_NEW_CODE, 0x0000);
		if (rc < 0)
			return rc;
	}
	mt9p017_ctrl->curr_lens_pos = 0x00;
	mt9p017_ctrl->curr_step_pos = 0x00;
	CDBG("mt9p017_set_default_focus exit....\n");
	return rc;
}


static int32_t mt9p017_sensor_setting(int update_type, int rt)
{
	int32_t rc = 0;

	struct msm_camera_csi_params mt9p017_csi_params;
	switch (update_type) {
	case REG_INIT:
		if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
			CAM_PRINTK(1, "Sensor setting Init = %d\n", rt);
			
			mt9p017_ctrl->fps = 30.8 * Q8;
			mt9p017_ctrl->fps_divider = 1 * 0x400;

			
			
			msleep(10);
			rc = mt9p017_i2c_write_b_sensor(REG_MODE_SELECT,
				MODE_SELECT_STANDBY_MODE);
			if (rc < 0)
				return rc;

			msleep(mt9p017_delay_msecs_stdby);
			rc = mt9p017_i2c_write_b_sensor(
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
			if (rc < 0)
				return rc;
			
			msleep(10);

			rc = mt9p017_i2c_write_w_table(mt9p017_regs.reg_pll,
				mt9p017_regs.reg_pll_size);
				if (rc < 0)
					return rc;
			msleep(10);

			if(!otpm){
			rc = mt9p017_i2c_write_w_table(mt9p017_regs.rec_settings,
				mt9p017_regs.rec_size);
			if (rc < 0)
				return rc;
			}
			else
			{
			rc = mt9p017_i2c_write_w_table(mt9p017_regs.rec_settings_rev3,
				mt9p017_regs.rec_size);
			if (rc < 0)
				return rc;
			}

			
			rc = mt9p017_set_lc();
			if (rc < 0)
				return rc;
			


			rc = mt9p017_i2c_write_b_sensor(
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD_OFF);
			if (rc < 0)
				return rc;
			CAM_PRINTK(1, "%s: %d\n", __func__, __LINE__);

			msleep(mt9p017_delay_msecs_stdby);
		}
		break;

	case UPDATE_PERIODIC:
		if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
			CAM_PRINTK(1, "%s: rt:%d\n", __func__, rt);
			
			rc = mt9p017_i2c_write_w_sensor(REG_VCM_CONTROL,
				REG_VCM_CONTROL_POWER_EN);
			if (rc < 0)
				return rc;
			rc = mt9p017_i2c_write_w_sensor(REG_VCM_ANALOG_POWER,
				REG_VCM_ANALOG_POWER_EN);
			if (rc < 0)
				return rc;

			
			msleep(20);
			CAM_PRINTK(1, "Sensor setting snap or preview = %d\n", rt);
			rc = mt9p017_i2c_write_b_sensor(REG_MODE_SELECT,
				MODE_SELECT_STANDBY_MODE);
			if (rc < 0)
				return rc;

			rc = mt9p017_i2c_write_b_sensor(
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
			if (rc < 0)
				return rc;
				
			msleep(60);
			
			if (config_csi == 0) {
				
				msm_camio_vfe_clk_rate_set(192000000);
				mt9p017_csi_params.lane_cnt = 2;
				mt9p017_csi_params.data_format = CSI_10BIT;
				mt9p017_csi_params.lane_assign = 0xe4;
				mt9p017_csi_params.dpcm_scheme = 0;
				mt9p017_csi_params.settle_cnt = 0x14;

				CAM_PRINTK(1, "mt9p017 configuring csi controller\n");
				rc = msm_camio_csi_config(&mt9p017_csi_params);
				if (rc < 0)
					CAM_PRINTK(1, "config csi controller failed\n");
				msleep(20);
				config_csi = 1;
			}
		

			
			if (rt == RES_PREVIEW) {
				rc = mt9p017_i2c_write_w_table(
				mt9p017_regs.reg_prev,
				mt9p017_regs.reg_prev_size);
			CAM_PRINTK(1, "MT9P017 Preview configs done\n");
			if (rc < 0)
				return rc;
			} else {
				rc = mt9p017_i2c_write_w_table(
				mt9p017_regs.reg_snap,
				mt9p017_regs.reg_snap_size);
				if (rc < 0)
					return rc;
				CAM_PRINTK(1, "MT9P017 Snapshot configs done\n");
			}
			
			
			msleep(10);
			
			rc = mt9p017_i2c_write_w_sensor(
				REG_RESET_REGISTER,	RESET_STREAM_READY);
			if (rc < 0)
				return rc;
			rc = mt9p017_i2c_write_b_sensor(
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD_OFF);
			if (rc < 0)
				return rc;
			CAM_PRINTK(1, " MT9P017 Turn on streaming\n");
			
			rc = mt9p017_i2c_write_b_sensor(REG_MODE_SELECT,
				MODE_SELECT_STREAM);
			if (rc < 0)
				return rc;
				
			
			msleep(60);
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}


static int32_t mt9p017_video_config(int mode)
{

	int32_t rc = 0;
	int rt;
	CDBG("mt9p017_video_config entering....\n");

	
	if (mt9p017_ctrl->prev_res == QTR_SIZE) {
			rt = RES_PREVIEW;
			mt9p017_delay_msecs_stdby =
			(((2 * 1000 * Q8 * mt9p017_ctrl->fps_divider) / 0x400) /
			mt9p017_ctrl->fps) + 1;
		} else {
			rt = RES_CAPTURE;
			mt9p017_delay_msecs_stdby =
			(((1000 * Q8 * mt9p017_ctrl->fps_divider) / 0x400) /
			mt9p017_ctrl->fps) + 1;
		}

		if (mt9p017_sensor_setting(UPDATE_PERIODIC, rt) < 0)
			return rc;


	mt9p017_ctrl->curr_res = mt9p017_ctrl->prev_res;
	mt9p017_ctrl->sensormode = mode;
	CAM_PRINTK(1, "mt9p017_video_config exit....\n");
	
	CAM_PRINTK(1, "[CAM----] %s: %d the angle is %d\n", __func__, __LINE__, mt9p017_ctrl->sensordata->sensor_platform_info->mount_angle);
	
	return rc;
}

static int32_t mt9p017_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	CAM_PRINTK(1, "mt9p017_snapshot_config entering....\n");

	
	if (mt9p017_ctrl->curr_res != mt9p017_ctrl->pict_res) {
		if (mt9p017_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;
			mt9p017_delay_msecs_stdby =
			(((2 * 1000 * Q8 * mt9p017_ctrl->fps_divider) / 0x400) /
			mt9p017_ctrl->fps) + 1;
		} else {
			rt = RES_CAPTURE;
			mt9p017_delay_msecs_stdby =
			(((1000 * Q8 * mt9p017_ctrl->fps_divider) / 0x400) /
			mt9p017_ctrl->fps) + 1;
		}

	CAM_PRINTK(1, "Calling mt9p017_snapshot_config\n");
	if (mt9p017_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	}
	mt9p017_ctrl->curr_res = mt9p017_ctrl->pict_res;
	mt9p017_ctrl->sensormode = mode;
	CAM_PRINTK(1, "mt9p017_snapshot_config exit....\n");
	return rc;
} 

static int32_t mt9p017_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	CAM_PRINTK(1, "mt9p017_raw_snapshot_config entering....\n");
	
	if (mt9p017_ctrl->curr_res != mt9p017_ctrl->pict_res) {
		if (mt9p017_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;
			mt9p017_delay_msecs_stdby =
				((2 * 1000 * Q8 *
				mt9p017_ctrl->fps_divider) /
				mt9p017_ctrl->fps) + 1;
		} else {
			rt = RES_CAPTURE;
			mt9p017_delay_msecs_stdby =
				((1000 * Q8 * mt9p017_ctrl->fps_divider)/
				mt9p017_ctrl->fps) + 1;
		}
		if (mt9p017_sensor_setting(UPDATE_PERIODIC, rt) < 0)
			return rc;
	}
	mt9p017_ctrl->curr_res = mt9p017_ctrl->pict_res;
	mt9p017_ctrl->sensormode = mode;
	CAM_PRINTK(1, "mt9p017_raw_snapshot_config exit....\n");

	return rc;
} 

static int32_t mt9p017_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	CAM_PRINTK(1, "mod is:%d, res is %d\n", mode, res);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mt9p017_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		rc = mt9p017_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = mt9p017_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	CAM_PRINTK(1, "rc:%d\n", rc);

	return rc;
}
static int32_t mt9p017_power_down(void)
{
  int32_t rc = 0;
	CAM_PRINTK(1, "mt9p017_entering power_down\n");
	mt9p017_enable_flash(false);
	mt9p017_enable_focus(false);





	return rc;
}

static int mt9p017_probe_init_done(const struct msm_camera_sensor_info *data)
{
	gpio_direction_output(data->sensor_reset, 0);
	gpio_free(data->sensor_reset);
	return 0;
}

static int mt9p017_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	unsigned short chipid = 0;
	CAM_PRINTK(1, " mt9p017_probe_init_sensor\n");
	rc = gpio_request(data->sensor_reset, "mt9p017");
	if (!rc) {
		gpio_tlmm_config(GPIO_CFG(37, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
		CAM_PRINTK(1, "sensor_reset = %d\n", rc);
		gpio_direction_output(data->sensor_reset, 0);
		msleep(10);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(20);
	} else {
		CAM_PRINTK(1, " mt9p017_probe_init_sensor 2\n");
		goto init_probe_done;
	}
	
	rc = mt9p017_i2c_read_w(mt9p017_client->addr,
		MT9P017_REG_MODEL_ID, &chipid);
	CAM_PRINTK(1, "mt9p017 model_id = 0x%x\n", chipid);
       cam_chipID = (int) chipid;
       rc = mt9p017_i2c_read_w(mt9p017_client->addr,
              0x31fe, &chipid);
	CAM_PRINTK(1, "mt9p017 revsion = 0x%x\n", chipid);

	rc = 0;

	goto init_probe_done;



init_probe_done:
	CAM_PRINTK(1, " mt9p017_probe_init_sensor finishes\n");
	return rc;
	}


int mt9p017_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	CAM_PRINTK(1, "enter\n");
	mt9p017_ctrl = kzalloc(sizeof(struct mt9p017_ctrl_t), GFP_KERNEL);
	if (!mt9p017_ctrl) {
		CAM_PRINTK(1, "mt9p017_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	mt9p017_ctrl->fps_divider = 1 * 0x00000400;
	mt9p017_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9p017_ctrl->fps = 30.8 * Q8;
	mt9p017_ctrl->set_test = TEST_OFF;
	mt9p017_ctrl->prev_res = QTR_SIZE;
	mt9p017_ctrl->pict_res = FULL_SIZE;
	mt9p017_ctrl->curr_res = INVALID_SIZE;
	config_csi = 0;

	if (data)
		mt9p017_ctrl->sensordata = data;
		
	msm_camio_clk_rate_set(MT9P017_MASTER_CLK_RATE);
	msleep(20);

	rc = mt9p017_probe_init_sensor(data);
	if (rc < 0) {
		CAM_PRINTK(1, "Calling mt9p017_sensor_open_init fail\n");
		goto init_fail;
	}
	CAM_PRINTK(1, "Calling mt9p017_af_init\n");

	rc = mt9p017_sensor_setting(REG_INIT, RES_PREVIEW);
       if(rc<0)
       {
       	CAM_PRINTK(1, "rc:%d, init preview failed\n", rc);
              goto init_fail;
       }
	mt9p017_af_init();
	mt9p017_enable_focus(true);
	mt9p017_enable_flash(true);
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;
init_fail:
	CAM_PRINTK(1, " mt9p017_sensor_open_init fail\n");
	
	mt9p017_probe_init_done(data);
	kfree(mt9p017_ctrl);
init_done:
	CAM_PRINTK(1, "mt9p017_sensor_open_init done\n");
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
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_PRINTK(1, "mt9p017 i2c_check_functionality failed\n");
		goto probe_failure;
	}

	mt9p017_sensorw = kzalloc(sizeof(struct mt9p017_work_t), GFP_KERNEL);
	if (!mt9p017_sensorw) {
		CAM_PRINTK(1, "kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9p017_sensorw);
	mt9p017_init_client(client);
	mt9p017_client = client;

	msleep(50);

	CAM_PRINTK(1, "mt9p017_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CAM_PRINTK(1, "mt9p017_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit mt9p017_i2c_remove(struct i2c_client *client)
{
	struct mt9p017_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	mt9p017_client = NULL;
	kfree(sensorw);
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
	CAM_PRINTK(1, "mt9p017_sensor_config: cfgtype = %d\n",
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
		case CFG_SET_LENS_SHADING:
			rc =
				mt9p017_lens_shading_enable(
				cdata.cfg.lens_shading);
			break;
		case CFG_GET_AF_MAX_STEPS:
			cdata.max_steps = MT9P017_STEPS_NEAR_TO_CLOSEST_INF;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_SET_EFFECT:
		default:
			rc = -EFAULT;
			break;
		}

	mutex_unlock(&mt9p017_mut);

	return rc;
}




static int mt9p017_sensor_release(void)
{
	int rc = -EBADF;
	CAM_PRINTK(1, "mt9p017_entering Sensor_release\n");
	mutex_lock(&mt9p017_mut);
	mt9p017_power_down();
	gpio_direction_output(mt9p017_ctrl->sensordata->sensor_reset,
		0);
	gpio_free(mt9p017_ctrl->sensordata->sensor_reset);
	kfree(mt9p017_ctrl);
	mt9p017_ctrl = NULL;
	CAM_PRINTK(1, "mt9p017_release completed\n");
	mutex_unlock(&mt9p017_mut);

	return rc;
}

static int mt9p017_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	
       CAM_PRINTK(1, "mt9p017_sensor_probe: SENSOR PROBE entered !\n");
	rc = i2c_add_driver(&mt9p017_i2c_driver);
	if (rc < 0 || mt9p017_client == NULL) {
		pr_info("i2c add driver is fail\n");
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	msm_camio_clk_rate_set(MT9P017_MASTER_CLK_RATE);
	msleep(200);
	rc = mt9p017_probe_init_sensor(info);
	if (rc < 0) {
		CAM_PRINTK(1, "mt9p017_sensor_probe: probe failed\n");
		goto probe_fail;
	}
	s->s_init    = mt9p017_sensor_open_init;
	s->s_release = mt9p017_sensor_release;
	s->s_config  = mt9p017_sensor_config;
	
	s->s_mount_angle = info->sensor_platform_info->mount_angle;
	
	CAM_PRINTK(1, "[CAM----] %s: %d the angle is %d\n", __func__, __LINE__, s->s_mount_angle);
	mt9p017_probe_init_done(info);
	CAM_PRINTK(1, "mt9p017_sensor_probe: SENSOR PROBE completed !\n");

	return rc;

probe_fail:
	CAM_PRINTK(1, "mt9p017_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static int __mt9p017_probe(struct platform_device *pdev)
{
	CAM_PRINTK(1, "__mt9p017_probe__mt9p017_probe is called\n");
	return msm_camera_drv_start(pdev, mt9p017_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9p017_probe,
	.driver = {
		.name = "msm_camera_mt9p017",
		.owner = THIS_MODULE,
	},
};



static int  read_mt9p017_device_id_s(void *data, u64 *val)
{
#if 0
  int type = (int)data;
  switch(type)
  {
    case DEVICE_ID:
      *val = cam_chipID;
      break;
    case FRONT_CAMERA_2D:

      *val = cam_reversion_id;
      break;
    default:
      CAM_PRINTK(1, "%s, invalid type = %d",__func__,type);
      *val = 0;
      break;
  }
  #endif
   *val = cam_chipID;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cam_mt9p017,  read_mt9p017_device_id_s, NULL, "%lld\n");

static int __init read_mt9p017_device_id(void)
{
  struct dentry *debugfs_base;
  debugfs_base = debugfs_create_dir("cam_mt9p017", 0);
  CAM_PRINTK(1, "%s+",__func__);
  if (IS_ERR(debugfs_base))
    return -ENOMEM;
  debugfs_create_file("cam_deviceID", 0444, debugfs_base, NULL, &cam_mt9p017);


  CAM_PRINTK(1, "%s-",__func__);
	return 0;
}


static int __init mt9p017_init(void)
{
       int rc = 0;
        cam_create_kernel_debuglevel();

        rc = read_mt9p017_device_id();
        if(rc != 0)
        CAM_PRINTK(1, "%s, init ead_mt9p017_device_id failed\n", __func__);
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9p017_init);

void mt9p017_exit(void)
{
	i2c_del_driver(&mt9p017_i2c_driver);
	cam_destroy_kernel_debuglevel();
}



module_param(otpm, int, 0644);

