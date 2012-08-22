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
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_chimei.h"
#include <linux/pwm.h>
#include <mach/socinfo.h>
#include <mach/pmic.h>
#include <mach/pm_log.h>
#include <linux/debugfs.h>

extern struct dentry *kernel_debuglevel_dir;
unsigned int LCD_HX8363A_DLL = 0;  

#define FB_MSM_MIPI_DSI_CHIMEI_PWMINFO_PES 1 

#if FB_MSM_MIPI_DSI_CHIMEI_PWMINFO_PES
#define CHIMEI_DEBUG_BUF	64
static int system_bl_level_old = 40; 
static char	debug_buf[CHIMEI_DEBUG_BUF];
#endif

struct platform_device *pdev_bkl_pwm;
struct msm_fb_data_type *mfd_bkl = NULL;
static atomic_t cap_bkl_emmode_enable = ATOMIC_INIT(0);
static int capkey_bl_on=0;
static struct msm_panel_common_pdata *mipi_chimei_pdata;
static struct dsi_buf chimei_tx_buf;
static struct dsi_buf chimei_rx_buf;
static int mipi_chimei_lcd_init(void);
static char enter_sleep[2] = {0x10, 0x00};
static char exit_sleep[2] = {0x11, 0x00};
static char display_off[2] = {0x28, 0x00};
static char display_on[2] = {0x29, 0x00};
static char setPixelFormat[2] = {0x3A, 0x70}; 
static char setRGBIF[2] = {0xB3, 0x00}; 
static char setPanel[2] = {0xCC, 0x07}; 
static char setMem[2] = {0x36, 0x00}; 
static char setEXTC[4] = {0xB9, 0xFF, 0x83, 0x63};
static char setPower[13] = {0xB1, 0x78, 0x34, 0x08, 0x32, 0x02, 0x13, 0x11, 0x11, 0x35, 0x3D, 0x3F, 0x3F};
static char setMipi[14] = {0xBA, 0x80, 0x00, 0x10, 0x08, 0x08, 0x10, 0x7C, 0x6E, 0x6D, 0x0A, 0x01, 0x84, 0x43};
static char setCYC[10] = {0xB4, 0x08, 0x12, 0x72, 0x12, 0x06, 0x03, 0x54, 0x03, 0x4E};
static char setGamma[31] = {0xE0,
0x01, 0x1F, 0x28, 0x38,
0x3E, 0x3F, 0x07, 0x8D,
0xCE, 0x91, 0x54, 0x12,
0x14, 0x8F, 0x12, 0x01,
0x1F, 0x28, 0x38, 0x3E,
0x3F, 0x07, 0x8D, 0xCE,
0x91, 0x54, 0x12, 0x14,
0x8F, 0x12};

#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
static char wr_disbv[2] = {0x51, 0x66};		
static char wr_ctrld[2] = {0x53, 0x24};		
static char wr_ctrld_off[2] = {0x53, 0x00};	
static char led_pwm_freq[4] = {0xC9, 0x1F, 0x3E, 0x1}; 
static struct dsi_cmd_desc chimei_cmd_backlight_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(wr_disbv), wr_disbv},
};
#endif

static struct dsi_cmd_desc chimei_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(setEXTC), setEXTC },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(setPower), setPower },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(setMipi), setMipi },
	{DTYPE_DCS_WRITE1, 1, 0, 0, 5,
		sizeof(setPixelFormat), setPixelFormat},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 5,
		sizeof(setRGBIF), setRGBIF},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(setCYC), setCYC },
	{DTYPE_DCS_WRITE1, 1, 0, 0, 5,
		sizeof(setPanel), setPanel},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 5,
		sizeof(setMem), setMem},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(setGamma), setGamma },
#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(led_pwm_freq), led_pwm_freq },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(wr_disbv), wr_disbv},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 5,
		sizeof(wr_ctrld), wr_ctrld},
#endif
	{DTYPE_DCS_WRITE, 1, 0, 0, 5,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc chimei_display_off_cmds[] = {
#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5,
		sizeof(wr_ctrld_off), wr_ctrld_off},
#endif
	{DTYPE_DCS_WRITE, 1, 0, 0, 5,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

static int mipi_chimei_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;

	mfd = platform_get_drvdata(pdev);
	
	if(!mfd_bkl)
	{
		mfd_bkl = mfd;
	}
	
	mipi  = &mfd->panel_info.mipi;

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (mipi->mode == DSI_VIDEO_MODE)
	{
		mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_video_on_cmds,
			ARRAY_SIZE(chimei_video_on_cmds));
	}
	else
		printk(KERN_ERR "%s: Only support DSI_VIDEO_MODE\n", __func__);

#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A

#else
	if (mipi_chimei_pdata && mipi_chimei_pdata->pmic_backlight)
		mipi_chimei_pdata->pmic_backlight(system_bl_level_old);
	else
		printk(KERN_ERR "%s: Backlight level set failed\n", __func__);
#endif

	PM_LOG_EVENT (PM_LOG_ON, PM_LOG_BL_LCD);

	return 0;
}

static int mipi_chimei_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A

#else
	if (mipi_chimei_pdata && mipi_chimei_pdata->pmic_backlight)
		mipi_chimei_pdata->pmic_backlight(0);
	else
		printk(KERN_ERR "%s: Backlight level set failed\n", __func__);
#endif

	mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_display_off_cmds,
			ARRAY_SIZE(chimei_display_off_cmds));

	if (!atomic_read(&cap_bkl_emmode_enable))
	{
		if (capkey_bl_on)
		{
			pmic_secure_mpp_config_i_sink(PM_MPP_3, PM_MPP__I_SINK__LEVEL_20mA, PM_MPP__I_SINK__SWITCH_DIS);
			PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_BL_KEYPAD);
			capkey_bl_on=0;
		}
	}

	PM_LOG_EVENT (PM_LOG_OFF, PM_LOG_BL_LCD);

	return 0;
}

#if FB_MSM_MIPI_DSI_CHIMEI_PWMINFO_PES

static char setGamma_20[31] = {0xE0,
0x01, 0x1E, 0x27, 0x2B,
0x3E, 0x3F, 0x07, 0x8D,
0x8F, 0xD2, 0x14, 0xD3,
0xD4, 0xD0, 0x19, 0x01,
0x1E, 0x27, 0x2B, 0x3E,
0x3F, 0x07, 0x8D, 0x8F,
0xD2, 0x14, 0xD3, 0xD4,
0xD0, 0x19};

static char setGamma_25[31] = {0xE0,
0x01, 0x20, 0x22, 0x32,
0x3F, 0x3F, 0x06, 0x8E,
0x8D, 0x91, 0x54, 0x12,
0x14, 0x11, 0x1A, 0x01,
0x20, 0x22, 0x32, 0x3F,
0x3F, 0x06, 0x8E, 0x8D,
0x91, 0x54, 0x12, 0x14,
0x11, 0x1A};

static struct dsi_cmd_desc chimei_cmd_gamma_default_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(setGamma), setGamma },
};

static struct dsi_cmd_desc chimei_cmd_gamma_20_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(setGamma_20), setGamma_20 },
};

static struct dsi_cmd_desc chimei_cmd_gamma_25_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(setGamma_25), setGamma_25 },
};

#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
static void chimei_lcd_bkl_onOff(unsigned int onOff)
{
	struct msm_fb_data_type *mfd = mfd_bkl;
	static struct dsi_cmd_desc chimei_cmd_backlight_on_cmds[] = {
		{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(wr_ctrld), wr_ctrld},
	};
	static struct dsi_cmd_desc chimei_cmd_backlight_off_cmds[] = {
		{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(wr_ctrld_off), wr_ctrld_off},
	};

	down(&mfd->dma->mutex);
	if (onOff)
	{
		mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_cmd_backlight_on_cmds,
		    ARRAY_SIZE(chimei_cmd_backlight_on_cmds));
	}
	else
	{
		mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_cmd_backlight_off_cmds,
		    ARRAY_SIZE(chimei_cmd_backlight_off_cmds));
	}
	up(&mfd->dma->mutex);
}
#else
static void chimei_lcd_bkl_onOff(unsigned int onOff)
{
	if (onOff)
	{
		if (mipi_chimei_pdata && mipi_chimei_pdata->pmic_backlight)
			mipi_chimei_pdata->pmic_backlight(system_bl_level_old);
		else
			printk(KERN_ERR "%s: Backlight level set failed\n", __func__);
	}
	else
	{
		if (mipi_chimei_pdata && mipi_chimei_pdata->pmic_backlight)
			mipi_chimei_pdata->pmic_backlight(0);
		else
			printk(KERN_ERR "%s: Backlight level set failed\n", __func__);
	}
}
#endif

static void chimei_lcd_onOff(unsigned int onOff)
{
	
	printk(KERN_ERR "%s = %d, function is disabled.\n", __func__, onOff);
#if 0
	down(&mfd_bkl->dma->mutex);
	if (onOff)
	{
		mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_video_on_cmds,
			ARRAY_SIZE(chimei_video_on_cmds));
	}
	else
	{
		mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_display_off_cmds,
			ARRAY_SIZE(chimei_display_off_cmds));
	}
	up(&mfd_bkl->dma->mutex);
#endif
}

static int mipi_chimei_pwm_open(struct inode *inode, struct file *file)
{
	
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_chimei_pwm_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_chimei_pwm_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	struct msm_fb_data_type *mfd = mfd_bkl;
	uint32 val, cnt;

	if((!mfd) || (!mfd->panel_power_on)) {
		return count;
	}


	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	

	sscanf(debug_buf, "%d %d", &val, &cnt);

	if (cnt <= 0)
		cnt = 1;

	
#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
	if (val > 0xff || val<0)
	{
		printk(KERN_ERR "%s: Backlight level should be 0~255\n", __func__);
		return -EINVAL;
	}
#else 
	if (val > 100 || val<0)
	{
		
		printk(KERN_ERR "%s: Backlight level should be 0~100\n", __func__);
		return -EINVAL;
	}
#endif
	system_bl_level_old = (int)val;

#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
	wr_disbv[1] = (unsigned char) system_bl_level_old;
	down(&mfd->dma->mutex);
		mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_cmd_backlight_cmds,
				ARRAY_SIZE(chimei_cmd_backlight_cmds));
	up(&mfd->dma->mutex);
#else
	if (mipi_chimei_pdata && mipi_chimei_pdata->pmic_backlight)
		mipi_chimei_pdata->pmic_backlight(system_bl_level_old);
	else
		printk(KERN_ERR "%s: Backlight level set failed\n", __func__);
#endif
	

	return count;
}

static ssize_t mipi_chimei_pwm_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;


	if (*ppos)
		return 0;	

	len = snprintf(debug_buf, sizeof(debug_buf), "pwm is %d\n", system_bl_level_old);

	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;	

	return len;
}

static const struct file_operations mipi_chimei_pwm_fops = {
	.open = mipi_chimei_pwm_open,
	.release = mipi_chimei_pwm_release,
	.read = mipi_chimei_pwm_read,
	.write = mipi_chimei_pwm_write,
};

static int mipi_chimei_mode_open(struct inode *inode, struct file *file)
{
	
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_chimei_mode_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_chimei_mode_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	struct msm_fb_data_type *mfd = mfd_bkl;
	int mode;
	uint32 val, cnt;

	if((!mfd) || (!mfd->panel_power_on)) {
		return count;
	}

	if(count < 1)
	{
		printk(KERN_ERR "%s: input invalid, count = %d\n", __func__, count);
		return -EFAULT;
	}

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	

	sscanf(debug_buf, "%d %d", &val, &cnt);

	switch(debug_buf[0])
	{
		case 'm':
			if(count<2) break;
			mode = debug_buf[1] - '0';
			
			break;

		case 'b': 
			if(debug_buf[1] == '0')
			chimei_lcd_bkl_onOff(0);
			else if(debug_buf[1] == '1')
			chimei_lcd_bkl_onOff(1);
			break;

		case 'l': 
			if (debug_buf[1] == '0')
			chimei_lcd_onOff(0);
			else if (debug_buf[1] == '1')
			chimei_lcd_onOff(1);
			break;
	}
	

	return count;
}

static const struct file_operations mipi_chimei_mode_fops = {
	.open = mipi_chimei_mode_open,
	.release = mipi_chimei_mode_release,
	.write = mipi_chimei_mode_write,
};

static int mipi_chimei_gamma_open(struct inode *inode, struct file *file)
{
	
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_chimei_gamma_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_chimei_gamma_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	struct msm_fb_data_type *mfd = mfd_bkl;
	uint32 val, cnt;

	if((!mfd) || (!mfd->panel_power_on)) {
		return count;
	}

	if(count < 1)
	{
		printk(KERN_ERR "%s: input invalid, count = %d\n", __func__, count);
		return -EFAULT;
	}

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	

	sscanf(debug_buf, "%d %d", &val, &cnt);

	
	if (val > 0xff || val<0)
	{
		printk(KERN_ERR "%s: Backlight level should be 0~255\n", __func__);
		return -EINVAL;
	}

	down(&mfd->dma->mutex);
	switch(val)
	{
		case 20:
			
			mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_cmd_gamma_20_cmds,
					ARRAY_SIZE(chimei_cmd_gamma_20_cmds));
			break;

		case 22:
			
			mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_cmd_gamma_default_cmds,
					ARRAY_SIZE(chimei_cmd_gamma_default_cmds));
			break;

		case 25:
			
			mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_cmd_gamma_25_cmds,
					ARRAY_SIZE(chimei_cmd_gamma_25_cmds));
			break;

		default:
			
			mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_cmd_gamma_default_cmds,
					ARRAY_SIZE(chimei_cmd_gamma_default_cmds));
			break;
	}
	up(&mfd->dma->mutex);
	

	return count;
}

static const struct file_operations mipi_chimei_gamma_fops = {
	.open = mipi_chimei_gamma_open,
	.release = mipi_chimei_gamma_release,
	.write = mipi_chimei_gamma_write,
};

static int mipi_chimei_capbkl_open(struct inode *inode, struct file *file)
{
	
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_chimei_capbkl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_chimei_capbkl_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 val, cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	

	sscanf(debug_buf, "%d %d", &val, &cnt);

	
	if (val > 7 || val<0)
	{
		printk(KERN_ERR "Capkey backlight value should be 0~7, %d\n",val);
		return -EINVAL;
	}

	if (val == 7)
	{
		atomic_set(&cap_bkl_emmode_enable,1);
		printk(KERN_ERR "Capkey backlight is locked in EM_mode\n");
	}

	if (val == 0)
	{
		atomic_set(&cap_bkl_emmode_enable,0);
		pmic_secure_mpp_config_i_sink(PM_MPP_3, PM_MPP__I_SINK__LEVEL_20mA, PM_MPP__I_SINK__SWITCH_DIS);
	}
	else {
		pmic_secure_mpp_config_i_sink(PM_MPP_3, val, PM_MPP__I_SINK__SWITCH_ENA);
	}
	

	return count;
}

static const struct file_operations mipi_chimei_capbkl_fops = {
	.open = mipi_chimei_capbkl_open,
	.release = mipi_chimei_capbkl_release,
	.write = mipi_chimei_capbkl_write,
};

int chimei_lcd_debugfs_init(void)
{
	struct dentry *dent = debugfs_create_dir("mipi_chimei", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return -1;
	}

	if (debugfs_create_file("pwm", 0666, dent, 0, &mipi_chimei_pwm_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("mode", 0222, dent, 0, &mipi_chimei_mode_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}
	if (debugfs_create_file("gamma", 0222, dent, 0, &mipi_chimei_gamma_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}
	if (debugfs_create_file("capbkl", 0222, dent, 0, &mipi_chimei_capbkl_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	return 0;
}
#endif

static void lcd_hx8363a_create_kernel_debuglevel_entries(void)
{
	printk(KERN_ERR "-- lcd create kernel debuglevel --\n");

	if (kernel_debuglevel_dir != NULL) {
		debugfs_create_u32("lcd_hx8363a_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&LCD_HX8363A_DLL));
	} else {
		printk(KERN_ERR "LCD_HX8363A kernel debuglevel dir falied\n");
	}
}

static int __devinit mipi_chimei_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_chimei_pdata = pdev->dev.platform_data;
		return 0;
	}

#if FB_MSM_MIPI_DSI_CHIMEI_PWMINFO_PES
	
	chimei_lcd_debugfs_init();

    	pdev_bkl_pwm = pdev;
#endif

	lcd_hx8363a_create_kernel_debuglevel_entries();

	msm_fb_add_device(pdev);

	return 0;
}

#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A

static void mipi_chimei_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	static int bl_level_old;

	mipi  = &mfd->panel_info.mipi;

	
	if (mfd->panel_power_on != TRUE) {
		return;
	}
	

	if (bl_level_old == mfd->bl_level)
	{
		return;
	}

	wr_disbv[1] = (unsigned char)(mfd->bl_level);

	down(&mfd->dma->mutex);
		mipi_dsi_cmds_tx(&chimei_tx_buf, chimei_cmd_backlight_cmds,
				ARRAY_SIZE(chimei_cmd_backlight_cmds));
	up(&mfd->dma->mutex);

	bl_level_old = mfd->bl_level;
#if FB_MSM_MIPI_DSI_CHIMEI_PWMINFO_PES
	system_bl_level_old = mfd->bl_level;
#endif

	if (!atomic_read(&cap_bkl_emmode_enable)) {
		unsigned int cap_key_bl_level = mfd->bl_level;
		cap_key_bl_level /= 10;
		cap_key_bl_level = (cap_key_bl_level > 8) ? 8 : cap_key_bl_level;
		if (cap_key_bl_level == 0)
		{
			pmic_secure_mpp_config_i_sink(PM_MPP_3, PM_MPP__I_SINK__LEVEL_20mA, PM_MPP__I_SINK__SWITCH_DIS);
			if (capkey_bl_on)
			{
				PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_BL_KEYPAD);
				capkey_bl_on=0;
			}
		}
		else
		{
			pmic_secure_mpp_config_i_sink(PM_MPP_3, (cap_key_bl_level-1), PM_MPP__I_SINK__SWITCH_ENA);
			if(!capkey_bl_on)
			{
				PM_LOG_EVENT(PM_LOG_ON, PM_LOG_BL_KEYPAD);
				capkey_bl_on=1;
			}
		}
	}
}
#else


static void mipi_chimei_set_backlight(struct msm_fb_data_type *mfd)
{
	int ret = -EPERM;
	int bl_level;

	bl_level = mfd->bl_level;

	if (mipi_chimei_pdata && mipi_chimei_pdata->pmic_backlight)
		ret = mipi_chimei_pdata->pmic_backlight(bl_level);
	else
		printk(KERN_ERR "%s: Backlight level set failed\n", __func__);

#if FB_MSM_MIPI_DSI_CHIMEI_PWMINFO_PES
	system_bl_level_old = mfd->bl_level;
#endif

	if (!atomic_read(&cap_bkl_emmode_enable)) {
		unsigned int cap_key_bl_level = mfd->bl_level;
		cap_key_bl_level /= 10;
		cap_key_bl_level = (cap_key_bl_level > 8) ? 8 : cap_key_bl_level;
		if (cap_key_bl_level == 0)
		{
			pmic_secure_mpp_config_i_sink(PM_MPP_3, PM_MPP__I_SINK__LEVEL_20mA, PM_MPP__I_SINK__SWITCH_DIS);
			if (capkey_bl_on)
			{
				PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_BL_KEYPAD);
				capkey_bl_on=0;
			}
		}
		else
		{
			pmic_secure_mpp_config_i_sink(PM_MPP_3, (cap_key_bl_level-1), PM_MPP__I_SINK__SWITCH_ENA);
			if(!capkey_bl_on)
			{
				PM_LOG_EVENT(PM_LOG_ON, PM_LOG_BL_KEYPAD);
				capkey_bl_on=1;
			}
		}
	}
}
#endif


void lsensor_update_backlight(int brightness_als)
{
	struct msm_fb_data_type *mfd = mfd_bkl;
	if(!mfd)
	{
		printk(KERN_ERR "%s: mfd_bkl = NULL (Error!!!)\n",__func__);
		return;
	}

#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
	mfd->bl_level = brightness_als;
#else
	if (brightness_als > ANDROID_MAX_BACKLIGHT_BRIGHTNESS)
		brightness_als = ANDROID_MAX_BACKLIGHT_BRIGHTNESS;

	
	mfd->bl_level = (2 * brightness_als * mfd->panel_info.bl_max + ANDROID_MAX_BACKLIGHT_BRIGHTNESS)
		/(2 * ANDROID_MAX_BACKLIGHT_BRIGHTNESS);

	if (!mfd->bl_level && brightness_als)
		mfd->bl_level = 1;
#endif
	mipi_chimei_set_backlight(mfd);
}
EXPORT_SYMBOL(lsensor_update_backlight);


static struct platform_driver this_driver = {
	.probe  = mipi_chimei_lcd_probe,
	.driver = {
		.name   = "mipi_chimei",
	},
};

static struct msm_fb_panel_data chimei_panel_data = {
	.on	= mipi_chimei_lcd_on,
	.off	= mipi_chimei_lcd_off,
	.set_backlight = mipi_chimei_set_backlight,
};

static int ch_used[3];

int mipi_chimei_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;
	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_chimei_lcd_init();
	if (ret) {
		printk(KERN_ERR "mipi_chimei_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_chimei", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	chimei_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &chimei_panel_data,
		sizeof(chimei_panel_data));
	if (ret) {
		printk(KERN_ERR "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_chimei_lcd_init(void)
{
	mipi_dsi_buf_alloc(&chimei_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&chimei_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}
