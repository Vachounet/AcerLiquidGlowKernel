/* Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/i2c/bq27520.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/msm-charger.h>

#include <mach/vreg.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/seq_file.h> 
#include <asm/atomic.h> 

#include <linux/reboot.h>	



#define CONFIG_BQ27520_TEST_ENABLE 1
#define DRIVER_VERSION			"1.1.0"

#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)


#define BQ27520_BATT_RECHARGING_THRESHOLD		90



#define	I2C_RETRY_MAX			5
#define	I2C_ADDR_NORMAL_MODE	(0xAA >> 1)
#define	I2C_ADDR_ROM_MODE		(0x16 >> 1)
#define	BQ27520_ID				0x0520
#define	BQ27520_FIRMWARE		0x0302
#define	BQ27520_HARDWARE		0x00a3

enum i2c_addr_mode {
	NORMAL_MODE,
	ROM_MODE,
};

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27520_device_info {
	struct device				*dev;
	int					id;
	struct bq27520_access_methods		*bus;
	struct i2c_client			*client;
	const struct bq27520_platform_data	*pdata;
	struct work_struct			counter;
	/* 300ms delay is needed after bq27520 is powered up
	 * and before any successful I2C transaction
	 */
	struct  delayed_work			hw_config;

	struct delayed_work rechg_mon_work;


	struct wake_lock bat_low_lock;

};

struct bq27520_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27520_device_info *di);
};

enum {
	GET_BATTERY_STATUS,
	GET_BATTERY_TEMPERATURE,
	GET_BATTERY_VOLTAGE,
	GET_BATTERY_CAPACITY,

	GET_BATTERY_CURRENT,
	GET_BATTERY_TIME_TO_EMPTY_NOW,
	GET_BATTERY_TIME_TO_EMPTY_AVG,
	GET_BATTERY_TIME_TO_FULL,

	NUM_OF_STATUS,
};

struct bq27520_status {
	/* Informations owned and maintained by Bq27520 driver, updated
	 * by poller or SOC_INT interrupt, decoupling from I/Oing
	 * hardware directly
	 */
	int			status[NUM_OF_STATUS];
	spinlock_t		lock;
	int polling_period;
	struct delayed_work	poller;
};

static struct bq27520_status curr_batt_status;
static struct bq27520_device_info *bq27520_di;
static u32 soc_irq;
static u32 bat_low_irq;
static int coulomb_counter;
static spinlock_t lock; /* protect access to coulomb_counter */
static struct timer_list timer; /* charge counter timer every 30 secs */

struct dev_chip_info {
	int dev_id;
	int fw_ver;
	int hw_ver;
};
static struct dev_chip_info bq27520_chip;


extern int QnooffchargeVariable;



static atomic_t it_enable_status = ATOMIC_INIT(0);


static int bq27520_read(u8 reg, int *rt_value, int b_single,
			struct bq27520_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}



static int bq27520_i2c_txsubcmd(u8 reg, unsigned short subcmd, struct bq27520_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	if (i2c_transfer(di->client->adapter, &msg, 1) < 0)
		return -EIO;

	return 0;
}

static int bq27520_cntl_cmd(struct bq27520_device_info *di, int subcmd)
{
#if 1
	int ret, i;

	for(i = 0; i < I2C_RETRY_MAX; i++) {
		ret = bq27520_i2c_txsubcmd(BQ27520_REG_CNTL, subcmd, di);
		if(ret < 0) {
			dev_err(di->dev, "-%d-: error %d writing cntl subcmd(%d) failed\n", i + 1 ,ret, subcmd);
			msleep(5);
		} else {
			msleep(2);
			return 0;
		}
}

	return ret;
#else
	int ret;

	ret = bq27520_i2c_txsubcmd(BQ27520_REG_CNTL, subcmd, di);
	if (ret < 0) {
		dev_err(di->dev, "error %d writing cntl command(%d) failed\n", ret, subcmd);
	return ret;
	}

	return 0;
#endif
}

#ifdef BQ275250_DFI_SUPPORT



#define BQ27520_DFI_USED_TO_PROJECT			0X4F52
#define BQ27520_DFI_MACH_INFO_LENGTH		12
#define BQ27520_DFI_MACH_NUM				6

#if defined(CONFIG_MACH_EVT3)
#define BQ27520_DFI_EVT						2
#define BQ27520_DFI_VERSION					0X0004
#elif defined(CONFIG_MACH_EVT2)
#define BQ27520_DFI_EVT						1
#define BQ27520_DFI_VERSION					0X0004
#elif defined(CONFIG_MACH_EVT1)
#define BQ27520_DFI_EVT						0
#define BQ27520_DFI_VERSION					0X0001
#endif

enum{
	BQ27520_MACH_EVT1,
	BQ27520_MACH_EVT2,
	BQ27520_MACH_EVT3,
	BQ27520_MACH_EVT4,
	BQ27520_MACH_EVT5,
	BQ27520_MACH_EVT6,
};

static u8 *yDataFlashImage;

static int bq27520_dfi_read_i2c(u8 reg, u8 *rt_value, u8 len,
			struct bq27520_device_info *di)
{
	struct i2c_client *client = di->client;
	int err;
	struct i2c_msg msgs[] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.buf	= (void *)&reg,
			.len	= 1,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.buf	= (void *)rt_value,
			.len	= len,
		},
	};

	if (!client->adapter)
		return -ENODEV;

	err = i2c_transfer(client->adapter, msgs, 2); 

	return err;
}

static bool bq27520_dfi_read_retry(u8 addr_mode, u8 reg, u8 *buf, u8 len)
{
	int i, ret;

	if (addr_mode == NORMAL_MODE)
		(bq27520_di->client)->addr = I2C_ADDR_NORMAL_MODE;
	else if (addr_mode == ROM_MODE)
		(bq27520_di->client)->addr = I2C_ADDR_ROM_MODE;
	else {
		dev_err(bq27520_di->dev, "%s: i2c addr mode(%d) error\n", __func__, addr_mode);
		return FALSE;
	}

	for(i = 0; i < I2C_RETRY_MAX; i++) {
		ret = bq27520_dfi_read_i2c(reg, buf, len, bq27520_di);
		if (ret < 0) {
				msleep(5);
			} else {
				msleep(2);
				return TRUE;
		}
	}

	return FALSE;
}

static int bq27520_dfi_write_i2c(u8 reg, u8 *wt_value, u8 len,
				struct bq27520_device_info *di)
{
	struct i2c_client *client = di->client;
	int i;
	unsigned char buf_w[100];
	struct i2c_msg msgs[] = {
		[0] = {
			.addr   = client->addr,
			.flags  = 0,
			.buf    = (void *)buf_w,
			.len    = len + 1
		}
	};

	if(len >= sizeof(buf_w))  
		return -ENOMEM;

	

	memset(buf_w, 0, sizeof(buf_w));

	buf_w[0] = reg;

	for(i = 0; i < len; i++) {
		buf_w[i + 1] = wt_value[i];
	
	}

	return i2c_transfer(client->adapter, msgs, 1);
}

static bool bq27520_dfi_write_retry(u8 addr_mode, u8 reg, u8 *buf, u8 len)
{
	int i, ret;

	if (addr_mode == NORMAL_MODE)
		(bq27520_di->client)->addr = I2C_ADDR_NORMAL_MODE;
	else if (addr_mode == ROM_MODE)
		(bq27520_di->client)->addr = I2C_ADDR_ROM_MODE;
	else {
		dev_err(bq27520_di->dev, "%s: i2c addr mode(%d) error\n", __func__, addr_mode);
		return FALSE;
	}

	for(i = 0; i < I2C_RETRY_MAX; i++) {
		ret = bq27520_dfi_write_i2c(reg, buf, len, bq27520_di);
		if(ret < 0) {
			msleep(5);
		} else {
			msleep(2);
			return TRUE;
		}
	}
	return FALSE;
}

static bool bq27520_enable_rom_mode(bool enable)
{
	u8 exit_rom[2] = {0x0F, 0x00};
	u8 enter_rom[2] = {0x00, 0x0F};
	int ret;

	if (enable) {
		ret = bq27520_dfi_write_retry(NORMAL_MODE, 0x00, enter_rom, sizeof(enter_rom));
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write 0x0f00 with command 0x00 failed\n", __func__);
			return FALSE;
		}
	} else {
		ret = bq27520_dfi_write_retry(ROM_MODE, 0x00, &exit_rom[0], 1);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write 0x0f with command 0x00 failed\n", __func__);
			return FALSE;
		}

		ret = bq27520_dfi_write_retry(ROM_MODE, 0x64, &exit_rom[0], 1);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write 0x0f with command 0x64 failed\n", __func__);
			return FALSE;
		}

		ret = bq27520_dfi_write_retry(ROM_MODE, 0x65, &exit_rom[1], 1);
		if(!ret) {
			dev_err(bq27520_di->dev, "%s: write 0x00 with command 0x65 failed\n", __func__);
			return FALSE;
		}
	}

	return TRUE;
}

static bool bq27520_erase_instruction_flash(void)
{
	u8 write_buf[2] = {0x03, 0x00}, *write_buf_ptr = write_buf;
	u8  read_buf = 0xFF, *read_buf_ptr = &read_buf;
	int i, ret;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --start--\n", __func__);

	for(i = 0; i < 3; i++) {
		
		ret = bq27520_dfi_write_retry(ROM_MODE, 0x00, write_buf_ptr, 1);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data 0x00 with command 0x03 failed\n",
				__func__);
			return ret;
		}
		
		 ret = bq27520_dfi_write_retry(ROM_MODE, 0x64, write_buf_ptr, 2);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data 0x03,0x00 with command 0x64 failed\n",
				__func__);
			return ret;
		}

		
		msleep(20);

		ret = bq27520_dfi_read_retry(ROM_MODE, 0x66, read_buf_ptr, 1);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: read  data with command 0x66 failed\n",
				__func__);
			return ret;
		}

		if(*read_buf_ptr == 0x00){
			break;
		}
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --end--\n", __func__);

	return TRUE;
}

static bool bq27520_erase_data_flash(void)
{
	u8 write_buf[2], *write_buf_ptr = write_buf;
	u8 read_buf = 0xFF, *read_buf_ptr = &read_buf;
	int i, ret;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --start--\n", __func__);

	for(i = 0; i < 3; i++) {
		
		*write_buf_ptr = 0x0C;

		ret = bq27520_dfi_write_retry(ROM_MODE, 0x00, write_buf_ptr, 1);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data 0x0c with command 0x00 failed\n",
				__func__);
			return ret;
		}

		
		*write_buf_ptr = 0x83;
		*(write_buf_ptr + 1) = 0xDE;

		ret = bq27520_dfi_write_retry(ROM_MODE, 0x04, write_buf_ptr, 2);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data 0x83, 0xde with command 0x04 failed\n",
				__func__);
			return ret;
		}

		
		*write_buf_ptr = 0x6D;
		
		*(write_buf_ptr + 1) = 0x01;

		ret = bq27520_dfi_write_retry(ROM_MODE, 0x64, write_buf_ptr, 2);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data 0x6d, 0x01 with command 0x64 failed\n",
				__func__);
			return ret;
		}

		
		msleep(200);

		
		ret = bq27520_dfi_read_retry(ROM_MODE, 0x66, read_buf_ptr, 1);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: read  data  with command 0x66 failed\n",
				__func__);
			return ret;
		}

		if(read_buf == 0x00) {
			break;
		}
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --end--\n", __func__);

	return TRUE;
}

static bool bq27520_dfi_write_data_flash(u16 *DFI_checksum)
{
	u8 read_buf, *read_buf_ptr = &read_buf;
	u8 write_buf[2], *write_buf_ptr = write_buf;
	u8 row, i, num, *dfi_ptr = (u8 *)yDataFlashImage;
	u16 checksum;
	u32 sum_of_dfi_buf;
	bool ret;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --start--\n", __func__);

	for(i = 0; i < 3; i++){
		for(row = 0; row < 0x400 / 32; row++) {
			
			*write_buf_ptr = 0x0A;
			*(write_buf_ptr + 1) = row;

			ret = bq27520_dfi_write_retry(ROM_MODE, 0x00, write_buf_ptr, 2);
			if (!ret) {
				dev_err(bq27520_di->dev, "%s: write data 0x0a,row with command 0x00 failed\n",
					__func__);
				return ret;
			}

			
			ret = bq27520_dfi_write_retry(ROM_MODE, 0x04, (dfi_ptr + row * 32), 32);
			if (!ret) {
				dev_err(bq27520_di->dev, "%s: write data DFI with command 0x04 failed\n",
					__func__);
				return ret;
			}
			sum_of_dfi_buf = 0;

			for(num = 0; num < 32; num++) {
				sum_of_dfi_buf += *(dfi_ptr + row * 32 + num);
			}

			checksum = (u16)((0x0A + row + sum_of_dfi_buf) % 0x10000);
			*DFI_checksum = (u16)((*DFI_checksum + sum_of_dfi_buf) % 0x10000);

			
			*write_buf_ptr = checksum & 0x00FF;
			*(write_buf_ptr + 1) = (checksum & 0xFF00) >> 8;

			ret = bq27520_dfi_write_retry(ROM_MODE, 0x64,  write_buf_ptr, 2);
			if (!ret) {
				dev_err(bq27520_di->dev, "%s: write data LSB, MSB, row with command 0x64 failed\n",
					__func__);
				return ret;
			}

			
			
			msleep(2);

			*read_buf_ptr = 0xFF;

			ret = bq27520_dfi_read_retry(ROM_MODE, 0x66, read_buf_ptr, 1);
			if (!ret) {
				dev_err(bq27520_di->dev, "%s: read data with command 0x66 failed\n", __func__);
				return ret;
			}

			if(*read_buf_ptr != 0x00) {
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: start writing image, read 0x66\n", __func__);
				break;
			}
		}

		if(row < 0x400 / 32) {
			if(!bq27520_erase_data_flash()) {
				return FALSE;
			}
		}
		else {
			return TRUE;
		}
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --end--\n", __func__);

	return FALSE;
}


static u8 bq27520_update_DFI(void)
{
	u8 instruction_bak[2][96], *instruction_bak_ptr = (u8 *)instruction_bak;
	u8 write_buf[2], *write_buf_ptr = write_buf;
	u8 read_buf[2] = {0xFF, 0xFF}, *read_buf_ptr = read_buf;
	u16 DFI_checksum, DFI_checksum_RB, checksum;
	u32 sum_of_dfi_bak;
	int row, i, num;
	bool ret;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --read and erase instruction flash--\n", __func__);

	if (!bq27520_enable_rom_mode(TRUE)) {
		dev_err(bq27520_di->dev, "%s: Can't enter rom mode\n", __func__);
		return FALSE;
	}

 	 
	for (row = 0; row < 2; row++) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Backup Instruction Flash (%d)\n", __func__, row);

		*write_buf_ptr = 0x00;

		ret = bq27520_dfi_write_retry(ROM_MODE, 0x00, write_buf_ptr, 1);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data 0x00 with command 0x00 failed\n",
				__func__);
			return ret;
		}

		*write_buf_ptr = row;  
		*(write_buf_ptr + 1) = 0x00; 

		
		
		ret = bq27520_dfi_write_retry(ROM_MODE, 0x01, write_buf_ptr, 2);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data row,0x00 with command 0x01 failed\n",
				__func__);
			return ret;
		}

	   	
		ret = bq27520_dfi_write_retry(ROM_MODE, 0x64, write_buf_ptr, 2);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data row,0x00 with command 0x64 failed\n",
				__func__);
			return ret;
		}

	    
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: read instruction flash into instruction_bak\n",
			__func__);

		ret = bq27520_dfi_read_retry(ROM_MODE, 0x04,
			instruction_bak_ptr + row * sizeof(instruction_bak[row]),
			sizeof(instruction_bak[row]));
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: read instruction flash failed\n", __func__);
			return ret;
		}

		
		msleep(20);
	}

	if (!bq27520_erase_instruction_flash()) {
		dev_err(bq27520_di->dev, "%s: erase instruction flash failed\n", __func__);
		return FALSE;
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --start writing image--\n", __func__);

	for (i = 0; i < 3; i++) {
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: Update DFI (%d)\n", __func__, i);

		DFI_checksum = 0;

		if (!bq27520_erase_data_flash()) {
			dev_err(bq27520_di->dev, "%s: erase data flash failed\n", __func__);
			return FALSE;
		}

		if (!bq27520_dfi_write_data_flash(&DFI_checksum)) {
			dev_err(bq27520_di->dev, "%s: write dfi data flash failed\n", __func__);
			return FALSE;
		}

		
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --setup data flash checksum--\n", __func__);

		*write_buf_ptr = 0x08;
		*(write_buf_ptr + 1) = 0x00;

		ret = bq27520_dfi_write_retry(ROM_MODE, 0x00, write_buf_ptr, 1);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data 0x08 with command 0x00 failed\n",
				__func__);
			return ret;
		}

		ret = bq27520_dfi_write_retry(ROM_MODE, 0x64, write_buf_ptr, 2);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: write data 0x08,0x00 with command 0x64 failed\n",
				__func__);
			return ret;
		}

		
		msleep(20);

		
		ret = bq27520_dfi_read_retry(ROM_MODE, 0x04, read_buf_ptr, 2);
		if (!ret) {
			dev_err(bq27520_di->dev, "%s: read data with command 0x04 failed\n", __func__);
			return ret;
		}

		DFI_checksum_RB = ((read_buf[1] << 8) & 0xFF00) | read_buf[0];

		
		if (DFI_checksum == DFI_checksum_RB) {
			msm_chg_printk(DBG_MSM_CHG_DFI, "%s: DFI checksum verify correct\n", __func__);
			break;
		} else {
			msm_chg_printk(DBG_MSM_CHG_DFI, "%s: DFI_checksum(%d) != DFI_checksum_RB(%d)\n",
				__func__, DFI_checksum, DFI_checksum_RB);
		}

	}

	if (i >= 3) {
		dev_err(bq27520_di->dev, "%s: Update DFI 3 times failed\n", __func__);
		return FALSE;
	}

	  
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --end writing image--\n", __func__);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: --restore instruction flash--\n", __func__);

	for (row = 1; row >= 0; row--) {
		for (i = 0; i < 3; i++) {
			
			*write_buf_ptr = 0x02;

			ret = bq27520_dfi_write_retry(ROM_MODE, 0x00, write_buf_ptr, 1);
			if (!ret) {
				dev_err(bq27520_di->dev, "*%s: write data 0x02 with command 0x00 failed\n",
					__func__);
				return ret;
			}

			
			*write_buf_ptr = row;
			*(write_buf_ptr + 1) = 0x00;

			ret = bq27520_dfi_write_retry(ROM_MODE, 0x01, write_buf_ptr, 2);
			if (!ret) {
				dev_err(bq27520_di->dev, "%s: write data row,0x00 with command 0x01 failed\n",
					__func__);
				return ret;
			}
			
			ret = bq27520_dfi_write_retry(ROM_MODE, 0x04, instruction_bak_ptr + row * sizeof(instruction_bak[row]), sizeof(instruction_bak[row]));
			if (!ret) {
				dev_err(bq27520_di->dev, "%s: write data IFrowdata with command 0x04 failed\n",
					__func__);
				return ret;
			}

			
			sum_of_dfi_bak = 0;
			for(num = 0; num < 96; num++) {
				sum_of_dfi_bak += *(instruction_bak_ptr + row * sizeof(instruction_bak[row]) + num);
			}

			
			checksum = (u16)((0x02 + row + sum_of_dfi_bak) % 0x10000);

			*write_buf_ptr = checksum & 0x00FF;
			*(write_buf_ptr + 1) = (checksum & 0xFF00) >> 8;

			ret = bq27520_dfi_write_retry(ROM_MODE, 0x64, write_buf_ptr, 2);
			if (!ret) {
				dev_err(bq27520_di->dev, "%s: write data LSB, MSB with command 0x64 failed\n",
					__func__);
				return ret;
			}

			
			msleep(20);

			
			*read_buf_ptr = 0xFF;

			ret = bq27520_dfi_read_retry(ROM_MODE, 0x66, read_buf_ptr, 1);
			if (!ret) {
				dev_err(bq27520_di->dev, "%s: read data with command 0x66 failed\n", __func__);
				return ret;
			}

			if (read_buf[0] == 0x00){
				break;
			}
		}

		
		if (i >= 3) {
			return FALSE;
		}
	}

	return TRUE;
}

u8 bq27520_version_DFI(void)
{
	bool ret = FALSE;
	u8 version;
	u8 block_data_control = 0x00, data_flash_class = 0x39, data_flash_block = 0x00;

	ret = bq27520_dfi_write_retry(NORMAL_MODE, 0x61, &block_data_control, 1);
	if (!ret) {
		dev_err(bq27520_di->dev, "%s: write data 0x00 with command 0x61 failed\n", __func__);
		return ret;
	}

	ret = bq27520_dfi_write_retry(NORMAL_MODE, 0x3E, &data_flash_class, 1);
	if (!ret) {
		dev_err(bq27520_di->dev, "%s: write data 0x39 with command 0x3e failed\n", __func__);
		return ret;
	}

	ret = bq27520_dfi_write_retry(NORMAL_MODE, 0x3F, &data_flash_block, 1);
	if (!ret) {
		dev_err(bq27520_di->dev, "%s: write data 0x00 with command 0x3f failed\n", __func__);
		return ret;
	}

	ret = bq27520_dfi_read_retry(NORMAL_MODE, 0x40, &version, 1);
	if (!ret) {
		dev_err(bq27520_di->dev, "%s: read data with command 0x40 failed\n", __func__);
		return ret;
	} else
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: ---bq27520 DFI version=0x%x---\n",__func__, version);

	return version;
}

static u8 bq27520_calibration_DFI(void)
{
	int ret, val = 0, cmd = 0;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	
	ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_DEVCIE_TYPE);
	if (ret < 0) {
		dev_err(bq27520_di->dev, "error %d writing subcmd device id failed\n", ret);
		return FALSE;
	}
	msleep(20);

	ret = bq27520_read(BQ27520_REG_CNTL, &val, 0, bq27520_di);
	if (ret) {
		dev_err(bq27520_di->dev, "error %d reading device id failed\n", ret);
		return FALSE;
	}

	if ( val != BQ27520_ID) {
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: check gauge id(0x%x) error!!\n", __func__, val);
		return FALSE;
	} else {
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: check gauge id(0x%x) success!!\n", __func__, val);
	}

	
	ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_FW_VER);
	if (ret < 0) {
		dev_err(bq27520_di->dev, "error %d writing subcmd firmware version failed\n", ret);
		return FALSE;
	}
	msleep(20);

	ret = bq27520_read(BQ27520_REG_CNTL, &val, 0, bq27520_di);
	if (ret) {
		dev_err(bq27520_di->dev, "error %d reading firmware version failed\n", ret);
		return FALSE;
	}

	if ( val != BQ27520_FIRMWARE) {
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: check firmware version(0x%x) error!!\n",
			__func__, val);
		return FALSE;
	} else {
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: check firmware version(0x%x) success!!\n",
			__func__, val);
	}

	
	ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_HW_VER);
	if (ret < 0) {
		dev_err(bq27520_di->dev, "error %d writing subcmd hardware version failed\n", ret);
		return FALSE;
	}
	msleep(20);

	ret = bq27520_read(BQ27520_REG_CNTL, &val, 0, bq27520_di);
	if (ret) {
		dev_err(bq27520_di->dev, "error %d reading hardware version failed\n", ret);
	return FALSE;
	}

	if ( val != BQ27520_HARDWARE) {
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: check hardware version(0x%x) error!!\n",
			__func__, val);
		return FALSE;
	} else {
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: check hardware version(0x%x) success!!\n",
			__func__, val);
	}

	ret = bq27520_update_DFI();
	if (!ret) {
		dev_err(bq27520_di->dev, "%s: ****** write Gauge data flash failed.******\n",
			__func__);
		return FALSE;
	}

	
	if (!bq27520_enable_rom_mode(FALSE)) {
		return FALSE;
	}

	msleep(20);

	
	cmd = 0x0041;

	if (!bq27520_dfi_write_retry(NORMAL_MODE, 0x00, (u8 *)&cmd, 2)) {
	 	return FALSE;
	}

	msleep(50);

	

	bq27520_version_DFI();

	return TRUE;

}

u8 bq27520_force_calibration_DFI(void)
{
	int ret, cmd = 0;

	bq27520_update_DFI();

	
	if (!bq27520_enable_rom_mode(FALSE)) {
		return FALSE;
	}

	msleep(20);

	
	cmd = 0x0041;

	if (!bq27520_dfi_write_retry(NORMAL_MODE, 0x00, (u8 *)&cmd, 2)) {
	 	return FALSE;
	}

	msleep(50);

	
	ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_ENABLE_IT);
	if (ret < 0) {
		dev_err(bq27520_di->dev, "error %d writing cntl command failed\n", ret);
		return FALSE;
	}

	bq27520_version_DFI();

	return TRUE;
}

static int bq27520_copy_dfi_user_to_kernel(const char *val, u16 size)
{
	const char *dfi_ptr = val;
	int i = 0, ret = 0;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	cancel_delayed_work_sync(&curr_batt_status.poller);

	yDataFlashImage = kmalloc(sizeof(u8)*size, GFP_KERNEL);
	if (yDataFlashImage == NULL) {
		dev_err(bq27520_di->dev, "%s: could'nt request enough memory!\n", __func__);
		return FALSE;
	}

	memset(yDataFlashImage, 0, sizeof(u8) *size);
	memcpy(yDataFlashImage, dfi_ptr, sizeof(u8) *size);

	while (i < size) {
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%02x ", *(yDataFlashImage + i));
		i++;
		if ((i % 16) == 0)
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "\n");
	}
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "\nDFI size is %d\n", (size - 1));

	
	ret = bq27520_calibration_DFI();
	if (!ret) {
		dev_err(bq27520_di->dev, "%s: ******calibration DFI failed.******\n", __func__);
		kfree(yDataFlashImage);
		return FALSE;
	} else
		msm_chg_printk(DBG_MSM_CHG_DFI, "%s: === DFI write completed ===\n", __func__);

	

	msleep(20);

	kfree(yDataFlashImage);

	
	schedule_delayed_work(&curr_batt_status.poller, 4000);

	return TRUE;
}
#endif


/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27520_battery_temperature(struct bq27520_device_info *di)
{
	int ret, temp = 0;

	ret = bq27520_read(BQ27520_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error %d reading temperature\n", ret);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: Temp=%d--\n",
		__func__, (temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN));

	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27520_battery_voltage(struct bq27520_device_info *di)
{
	int ret, volt = 0;

	ret = bq27520_read(BQ27520_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error %d reading voltage\n", ret);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: Volt=%d--\n", __func__, volt);

	return volt;
}


static int bq27520_battery_rsoc(struct bq27520_device_info *di)
{
	int ret, rsoc = 0;

	ret = bq27520_read(BQ27520_REG_SOC, &rsoc, 0, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading relative State-of-Charge\n", ret);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: SOC=%d--\n", __func__, rsoc);

	return rsoc;
}


static int bq27520_battery_remain_capacity(struct bq27520_device_info *di)
{
	int ret, rm = 0;

	ret = bq27520_read(BQ27520_REG_RM, &rm, 0, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading relative Remaining Capacity\n", ret);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: RM=%d--\n", __func__, rm);

	return rm;
}


static int bq27520_battery_full_charge_capacity(struct bq27520_device_info *di)
{
	int ret, fcc = 0;

	ret = bq27520_read(BQ27520_REG_FCC, &fcc, 0, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading relative Full Charge Capacity\n", ret);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: RM=%d--\n", __func__, fcc);

	return fcc;
}



static int bq27520_battery_curr(struct bq27520_device_info *di, int *value)
{
	int ret, curr = 0;

	ret = bq27520_read(BQ27520_REG_AI, &curr, 0, di);
	if (ret) {
		dev_err(di->dev,
			"error %d reading relative Average-Current\n", ret);
		return ret;
	}

	*value = (int)(s16)curr;

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: Curr=%d--\n", __func__, *value);

	return 0;
}


static int bq27520_battery_time_to_empty(struct bq27520_device_info *di)
{
	int ret, times = 0;

	ret = bq27520_read(BQ27520_REG_TTE, &times, 0, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading relative Time-to-Empty\n", times);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: TTE=%d--\n", __func__, times);

	return times;
}


static int bq27520_battery_TTE_at_constant_power(struct bq27520_device_info *di)
{
	int ret, times = 0;

	ret = bq27520_read(BQ27520_REG_TTECP, &times, 0, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading relative TTE-at-Constant-Power\n", times);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: TTEC=%d--\n", __func__, times);

	return times;
}


static int bq27520_battery_time_to_full(struct bq27520_device_info *di)
{
	int ret, times = 0;

	ret = bq27520_read(BQ27520_REG_TTF, &times, 0, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading relative Time-to-Full\n", times);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: TTF=%d--\n", __func__, times);

	return times;
}


static int bq27520_battery_flags(struct bq27520_device_info *di)
{
	int ret, flags = 0;

	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 0, di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: flags=0x%x\n", __func__, flags);
	if (ret) {
		dev_err(di->dev, "error %d reading register %02x\n", ret, BQ27520_REG_FLAGS);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: flags=0x%x--\n", __func__, flags);

	return flags;
}



static int bq27520_battery_extcmd_APPSTAT(struct bq27520_device_info *di)
{
	int ret, temp = 0;

	ret = bq27520_read(BQ27520_EXTCMD_APP_STATUS, &temp, 1, di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: APPSTAT=0x%x\n", __func__, temp);
	if (ret) {
		dev_err(di->dev, "error %d reading register %02x\n",
			ret, BQ27520_EXTCMD_APP_STATUS);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: APPSTAT=0x%x--\n", __func__, temp);

	return temp;
}


static int bq27520_battery_extcmd_OPERCON(struct bq27520_device_info *di)
{
	int ret, temp = 0;

	ret = bq27520_read(BQ27520_EXTCMD_OPERATOR_CONFIG, &temp, 1, di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: OPERCON=0x%x\n", __func__, temp);
	if (ret) {
		dev_err(di->dev, "error %d reading register %02x\n",
			ret, BQ27520_EXTCMD_APP_STATUS);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: OPERCON=0x%x--\n", __func__, temp);

	return temp;
}

/*
 * Return the battery Relative Nominal Available Capacity
 * Or < 0 if something fails.
 */
static int bq27520_battery_NAC(struct bq27520_device_info *di)
{
	int ret, temp = 0;

	ret = bq27520_read(BQ27520_REG_NAC, &temp, 0, di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: NAC=0x%x\n", __func__, temp);
	if (ret) {
		dev_err(di->dev, "error %d reading register %02x\n",
			ret, BQ27520_EXTCMD_APP_STATUS);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: NAC=0x%x--\n", __func__, temp);

	return temp;
}

/*
 * Return the battery Relative State of Health
 * Or < 0 if something fails.
 */
static int bq27520_battery_SOH(struct bq27520_device_info *di)
{
	int ret, temp = 0;

	ret = bq27520_read(BQ27520_REG_SOH, &temp, 0, di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: SOH=0x%x\n", __func__, temp);
	if (ret) {
		dev_err(di->dev, "error %d reading register %02x\n",
			ret, BQ27520_EXTCMD_APP_STATUS);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_UPDATE, "--%s: SOH=0x%x--\n", __func__, temp);

	return temp;
}



int engineer_gauge_remain_capacity(void)
{
	int ret = 0;

	ret = bq27520_battery_remain_capacity(bq27520_di);

	return ret;
}

int engineer_gauge_full_charge_capacity(void)
{
	int ret = 0;

	ret = bq27520_battery_full_charge_capacity(bq27520_di);

	return ret;
}

int engineer_gauge_fw_version(void)
{
	int ret = 0, fw_version = 0;;

	if(bq27520_chip.fw_ver != BQ27520_FIRMWARE) {
		ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_FW_VER);
		if (ret < 0) {
			dev_err(bq27520_di->dev, "error %d writing subcomd firmware version failed\n", ret);
			return ret;
		}
		udelay(66);

		ret = bq27520_read(BQ27520_REG_CNTL, &fw_version, 0, bq27520_di);
		if(ret) {
			dev_err(bq27520_di->dev, "error %d reading device id failed\n", ret);
			return ret;
		}

		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: fw_version=0x%x\n", __func__, fw_version);

		if (fw_version != BQ27520_FIRMWARE)
			fw_version = 0;
	}
	else
		fw_version = bq27520_chip.fw_ver;

	return fw_version;
}

int engineer_gauge_flags(void)
{
	int ret = 0;

	ret = bq27520_battery_flags(bq27520_di);

	return ret;
}

int engineer_gauge_cntl_status(void)
{
	int ret = 0;

	ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_CTNL_STATUS);
	if (ret < 0) {
		dev_err(bq27520_di->dev, "error %d writing subcmd ctnl status failed\n", ret);
		return FALSE;
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "--%s: cntl_status=%d--\n", __func__, ret);

	return ret;
}



static int bq27520_chip_config(struct bq27520_device_info *di)
{
	int flags = 0, ret = 0;

	if (di->pdata->enable_it || di->pdata->enable_dlog) {
		ret = bq27520_cntl_cmd(di, BQ27520_SUBCMD_CTNL_STATUS);
		if (ret < 0) {
			dev_err(di->dev, "error %d writing subcmd cntl status failed\n", ret);
			return ret;
		}
		udelay(66);

		ret = bq27520_read(BQ27520_REG_CNTL, &flags, 0, di);
		if (ret < 0) {
			dev_err(di->dev, "error %d reading register %02x\n",
				 ret, BQ27520_REG_CNTL);
			return ret;
		}
		udelay(66);

		
		msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: control_status=%d\n", __func__, flags);

		if (di->pdata->enable_it && !(flags & BQ27520_CS_QEN)) {
			ret = bq27520_cntl_cmd(di, BQ27520_SUBCMD_ENABLE_IT);
			if (ret < 0) {
				dev_err(di->dev, "error %d writing subcmd enable it failed\n", ret);
				return ret;
			}

			msm_chg_printk(DBG_MSM_CHG_PROBE, "--%s,%d: Enable IT--\n", __func__, __LINE__);
			udelay(66);

			ret = bq27520_cntl_cmd(di, BQ27520_SUBCMD_RESET);
			if (ret < 0) {
				dev_err(di->dev, "error %d writing subcomd reset failed\n", ret);
				return ret;
			}
			msm_chg_printk(DBG_MSM_CHG_PROBE, "--%s,%d: RESET--\n", __func__, __LINE__);
		}
		

		if (di->pdata->enable_dlog && !(flags & BQ27520_CS_DLOGEN)) {
			ret = bq27520_cntl_cmd(di, BQ27520_SUBCMD_ENABLE_DLOG);
			if (ret < 0) {
				dev_err(di->dev, "error %d writing subcomd enable dlog failed\n", ret);
				return ret;
			}

			msm_chg_printk(DBG_MSM_CHG_PROBE, "--%s,%d: Enable DLOG--\n", __func__, __LINE__);
			udelay(66);
		}
	}

	return 0;
}

static void bq27520_every_30secs(unsigned long data)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;

	msm_chg_printk(DBG_MSM_CHG_WROKQUEUE, "--%s--\n", __func__);

	schedule_work(&di->counter);
	mod_timer(&timer, jiffies + (HZ * BQ27520_COULOMB_POLL));
}

static void bq27520_coulomb_counter_work(struct work_struct *work)
{
	int value = 0, temp = 0, index = 0, ret = 0, count = 0;
	struct bq27520_device_info *di;
	unsigned long flags;

	msm_chg_printk(DBG_MSM_CHG_WROKQUEUE, "--%s--\n", __func__);

	di = container_of(work, struct bq27520_device_info, counter);

	/* retrieve 30 values from FIFO of coulomb data logging buffer
	 * and average over time
	 */
	do {
		ret = bq27520_read(BQ27520_REG_LOGBUF, &temp, 0, di);
		if (ret < 0)
			break;
		if (temp != 0x7FFF) {
			++count;
			value += temp;
		}

		udelay(66);

		ret = bq27520_read(BQ27520_REG_LOGIDX, &index, 0, di);
		if (ret < 0)
			break;

		udelay(66);

	} while (index != 0 || temp != 0x7FFF);

	if (ret < 0) {
		dev_err(di->dev, "Error %d reading datalog register\n", ret);
		return;
	}

	if (count) {
		spin_lock_irqsave(&lock, flags);
		coulomb_counter = value/count;
		spin_unlock_irqrestore(&lock, flags);
	}
}

static int bq27520_is_battery_present(void)
{
#if 1

	int ret, flags = 0;

	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 0, bq27520_di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: flags=0x%x\n", __func__, flags);
	if (ret) {
		dev_err(bq27520_di->dev, "error %d reading flags\n", ret);
		return ret;
	}

	
	if (!(flags & BQ27520_FLAG_BAT_DET) && !(flags & BQ27520_FLAG_OCV_GD)) {
		ret = bq27520_battery_temperature(bq27520_di);
		if (ret < BATT_INVALID_TEMP_THRESHOLD) {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Battery absent!!\n", __func__);
			return 0;
		}

		if (flags & BQ27520_FLAG_WAIT_ID) {
			msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Waiting to identity inserted battery!!\n",
				__func__);
		}
	}

	return 1;

#else
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	return 1;
#endif
}

static int bq27520_is_battery_temp_within_range(void)
{
#if 1

	int ret, flags = 0, temp = 0;
	static int overtemp = 0;

	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 0, bq27520_di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: flags=0x%x\n", __func__, flags);
	if (ret) {
		dev_err(bq27520_di->dev, "error %d reading temperature\n", ret);
		return ret;
	}

	
	if ((flags & BQ27520_FLAG_CHG_INH) &&
		((flags & BQ27520_FLAG_BAT_DET) || (flags & BQ27520_FLAG_OCV_GD))) {
		temp = bq27520_battery_temperature(bq27520_di);
		if (temp > BATT_INVALID_TEMP_THRESHOLD) {
			if ((temp < DEFAULT_BATT_LOW_TEMP || temp > DEFAULT_BATT_HIGH_TEMP)) {
				msm_chg_printk(DBG_MSM_CHG_UPDATE, "%s: Charger inhibit temp(%d)!!\n",
					__func__, temp);

				if (flags & BQ27520_FLAG_OTC) {
					msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Over temp in charge condition!!\n",
						__func__);
				}

				if (flags & BQ27520_FLAG_OTD) {
					msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Over temp in discharge condition!!\n",
						__func__);
				}

				overtemp = 1;
				return 0;
			}
		} else {
			overtemp = 0;
		}
	}

	if (overtemp == 1) {
		if (flags & BQ27520_FLAG_CHG_INH) 
			return 0;
		else
			overtemp = 0;
	}

	return 1;

#else
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);
	return 1;
#endif
}

static int bq27520_is_battery_id_valid(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	return 1;
}

static int bq27520_status_getter(int function)
{
	int status = 0;
	unsigned long flags;

	spin_lock_irqsave(&curr_batt_status.lock, flags);
	status = curr_batt_status.status[function];
	spin_unlock_irqrestore(&curr_batt_status.lock, flags);

	return status;
}

static int bq27520_get_battery_mvolts(void)
{
	return bq27520_status_getter(GET_BATTERY_VOLTAGE);
}

static int bq27520_get_battery_temperature(void)
{
	return bq27520_status_getter(GET_BATTERY_TEMPERATURE);
}

static int bq27520_get_battery_status(void)
{
	return bq27520_status_getter(GET_BATTERY_STATUS);
}

static int bq27520_get_remaining_capacity(void)
{
	return bq27520_status_getter(GET_BATTERY_CAPACITY);
}


static int bq27520_get_average_current(void)
{
	return bq27520_status_getter(GET_BATTERY_CURRENT);
}

static int bq27520_get_time_to_empty_now(void)
{
	return bq27520_status_getter(GET_BATTERY_TIME_TO_EMPTY_NOW);
}

static int bq27520_get_time_to_empty_avg(void)
{
	return bq27520_status_getter(GET_BATTERY_TIME_TO_EMPTY_AVG);
}

static int bq27520_get_time_to_full(void)
{
	return bq27520_status_getter(GET_BATTERY_TIME_TO_FULL);
}

static void bq27520_updating_poller(int polling_time)
{
	cancel_delayed_work_sync(&curr_batt_status.poller);
	curr_batt_status.polling_period = (polling_time / 10) - 1; 
	schedule_delayed_work(&curr_batt_status.poller, curr_batt_status.polling_period);
}



static int bq27520_monitor_for_recharging(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);





	return 1;
}


static struct msm_battery_gauge bq27520_batt_gauge = {
	.get_battery_mvolts		= bq27520_get_battery_mvolts,
	.get_battery_temperature	= bq27520_get_battery_temperature,
	.is_battery_present		= bq27520_is_battery_present,
	.is_battery_temp_within_range	= bq27520_is_battery_temp_within_range,
	.is_battery_id_valid		= bq27520_is_battery_id_valid,
	.get_battery_status		= bq27520_get_battery_status,
	.get_batt_remaining_capacity	= bq27520_get_remaining_capacity,

	.monitor_for_recharging = bq27520_monitor_for_recharging,


	.get_average_current = bq27520_get_average_current,
	.get_time_to_empty_now = bq27520_get_time_to_empty_now,
	.get_time_to_empty_avg = bq27520_get_time_to_empty_avg,
	.get_time_to_full_now = bq27520_get_time_to_full,
	.updating_batt_info = bq27520_updating_poller,
	.copy_dfi_from_user_to_kernel = bq27520_copy_dfi_user_to_kernel,

};


#include "../../arch/arm/mach-msm/smd_private.h"
#include <oem/oem_smem_struct.h>
static smem_bat_info_data smem_bat_info;
static smem_vendor_id1_apps_data *smem_vendor1_data = NULL;
static DEFINE_SPINLOCK(smem_bat_lock);


static int update_current_battery_status(int data)
{

	int status[NUM_OF_STATUS], ret = 0, noupdate = 0, temp_curr = 0;

	unsigned long flag;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d: status=%d--\n", __func__, __LINE__, data);

	memset(status, 0, sizeof status);

	ret = bq27520_battery_rsoc(bq27520_di);
	if (ret < 0)
		SET_BIT(noupdate, GET_BATTERY_CAPACITY);
	else
		status[GET_BATTERY_CAPACITY] = ret;

	ret = bq27520_battery_voltage(bq27520_di);
	if (ret < 0)
		SET_BIT(noupdate, GET_BATTERY_VOLTAGE);
	else
		status[GET_BATTERY_VOLTAGE] = ret;

	ret = bq27520_battery_temperature(bq27520_di);
	if (ret < 0)
		SET_BIT(noupdate, GET_BATTERY_TEMPERATURE);
	else
		status[GET_BATTERY_TEMPERATURE] = ret;

	ret = bq27520_battery_curr(bq27520_di, &temp_curr);
	if (ret < 0)
		SET_BIT(noupdate, GET_BATTERY_CURRENT);
	else
		status[GET_BATTERY_CURRENT] = temp_curr;

	ret = bq27520_battery_time_to_empty(bq27520_di);
	if (ret < 0)
		SET_BIT(noupdate, GET_BATTERY_TIME_TO_EMPTY_NOW);
	else
		status[GET_BATTERY_TIME_TO_EMPTY_NOW] = ret;

	ret = bq27520_battery_TTE_at_constant_power(bq27520_di);
	if (ret < 0)
		SET_BIT(noupdate, GET_BATTERY_TIME_TO_EMPTY_AVG);
	else
		status[GET_BATTERY_TIME_TO_EMPTY_AVG] = ret;

	ret = bq27520_battery_time_to_full(bq27520_di);
	if (ret < 0)
		SET_BIT(noupdate, GET_BATTERY_TIME_TO_FULL);
	else
		status[GET_BATTERY_TIME_TO_FULL] = ret;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s,%d--\n", __func__, __LINE__);

	spin_lock_irqsave(&curr_batt_status.lock, flag);
	if (TST_BIT(noupdate, GET_BATTERY_CAPACITY) ||
		status[GET_BATTERY_CAPACITY] < 0 || status[GET_BATTERY_CAPACITY] > 100) {
		ret = 0;
	} else {
		ret = (status[GET_BATTERY_CAPACITY] !=
			curr_batt_status.status[GET_BATTERY_CAPACITY]);

		curr_batt_status.status[GET_BATTERY_CAPACITY] = status[GET_BATTERY_CAPACITY];
	}

	curr_batt_status.status[GET_BATTERY_STATUS] = data;

	if (!TST_BIT(noupdate, GET_BATTERY_VOLTAGE))
		curr_batt_status.status[GET_BATTERY_VOLTAGE] = status[GET_BATTERY_VOLTAGE];

	if (!TST_BIT(noupdate, GET_BATTERY_TEMPERATURE))
		curr_batt_status.status[GET_BATTERY_TEMPERATURE] = status[GET_BATTERY_TEMPERATURE];

	if (!TST_BIT(noupdate, GET_BATTERY_CURRENT))
		curr_batt_status.status[GET_BATTERY_CURRENT] = status[GET_BATTERY_CURRENT];

	if (!TST_BIT(noupdate, GET_BATTERY_TIME_TO_EMPTY_NOW))
		curr_batt_status.status[GET_BATTERY_TIME_TO_EMPTY_NOW] = status[GET_BATTERY_TIME_TO_EMPTY_NOW];

	if (!TST_BIT(noupdate, GET_BATTERY_TIME_TO_EMPTY_AVG))
		curr_batt_status.status[GET_BATTERY_TIME_TO_EMPTY_AVG] = status[GET_BATTERY_TIME_TO_EMPTY_AVG];

	if (!TST_BIT(noupdate, GET_BATTERY_TIME_TO_FULL))
		curr_batt_status.status[GET_BATTERY_TIME_TO_FULL] = status[GET_BATTERY_TIME_TO_FULL];


	spin_unlock_irqrestore(&curr_batt_status.lock, flag);

	
	if (smem_vendor1_data == NULL) {
		smem_vendor1_data = (smem_vendor_id1_apps_data *) smem_alloc (SMEM_ID_VENDOR1, sizeof (smem_vendor_id1_apps_data));
	}

	if (smem_vendor1_data != NULL) {
		struct timeval gauge_timeval;
		if (smem_vendor1_data->diag_pm_batt.cap != status[GET_BATTERY_CAPACITY]) {
			do_gettimeofday(&gauge_timeval);
			smem_bat_info.cap = status[GET_BATTERY_CAPACITY];
			smem_bat_info.volt = status[GET_BATTERY_VOLTAGE];
			smem_bat_info.ai = status[GET_BATTERY_CURRENT];
			smem_bat_info.time = gauge_timeval.tv_sec;
			smem_bat_info.ischarging = (data == POWER_SUPPLY_STATUS_CHARGING)?TRUE:FALSE;
			spin_lock_irqsave(&smem_bat_lock, flag);
			memcpy (&(smem_vendor1_data->diag_pm_batt), &smem_bat_info, sizeof (smem_bat_info));
			spin_unlock_irqrestore (&smem_bat_lock, flag);
		}
	}
	

	return ret;
}


enum {
	DUMP_GAUGE_CONSTAT,
	DUMP_GAUGE_FLAGS,
	DUMP_GAUGE_APPSTAT,
	DUMP_GAUGE_OPERSTAT,
	DUMP_GAUGE_TEMP,
	DUMP_GAUGE_VOLT,
	DUMP_GAUGE_NAC,
	DUMP_GAUGE_FCC,
	DUMP_GAUGE_RM,
	DUMP_GAUGE_CURR,
	DUMP_GAUGE_SOH,
	DUMP_GAUGE_SOC,
	NUM_OF_DUMP_GAUGE,
};

void dump_gauge_log(void)
{
	int ret, value[NUM_OF_DUMP_GAUGE];

	
	value[DUMP_GAUGE_CONSTAT] = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_CTNL_STATUS);;

	
	value[DUMP_GAUGE_FLAGS] = bq27520_battery_flags(bq27520_di);

	
	value[DUMP_GAUGE_APPSTAT] = bq27520_battery_extcmd_APPSTAT(bq27520_di);

	
	value[DUMP_GAUGE_OPERSTAT] = bq27520_battery_extcmd_OPERCON(bq27520_di);

	
	value[DUMP_GAUGE_TEMP] = bq27520_battery_temperature(bq27520_di);

	
	value[DUMP_GAUGE_VOLT] = bq27520_battery_voltage(bq27520_di);

	
	value[DUMP_GAUGE_NAC] = bq27520_battery_NAC(bq27520_di);

	
	value[DUMP_GAUGE_FCC] = bq27520_battery_full_charge_capacity(bq27520_di);

	
	value[DUMP_GAUGE_RM] = bq27520_battery_remain_capacity(bq27520_di);

	
	ret = bq27520_battery_curr(bq27520_di, &value[DUMP_GAUGE_CURR]);
	if (ret < 0)
		value[DUMP_GAUGE_CURR] = ret;

	
	value[DUMP_GAUGE_SOH] = bq27520_battery_SOH(bq27520_di);

	
	value[DUMP_GAUGE_SOC] = bq27520_battery_rsoc(bq27520_di);

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "[GAUGE]:Temperature(%d), Capacity(%d), Voltage(%d), Current(%d), ",
		value[DUMP_GAUGE_TEMP], value[DUMP_GAUGE_SOC], value[DUMP_GAUGE_VOLT], value[DUMP_GAUGE_CURR]);

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "NAC(%d), FCC(%d), RM(%d)\n",
		value[DUMP_GAUGE_NAC], value[DUMP_GAUGE_FCC], value[DUMP_GAUGE_RM]);

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "[GAUGE]:Flag(%02x), ConStat(%02x), AppStat(%02x), OperStat(%02x), ",
		value[DUMP_GAUGE_FLAGS], value[DUMP_GAUGE_CONSTAT], value[DUMP_GAUGE_APPSTAT], value[DUMP_GAUGE_OPERSTAT]);

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "SOH(percent:%d),(status:%d)\n",
		(value[DUMP_GAUGE_SOH]& 0xFF), ((value[DUMP_GAUGE_SOH] >> 8) & 0xFF));
}


/* only if battery charging satus changes then notify msm_charger. otherwise
 * only refresh current_batter_status
 */
static int if_notify_msm_charger(int *data)
{
	int ret = 0, flags = 0, curr = 0;
	unsigned long flag;
	
	static int old_flags = 0, status = 0;
	


	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 0, bq27520_di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: flags=0x%x\n", __func__, flags);
	if (ret) {
		dev_err(bq27520_di->dev, "error %d reading register %02x\n", ret, BQ27520_REG_FLAGS);
		*data = status; 
		return 0; 

	} else {
	
		if (flags & BQ27520_FLAG_CHG_INH) {
			
			if (!(flags & BQ27520_FLAG_OCV_GD) && !(flags & BQ27520_FLAG_BAT_DET)) {
				msm_chg_printk(DBG_MSM_CRITICAL, "%s: Invalid battery detected !!\n",
					__func__);
				ret = bq27520_battery_voltage(bq27520_di);
				if (ret > 0 &&  ret < BATT_INVALID_VOLT_THRESHOLD) {
					ret = bq27520_battery_temperature(bq27520_di);
					if (ret < BATT_INVALID_TEMP_THRESHOLD) {
						
						if (!QnooffchargeVariable) {
							msm_chg_printk(DBG_MSM_CRITICAL, "%s: Battery removed, do shutdown !!\n",
								__func__);
							kernel_power_off("batt removed");
							return 0;
						}
					}
				}
			}
			

			if (flags & BQ27520_FLAG_OTC) {
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Over Temperature in charging condition !!\n", __func__);
			}

			if (flags & BQ27520_FLAG_OTD) {
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Over Temperature in discharging condition !!\n", __func__);
			}

			if (flags & BQ27520_FLAG_DSC) {
				status = POWER_SUPPLY_STATUS_DISCHARGING;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Discharging !!\n", __func__);
			} else {
				status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Not charging-inhibit !!\n", __func__);
			}
		} else {
			
			if (flags & BQ27520_FLAG_SOC1) {
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: SOC1 reached !!\n", __func__);
				if (!(old_flags & BQ27520_FLAG_SOC1)){
					ret = bq27520_get_remaining_capacity();
					if (ret <= 5) {
						msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_CRITICAL_LOW--\n", __func__);
						msm_charger_notify_event(NULL, CHG_BATT_CRITICAL_LOW);
					} else if (ret <= 15) {
						msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_LOW--\n", __func__);
						msm_charger_notify_event(NULL, CHG_BATT_LOW);
					} else {
						msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: capacity(%d)--\n",
							__func__, ret);
					}
				}
			} else {
				if (old_flags & BQ27520_FLAG_SOC1) {
					msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: SOC1 recovery !!\n", __func__);
				}
			}

			#if 0 
			if (flags & BQ27520_FLAG_SYSDOWN) {
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: SYSDOWN reached !!\n", __func__);
			} else {
				if (old_flags & BQ27520_FLAG_SYSDOWN) {
					msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: SYSDOWN recovery !!\n", __func__);
					wake_unlock(&bq27520_di->bat_low_lock);
				}
			}
			#endif
			

			
			if ((flags & BQ27520_FLAG_FC) && bq27520_battery_rsoc(bq27520_di) == 100) {
			
				status = POWER_SUPPLY_STATUS_FULL;
				msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Charging Full !!\n", __func__);
			} else {
			
				ret = bq27520_battery_curr(bq27520_di, &curr);
				if (ret) {
					dev_err(bq27520_di->dev, "read current failed!!\n");
					*data = status; 
					return 0; 
				}

				
				if (curr < 0) {
					if (flags & BQ27520_FLAG_DSC) {
						status = POWER_SUPPLY_STATUS_DISCHARGING;
						msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Discharging !!\n", __func__);
					} else {
						status = POWER_SUPPLY_STATUS_NOT_CHARGING;
						msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Not charging !!\n", __func__);
					}
				} else if (curr == 0) {
					status = POWER_SUPPLY_STATUS_NOT_CHARGING;
					msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Not charging !!\n", __func__);
				} else { 
					status = POWER_SUPPLY_STATUS_CHARGING;
					msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: Charging !!\n", __func__);
				}
				
			
			}
		}
	}
	

	*data = status;
	spin_lock_irqsave(&curr_batt_status.lock, flag);
	
	old_flags = flags;
	
	ret = (status != curr_batt_status.status[GET_BATTERY_STATUS]);
	spin_unlock_irqrestore(&curr_batt_status.lock, flag);
	return ret;
}

static void battery_status_poller(struct work_struct *work)
{
	int status = 0, temp = 0, update = 0;

	msm_chg_printk(DBG_MSM_CHG_WROKQUEUE, "--%s--\n", __func__);

	temp = if_notify_msm_charger(&status);
	update = update_current_battery_status(status);

	if (temp) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_STATUS_CHANGE--\n", __func__);
		msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);
	} else if (update) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_CAPACITY_CHANGE--\n", __func__);
		msm_charger_notify_event(NULL, CHG_BATT_CAPACITY_CHANGE);
	} else {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: NO CHANGE--\n", __func__);
	}

	schedule_delayed_work(&curr_batt_status.poller, curr_batt_status.polling_period);
}

static void bq27520_hw_config(struct work_struct *work)
{

	int ret = 0, flags = 0, status = 0, temp = 0, update = 0;
	struct bq27520_device_info *di;

	di  = container_of(work, struct bq27520_device_info, hw_config.work);

	ret = bq27520_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to config Bq27520 IT/DLOG ret = %d\n", ret);
	}


	msm_battery_gauge_register(&bq27520_batt_gauge);


	
	temp = if_notify_msm_charger(&status);
	update = update_current_battery_status(status);

	if (temp) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_STATUS_CHANGE--\n", __func__);
		msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);
	} else if (update) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_CAPACITY_CHANGE--\n", __func__);
		msm_charger_notify_event(NULL, CHG_BATT_CAPACITY_CHANGE);
	} else {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: NO CHANGE--\n", __func__);
	}


	if (di->pdata->soc_int) {
		enable_irq(soc_irq);
	}


	if (di->pdata->bat_low) {
		enable_irq(bat_low_irq);
	}


	
	curr_batt_status.polling_period = (BQ27520_POLLING_DISCHARGING / 10) - 1;
	schedule_delayed_work(&curr_batt_status.poller, 100);

	if (di->pdata->enable_dlog) {
		schedule_work(&di->counter);
		init_timer(&timer);
		timer.function = &bq27520_every_30secs;
		timer.data = (unsigned long)di;
		timer.expires = jiffies + (HZ * BQ27520_COULOMB_POLL);
		add_timer(&timer);
	}

	memset(&bq27520_chip, 0, sizeof(bq27520_chip));

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27520_read(BQ27520_REG_CNTL, &flags, 0, di);

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_DEVCIE_TYPE);
	udelay(66);
	bq27520_read(BQ27520_REG_CNTL, &bq27520_chip.dev_id, 0, di);

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_FW_VER);
	udelay(66);
	bq27520_read(BQ27520_REG_CNTL, &bq27520_chip.fw_ver, 0, di);

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_HW_VER);
	udelay(66);
	bq27520_read(BQ27520_REG_CNTL, &bq27520_chip.hw_ver, 0, di);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s: DEVICE_TYPE:0x%02X, FIRMWARE_VERSION:0x%02X, ",
		__func__, bq27520_chip.dev_id, bq27520_chip.fw_ver);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "HW_VERSION:0x%02X, STATUS:0x%02X\n",
		bq27520_chip.hw_ver, flags);


}


static void bq27520_recharging_monitor(struct work_struct *work)
{
	int capacity = 0;
	struct bq27520_device_info *di;

	di  = container_of(work, struct bq27520_device_info, hw_config.work);

	msm_chg_printk(DBG_MSM_CHG_WROKQUEUE, "--%s--\n", __func__ );

	capacity = bq27520_get_remaining_capacity();
	if (capacity <= BQ27520_BATT_RECHARGING_THRESHOLD) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_NEEDS_RECHARGING, capacity(%d)--\n",
			__func__, capacity);
		msm_charger_notify_event(NULL, CHG_BATT_NEEDS_RECHARGING);
	} else {
		msm_chg_printk(DBG_MSM_CHG_WROKQUEUE, "--%s: continue to moniter recharging threshold--\n",
			__func__);
		schedule_delayed_work(&di->hw_config, BQ27520_POLLING_RECHG_MON / 10);
	}
}




#if 1
static int bq27520_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27520_device_info *di)
{
	struct i2c_client *client = di->client;
	int err;
	unsigned char data[2];
	struct i2c_msg msgs[] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.buf	= (void *)&reg,
			.len	= 1,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.buf	= data,
			.len	= 2,
		},
	};


	if (!b_single) {
		msgs[1].len = 2;
	}
	else {
		msgs[1].len = 1;
	}

	memset(data, 0, sizeof(data));

	if (!client->adapter)
		return -ENODEV;

	err = i2c_transfer(client->adapter, msgs, 2);
	if (err >= 0) {
		*rt_value = get_unaligned_le16(data);
		
		if (*rt_value == 0xFF00) {
			msm_chg_printk(DBG_MSM_CHG_PROBE, "--%s: error data--\n", __func__);
			return -ENODATA;
		}
		
		return 0;
	}

	return err;
}


static int bq27520_read_i2c_retry(u8 reg, int *rt_value, int b_single,
			struct bq27520_device_info *di)
{
	int i, err;

	for(i = 0; i < I2C_RETRY_MAX; i++) {
		err = bq27520_read_i2c(reg, rt_value, b_single, bq27520_di);
		if (err < 0) {
			msleep(10);
		} else {
			return 0;
		}
	}
	return err;
}

#else
static int bq27520_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}
#endif


#ifdef CONFIG_BQ27520_TEST_ENABLE
static int reg;
static int subcmd;
static ssize_t bq27520_read_stdcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27520_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (reg <= BQ27520_REG_ICR && reg > 0x00) {
		ret = bq27520_read(reg, &temp, 0, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27520_write_stdcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	reg = cmd;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s: recv'd cmd is 0x%02X--\n", __func__, reg);

	return ret;
}

static ssize_t bq27520_read_subcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret, temp = 0;
	struct platform_device *client;
	struct bq27520_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (subcmd == BQ27520_SUBCMD_DEVCIE_TYPE ||
		 subcmd == BQ27520_SUBCMD_FW_VER ||
		 subcmd == BQ27520_SUBCMD_HW_VER ||
		 subcmd == BQ27520_SUBCMD_CHEM_ID) {

		ret = bq27520_cntl_cmd(di, subcmd);
		if (ret < 0) {
			ret = snprintf(buf, PAGE_SIZE, "Write Error!\n");
			return ret;
		}
		udelay(66);

		ret = bq27520_read(BQ27520_REG_CNTL, &temp, 0, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27520_write_subcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	subcmd = cmd;
	return ret;
}

static DEVICE_ATTR(std_cmd, 0664, bq27520_read_stdcmd,
	bq27520_write_stdcmd);
static DEVICE_ATTR(sub_cmd, 0664, bq27520_read_subcmd,
	bq27520_write_subcmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_std_cmd.attr,
	&dev_attr_sub_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static struct platform_device this_device = {
	.name = "bq27520-test",
	.id = -1,
	.dev.platform_data = NULL,
};
#endif


#ifdef CONFIG_DEBUG_FS
static struct dentry *dent;

static int gauge_id_get(void *data, u64 *value)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;
	int ret = 0, type = 0;


	if (bq27520_chip.dev_id != BQ27520_ID) {
		bq27520_cntl_cmd(di, BQ27520_SUBCMD_DEVCIE_TYPE);
		if (ret < 0) {
			dev_err(di->dev, "error %d writing subcomd device id failed\n", ret);
			return ret;
		}
		udelay(66);

		ret = bq27520_read(BQ27520_REG_CNTL, &type, 0, di);
		if(ret) {
			dev_err(di->dev, "error %d reading device id failed\n", ret);
			return ret;
		}

		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: id=0x%x\n", __func__, type);

		if (type != BQ27520_ID)
			*value = 0;
	}
	else
		*value = bq27520_chip.dev_id;


	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bq27520_gauge_id_fops, gauge_id_get, NULL, "%llu\n");


static int gauge_it_en_get(void *data, u64 *value)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;
	int ret, flags = 0;

	if (atomic_read(&it_enable_status) != 0) {
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: wait for finishing RESET cmd\n",
			__func__);
		return -EBUSY;
	}

	ret = bq27520_cntl_cmd(di, BQ27520_SUBCMD_CTNL_STATUS);
	if (ret < 0) {
		dev_err(di->dev, "error %d writing subcomd ctnl status failed\n", ret);
		return ret;
	}
	udelay(66);

	ret = bq27520_read(BQ27520_REG_CNTL, &flags, 0, di);
	if(ret) {
		dev_err(di->dev, "error %d reading ctnl status failed\n", ret);
		return ret;
	}

	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: it_en=%d\n", __func__, (flags & 0x1));


	*value = (flags & 0x1);


	return 0;
}

static int gauge_it_en_set(void *data, u64 value)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;
	int ret = 0, flags = 0, count = 0;

	
	if (value != 1) {
		dev_err(di->dev, "set value(%lld) error(only accept 1 to enable)\n", value);
		return -EINVAL;
	}

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_ENABLE_IT);
	if (ret < 0) {
		dev_err(di->dev, "error %d writing subcomd enable it failed\n", ret);
		return ret;
	}
	udelay(66);

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_RESET);
	if (ret < 0) {
		dev_err(di->dev, "error %d writing subcomd reset failed\n", ret);
		return ret;
	}
	udelay(66);

	atomic_set(&it_enable_status, 1);

	do
	{
		ret = bq27520_cntl_cmd(di, BQ27520_SUBCMD_CTNL_STATUS);
		if (ret < 0) {
			dev_err(di->dev, "error %d writing subcomd ctnl status failed\n", ret);
			msleep(5);
			continue;
		}
		udelay(66);

		ret = bq27520_read(BQ27520_REG_CNTL, &flags, 0, di);
		if(ret) {
			dev_err(di->dev, "error %d reading ctnl status failed\n", ret);
			msleep(5);
			continue;
		}

		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: count(%d)\n", __func__, count);
		count++;
		msleep(1000);
	} while(!(flags & BQ27520_CS_QEN));

	atomic_set(&it_enable_status, 0);
	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set it_en\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bq27520_gauge_it_en_fops, gauge_it_en_get, gauge_it_en_set, "%llu\n");


static int gauge_volt_get(void *data, u64 *value)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;
	int ret = 0;

	ret = bq27520_battery_voltage(di);

	*value = ret;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bq27520_gauge_volt_fops, gauge_volt_get, NULL, "%llu\n");

static int gauge_temp_get(void *data, u64 *value)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;
	int ret = 0;

	ret = bq27520_battery_temperature(di);

	*value = ret;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bq27520_gauge_temp_fops, gauge_temp_get, NULL, "%llu\n");

static int gauge_capacity_get(void *data, u64 *value)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;
	int ret = 0;

	ret = bq27520_battery_rsoc(di);

	if (ret < 0)
		ret = 0;

	*value = ret;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bq27520_gauge_capacity_fops, gauge_capacity_get, NULL, "%llu\n");

static int gauge_polling_time_get(void *data, u64 *value)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: batt polling_period=%d\n",
		__func__, curr_batt_status.polling_period);

	*value = curr_batt_status.polling_period;

	return 0;
}

static int gauge_polling_time_set(void *data, u64 value)
{
	if (value > 3600) {
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set too big time(%lld), set as max(3600) secs\n",
			__func__, value);

		curr_batt_status.polling_period = 3600 * 100;
	} else {
		msm_chg_printk(DBG_MSM_CHG_DEBUGFS, "%s: set batt polling_period as %lld secs\n",
			__func__, value);

		curr_batt_status.polling_period = value * 100;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bq27520_polling_time_fops, gauge_polling_time_get, gauge_polling_time_set, "%llu\n");


static int gauge_curr_get(struct seq_file *s, void *unused)
{
	int ret = 0, curr = 0;

	ret = bq27520_battery_curr(bq27520_di, &curr);

	if (ret)
		seq_printf(s, "-0\n");
	else
		seq_printf(s, "%d\n", curr);

	return 0;
}

static int gauge_curr_get_open(struct inode *inode, struct file *file)
{
	return single_open(file, gauge_curr_get, NULL);
}

static const struct file_operations bq27520_batt_operations = {
	.open		= gauge_curr_get_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int gauge_dfi_version_get(struct seq_file *s, void *unused)
{
	u8 ret = 0;

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	seq_printf(s, "DFI version: ");

	ret = bq27520_version_DFI();
	if (!ret)
		seq_printf(s, "Get failed!!");
	else
		seq_printf(s, "0x%x", ret);

	seq_printf(s, "\n"); 

	return 0;
}

static int gauge_dfi_version_get_open(struct inode *inode, struct file *file)
{
	return single_open(file, gauge_dfi_version_get, NULL);
}

static const struct file_operations gauge_dfi_version_operations = {
	.open		= gauge_dfi_version_get_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static void bq27520_create_debugfs_entries(struct bq27520_device_info *di)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	dent = debugfs_create_dir(BQ27520_DRIVER_NAME, NULL);
	if (dent) {
		debugfs_create_file("gauge_id", S_IRUGO, dent, di, &bq27520_gauge_id_fops);
		debugfs_create_file("gauge_it_en", S_IRUGO | S_IWUGO, dent, di, &bq27520_gauge_it_en_fops);
		debugfs_create_file("gauge_volt", S_IRUGO, dent, di, &bq27520_gauge_volt_fops);
		debugfs_create_file("gauge_temp", S_IRUGO, dent, di, &bq27520_gauge_temp_fops);
		debugfs_create_file("gauge_curr", S_IFREG | S_IRUGO, dent, NULL, &bq27520_batt_operations);
		debugfs_create_file("gauge_capacity", S_IRUGO, dent, di, &bq27520_gauge_capacity_fops);
		debugfs_create_file("polling_time", S_IRUGO | S_IWUGO, dent, di, &bq27520_polling_time_fops);
		
		debugfs_create_file("gauge_dfi_version", S_IRUGO | S_IWUGO, dent, NULL, &gauge_dfi_version_operations);
		
	}
}


static void bq27520_destroy_debugfs_entries(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (dent)
		debugfs_remove_recursive(dent);
}
#else
static void bq27520_create_debugfs_entries(struct bq27520_device_info *di)
{
}
static void bq27520_destroy_debugfs_entries(void)
{
}
#endif


static irqreturn_t soc_irqhandler(int irq, void *dev_id)
{
	int status = 0, temp = 0, update = 0;

	msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s --\n", __func__);

	temp = if_notify_msm_charger(&status);
	update = update_current_battery_status(status);

	if (temp) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_STATUS_CHANGE--\n", __func__);
		msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);
	} else if (update) {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_CAPACITY_CHANGE--\n", __func__);
		msm_charger_notify_event(NULL, CHG_BATT_CAPACITY_CHANGE);
	} else {
		msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: NO CHANGE--\n", __func__);
	}

	return IRQ_HANDLED;
}


static irqreturn_t bat_low_irqhandler(int irq, void *dev_id)
{
	int flags = 0, ret = 0;

	msm_chg_printk(DBG_MSM_CHG_INTERRUPT, "--%s --\n", __func__);

	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 0, bq27520_di);
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "%s: flags=0x%x\n", __func__, flags);
	if (ret) {
		dev_err(bq27520_di->dev, "error %d reading register %02x\n", ret, BQ27520_REG_FLAGS);
	} else {
		if (flags & BQ27520_FLAG_SOC1) {
			ret = bq27520_get_remaining_capacity();
			if (ret <= 15) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_LOW--\n",	__func__);
			msm_charger_notify_event(NULL, CHG_BATT_LOW);
			} else {
				msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: capacity(%d)--\n",
					__func__, ret);
			}
		}

		#if 0 
		if (flags & BQ27520_FLAG_SYSDOWN) {
			msm_chg_printk(DBG_MSM_CHG_EVENT, "--%s: CHG_BATT_CRITICAL_LOW--\n", __func__);
			wake_lock(&bq27520_di->bat_low_lock);
			msm_charger_notify_event(NULL, CHG_BATT_CRITICAL_LOW);
		}
		#endif
	}

	return IRQ_HANDLED;
}



static struct regulator *vreg_bq27520;
static int bq27520_power(bool enable, struct bq27520_device_info *di)
{
	int rc = 0;
	const struct bq27520_platform_data *platdata;

	platdata = di->pdata;

	if (enable) {

		/* switch on Vreg_S3 */
		rc = regulator_enable(vreg_bq27520);
		if (rc < 0) {
			dev_err(di->dev, "%s: vreg %s %s failed (%d)\n",
				__func__, platdata->vreg_name, "enable", rc);
			goto vreg_fail;
		}

		if(platdata->chip_en) {
			
			rc = gpio_request(platdata->chip_en, "GAUGE_EN");
			if (rc) {
				dev_err(di->dev, "%s: fail to request gpio %d (%d)\n",
					__func__, platdata->chip_en, rc);
				goto vreg_fail;
			}

			msm_chg_printk(DBG_MSM_CHG_PROBE, "--%s,reguest chip_en--\n", __func__);

			gpio_direction_output(platdata->chip_en, 0);
			gpio_set_value(platdata->chip_en, 1);
		}

		if(platdata->soc_int) {
			rc = gpio_request(platdata->soc_int, "GAUGE_SOC_INT");
			if (rc) {
				dev_err(di->dev, "%s: fail to request gpio %d (%d)\n",
					__func__, platdata->soc_int, rc);
					goto chip_en_gpio_fail;
			}

			msm_chg_printk(DBG_MSM_CHG_PROBE, "--%s,reguest soc_int--\n", __func__);

			gpio_direction_input(platdata->soc_int);
			soc_irq = gpio_to_irq(platdata->soc_int);
			rc = request_threaded_irq(soc_irq, NULL, soc_irqhandler,
					IRQF_TRIGGER_FALLING,
					"BQ27520_SOC_IRQ", di);
			if (rc) {
				dev_err(di->dev, "%s: fail to request irq %d (%d)\n",
					__func__, platdata->soc_int, rc);
					goto soc_irq_fail;
			} else {
				disable_irq_nosync(soc_irq);
			}

			rc = irq_set_irq_wake(soc_irq, 1);
			if (rc) {
				dev_err(di->dev, "%s failed for irq_set_irq_wake %d ret =%d\n",
					 __func__, soc_irq, rc);
				goto bat_low_gpio_fail;
			}
		}

		
		if(platdata->bat_low) {
			rc = gpio_request(platdata->bat_low, "BAT_LOW_INT");
			if (rc) {
				dev_err(di->dev, "%s: fail to request gpio %d (%d)\n",
					__func__, platdata->bat_low, rc);
				goto bat_low_gpio_fail;
			}

			msm_chg_printk(DBG_MSM_CHG_PROBE, "--%s,reguest bat_low--\n", __func__);

			gpio_direction_input(platdata->bat_low);
			bat_low_irq = gpio_to_irq(platdata->bat_low);
			rc = request_threaded_irq(bat_low_irq, NULL, bat_low_irqhandler,
					IRQF_TRIGGER_RISING,
					"BQ27520_BAT_LOW_IRQ", di);
			if (rc) {
				dev_err(di->dev, "%s: fail to request irq %d (%d)\n",
					__func__, platdata->bat_low, rc);
				goto bat_low_irq_fail;
			} else {
				disable_irq_nosync(bat_low_irq);
			}

			rc = irq_set_irq_wake(bat_low_irq, 1);
			if (rc) {
				dev_err(di->dev, "%s failed for irq_set_irq_wake %d ret =%d\n",
					 __func__, bat_low_irq, rc);
				goto bat_low_irq_wake_fail;
			}
		}
		
	} else {

		
		if(platdata->bat_low) {
			free_irq(bat_low_irq , di);
			gpio_free(platdata->bat_low);
		}
		

		if(platdata->soc_int) {
			free_irq(soc_irq, di);
			gpio_free(platdata->soc_int);
		}

		if(platdata->chip_en) {
			
			gpio_set_value(platdata->chip_en, 0);
			gpio_free(platdata->chip_en);
		}

		/* switch off Vreg_S3 */
		rc = regulator_disable(vreg_bq27520);
		if (rc < 0) {
			dev_err(di->dev, "%s: vreg %s %s failed (%d)\n",
				__func__, platdata->vreg_name, "disable", rc);
			goto vreg_fail;
		}
	}

	return rc;

bat_low_irq_wake_fail:
	if(platdata->bat_low)
		free_irq(bat_low_irq, di);
bat_low_irq_fail:
	if(platdata->bat_low)
		gpio_free(platdata->bat_low);
bat_low_gpio_fail:
	if (platdata->soc_int)
		free_irq(soc_irq, di);
soc_irq_fail:
	if(platdata->soc_int)
	gpio_free(platdata->soc_int);
chip_en_gpio_fail:
	if(platdata->chip_en) {
	gpio_set_value(platdata->chip_en, 0);
	gpio_free(platdata->chip_en);
	}
vreg_fail:
	return rc;
}

static int bq27520_dev_setup(bool enable, struct bq27520_device_info *di)
{
	int rc;
	const struct bq27520_platform_data *platdata;

	platdata = di->pdata;

	if (enable) {

		/* enable and set voltage Vreg_S3 */
		vreg_bq27520 = regulator_get(NULL, platdata->vreg_name);
		if (IS_ERR(vreg_bq27520)) {
			dev_err(di->dev, "%s: regulator_get of %s failed (%ld)\n",
				__func__, platdata->vreg_name, PTR_ERR(vreg_bq27520));
			rc = PTR_ERR(vreg_bq27520);
			goto vreg_get_fail;
		}

		rc = regulator_set_voltage(vreg_bq27520, platdata->vreg_value, platdata->vreg_value);
		if (rc) {
			dev_err(di->dev, "%s: regulator_set_voltage(%s) failed (%d)\n",
				__func__, platdata->vreg_name, rc);
			goto vreg_get_fail;
		}
	} else {

		regulator_put(vreg_bq27520);
	}

	return 0;

vreg_get_fail:
	regulator_put(vreg_bq27520);
	return rc;
}

static void init_battery_status(void)
{
	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	spin_lock_init(&curr_batt_status.lock);
	curr_batt_status.status[GET_BATTERY_STATUS] = POWER_SUPPLY_STATUS_UNKNOWN;
}


static int bq27520_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27520_device_info *di;
	struct bq27520_access_methods *bus;
	const struct bq27520_platform_data  *pdata;
	int num, retval = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c check function failed\n");
		return -ENODEV;
	}

	pdata = client->dev.platform_data;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}
	di->id = num;
	di->pdata = pdata;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;


	bus->read = &bq27520_read_i2c_retry;

	di->bus = bus;
	di->client = client;

#ifdef CONFIG_BQ27520_TEST_ENABLE
	platform_set_drvdata(&this_device, di);
	retval = platform_device_register(&this_device);
	if (!retval) {
		retval = sysfs_create_group(&this_device.dev.kobj,&fs_attr_group);
		if (retval)
			goto batt_failed_3;
	} else
		goto batt_failed_3;
#endif

	bq27520_di = di;

	retval = bq27520_dev_setup(true, di);
	if (retval) {
		dev_err(&client->dev, "failed to setup ret = %d\n", retval);
		goto batt_failed_3;
	}

	retval = bq27520_power(true, di);
	if (retval) {
		dev_err(&client->dev, "failed to powerup ret = %d\n", retval);
		goto batt_failed_3;
	}

	spin_lock_init(&lock);


	
	init_battery_status();


	if (pdata->enable_dlog)
		INIT_WORK(&di->counter, bq27520_coulomb_counter_work);


	bq27520_create_debugfs_entries(di);


	INIT_DELAYED_WORK(&curr_batt_status.poller, battery_status_poller);
	INIT_DELAYED_WORK(&di->hw_config, bq27520_hw_config);

	INIT_DELAYED_WORK(&di->rechg_mon_work, bq27520_recharging_monitor);


	wake_lock_init(&di->bat_low_lock, WAKE_LOCK_SUSPEND, "bq27520_battery");


	schedule_delayed_work(&di->hw_config, BQ27520_INIT_DELAY / 10); 

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s probe success!\n", __func__);

	return 0;

batt_failed_3:
	kfree(bus);
batt_failed_2:
	kfree(di);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27520_battery_remove(struct i2c_client *client)
{
	struct bq27520_device_info *di = i2c_get_clientdata(client);
	int ret = 0;

	if (di->pdata->enable_dlog) {
		del_timer_sync(&timer);
		cancel_work_sync(&di->counter);
		bq27520_cntl_cmd(di, BQ27520_SUBCMD_DISABLE_DLOG);
		if (ret < 0)
			dev_err(di->dev, "error %d writing subcomd disable dlog failed\n", ret);
		udelay(66);
	}

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_DISABLE_IT);
	if (ret < 0)
		dev_err(di->dev, "error %d writing subcomd diable it failed\n", ret);
	cancel_delayed_work_sync(&di->hw_config);
	cancel_delayed_work_sync(&curr_batt_status.poller);

	bq27520_dev_setup(false, di);
	bq27520_power(false, di);

	bq27520_destroy_debugfs_entries();


	wake_lock_destroy(&di->bat_low_lock);

	kfree(di->bus);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s remove success!\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int bq27520_suspend(struct device *dev)
{
	struct bq27520_device_info *di = dev_get_drvdata(dev);

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (di->pdata->enable_dlog) {
		del_timer_sync(&timer);
		cancel_work_sync(&di->counter);
	}

	cancel_delayed_work_sync(&curr_batt_status.poller);
	return 0;
}

static int bq27520_resume(struct device *dev)
{
	struct bq27520_device_info *di = dev_get_drvdata(dev);

	msm_chg_printk(DBG_MSM_CHG_DEBUG, "--%s--\n", __func__);

	if (di->pdata->enable_dlog)
		add_timer(&timer);

	schedule_delayed_work(&curr_batt_status.poller, 1); 
	return 0;
}

static const struct dev_pm_ops bq27520_pm_ops = {
	.suspend = bq27520_suspend,
	.resume = bq27520_resume,
};
#endif

static const struct i2c_device_id bq27520_id[] = {
	{ BQ27520_DRIVER_NAME, 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27520_id);

static struct i2c_driver bq27520_battery_driver = {
	.driver = {
		.name = BQ27520_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &bq27520_pm_ops,
#endif
	},
	.probe = bq27520_battery_probe,
	.remove = bq27520_battery_remove,
	.id_table = bq27520_id,
};

static int __init bq27520_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27520_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register driver ret = %d\n", ret);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s initializing success!\n", __func__);

	return ret;
}
module_init(bq27520_battery_init);

static void __exit bq27520_battery_exit(void)
{
	i2c_del_driver(&bq27520_battery_driver);
	msm_battery_gauge_unregister(&bq27520_batt_gauge);

	msm_chg_printk(DBG_MSM_CHG_PROBE, "%s exit success!\n", __func__);
}
module_exit(bq27520_battery_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27520 battery monitor driver");
