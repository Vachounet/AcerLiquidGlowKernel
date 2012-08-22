/*  Date: 2011/7/4 17:00:00
 *  Revision: 2.7
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http:

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */




#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/i2c.h>

#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/bma250.h>

#include <mach/pm_log.h>

#include <linux/debugfs.h>

#define CONFIG_PM 1
#define CONFIG_HAS_EARLYSUSPEND 1

#define BMA250_MAX_DELAY		100
#define BMA250_CHIP_ID			3
#define BMA250_RANGE_SET		0
#define BMA250_BW_SET			2


#define BMAMSG(format, arg...) {if(GSENSOR_DLL)  printk(KERN_INFO "[BMA250]" format "\n", ## arg);}

static unsigned char gsensor_chipid = 0;
static struct i2c_client *this_client;
static atomic_t reserve_enable_flag;

extern struct dentry *kernel_debuglevel_dir;
static unsigned int GSENSOR_DLL=0;


static void gsensor_create_kernel_debuglevel(void)
{
	BMAMSG("create kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir!=NULL) {
		debugfs_create_u32("bma250_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&GSENSOR_DLL));
	} else {
		printk(KERN_ERR "failed to create Bosch Sensortec GSensor BMA250 dll in kernel_debuglevel_dir!!!\n");
	}
}


static void gsensor_destroy_kernel_debuglevel(void)
{
	BMAMSG("destroy kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}

static struct regulator *vreg_smps3_1p8;		
static struct regulator *vreg_ldo12_2p85; 

static int bma250_power(int on)
{
	int rc = 0;
#define _GET_REGULATOR(var, name) do {				\
		var = regulator_get(NULL, name);			\
		if (IS_ERR(var)) {					\
			pr_info("'%s' regulator not found, rc=%ld\n",	\
				name, IS_ERR(var)); 		\
			var = NULL; 				\
			return -ENODEV; 				\
		}							\
	} while (0)

	if (!vreg_ldo12_2p85)
		_GET_REGULATOR(vreg_ldo12_2p85, "ldo12");
	if (!vreg_smps3_1p8)
		_GET_REGULATOR(vreg_smps3_1p8, "smps3");
#undef _GET_REGULATOR

	if (on) {
		rc = regulator_set_voltage(vreg_ldo12_2p85, 2850000, 2850000);
		if (rc) {
			pr_info("%s: '%s' regulator set voltage failed,\
				rc=%d\n", __func__, "vreg_ldo12_2p85", rc);
			return rc;
		}

		rc = regulator_enable(vreg_ldo12_2p85);
		if (rc) {
			pr_err("%s: '%s' regulator enable failed,\
				rc=%d\n", __func__, "vreg_ldo12_2p85", rc);
			return rc;
		}
	} else {
		rc = regulator_disable(vreg_ldo12_2p85);
		if (rc)
			pr_warning("%s: '%s' regulator disable failed, rc=%d\n",
				__func__, "vreg_ldo12_2p85", rc);
	}

	if (on) {
		rc = regulator_set_voltage(vreg_smps3_1p8, 1800000, 1800000);
		if (rc) {
			pr_info("%s: '%s' regulator set voltage failed,\
				rc=%d\n", __func__, "vreg_smps3_1p8", rc);
			return rc;
		}

		rc = regulator_enable(vreg_smps3_1p8);
		if (rc) {
			pr_err("%s: '%s' regulator enable failed,\
				rc=%d\n", __func__, "vreg_smps3_1p8", rc);
			return rc;
		}
	} else {
		rc = regulator_disable(vreg_smps3_1p8);
		if (rc)
			pr_warning("%s: '%s' regulator disable failed, rc=%d\n",
				__func__, "vreg_smps3_1p8", rc);
	}

	return rc;
}

static int bma250_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma250_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma250_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma250_reset(struct i2c_client *client)
{
	int comres = 0;
	unsigned char data = 0xB6;

	comres = bma250_smbus_write_byte(client, BMA250_RESET_REG, &data);

	if (comres < 0)
		return -1;
	return 0;
}

static int bma250_set_mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data1;


	
	PM_LOG_EVENT ( (Mode==BMA250_MODE_NORMAL)?PM_LOG_ON:PM_LOG_OFF, PM_LOG_G_SENSOR);
	
	if (Mode < 3) {
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_LOW_POWER__REG, &data1);
		switch (Mode) {
		case BMA250_MODE_NORMAL:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 0);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 0);
			break;
		case BMA250_MODE_LOWPOWER:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 1);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 0);
			break;
		case BMA250_MODE_SUSPEND:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 0);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 1);
			break;
		default:
			break;
		}

		comres += bma250_smbus_write_byte(client,
				BMA250_EN_LOW_POWER__REG, &data1);
	} else{
		comres = -1;
	}


	return comres;
}
#ifdef BMA250_ENABLE_INT1
static int bma250_set_int1_pad_sel(struct i2c_client *client,unsigned char int1sel)
{
	int comres=0;
	unsigned char data;
	unsigned char state;
	state = 0x01;


	switch (int1sel) {
	case 0:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT1_PAD_LOWG__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_LOWG, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT1_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT1_PAD_HIGHG__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_HIGHG, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT1_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT1_PAD_SLOPE__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_SLOPE, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT1_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT1_PAD_DB_TAP__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_DB_TAP, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT1_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT1_PAD_SNG_TAP__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_SNG_TAP, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT1_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT1_PAD_ORIENT__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_ORIENT, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT1_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT1_PAD_FLAT__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_FLAT, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT1_PAD_FLAT__REG, &data);
		break;
	default:
		break;
	}

	return comres;
}
#endif 
#ifdef BMA250_ENABLE_INT2
static int bma250_set_int2_pad_sel(struct i2c_client *client,unsigned char int2sel)
{
	int comres=0;
	unsigned char data;
	unsigned char state;
	state = 0x01;


	switch (int2sel) {
	case 0:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT2_PAD_LOWG__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_LOWG, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT2_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT2_PAD_HIGHG__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_HIGHG, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT2_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT2_PAD_SLOPE__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_SLOPE, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT2_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT2_PAD_DB_TAP__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_DB_TAP, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT2_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT2_PAD_SNG_TAP__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_SNG_TAP, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT2_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT2_PAD_ORIENT__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_ORIENT, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT2_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma250_smbus_read_byte(client, BMA250_EN_INT2_PAD_FLAT__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_FLAT, state );
		comres = bma250_smbus_write_byte(client, BMA250_EN_INT2_PAD_FLAT__REG, &data);
		break;
	default:
		break;
	}

	return comres;
}
#endif 

static int bma250_set_Int_Enable(struct i2c_client *client, unsigned char InterruptType , unsigned char value )
{
	int comres = 0;
	unsigned char data1,data2;


	comres = bma250_smbus_read_byte(client, BMA250_INT_ENABLE1_REG, &data1);
	comres = bma250_smbus_read_byte(client, BMA250_INT_ENABLE2_REG, &data2);

	value = value & 1;
	switch (InterruptType)
	{
		case 0:
			
			data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_LOWG_INT, value );
			break;
		case 1:
			

			data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_X_INT, value );
			break;
		case 2:
			

			data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_Y_INT, value );
			break;
		case 3:
			

			data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_Z_INT, value );
			break;
		case 4:
			

			data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_NEW_DATA_INT, value );
			break;
		case 5:
			

			data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_X_INT, value );
			break;
		case 6:
			

			data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_Y_INT, value );
			break;
		case 7:
			

			data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_Z_INT, value );
			break;
		case 8:
			

			data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SINGLE_TAP_INT, value );
			break;
		case 9:
			

			data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_DOUBLE_TAP_INT, value );
			break;
		case 10:
			

			data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_ORIENT_INT, value );
			break;
		case 11:
			

			data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_FLAT_INT, value );
			break;
		default:
			break;
	}
	comres = bma250_smbus_write_byte(client, BMA250_INT_ENABLE1_REG, &data1);
	comres = bma250_smbus_write_byte(client, BMA250_INT_ENABLE2_REG, &data2);

	return comres;
}


static int bma250_get_mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;


	comres = bma250_smbus_read_byte(client,
			BMA250_EN_LOW_POWER__REG, Mode);
	*Mode  = (*Mode) >> 6;


	return comres;
}

static int bma250_set_range(struct i2c_client *client, unsigned char Range)
{
	int comres = 0;
	unsigned char data1;
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (Range < 4) {
		comres = bma250_smbus_read_byte(client,
				BMA250_RANGE_SEL_REG, &data1);
		switch (Range) {
			case BMA250_RANGE_2G:
				data1  = BMA250_SET_BITSLICE(data1,
						BMA250_RANGE_SEL, 3);
				break;
			case BMA250_RANGE_4G:
				data1  = BMA250_SET_BITSLICE(data1,
						BMA250_RANGE_SEL, 5);
				break;
			case BMA250_RANGE_8G:
				data1  = BMA250_SET_BITSLICE(data1,
						BMA250_RANGE_SEL, 8);
				break;
			case BMA250_RANGE_16G:
				data1  = BMA250_SET_BITSLICE(data1,
						BMA250_RANGE_SEL, 12);
				break;
			default:
				break;
		}
		comres += bma250_smbus_write_byte(client,
				BMA250_RANGE_SEL_REG, &data1);
		if(!comres)
      atomic_set(&bma250->range, (unsigned int) Range);
	} else{
		comres = -1;
	}


	return comres;
}

static int bma250_get_range(struct i2c_client *client, unsigned char *Range)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client, BMA250_RANGE_SEL__REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_RANGE_SEL);
	*Range = data;


	return comres;
}


static int bma250_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data;
	int Bandwidth = 0;
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (BW < 8) {
		switch (BW) {
			case 0:
				Bandwidth = BMA250_BW_7_81HZ;
				break;
			case 1:
				Bandwidth = BMA250_BW_15_63HZ;
				break;
			case 2:
				Bandwidth = BMA250_BW_31_25HZ;
				break;
			case 3:
				Bandwidth = BMA250_BW_62_50HZ;
				break;
			case 4:
				Bandwidth = BMA250_BW_125HZ;
				break;
			case 5:
				Bandwidth = BMA250_BW_250HZ;
				break;
			case 6:
				Bandwidth = BMA250_BW_500HZ;
				break;
			case 7:
				Bandwidth = BMA250_BW_1000HZ;
				break;
			default:
				break;
		}
		comres = bma250_smbus_read_byte(client,
				BMA250_BANDWIDTH__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_BANDWIDTH,
				Bandwidth);
		comres += bma250_smbus_write_byte(client,
				BMA250_BANDWIDTH__REG, &data);
		if(!comres)
			atomic_set(&bma250->bandwidth, (unsigned int) BW);
	} else{
		comres = -1;
	}


	return comres;
}

static int bma250_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client, BMA250_BANDWIDTH__REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_BANDWIDTH);
	if (data <= 8) {
		*BW = 0;
	} else{
		if (data >= 0x0F)
			*BW = 7;
		else
			*BW = data - 8;

	}

	return comres;
}

#if defined(BMA250_ENABLE_INT1)||defined(BMA250_ENABLE_INT2)
static int bma250_get_interruptstatus1(struct i2c_client *client, unsigned char *intstatus )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS1_REG, &data);
	*intstatus = data;

	return comres;
}


static int bma250_get_HIGH_first(struct i2c_client *client, unsigned char param,unsigned char *intstatus )
{
	int comres = 0 ;
	unsigned char data;

	switch (param)
	{
		case 0:
			comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG, &data);
			data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_FIRST_X);
			*intstatus = data;
			break;
		case 1:
			comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG, &data);
			data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_FIRST_Y);
			*intstatus = data;
			break;
		case 2:
			comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG, &data);
			data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_FIRST_Z);
			*intstatus = data;
			break;
		default:
			break;
	}

	return comres;
}

static int bma250_get_HIGH_sign(struct i2c_client *client, unsigned char *intstatus )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_SIGN_S);
	*intstatus = data;

	return comres;
}


static int bma250_get_slope_first(struct i2c_client *client, unsigned char param,unsigned char *intstatus )
{
	int comres = 0 ;
	unsigned char data;

	switch (param)
	{
		case 0:
			comres = bma250_smbus_read_byte(client, BMA250_STATUS_TAP_SLOPE_REG, &data);
			data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_FIRST_X);
			*intstatus = data;
			break;
		case 1:
			comres = bma250_smbus_read_byte(client, BMA250_STATUS_TAP_SLOPE_REG, &data);
			data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_FIRST_Y);
			*intstatus = data;
			break;
		case 2:
			comres = bma250_smbus_read_byte(client, BMA250_STATUS_TAP_SLOPE_REG, &data);
			data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_FIRST_Z);
			*intstatus = data;
			break;
		default:
			break;
	}

	return comres;
}

static int bma250_get_slope_sign(struct i2c_client *client, unsigned char *intstatus )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS_TAP_SLOPE_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_SIGN_S);
	*intstatus = data;

	return comres;
}

static int bma250_get_orient_status(struct i2c_client *client, unsigned char *intstatus )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_ORIENT_S);
	*intstatus = data;

	return comres;
}

static int bma250_get_orient_flat_status(struct i2c_client *client, unsigned char *intstatus )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_FLAT_S);
	*intstatus = data;

	return comres;
}
#endif 
static int bma250_set_Int_Mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client,
			BMA250_INT_MODE_SEL__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_INT_MODE_SEL, Mode);
	comres = bma250_smbus_write_byte(client,
			BMA250_INT_MODE_SEL__REG, &data);

	return comres;
}

static int bma250_get_Int_Mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_INT_MODE_SEL__REG, &data);
	data  = BMA250_GET_BITSLICE(data, BMA250_INT_MODE_SEL);
	*Mode = data;


	return comres;
}
static int bma250_set_slope_duration(struct i2c_client *client, unsigned char duration)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_SLOPE_DUR__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_SLOPE_DUR, duration);
	comres = bma250_smbus_write_byte(client,
			BMA250_SLOPE_DUR__REG, &data);


	return comres;
}

static int bma250_get_slope_duration(struct i2c_client *client, unsigned char *status)
{
	int comres = 0 ;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_SLOPE_DURN_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_DUR);
	*status = data;


	return comres;
}

static int bma250_set_slope_threshold(struct i2c_client *client,
		unsigned char threshold)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_SLOPE_THRES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_SLOPE_THRES, threshold);
	comres = bma250_smbus_write_byte(client,
			BMA250_SLOPE_THRES__REG, &data);


	return comres;
}

static int bma250_get_slope_threshold(struct i2c_client *client,
		unsigned char *status)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_SLOPE_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_THRES);
	*status = data;


	return comres;
}
static int bma250_set_low_g_duration(struct i2c_client *client, unsigned char duration)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_LOWG_DUR__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_LOWG_DUR, duration );
	comres = bma250_smbus_write_byte(client, BMA250_LOWG_DUR__REG, &data);

	return comres;
}

static int bma250_get_low_g_duration(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_LOW_DURN_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_LOWG_DUR);
	*status = data;

	return comres;
}

static int bma250_set_low_g_threshold(struct i2c_client *client, unsigned char threshold)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_LOWG_THRES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_LOWG_THRES, threshold );
	comres = bma250_smbus_write_byte(client, BMA250_LOWG_THRES__REG, &data);

	return comres;
}

static int bma250_get_low_g_threshold(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_LOW_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_LOWG_THRES);
	*status = data;

	return comres;
}

static int bma250_set_high_g_duration(struct i2c_client *client, unsigned char duration)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_HIGHG_DUR__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_HIGHG_DUR, duration );
	comres = bma250_smbus_write_byte(client, BMA250_HIGHG_DUR__REG, &data);

	return comres;
}

static int bma250_get_high_g_duration(struct i2c_client *client, unsigned char *status )
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_HIGH_DURN_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_DUR);
	*status = data;

	return comres;
}

static int bma250_set_high_g_threshold(struct i2c_client *client, unsigned char threshold)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_HIGHG_THRES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_HIGHG_THRES, threshold );
	comres = bma250_smbus_write_byte(client, BMA250_HIGHG_THRES__REG, &data);

	return comres;
}

static int bma250_get_high_g_threshold(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_HIGH_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_THRES);
	*status = data;

	return comres;
}


static int bma250_set_tap_duration(struct i2c_client *client, unsigned char duration)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_DUR__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_DUR, duration );
	comres = bma250_smbus_write_byte(client, BMA250_TAP_DUR__REG, &data);

	return comres;
}

static int bma250_get_tap_duration(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_DUR);
	*status = data;

	return comres;
}

static int bma250_set_tap_shock(struct i2c_client *client, unsigned char setval)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_SHOCK_DURN__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_SHOCK_DURN, setval );
	comres = bma250_smbus_write_byte(client, BMA250_TAP_SHOCK_DURN__REG, &data);

	return comres;
}

static int bma250_get_tap_shock(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_SHOCK_DURN);
	*status = data;

	return comres;
}

static int bma250_set_tap_quiet(struct i2c_client *client, unsigned char duration)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_QUIET_DURN__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_QUIET_DURN, duration );
	comres = bma250_smbus_write_byte(client, BMA250_TAP_QUIET_DURN__REG, &data);

	return comres;
}

static int bma250_get_tap_quiet(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_QUIET_DURN);
	*status = data;

	return comres;
}

static int bma250_set_tap_threshold(struct i2c_client *client, unsigned char threshold)
{
	int comres= 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_THRES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_THRES, threshold );
	comres = bma250_smbus_write_byte(client, BMA250_TAP_THRES__REG, &data);

	return comres;
}

static int bma250_get_tap_threshold(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_THRES);
	*status = data;

	return comres;
}

static int bma250_set_tap_samp(struct i2c_client *client, unsigned char samp)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_SAMPLES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_SAMPLES, samp );
	comres = bma250_smbus_write_byte(client, BMA250_TAP_SAMPLES__REG, &data);

	return comres;
}

static int bma250_get_tap_samp(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_SAMPLES);
	*status = data;

	return comres;
}

static int bma250_set_orient_mode(struct i2c_client *client, unsigned char mode)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_MODE__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_ORIENT_MODE, mode );
	comres = bma250_smbus_write_byte(client, BMA250_ORIENT_MODE__REG, &data);

	return comres;
}

static int bma250_get_orient_mode(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_ORIENT_MODE);
	*status = data;

	return comres;
}

static int bma250_set_orient_blocking(struct i2c_client *client, unsigned char samp)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_BLOCK__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_ORIENT_BLOCK, samp );
	comres = bma250_smbus_write_byte(client, BMA250_ORIENT_BLOCK__REG, &data);

	return comres;
}

static int bma250_get_orient_blocking(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_ORIENT_BLOCK);
	*status = data;

	return comres;
}

static int bma250_set_orient_hyst(struct i2c_client *client, unsigned char orienthyst)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_HYST__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_ORIENT_HYST, orienthyst );
	comres = bma250_smbus_write_byte(client, BMA250_ORIENT_HYST__REG, &data);

	return comres;
}

static int bma250_get_orient_hyst(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_ORIENT_HYST);
	*status = data;

	return comres;
}
static int bma250_set_theta_blocking(struct i2c_client *client, unsigned char thetablk)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_THETA_BLOCK__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_THETA_BLOCK, thetablk );
	comres = bma250_smbus_write_byte(client, BMA250_THETA_BLOCK__REG, &data);

	return comres;
}

static int bma250_get_theta_blocking(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_THETA_BLOCK_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_THETA_BLOCK);
	*status = data;

	return comres;
}

static int bma250_set_theta_flat(struct i2c_client *client, unsigned char thetaflat)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_THETA_FLAT__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_THETA_FLAT, thetaflat );
	comres = bma250_smbus_write_byte(client, BMA250_THETA_FLAT__REG, &data);

	return comres;
}

static int bma250_get_theta_flat(struct i2c_client *client, unsigned char *status )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_THETA_FLAT_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_THETA_FLAT);
	*status = data;

	return comres;
}

static int bma250_set_flat_hold_time(struct i2c_client *client, unsigned char holdtime)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_FLAT_HOLD_TIME__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_FLAT_HOLD_TIME, holdtime );
	comres = bma250_smbus_write_byte(client, BMA250_FLAT_HOLD_TIME__REG, &data);

	return comres;
}

static int bma250_get_flat_hold_time(struct i2c_client *client, unsigned char *holdtime )
{
	int comres= 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_FLAT_HOLD_TIME_REG, &data);
	data  = BMA250_GET_BITSLICE(data, BMA250_FLAT_HOLD_TIME);
	*holdtime = data ;

	return comres;
}

static int bma250_write_reg(struct i2c_client *client, unsigned char addr, unsigned char *data)
{
	int comres = 0 ;
	comres = bma250_smbus_write_byte(client, addr, data);

	return comres;
}


static int bma250_set_offset_target_x(struct i2c_client *client, unsigned char offsettarget)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_COMP_TARGET_OFFSET_X__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_X, offsettarget );
	comres = bma250_smbus_write_byte(client, BMA250_COMP_TARGET_OFFSET_X__REG, &data);

	return comres;
}



static int bma250_set_offset_target_y(struct i2c_client *client, unsigned char offsettarget)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_COMP_TARGET_OFFSET_Y__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Y, offsettarget );
	comres = bma250_smbus_write_byte(client, BMA250_COMP_TARGET_OFFSET_Y__REG, &data);

	return comres;
}



static int bma250_set_offset_target_z(struct i2c_client *client, unsigned char offsettarget)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_COMP_TARGET_OFFSET_Z__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Z, offsettarget );
	comres = bma250_smbus_write_byte(client, BMA250_COMP_TARGET_OFFSET_Z__REG, &data);

	return comres;
}



static int bma250_get_cal_ready(struct i2c_client *client, unsigned char *calrdy )
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_OFFSET_CTRL_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_FAST_COMP_RDY_S);
	*calrdy = data;

	return comres;
}

static int bma250_set_cal_trigger(struct i2c_client *client, unsigned char caltrigger)
{
	int comres=0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_EN_FAST_COMP__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_EN_FAST_COMP, caltrigger );
	comres = bma250_smbus_write_byte(client, BMA250_EN_FAST_COMP__REG, &data);

	return comres;
}

static int bma250_set_selftest_st(struct i2c_client *client, unsigned char selftest)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_EN_SELF_TEST__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_EN_SELF_TEST, selftest );
	comres = bma250_smbus_write_byte(client, BMA250_EN_SELF_TEST__REG, &data);

	return comres;
}

static int bma250_set_selftest_stn(struct i2c_client *client, unsigned char stn)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_NEG_SELF_TEST__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_NEG_SELF_TEST, stn );
	comres = bma250_smbus_write_byte(client, BMA250_NEG_SELF_TEST__REG, &data);

	return comres;
}
static int bma250_read_accel_x(struct i2c_client *client, short *a_x)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client, BMA250_ACC_X_LSB__REG, data, 2);
	*a_x = BMA250_GET_BITSLICE(data[0],BMA250_ACC_X_LSB) | (BMA250_GET_BITSLICE(data[1],BMA250_ACC_X_MSB)<<BMA250_ACC_X_LSB__LEN);
	*a_x = *a_x << (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN+BMA250_ACC_X_MSB__LEN));
	*a_x = *a_x >> (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN+BMA250_ACC_X_MSB__LEN));

	return comres;
}
static int bma250_read_accel_y(struct i2c_client *client, short *a_y)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client, BMA250_ACC_Y_LSB__REG, data, 2);
	*a_y = BMA250_GET_BITSLICE(data[0],BMA250_ACC_Y_LSB) | (BMA250_GET_BITSLICE(data[1],BMA250_ACC_Y_MSB)<<BMA250_ACC_Y_LSB__LEN);
	*a_y = *a_y << (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN+BMA250_ACC_Y_MSB__LEN));
	*a_y = *a_y >> (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN+BMA250_ACC_Y_MSB__LEN));

	return comres;
}

static int bma250_read_accel_z(struct i2c_client *client, short *a_z)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client, BMA250_ACC_Z_LSB__REG, data, 2);
	*a_z = BMA250_GET_BITSLICE(data[0],BMA250_ACC_Z_LSB) | BMA250_GET_BITSLICE(data[1],BMA250_ACC_Z_MSB)<<BMA250_ACC_Z_LSB__LEN;
	*a_z = *a_z << (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN+BMA250_ACC_Z_MSB__LEN));
	*a_z = *a_z >> (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN+BMA250_ACC_Z_MSB__LEN));

	return comres;
}


static int bma250_read_accel_xyz(struct i2c_client *client,
		struct bma250acc *acc)
{
	int comres;
	unsigned char data[6];

	comres = bma250_smbus_read_byte_block(client,
			BMA250_ACC_X_LSB__REG, data, 6);

	acc->x = BMA250_GET_BITSLICE(data[0], BMA250_ACC_X_LSB)
		|(BMA250_GET_BITSLICE(data[1],
					BMA250_ACC_X_MSB)<<BMA250_ACC_X_LSB__LEN);
	acc->x = acc->x << (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
				+ BMA250_ACC_X_MSB__LEN));
	acc->x = acc->x >> (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
				+ BMA250_ACC_X_MSB__LEN));
	acc->y = BMA250_GET_BITSLICE(data[2], BMA250_ACC_Y_LSB)
		| (BMA250_GET_BITSLICE(data[3],
					BMA250_ACC_Y_MSB)<<BMA250_ACC_Y_LSB__LEN);
	acc->y = acc->y << (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
				+ BMA250_ACC_Y_MSB__LEN));
	acc->y = acc->y >> (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
				+ BMA250_ACC_Y_MSB__LEN));

	acc->z = BMA250_GET_BITSLICE(data[4], BMA250_ACC_Z_LSB)
		| (BMA250_GET_BITSLICE(data[5],
					BMA250_ACC_Z_MSB)<<BMA250_ACC_Z_LSB__LEN);
	acc->z = acc->z << (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
				+ BMA250_ACC_Z_MSB__LEN));
	acc->z = acc->z >> (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
				+ BMA250_ACC_Z_MSB__LEN));


	return comres;
}

static void bma250_work_func(struct work_struct *work)
{
	#define ACC_NUM 1
	int i;
	struct bma250acc acc[ACC_NUM], acc_avg;
	int acc_sum_x=0, acc_sum_y=0, acc_sum_z=0;

	struct bma250_data *bma250 = container_of((struct delayed_work *)work,
			struct bma250_data, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma250->delay));

	
	if(atomic_read(&bma250->enable)==1)
	{
		for(i=0; i<ACC_NUM; i++)
		{
		  bma250_read_accel_xyz(bma250->bma250_client, &acc[i]);
			acc_sum_x += acc[i].x;
			acc_sum_y += acc[i].y;
			acc_sum_z += acc[i].z;
    	
			usleep(200);
		}
		acc_avg.x = (acc_sum_x)/ACC_NUM;
		acc_avg.y = (acc_sum_y)/ACC_NUM;
		acc_avg.z = (acc_sum_z)/ACC_NUM;
    

		mutex_lock(&bma250->value_mutex);
		bma250->value = acc_avg;
		mutex_unlock(&bma250->value_mutex);
		schedule_delayed_work(&bma250->work, delay);
	}
}


static ssize_t bma250_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	sscanf(buf, "%d%d", &address, &value);

	if (bma250_write_reg(bma250->bma250_client, (unsigned char)address, (unsigned char *)&value) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	size_t count = 0;
	u8 reg[0x3d];
	int i;

	for(i =0; i <= 0x3d; i++) {
		bma250_smbus_read_byte(bma250->bma250_client,i,reg+i);

		count += sprintf(&buf[count], "0x%x: %d\n", i, reg[i]);
	}
	return count;


}

static ssize_t bma250_chipid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gsensor_chipid);
}

static ssize_t bma250_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_range(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_range(bma250->bma250_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_bandwidth(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_bandwidth(bma250->bma250_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_mode(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_mode(bma250->bma250_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}


static ssize_t bma250_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma250_data *bma250 = input_get_drvdata(input);
	struct bma250acc acc_value;

	mutex_lock(&bma250->value_mutex);
	acc_value = bma250->value;
	mutex_unlock(&bma250->value_mutex);

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t bma250_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma250->delay));

}

static ssize_t bma250_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > BMA250_MAX_DELAY)
		data = BMA250_MAX_DELAY;
	atomic_set(&bma250->delay, (unsigned int) data);

	return count;
}


static ssize_t bma250_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma250->enable));

}

static void bma250_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&bma250->enable);

	mutex_lock(&bma250->enable_mutex);
	if (enable) {
		if (pre_enable ==0) {
			bma250_set_mode(bma250->bma250_client,
					BMA250_MODE_NORMAL);
			schedule_delayed_work(&bma250->work,
					msecs_to_jiffies(atomic_read(&bma250->delay)));
			atomic_set(&bma250->enable, 1);
		}

	} else {
		if (pre_enable ==1) {
			bma250_set_mode(bma250->bma250_client,
					BMA250_MODE_SUSPEND);
			cancel_delayed_work_sync(&bma250->work);
			atomic_set(&bma250->enable, 0);
		}
	}
	mutex_unlock(&bma250->enable_mutex);

}

static ssize_t bma250_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0)||(data==1)) {
		bma250_set_enable(dev,data);
	}

	return count;
}

static ssize_t bma250_enable_int_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int type, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	sscanf(buf, "%d%d", &type, &value);

	if (bma250_set_Int_Enable(bma250->bma250_client, type, value) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_int_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_Int_Mode(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_int_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	data = (unsigned char)simple_strtoul(buf, NULL, 10);

	if (bma250_set_Int_Mode(bma250->bma250_client, data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_slope_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_slope_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_slope_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_slope_duration(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_slope_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_slope_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_slope_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_slope_threshold(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_high_g_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_high_g_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_high_g_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_high_g_duration(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_high_g_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_high_g_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_high_g_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_high_g_threshold(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_low_g_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_low_g_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_low_g_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_low_g_duration(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_low_g_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_low_g_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_low_g_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_low_g_threshold(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_tap_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_tap_threshold(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_tap_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_tap_duration(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_tap_quiet_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_quiet(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_quiet_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_tap_quiet(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_tap_shock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_shock(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_shock_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_tap_shock(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_tap_samp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_samp(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_samp_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_tap_samp(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_orient_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_orient_mode(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_orient_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_orient_mode(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_orient_blocking_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_orient_blocking(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_orient_blocking_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_orient_blocking(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_orient_hyst_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_orient_hyst(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_orient_hyst_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_orient_hyst(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_orient_theta_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_theta_blocking(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_orient_theta_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_theta_blocking(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_flat_theta_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_theta_flat(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_flat_theta_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_theta_flat(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_flat_hold_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_flat_hold_time(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_flat_hold_time_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_flat_hold_time(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}


static ssize_t bma250_fast_calibration_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{



	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);



	return sprintf(buf, "%d\n", bma250->cal_data.x_cal);

}

static ssize_t bma250_fast_calibration_x_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	char value[2];
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	bma250->cal_data.status &= 0x0E;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_offset_target_x(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma250_set_cal_trigger(bma250->bma250_client, 1) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma250_get_cal_ready(bma250->bma250_client, &tmp);

		BMAMSG("wait 2ms and got cal ready flag is %d\n",tmp);
		timeout++;
		if(timeout==250) {
			printk(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		};

	}while(tmp==0);

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_X_REG,
								&value[0]) < 0)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_X_REG,
								&value[1]) < 0)
		return -1;

	bma250->cal_data.x_cal = (short)(value[1]*256) + (short)value[0];
	bma250->cal_data.status |= 0x01;

	BMAMSG("x axis fast calibration finished\n");
	return count;
}

static ssize_t bma250_fast_calibration_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{



	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);


	

	return sprintf(buf, "%d\n", bma250->cal_data.y_cal);

}

static ssize_t bma250_fast_calibration_y_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	char value[2];
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);


	bma250->cal_data.status &= 0x0D;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_offset_target_y(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma250_set_cal_trigger(bma250->bma250_client, 2) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma250_get_cal_ready(bma250->bma250_client, &tmp);

		BMAMSG("wait 2ms and got cal ready flag is %d\n",tmp);
		timeout++;
		if(timeout==250) {
			printk(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		};

	}while(tmp==0);

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_Y_REG,
								&value[0]) < 0)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_Y_REG,
								&value[1]) < 0)
		return -1;

	bma250->cal_data.y_cal = (short)(value[1]*256) + (short)value[0];
	bma250->cal_data.status |= 0x02;

	BMAMSG("y axis fast calibration finished\n");
	return count;
}

static ssize_t bma250_fast_calibration_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{



	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);




	return sprintf(buf, "%d\n", bma250->cal_data.z_cal);

}

static ssize_t bma250_fast_calibration_z_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	char value[2];
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	bma250->cal_data.status &= 0x0B;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_offset_target_z(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	if (bma250_set_cal_trigger(bma250->bma250_client, 3) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma250_get_cal_ready(bma250->bma250_client, &tmp);

		BMAMSG("wait 2ms and got cal ready flag is %d\n",tmp);
		timeout++;
		if(timeout==250) {
			printk(KERN_INFO "get fast calibration ready error\n");
			return -EINVAL;
		};

	}while(tmp==0);

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_Z_REG,
								&value[0]) < 0)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_Z_REG,
								&value[1]) < 0)
		return -1;

	bma250->cal_data.z_cal = (short)(value[1]*256) + (short)value[0];
	bma250->cal_data.status |= 0x04;

	BMAMSG("z axis fast calibration finished\n");
	return count;
}

static int get_calibration_data(struct bma250_data *bma250)
{
	char value[2];

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_X_REG,
								&value[0]) < 0)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_X_REG,
								&value[1]) < 0)
		return -1;

	bma250->cal_data.x_cal = (short)(value[1]*256) + (short)value[0];
	bma250->cal_data.status &= ~0x01;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_Y_REG,
								&value[0]) < 0)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_Y_REG,
								&value[1]) < 0)
		return -1;

	bma250->cal_data.y_cal = (short)(value[1]*256) + (short)value[0];
	bma250->cal_data.status &= ~0x02;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_Z_REG,
								&value[0]) < 0)
		return -1;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_Z_REG,
								&value[1]) < 0)
		return -1;

	bma250->cal_data.z_cal = (short)(value[1]*256) + (short)value[0];
	bma250->cal_data.status &= ~0x04;

	return 0;
}

static ssize_t bma250_cal_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma250_data *bma250 = input_get_drvdata(input);
    unsigned short cal_status;

	cal_status = bma250->cal_data.status;

	return sprintf(buf, "%d\n", cal_status);
}
static ssize_t bma250_selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma250->selftest_result));

}

static ssize_t bma250_selftest_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

	unsigned long data;
	unsigned char clear_value = 0;
	int error;
	short value1 = 0;
	short value2 = 0;
	short diff = 0;
	unsigned long result = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if(data!=1)
	{
		printk("[BMA250] selftest command error! data:%ld\n", data);
		return -EINVAL;
	}
	atomic_set(&bma250->selftest_result, 0);
	
	if (bma250_set_range(bma250->bma250_client, 0) < 0)
		return -EINVAL;

	bma250_write_reg(bma250->bma250_client,0x32,&clear_value);

	bma250_set_selftest_st(bma250->bma250_client,1); 
	bma250_set_selftest_stn(bma250->bma250_client,0); 
	mdelay(50);
	bma250_read_accel_x(bma250->bma250_client,&value1);
	bma250_set_selftest_stn(bma250->bma250_client,1); 
	mdelay(50);
	bma250_read_accel_x(bma250->bma250_client,&value2);
	diff = value1-value2;

	printk(KERN_INFO "diff x is %d,value1 is %d, value2 is %d\n",diff,value1,value2);

	if(abs(diff)>204) result |= 1;

	bma250_set_selftest_st(bma250->bma250_client,2); 
	bma250_set_selftest_stn(bma250->bma250_client,0); 
	mdelay(50);
	bma250_read_accel_y(bma250->bma250_client,&value1);
	bma250_set_selftest_stn(bma250->bma250_client,1); 
	mdelay(50);
	bma250_read_accel_y(bma250->bma250_client,&value2);
	diff = value1-value2;
	printk(KERN_INFO "diff y is %d,value1 is %d, value2 is %d\n",diff,value1,value2);
	if(abs(diff)>204) result |= 2;


	bma250_set_selftest_st(bma250->bma250_client,3); 
	bma250_set_selftest_stn(bma250->bma250_client,0); 
	mdelay(50);
	bma250_read_accel_z(bma250->bma250_client,&value1);
	bma250_set_selftest_stn(bma250->bma250_client,1); 
	mdelay(50);
	bma250_read_accel_z(bma250->bma250_client,&value2);
	diff = value1-value2;

	printk(KERN_INFO "diff z is %d,value1 is %d, value2 is %d\n",diff,value1,value2);
	if(abs(diff)>102) result |= 4;

	atomic_set(&bma250->selftest_result, (unsigned int) result);
	
	bma250_write_reg(bma250->bma250_client,0x32,&clear_value);

	printk(KERN_INFO "[BMA250] self test finished. result:%d\n",atomic_read(&bma250->selftest_result));

	return count;
}

static DEVICE_ATTR(chipid, S_IRUGO,
		bma250_chipid_show, NULL);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_range_show, bma250_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_bandwidth_show, bma250_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_mode_show, bma250_mode_store);
static DEVICE_ATTR(value, S_IRUGO,
		bma250_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_delay_show, bma250_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_enable_show, bma250_enable_store);
static DEVICE_ATTR(enable_int, S_IWUSR|S_IWGRP,
		NULL, bma250_enable_int_store);
static DEVICE_ATTR(int_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_int_mode_show, bma250_int_mode_store);
static DEVICE_ATTR(slope_duration, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_slope_duration_show, bma250_slope_duration_store);
static DEVICE_ATTR(slope_threshold, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_slope_threshold_show, bma250_slope_threshold_store);
static DEVICE_ATTR(high_g_duration, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_high_g_duration_show, bma250_high_g_duration_store);
static DEVICE_ATTR(high_g_threshold, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_high_g_threshold_show, bma250_high_g_threshold_store);
static DEVICE_ATTR(low_g_duration, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_low_g_duration_show, bma250_low_g_duration_store);
static DEVICE_ATTR(low_g_threshold, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_low_g_threshold_show, bma250_low_g_threshold_store);
static DEVICE_ATTR(tap_duration, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_tap_duration_show, bma250_tap_duration_store);
static DEVICE_ATTR(tap_threshold, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_tap_threshold_show, bma250_tap_threshold_store);
static DEVICE_ATTR(tap_quiet, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_tap_quiet_show, bma250_tap_quiet_store);
static DEVICE_ATTR(tap_shock, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_tap_shock_show, bma250_tap_shock_store);
static DEVICE_ATTR(tap_samp, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_tap_samp_show, bma250_tap_samp_store);
static DEVICE_ATTR(orient_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_orient_mode_show, bma250_orient_mode_store);
static DEVICE_ATTR(orient_blocking, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_orient_blocking_show, bma250_orient_blocking_store);
static DEVICE_ATTR(orient_hyst, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_orient_hyst_show, bma250_orient_hyst_store);
static DEVICE_ATTR(orient_theta, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_orient_theta_show, bma250_orient_theta_store);
static DEVICE_ATTR(flat_theta, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_flat_theta_show, bma250_flat_theta_store);
static DEVICE_ATTR(flat_hold_time, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_flat_hold_time_show, bma250_flat_hold_time_store);
static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_register_show, bma250_register_store);
static DEVICE_ATTR(fast_calibration_x, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_fast_calibration_x_show, bma250_fast_calibration_x_store);
static DEVICE_ATTR(fast_calibration_y, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_fast_calibration_y_show, bma250_fast_calibration_y_store);
static DEVICE_ATTR(fast_calibration_z, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_fast_calibration_z_show, bma250_fast_calibration_z_store);
static DEVICE_ATTR(calibration_status, S_IRUGO,
		bma250_cal_status_show, NULL);
static DEVICE_ATTR(selftest, S_IRUGO|S_IWUSR|S_IWGRP,
		bma250_selftest_show, bma250_selftest_store);

static struct attribute *bma250_attributes[] = {
	&dev_attr_chipid.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_enable_int.attr,
	&dev_attr_int_mode.attr,
	&dev_attr_slope_duration.attr,
	&dev_attr_slope_threshold.attr,
	&dev_attr_high_g_duration.attr,
	&dev_attr_high_g_threshold.attr,
	&dev_attr_low_g_duration.attr,
	&dev_attr_low_g_threshold.attr,
	&dev_attr_tap_threshold.attr,
	&dev_attr_tap_duration.attr,
	&dev_attr_tap_quiet.attr,
	&dev_attr_tap_shock.attr,
	&dev_attr_tap_samp.attr,
	&dev_attr_orient_mode.attr,
	&dev_attr_orient_blocking.attr,
	&dev_attr_orient_hyst.attr,
	&dev_attr_orient_theta.attr,
	&dev_attr_flat_theta.attr,
	&dev_attr_flat_hold_time.attr,
	&dev_attr_reg.attr,
	&dev_attr_fast_calibration_x.attr,
	&dev_attr_fast_calibration_y.attr,
	&dev_attr_fast_calibration_z.attr,
	&dev_attr_calibration_status.attr,
	&dev_attr_selftest.attr,
	NULL
};

static struct attribute_group bma250_attribute_group = {
	.attrs = bma250_attributes
};


#if defined(BMA250_ENABLE_INT1)||defined(BMA250_ENABLE_INT2)
unsigned char *orient[]={"upward looking portrait upright",   \
	"upward looking portrait upside-down",   \
		"upward looking landscape left",   \
		"upward looking landscape right",   \
		"downward looking portrait upright",   \
		"downward looking portrait upside-down",   \
		"downward looking landscape left",   \
		"downward looking landscape right"};

static void bma250_irq_work_func(struct work_struct *work)
{
	struct bma250_data *bma250 = container_of((struct work_struct *)work,
			struct bma250_data, irq_work);

	unsigned char status = 0;
	unsigned char i;
	unsigned char first_value = 0;
	unsigned char sign_value = 0;

	bma250_get_interruptstatus1(bma250->bma250_client, &status);

	switch(status) {

	case 0x01: BMAMSG("Low G interrupt happened\n");
		   input_report_rel(bma250->input,LOW_G_INTERRUPT,LOW_G_INTERRUPT_HAPPENED);
		   break;
	case 0x02: for(i=0;i<3;i++)
		   {
			   bma250_get_HIGH_first(bma250->bma250_client,i,&first_value);
			   if(first_value==1){

				   bma250_get_HIGH_sign(bma250->bma250_client,&sign_value);

				   if(sign_value==1){
					   if(i==0)input_report_rel(bma250->input,HIGH_G_INTERRUPT,HIGH_G_INTERRUPT_X_NEGATIVE_HAPPENED);
					   if(i==1)input_report_rel(bma250->input,HIGH_G_INTERRUPT,HIGH_G_INTERRUPT_Y_NEGATIVE_HAPPENED);
					   if(i==2)input_report_rel(bma250->input,HIGH_G_INTERRUPT,HIGH_G_INTERRUPT_Z_NEGATIVE_HAPPENED);
				   } else {
					   if(i==0)input_report_rel(bma250->input,HIGH_G_INTERRUPT,HIGH_G_INTERRUPT_X_HAPPENED);
					   if(i==1)input_report_rel(bma250->input,HIGH_G_INTERRUPT,HIGH_G_INTERRUPT_Y_HAPPENED);
					   if(i==2)input_report_rel(bma250->input,HIGH_G_INTERRUPT,HIGH_G_INTERRUPT_Z_HAPPENED);

				   }
			   }

			   BMAMSG("High G interrupt happened,exis is %d,first is %d,sign is %d\n",i,first_value,sign_value);
		   }
		   break;
	case 0x04: for(i=0;i<3;i++)
		   {
			   bma250_get_slope_first(bma250->bma250_client,i,&first_value);
			   if(first_value==1){

				   bma250_get_slope_sign(bma250->bma250_client,&sign_value);

				   if(sign_value==1){
					   if(i==0)input_report_rel(bma250->input,SLOP_INTERRUPT,SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED);
					   else if(i==1)input_report_rel(bma250->input,SLOP_INTERRUPT,SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED);
					   else if(i==2)input_report_rel(bma250->input,SLOP_INTERRUPT,SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED);
				   } else {
					   if(i==0)input_report_rel(bma250->input,SLOP_INTERRUPT,SLOPE_INTERRUPT_X_HAPPENED);
					   else if(i==1)input_report_rel(bma250->input,SLOP_INTERRUPT,SLOPE_INTERRUPT_Y_HAPPENED);
					   else if(i==2)input_report_rel(bma250->input,SLOP_INTERRUPT,SLOPE_INTERRUPT_Z_HAPPENED);

				   }
			   }

			   BMAMSG("Slop interrupt happened,exis is %d,first is %d,sign is %d\n",i,first_value,sign_value);
		   }
		   break;

	case 0x10: 	BMAMSG("double tap interrupt happened\n");
			input_report_rel(bma250->input,DOUBLE_TAP_INTERRUPT,DOUBLE_TAP_INTERRUPT_HAPPENED);
			break;
	case 0x20: 	BMAMSG("single tap interrupt happened\n");
			input_report_rel(bma250->input,SINGLE_TAP_INTERRUPT,SINGLE_TAP_INTERRUPT_HAPPENED);
			break;
	case 0x40:  bma250_get_orient_status(bma250->bma250_client,&first_value);
		   BMAMSG("orient interrupt happened,%s\n",orient[first_value]);
		    if(first_value==0)input_report_abs(bma250->input,ORIENT_INTERRUPT,UPWARD_PORTRAIT_UP_INTERRUPT_HAPPENED);
		    else if(first_value==1)input_report_abs(bma250->input,ORIENT_INTERRUPT,UPWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED);
		    else if(first_value==2)input_report_abs(bma250->input,ORIENT_INTERRUPT,UPWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED);
		    else if(first_value==3)input_report_abs(bma250->input,ORIENT_INTERRUPT,UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED);
		    else if(first_value==4)input_report_abs(bma250->input,ORIENT_INTERRUPT,DOWNWARD_PORTRAIT_UP_INTERRUPT_HAPPENED);
		    else if(first_value==5)input_report_abs(bma250->input,ORIENT_INTERRUPT,DOWNWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED);
		    else if(first_value==6)input_report_abs(bma250->input,ORIENT_INTERRUPT,DOWNWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED);
		    else if(first_value==7)input_report_abs(bma250->input,ORIENT_INTERRUPT,DOWNWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED);
		    break;
	case 0x80:  bma250_get_orient_flat_status(bma250->bma250_client,&sign_value);
		    BMAMSG("flat interrupt happened,flat status is %d\n",sign_value);
		    if(sign_value==1) {
			    input_report_abs(bma250->input,FLAT_INTERRUPT,FLAT_INTERRUPT_TURE_HAPPENED);
		    } else {
			    input_report_abs(bma250->input,FLAT_INTERRUPT,FLAT_INTERRUPT_FALSE_HAPPENED);
		    }
		    break;
	default: break;
	}

}

static irqreturn_t bma250_irq_handler(int irq, void *handle)
{
	struct bma250_data *data = handle;

	

	if (data == NULL)
		return IRQ_HANDLED;
	if (data->bma250_client == NULL)
		return IRQ_HANDLED;

	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}
#endif 

int load_caldata_to_reg(void)
{
	struct bma250_data *bma250 = i2c_get_clientdata(this_client);
	unsigned char data;

	data = bma250->cal_data.x_cal && 0xff;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_X_REG,
								&data) < 0)
		return -1;

	data = bma250->cal_data.x_cal >> 8;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_X_REG,
								&data) < 0)
		return -1;

	data = bma250->cal_data.y_cal && 0xff;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_Y_REG,
								&data) < 0)
		return -1;

	data = bma250->cal_data.y_cal >> 8;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_Y_REG,
								&data) < 0)
		return -1;

	data = bma250->cal_data.z_cal && 0xff;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_OFFSET_FILT_Z_REG,
								&data) < 0)
		return -1;

	data = bma250->cal_data.z_cal >> 8;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_OFFSET_UNFILT_Z_REG,
								&data) < 0)
		return -1;

	return 0;
}

static int store_caldata_to_e2prom(void)
{
	struct bma250_data *bma250 = i2c_get_clientdata(this_client);
	unsigned char data = 0, tmp = 0, timeout = 0;

	if (bma250_smbus_read_byte(bma250->bma250_client,
								BMA250_EEPROM_CTRL_REG,
								&data) < 0)
		return -1;

	data = data | 0x01 ;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_EEPROM_CTRL_REG,
								&data) < 0)
		return -1;
	mdelay(500);
	data = data | 0x02 ;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_EEPROM_CTRL_REG,
								&data) < 0)
		return -1;
	mdelay(500);

	do {
		mdelay(2);
		bma250_smbus_read_byte(bma250->bma250_client,
										BMA250_EEPROM_CTRL_REG,
										&data);
		tmp = data & 0x04;

		timeout++;
		if(timeout==50) {
			BMAMSG("store_caldata_to_e2prom error\n");
			return -EINVAL;
		};
	}while(tmp==0);

	data = data & 0xFE ;
	if (bma250_smbus_write_byte(bma250->bma250_client,
								BMA250_EEPROM_CTRL_REG,
								&data) < 0)
		return -1;

	return 0;
}


static int bma250_open(struct inode *inode, struct file *file)
{
	BMAMSG("bma250_open");
	return 0;
}

static int bma250_release(struct inode *inode, struct file *file)
{
	BMAMSG("bma250_release");
	return 0;
}

static long
bma250_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	struct bma250_data *bma250 = i2c_get_clientdata(this_client);
	struct device *bma_dev = &(bma250->input->dev);
	struct bma250acc acc_value;
	char buf[10];
	unsigned short delay = 0, range = 0;

	void __user *argp = (void __user *)arg;

	switch (cmd) {
		case GSENSOR_IOC_GET_ACCEL:
			mutex_lock(&bma250->value_mutex);
			acc_value = bma250->value;
			
			mutex_unlock(&bma250->value_mutex);
			if (copy_to_user(argp, &acc_value, sizeof(acc_value))) {
				BMAMSG("copy accel value failed.");
				return -EFAULT;
			}
	  	return 0;
		case GSENSOR_IOC_ENABLE:
			bma250_set_enable(bma_dev, 1);
			break;
		case GSENSOR_IOC_DISABLE:
			bma250_set_enable(bma_dev, 0);
			break;
		case GSENSOR_IOC_SET_CALDAT:
			if (argp == NULL) {
				BMAMSG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&(bma250->cal_data), argp, sizeof(struct gsensor_cal_data))) {
				BMAMSG("copy_from_user failed.");
				return -EFAULT;
			}
			if(load_caldata_to_reg())
			{
				BMAMSG("load calibration data Failed.");
				return -EFAULT;
			}
			return 0;
	  case GSENSOR_IOC_CALIBRATION:
			sprintf(buf, "%d", 0);
			bma250_fast_calibration_x_store(bma_dev, NULL, buf, 0);
			sprintf(buf, "%d", 0);
			bma250_fast_calibration_y_store(bma_dev, NULL, buf, 0);
			sprintf(buf, "%d", 2);
			bma250_fast_calibration_z_store(bma_dev, NULL, buf, 0);

			if(bma250->cal_data.status == 7)
			{
				if (copy_to_user(argp, &(bma250->cal_data), sizeof(struct gsensor_cal_data))) {
						printk("[BMA250] copy cal_data failed\n.");
						return -EFAULT;
				}			
				BMAMSG("cal_data default: %d %d %d %d",
					bma250->cal_data.x_cal,
					bma250->cal_data.y_cal,
					bma250->cal_data.z_cal,
					bma250->cal_data.status);
				if(store_caldata_to_e2prom() == 0)
					BMAMSG("store calibration data to e2prom success.");
			  	return 0;
			}
	  	return -EFAULT;
		case GSENSOR_IOC_GET_CHIPID:
			if (copy_to_user(argp, &gsensor_chipid, 1)) {
				BMAMSG("copy gsensor_chipid failed.");
				return -EFAULT;
			}
	  	return 0;
		case GSENSOR_IOC_SET_DELAY:
			if (copy_from_user(&delay, argp, sizeof(delay))) {
				BMAMSG("copy delay value failed.");
				return -EFAULT;
			}
			BMAMSG("Set gsensor delay is %d ms.", delay);
			if (delay > BMA250_MAX_DELAY)
				delay = BMA250_MAX_DELAY;
			atomic_set(&bma250->delay, (short)delay);
			return 0;
		case GSENSOR_IOC_GET_RANGE:
			range =	atomic_read(&bma250->range);
			if (copy_to_user(argp, &range, sizeof(range))) {
				BMAMSG("copy range failed.");
				return -EFAULT;
			}
	 		return 0;
		
		case GSENSOR_IOC_UPDATE_CAL_DATA:
			BMAMSG("%s: GSENSOR_IOC_UPDATE_CAL_DATA\n", __func__);
			if (argp == NULL) {
				printk("invalid argument.\n");
				return -EINVAL;
			} 
			if (copy_from_user(&(bma250->cal_fa_data), argp, sizeof(struct gsensor_cal_data))) {
				printk("[BMA]copy_from_user failed.");
				return -EFAULT;
			}
			
			BMAMSG("%s: x_cal=0x%02X, y_cal=0x%02X, z_cal=0x%02X,status=%d\n",__func__, 
				bma250->cal_fa_data.x_cal, bma250->cal_fa_data.y_cal,bma250->cal_fa_data.z_cal,bma250->cal_fa_data.status);

			if (bma250->cal_fa_data.status == 7) {
				get_calibration_data(bma250);
				BMAMSG("%s: x_cal=0x%02X, y_cal=0x%02X, z_cal=0x%02X\n",__func__, 
				bma250->cal_data.x_cal, bma250->cal_data.y_cal, bma250->cal_data.z_cal);

				if ((bma250->cal_fa_data.x_cal == bma250->cal_data.x_cal) &&
					(bma250->cal_fa_data.y_cal == bma250->cal_data.y_cal) &&
					(bma250->cal_fa_data.z_cal == bma250->cal_data.z_cal) ) {
					
					printk("[BMA250]get cal_data from eeprom!\n");
				}
				else
				{
					printk("[BMA250]rewrite the cal_data of eeprom!\n");
					bma250->cal_data.x_cal = bma250->cal_fa_data.x_cal;
					bma250->cal_data.y_cal = bma250->cal_fa_data.y_cal;
					bma250->cal_data.z_cal = bma250->cal_fa_data.z_cal;

					if(load_caldata_to_reg())
					{
						printk("[BMA250] load calibration data Failed.");
					}

					if(store_caldata_to_e2prom() == 0)
						BMAMSG("store calibration data to e2prom success.");
				}
			}
			
			return 0;
					
		default:
			break;
	}

	return 0;
}

static struct file_operations bma250_fops = {
	.owner = THIS_MODULE,
	.open = bma250_open,
	.release = bma250_release,
	.unlocked_ioctl = bma250_ioctl,
};

static struct miscdevice bma250_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma250_dev",
	.fops = &bma250_fops,
};


static int bma250_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	unsigned char tempvalue;
	struct bma250_data *data;
	struct input_dev *dev;
	unsigned char  wdt=0x06;

	bma250_power(1);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	
	tempvalue = i2c_smbus_read_byte_data(client, BMA250_CHIP_ID_REG);

	if (tempvalue == BMA250_CHIP_ID) {
		BMAMSG(KERN_INFO "Bosch Sensortec Device detected!\n"
				"BMA250 registered I2C driver!\n");
		gsensor_chipid = BMA250_CHIP_ID;
	} else{
		printk(KERN_INFO "Bosch Sensortec Device not found "
				"i2c error %d \n", tempvalue);
		err = -ENODEV;
		goto kfree_exit;
	}
	i2c_set_clientdata(client, data);
	data->bma250_client = client;
	this_client = client;
	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	if (bma250_reset(this_client)) {
		printk("[BMA250] Gsensor reset failed.");
		err = -EFAULT;
		goto exit;
	}
	
	
	msleep(5);
	

	bma250_set_bandwidth(client, BMA250_BW_SET);
	bma250_set_range(client, BMA250_RANGE_SET);
	bma250_smbus_write_byte(client,
				BMA250_SERIAL_CTRL_REG, &wdt);

#if defined(BMA250_ENABLE_INT1)||defined(BMA250_ENABLE_INT2)
	bma250_set_Int_Mode(client,1);
#endif
	
	
	
	
	
	
	
	bma250_set_Int_Enable(client,8, 1);
	bma250_set_Int_Enable(client,10, 1);
	bma250_set_Int_Enable(client,11, 1);

#ifdef BMA250_ENABLE_INT1
	
	bma250_set_int1_pad_sel(client,PAD_LOWG);
	bma250_set_int1_pad_sel(client,PAD_HIGHG);
	bma250_set_int1_pad_sel(client,PAD_SLOP);
	bma250_set_int1_pad_sel(client,PAD_DOUBLE_TAP);
	bma250_set_int1_pad_sel(client,PAD_SINGLE_TAP);
	bma250_set_int1_pad_sel(client,PAD_ORIENT);
	bma250_set_int1_pad_sel(client,PAD_FLAT);
#endif

#ifdef BMA250_ENABLE_INT2
	
	bma250_set_int2_pad_sel(client,PAD_LOWG);
	bma250_set_int2_pad_sel(client,PAD_HIGHG);
	bma250_set_int2_pad_sel(client,PAD_SLOP);
	bma250_set_int2_pad_sel(client,PAD_DOUBLE_TAP);
	bma250_set_int2_pad_sel(client,PAD_SINGLE_TAP);
	bma250_set_int2_pad_sel(client,PAD_ORIENT);
	bma250_set_int2_pad_sel(client,PAD_FLAT);
#endif

#if defined(BMA250_ENABLE_INT1)||defined(BMA250_ENABLE_INT2)
	INIT_WORK(&data->irq_work, bma250_irq_work_func);
	data->IRQ = client->irq;
	err = request_irq(data->IRQ, bma250_irq_handler, IRQF_TRIGGER_RISING, "bma250", data);
	if (err) {
		printk(KERN_ERR "could not request irq\n");
	}
#endif

	INIT_DELAYED_WORK(&data->work, bma250_work_func);
	atomic_set(&data->delay, BMA250_MAX_DELAY);
	atomic_set(&data->enable, 0);

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, LOW_G_INTERRUPT);
	input_set_capability(dev, EV_REL, HIGH_G_INTERRUPT);
	input_set_capability(dev, EV_REL, SLOP_INTERRUPT);
	input_set_capability(dev, EV_REL, DOUBLE_TAP_INTERRUPT);
	input_set_capability(dev, EV_REL, SINGLE_TAP_INTERRUPT);
	input_set_capability(dev, EV_ABS, ORIENT_INTERRUPT);
	input_set_capability(dev, EV_ABS, FLAT_INTERRUPT);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);

	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		goto kfree_exit;
	}

	data->input = dev;
	
	err = sysfs_create_group(&data->input->dev.kobj,
			&bma250_attribute_group);
	if (err < 0)
		goto error_sysfs;

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = bma250_early_suspend;
	data->early_suspend.resume = bma250_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	bma250_set_enable(&(data->input->dev), 1);

	
	err = misc_register(&bma250_device);
	if (err) {
		printk(KERN_ERR "bma250_device register failed\n");
		goto exit;
	}


	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	return 0;

error_sysfs:
	input_unregister_device(data->input);

kfree_exit:
	kfree(data);
exit:
	return err;
}


void bma250_shutdown(struct i2c_client *client)
{
	bma250_set_enable(&client->dev, 0);
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h)
{
	struct bma250_data *data =
		container_of(h, struct bma250_data, early_suspend);

	atomic_set(&reserve_enable_flag, atomic_read(&data->enable));
	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable)==1) {
		atomic_set(&data->enable, 0);
		bma250_set_mode(data->bma250_client, BMA250_MODE_SUSPEND);
		cancel_delayed_work_sync(&data->work);
	}
	mutex_unlock(&data->enable_mutex);
}


static void bma250_late_resume(struct early_suspend *h)
{
	struct bma250_data *data =
		container_of(h, struct bma250_data, early_suspend);
	int range = atomic_read(&data->range);
	int bandwidth = atomic_read(&data->bandwidth);

  msleep(1);
	atomic_set(&data->enable, atomic_read(&reserve_enable_flag));
	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable)==1) {
		bma250_set_range(data->bma250_client, (unsigned char)range);
		bma250_set_bandwidth(data->bma250_client, (unsigned char)bandwidth);
		bma250_set_mode(data->bma250_client, BMA250_MODE_NORMAL);
		schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)+300));
	}
	mutex_unlock(&data->enable_mutex);
}
#endif

static int __devexit bma250_remove(struct i2c_client *client)
{
	struct bma250_data *data = i2c_get_clientdata(client);

	bma250_set_enable(&client->dev, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &bma250_attribute_group);
	input_unregister_device(data->input);

	kfree(data);
	bma250_power(0);
	regulator_put(vreg_smps3_1p8);
	regulator_put(vreg_ldo12_2p85);

	return 0;
}
#ifdef CONFIG_PM

static int bma250_suspend(struct i2c_client *client, pm_message_t mesg)
{
	bma250_power(0);

	return 0;
}

static int bma250_resume(struct i2c_client *client)
{
	bma250_power(1);

	return 0;
}

#else

#define bma250_suspend		NULL
#define bma250_resume		NULL

#endif 

static const struct i2c_device_id bma250_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);

static struct i2c_driver bma250_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
	},
	.suspend	= bma250_suspend,
	.resume		= bma250_resume,
	.id_table	= bma250_id,
	.probe		= bma250_probe,
	.shutdown	= bma250_shutdown,
	.remove		= __devexit_p(bma250_remove),

};

static int __init BMA250_init(void)
{
	return i2c_add_driver(&bma250_driver);
}

static void __exit BMA250_exit(void)
{
	i2c_del_driver(&bma250_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA250 accelerometer sensor driver");
MODULE_LICENSE("GPL");

module_init(BMA250_init);
module_exit(BMA250_exit);

