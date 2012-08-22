/* Source for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * drivers/input/touchscreen/cyttsp_touch.c
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

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <mach/rpc_pmapp.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <mach/touch_TMA340_cypress.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <mach/chicago_hwid.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
#define ATMEL_POR_DELAY         100 
#include <mach/pm_log.h>


extern struct dentry *kernel_debuglevel_dir;
static unsigned int CYTTSP_DLL=0;

#if 0
module_param_named(
	DLL, DLL,
 	int, S_IRUGO | S_IWUSR | S_IWGRP
);
#endif
#define ERR_LEVEL   0
#define INFO_LEVEL  1
#define DEBUG_LEVEL 2

#define CYTTSP_PRINTK(level, fmt, args...) if(level <= CYTTSP_DLL) printk("%s:" fmt, __func__, ##args);



struct cyttsp {
	struct i2c_client *client;
	struct input_dev *input;
	struct input_dev *keyarray_input;
	struct work_struct work;
	struct workqueue_struct *cyttsp_ts_wq;
	struct delayed_work ts_work;
	struct timer_list timer;
	struct mutex mutex;
	struct cyttsp_op_mode_data_t op_mode_data;
	struct cyttsp_sysinfo_data_t sysinfo_data;
	struct cyttsp_touch_point_status_t msg[CY_NUM_MT_TCH_ID];
	struct cyttsp_cap_key_point_status_t key_msg[CY_NUM_OF_CAP_KEY];
	enum cyttsp_pwr_state_t power_state;
	u16 cyttsp_x_max,cyttsp_y_max;
	u8 fw_ver; 
	u8 sleep_mode;
	int gpio_num[CY_MAX_NUM_GPIO];
	int is_suspended;
	int open_count; 
	int key_open_count; 
	int misc_open_count; 
	int	irq; 
	
	
	
	struct regulator *vdd_regulator; 
	struct regulator *avdd_1_regulator; 
	struct regulator *avdd_2_regulator; 
	uint8_t i2c_addr;
	int chk_k_flag;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend ts_early_suspend;
#endif 
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_suspend(struct early_suspend *handler);
static void cyttsp_resume(struct early_suspend *handler);
#endif 

struct workqueue_struct *cyttsp_ts_wq;
static struct cyttsp *g_ts;
const char *gpio_name[] = {"cyttsp_ts_irq", "cyttsp_ts_rst", "cyttsp_ts_issp_sclk ", "cyttsp_ts_issp_sdata "};
enum {IRQ_GPIO=0, RST_GPIO, ISSP_CLK, ISSP_SDATA};


static void cyttsp_irqWorkHandler(struct work_struct *work);
static irqreturn_t cyttsp_irq(int irq, void *handle);
static int __devinit cyttsp_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int __devexit cyttsp_remove(struct i2c_client *client);

#define TOUCH_RETRY_COUNT 5
static int ts_write_i2c( struct i2c_client *client,
                         uint8_t           regBuf,
                         uint8_t           *dataBuf,
                         uint8_t           dataLen )
{
    int     result = 0;
    uint8_t *buf = NULL;
    int     retryCnt = TOUCH_RETRY_COUNT;


    struct  i2c_msg msgs[] = {
        [0] = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)buf,
            .len    = 0
        }
    };

    buf = kzalloc( dataLen+sizeof(regBuf), GFP_KERNEL );
    if( NULL == buf )
    {
        CYTTSP_PRINTK(0, "alloc memory failed\n");
        return -EFAULT;
    }

    buf[0] = regBuf;
    memcpy( &buf[1], dataBuf, dataLen );
    msgs[0].buf = buf;
    msgs[0].len = dataLen+1;

    while(retryCnt)
    {
        result = i2c_transfer( client->adapter, msgs, 1 );
        if( result != ARRAY_SIZE(msgs) )
        {
            CYTTSP_PRINTK(0, "write %Xh %d bytes return failure, %d\n", buf[0], dataLen, result);
            
            
            if( -ETIMEDOUT == result ) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
        }
    }

    if( (result == 0) && (retryCnt < TOUCH_RETRY_COUNT) )
        CYTTSP_PRINTK(0, "write %Xh %d bytes retry at %d\n", buf[0], dataLen, TOUCH_RETRY_COUNT-retryCnt);

    kfree( buf );
    return result;
}

static int ts_read_i2c( struct i2c_client *client,
                        uint8_t           regBuf,
                        uint8_t           *dataBuf,
                        uint8_t           dataLen )
{
    int     result = 0;
    int     retryCnt = TOUCH_RETRY_COUNT;

    struct  i2c_msg msgs[] = {
        [0] = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)&regBuf,
            .len    = 1
        },
        [1] = {
            .addr   = client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)dataBuf,
            .len    = dataLen
        }
    };

    while(retryCnt)
    {
        result = i2c_transfer( client->adapter, msgs, 2 );
        if( result != ARRAY_SIZE(msgs) )
        {
            CYTTSP_PRINTK(0, "read %Xh %d bytes return failure, %d\n", regBuf, dataLen, result );
            
            if( -ETIMEDOUT == result ) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
        }
    }

    if( (result == 0) && (retryCnt < TOUCH_RETRY_COUNT) )
        CYTTSP_PRINTK(0, "read %Xh %d bytes retry at %d\n", regBuf, dataLen, TOUCH_RETRY_COUNT-retryCnt);

    return result;
}

static const struct i2c_device_id cyttsp_id[] = {
	{ CY_I2C_NAME, 0 },  { }
};

MODULE_DEVICE_TABLE(i2c, cyttsp_id);

static struct i2c_driver i2c_ts_driver = {
	.driver = {
		.name = CY_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp_probe,
	.remove = __devexit_p(cyttsp_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = cyttsp_suspend,
	.resume = cyttsp_resume,
#endif
	.id_table = cyttsp_id,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver");
MODULE_AUTHOR("Cypress");

static void cyttsp_report_capkey(int id, struct cyttsp *g_ts)
{


	if (g_ts->key_msg[0].key_coord.x == 0 && g_ts->key_msg[0].key_coord.y == 0) {
		g_ts->key_msg[0].key_state = KEY_RELEASE;
		CYTTSP_PRINTK(1,"CAP KEYs coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
		CYTTSP_PRINTK(1," k[%d]:KEY_HOME is released\n",KEY_HOME);
		input_report_key(g_ts->keyarray_input, KEY_HOME, 0);
		CYTTSP_PRINTK(1," k[%d]:KEY_SEARCH is released\n",KEY_SEARCH);
		input_report_key(g_ts->keyarray_input, KEY_SEARCH, 0);
		CYTTSP_PRINTK(1," k[%d]:KEY_BACK is released\n",KEY_BACK);
		input_report_key(g_ts->keyarray_input, KEY_BACK, 0);
		CYTTSP_PRINTK(1," k[%d]:KEY_MENU is released\n",KEY_MENU);
		input_report_key(g_ts->keyarray_input, KEY_MENU, 0);
	}
	else {
		if (g_ts->key_msg[0].key_coord.y >= 870 && g_ts->key_msg[0].key_coord.y < 960) {
			if (g_ts->key_msg[0].key_coord.x >= 5 && g_ts->key_msg[0].key_coord.x < 95) {
				g_ts->key_msg[0].key_state = KEY_PRESS;
				CYTTSP_PRINTK(1," k[%d]:KEY_HOME is pressed\n",KEY_HOME);
				CYTTSP_PRINTK(1,"HOME KEY coord x:%d coord y:%d\n",
						g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				input_report_key(g_ts->keyarray_input, KEY_HOME, 1);
			}
			else {
				g_ts->key_msg[0].key_state = KEY_RELEASE;
				CYTTSP_PRINTK(1," k[%d]:KEY_HOME is released\n",KEY_HOME);
				CYTTSP_PRINTK(1,"HOME KEY coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				input_report_key(g_ts->keyarray_input, KEY_HOME, 0);
			}
			if (g_ts->key_msg[0].key_coord.x >= 140 && g_ts->key_msg[0].key_coord.x < 220) {
				g_ts->key_msg[0].key_state = KEY_PRESS;
				CYTTSP_PRINTK(1," k[%d]:KEY_SEARCH is pressed\n",KEY_SEARCH);
				CYTTSP_PRINTK(1,"HOME KEY coord x:%d coord y:%d\n",
						g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				input_report_key(g_ts->keyarray_input, KEY_SEARCH, 1);
			}
			else {
				g_ts->key_msg[0].key_state = KEY_RELEASE;
				CYTTSP_PRINTK(1," k[%d]:KEY_SEARCH is released\n",KEY_SEARCH);
				CYTTSP_PRINTK(1,"HOME KEY coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				input_report_key(g_ts->keyarray_input, KEY_SEARCH, 0);
			}
			if (g_ts->key_msg[0].key_coord.x >= 260 && g_ts->key_msg[0].key_coord.x < 340) {
				g_ts->key_msg[0].key_state = KEY_PRESS;
				CYTTSP_PRINTK(1," k[%d]:KEY_BACK is pressed\n",KEY_BACK);
				CYTTSP_PRINTK(1,"BACK KEY coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				input_report_key(g_ts->keyarray_input, KEY_BACK, 1);
			}
			else {
				g_ts->key_msg[0].key_state = KEY_RELEASE;
				CYTTSP_PRINTK(1," k[%d]:KEY_BACK is released\n",KEY_BACK);
				CYTTSP_PRINTK(1,"BACK KEY coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				input_report_key(g_ts->keyarray_input, KEY_BACK, 0);
			}
			if (g_ts->key_msg[0].key_coord.x >= 379 && g_ts->key_msg[0].key_coord.x < 475) {
				g_ts->key_msg[0].key_state = KEY_PRESS;
				CYTTSP_PRINTK(1," k[%d]:KEY_MENU is pressed\n",KEY_MENU);
				CYTTSP_PRINTK(1,"HOME KEY coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				input_report_key(g_ts->keyarray_input, KEY_MENU, 1);
			}
			else {
				g_ts->key_msg[0].key_state = KEY_RELEASE;
				CYTTSP_PRINTK(1," k[%d]:KEY_MENU is released\n",KEY_MENU);
				CYTTSP_PRINTK(1,"BACK KEY coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				input_report_key(g_ts->keyarray_input, KEY_MENU, 0);
			}
			#if 0
			if (g_ts->key_msg[0].key_coord.x == 0 && g_ts->key_msg[0].key_coord.y == 0) {
				g_ts->key_msg[0].key_state = KEY_RELEASE;
				CYTTSP_PRINTK(1,"CAP KEYs coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				CYTTSP_PRINTK(1," k[%d]:KEY_HOME is released\n",KEY_HOME);
				input_report_key(g_ts->keyarray_input, KEY_HOME, 0);
				CYTTSP_PRINTK(1," k[%d]:KEY_SEARCH is released\n",KEY_SEARCH);
				input_report_key(g_ts->keyarray_input, KEY_SEARCH, 0);
				CYTTSP_PRINTK(1," k[%d]:KEY_BACK is released\n",KEY_BACK);
				input_report_key(g_ts->keyarray_input, KEY_BACK, 0);
				CYTTSP_PRINTK(1," k[%d]:KEY_MENU is released\n",KEY_MENU);
				input_report_key(g_ts->keyarray_input, KEY_MENU, 0);
			}
			#endif
			if (id < 0) {
				g_ts->key_msg[0].key_state = KEY_RELEASE;
				CYTTSP_PRINTK(1,"CAP KEYs coord x:%d coord y:%d\n",
					g_ts->key_msg[0].key_coord.x, g_ts->key_msg[0].key_coord.y);
				CYTTSP_PRINTK(1," k[%d]:KEY_HOME is released\n",KEY_HOME);
				input_report_key(g_ts->keyarray_input, KEY_HOME, 0);
				CYTTSP_PRINTK(1," k[%d]:KEY_SEARCH is released\n",KEY_SEARCH);
				input_report_key(g_ts->keyarray_input, KEY_SEARCH, 0);
				CYTTSP_PRINTK(1," k[%d]:KEY_BACK is released\n",KEY_BACK);
				input_report_key(g_ts->keyarray_input, KEY_BACK, 0);
				CYTTSP_PRINTK(1," k[%d]:KEY_MENU is released\n",KEY_MENU);
				input_report_key(g_ts->keyarray_input, KEY_MENU, 0);
			}
		}
	}
	input_sync(g_ts->keyarray_input);
	return;
}

static void cyttsp_report_mt_protocol(int id, struct cyttsp *g_ts)
{
	int i;

	for(i = 0; i < CY_NUM_MT_TCH_ID; i++) {
		
			
        #if 0
		if (g_ts->msg[i].coord.x == 0 && g_ts->msg[i].coord.y == 0) {
			g_ts->msg[i].state = TS_RELEASE;
			input_report_key(g_ts->input, BTN_TOUCH, 0);
			input_report_abs(g_ts->input, ABS_MT_POSITION_X, g_ts->msg[i].coord.x);
			input_report_abs(g_ts->input, ABS_MT_POSITION_Y, g_ts->msg[i].coord.y);
			input_report_abs(g_ts->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(g_ts->input, ABS_MT_WIDTH_MAJOR, g_ts->msg[i].w);
			input_report_abs(g_ts->input, ABS_MT_PRESSURE, 0);
			
			
		}
		else {
			if (g_ts->msg[i].coord.y <= 830) {
				g_ts->msg[i].state = TS_PRESS;
				input_report_key(g_ts->input, BTN_TOUCH, 1);
				input_report_abs(g_ts->input, ABS_MT_POSITION_X, g_ts->msg[i].coord.x);
				input_report_abs(g_ts->input, ABS_MT_POSITION_Y, g_ts->msg[i].coord.y);
				input_report_abs(g_ts->input, ABS_MT_TOUCH_MAJOR, g_ts->msg[i].coord.z);
				input_report_abs(g_ts->input, ABS_MT_WIDTH_MAJOR, g_ts->msg[i].w);
				input_report_abs(g_ts->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(g_ts->input, ABS_MT_PRESSURE, g_ts->msg[i].coord.z);
			}
			else {
				g_ts->msg[i].state =TS_RELEASE;
				input_report_key(g_ts->input, BTN_TOUCH, 0);
				input_report_abs(g_ts->input, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(g_ts->input, ABS_MT_PRESSURE, 0);
				g_ts->msg[i].coord.x = 0;
				g_ts->msg[i].coord.y = 0;
				g_ts->msg[i].coord.z = 0;
				g_ts->msg[i].coord.tch_id = 0;
				g_ts->msg[i].w = 0;
			}
		}
		#endif
		if (id < 0) {
			g_ts->msg[i].state =TS_RELEASE;
			input_report_key(g_ts->input, BTN_TOUCH, 0);
			input_report_abs(g_ts->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(g_ts->input, ABS_MT_PRESSURE, 0);
		}
		else if (id >= 0 && id < CY_NUM_MT_TCH_ID) {
			if (g_ts->msg[i].coord.y <= 830) {
				g_ts->msg[i].state = TS_PRESS;
				input_report_key(g_ts->input, BTN_TOUCH, 1);
				input_report_abs(g_ts->input, ABS_MT_POSITION_X, g_ts->msg[i].coord.x);
				input_report_abs(g_ts->input, ABS_MT_POSITION_Y, g_ts->msg[i].coord.y);
				input_report_abs(g_ts->input, ABS_MT_TOUCH_MAJOR, g_ts->msg[i].coord.z);
				input_report_abs(g_ts->input, ABS_MT_WIDTH_MAJOR, g_ts->msg[i].w);
				input_report_abs(g_ts->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(g_ts->input, ABS_MT_PRESSURE, g_ts->msg[i].coord.z);
			}
		}
		
		if (g_ts->msg[i].state == TS_PRESS) {
			CYTTSP_PRINTK(1, "touch press down!!!\n");
			CYTTSP_PRINTK(1,"num_0f_tch[%d] coord x:%d coord y:%d z:%d w:%d tch_id:0x%x\n",
					i, g_ts->msg[i].coord.x, g_ts->msg[i].coord.y, g_ts->msg[i].coord.z,
						g_ts->msg[i].w,  g_ts->msg[i].coord.tch_id);
		}
		else if (g_ts->msg[i].state == TS_RELEASE) {
			CYTTSP_PRINTK(1, "touch release!!!\n");
			CYTTSP_PRINTK(1,"num_0f_tch[%d] coord x:%d coord y:%d z:%d w:%d tch_id:0x%x\n",
					i, g_ts->msg[i].coord.x, g_ts->msg[i].coord.y, g_ts->msg[i].coord.z,
						g_ts->msg[i].w,  g_ts->msg[i].coord.tch_id);
		}
		input_mt_sync(g_ts->input);
		
		
			
	}
	input_sync(g_ts->input);
}

static void cyttsp_report_coord(u8 id, struct cyttsp *g_ts)
{

	int i;

	for (i = 0; i < CY_MAX_REPORT_ID; i++) {
		if (id < 0) { 
			
			g_ts->msg[i].coord.x = 0;
			g_ts->msg[i].coord.y = 0;
			g_ts->msg[i].coord.z = 0;
			g_ts->msg[i].coord.tch_id = 0;
			g_ts->msg[i].w = 0x0;
			
			g_ts->key_msg[0].key_coord.x = 0x0;
			g_ts->key_msg[0].key_coord.y = 0x0;
		}
		else {
			if (id == 0) {
				g_ts->key_msg[i].key_coord.x = (int)g_ts->op_mode_data.x[i];
				g_ts->key_msg[i].key_coord.y = (int)g_ts->op_mode_data.y[i];
				g_ts->msg[i].coord.x = (int)g_ts->op_mode_data.x[i];
				g_ts->msg[i].coord.y = (int)g_ts->op_mode_data.y[i];
				g_ts->msg[i].coord.z = (int)g_ts->op_mode_data.z[i];
				g_ts->msg[i].coord.tch_id = (int)g_ts->op_mode_data.tch_id[i];
			}
			else {
				g_ts->msg[i].coord.x = (int)g_ts->op_mode_data.x[i];
				g_ts->msg[i].coord.y = (int)g_ts->op_mode_data.y[i];
				g_ts->msg[i].coord.z = (int)g_ts->op_mode_data.z[i];
				g_ts->msg[i].coord.tch_id = (int)g_ts->op_mode_data.tch_id[i];
				g_ts->key_msg[0].key_coord.x = 0;
				g_ts->key_msg[0].key_coord.y = 0;
			}
			if (i <= id && id != 0x0F) {
					g_ts->msg[i].w = CY_LARGE_TOOL_WIDTH;
				}
			else {
					g_ts->msg[i].w = 0x0;
				}
		}
    }
	cyttsp_report_mt_protocol(id, g_ts);
	cyttsp_report_capkey(id, g_ts);
}


static void cyttsp_irqWorkHandler(struct work_struct *work)
{

	struct i2c_client           *client = g_ts->client;
    uint8_t value[NUM_OF_OP_MODE_SIZE];
	
    
	uint8_t num_of_tch = 0;
	int id = 0;
	int retval = 0;
	int i;

	CYTTSP_PRINTK(2,"CYTTSP irqWorkHandler\n");
	
	#if 0
	if(!rt_task(current))
    {
 		if(sched_setscheduler_nocheck(current, SCHED_FIFO, &s)!=0)
		{
			CYTTSP_PRINTK(0,"fail to set rt pri...\n" );
		}
		else
		{
			CYTTSP_PRINTK(1,"set rt pri...\n" );
		}
    }
	#endif
	mutex_lock(&g_ts->mutex);
	
    for (i = 0; i < CY_MAX_REPORT_ID; i++)
	{
		g_ts->msg[i].state = TS_RELEASE;
		g_ts->op_mode_data.x[i] = 0x0;
		g_ts->op_mode_data.y[i] = 0x0;
		g_ts->op_mode_data.z[i] = 0x0;
		g_ts->op_mode_data.tch_id[i] = 0x0;
	}

    retval = ts_read_i2c(client, CY_OP_MODE, &value[0], NUM_OF_OP_MODE_SIZE);
    if (retval) {
		CYTTSP_PRINTK(0, "failed to read operation mode!!!\n");
    }
	
	CYTTSP_PRINTK(1, "hst_mode:0x%x tt_mode:0x%x tt_stat:0x%x\n",value[0],value[1],value[2]);

	
	
	
	
			

	
	num_of_tch = value[2] & 0x0F;
	
	CYTTSP_PRINTK(2," num_of_tch:0x%x\n",num_of_tch);

	
	if (num_of_tch == 0x01) {
		id = ((int)num_of_tch) - 1;
		CYTTSP_PRINTK(2," id:%d\n",id);
	}
	else if (num_of_tch == 0x02 ) {
		id = ((int)num_of_tch) - 1;
		CYTTSP_PRINTK(2," id:%d\n",id);
	}
	else if (num_of_tch == 0x03) {
		id = ((int)num_of_tch) - 1;
		CYTTSP_PRINTK(2," id:%d\n",id);
	}
	else if (num_of_tch == 0x04) {
		id = ((int)num_of_tch) - 1;
		CYTTSP_PRINTK(2," id:%d\n",id);
	}
	else if (num_of_tch == 0x00) {
		id = (int)num_of_tch -1;
		CYTTSP_PRINTK(2," id:%d\n",id);
	}

	
	for (i = 0; i < CY_MAX_REPORT_ID; i++) {
		#if 0
		if (id >= 0 && i <= id) {
			g_ts->msg[i].state = TS_PRESS;
		}
		#endif
		
		if (id < 0){
			g_ts->msg[i].state = TS_RELEASE;
			g_ts->key_msg[0].key_state = KEY_RELEASE;
			g_ts->op_mode_data.x[i] = 0x0;
			g_ts->op_mode_data.y[i] = 0x0;
			g_ts->op_mode_data.z[i] = 0x0;
			g_ts->op_mode_data.tch_id[i] = 0x0;
		}
	}

	
	g_ts->op_mode_data.x[0] = value[3] << 8 | value[4];
	g_ts->op_mode_data.y[0] = value[5] << 8 | value[6];
	g_ts->op_mode_data.z[0] = value[7];
	g_ts->op_mode_data.tch_id[0] = (value[8] & 0xF0) >> 4;
	CYTTSP_PRINTK(2," value[3]:0x%x\n",value[3]);
	CYTTSP_PRINTK(2," value[4]:0x%x\n",value[4]);
	CYTTSP_PRINTK(2," value[5]:0x%x\n",value[5]);
	CYTTSP_PRINTK(2," value[6]:0x%x\n",value[6]);
	CYTTSP_PRINTK(2," value[7]:0x%x\n",value[7]);
	CYTTSP_PRINTK(2," op_mode_data.x[0]:0x%x\n",g_ts->op_mode_data.x[0]);
	CYTTSP_PRINTK(2," op_mode_data.y[0]:0x%x\n",g_ts->op_mode_data.y[0]);
	CYTTSP_PRINTK(2," g_ts->op_mode_data.z[0]:0x%x\n",g_ts->op_mode_data.z[0]);
	CYTTSP_PRINTK(2," op_mode_data.tch_id[0]:0x%x\n",g_ts->op_mode_data.tch_id[0]);
	
	
	

	g_ts->op_mode_data.tch_id[1]= (value[8] & 0x0F);
	g_ts->op_mode_data.x[1] = value[9] << 8 | value[10];
	g_ts->op_mode_data.y[1] = value[11] << 8 | value[12];
	g_ts->op_mode_data.z[1] = value[13];
	
	
	
	g_ts->op_mode_data.x[2] = value[16] << 8 | value[17];
	g_ts->op_mode_data.y[2] = value[18] << 8 | value[19];
	g_ts->op_mode_data.z[2] = value[20];
	g_ts->op_mode_data.tch_id[2]  = (value[21] & 0xF0) >> 4;
	
	
	
	g_ts->op_mode_data.tch_id[3]  = (value[21] & 0x0F);
	g_ts->op_mode_data.x[3] = value[22] << 8 | value[23];
	g_ts->op_mode_data.y[3] = value[24] << 8 | value[25];
	g_ts->op_mode_data.z[3] = value[26];
	
	
	
	cyttsp_report_coord(id, g_ts);

   	enable_irq(g_ts->irq);
	mutex_unlock(&g_ts->mutex);
	return;
}



static irqreturn_t cyttsp_irq(int irq, void *dev_id)
{
	
	struct cyttsp *g_ts = dev_id;

	
	CYTTSP_PRINTK(1, "%s: Got IRQ\n", CY_I2C_NAME);

	
	disable_irq_nosync(g_ts->irq);

	
	
	queue_delayed_work(g_ts->cyttsp_ts_wq, &g_ts->ts_work, 0);
	return IRQ_HANDLED;
}

#if 0

static int cyttsp_read_sys_info(struct cyttsp *ts)
{
	int rc = 0;
	u8 host_reg;
	u8 value[NUM_OF_SYS_INFO_SIZE];

	CYTTSP_PRINTK(1,"Power up \n");

	mdelay(15);

	
	if (!(rc < 0)) {
		CYTTSP_PRINTK(1,"switch to sysinfo mode \n");
		host_reg = CY_SYSINFO_MODE;
		rc = ts_read_i2c(ts->client, CY_REG_BASE, &host_reg, sizeof(host_reg));
		if (rc) {
			CYTTSP_PRINTK(0,"fail to read sysinfo mode I \n");
			return rc;
		}
		
		mdelay(15);
		rc = ts_read_i2c(ts->client, CY_REG_BASE, &value[0], 6);
		if (rc) {
			CYTTSP_PRINTK(0,"fail to read sysinfo mode I \n");
			return rc;
		}
		ts->sysinfo_data.hst_mode = value[0];
		ts->sysinfo_data.mfg_cmd = value[1];
		ts->sysinfo_data.mfg_stat = value[2];
		ts->sysinfo_data.cid[0] = value[3];
		ts->sysinfo_data.cid[1] = value[4];
		ts->sysinfo_data.cid[2] = value[5];

		rc = ts_read_i2c(ts->client, CY_REG_BL_VER, &value[16], 8);
		if (rc) {
			CYTTSP_PRINTK(0,"fail to read sysinfo mode II \n");
			return rc;
		}
		ts->sysinfo_data.bl_ver = value[16] | value[15] << 8;
		ts->sysinfo_data.tts_ver = value[18] | value[17] << 8;
		ts->sysinfo_data.app_id = value[20] | value[19] << 8;
		ts->sysinfo_data.app_ver = value[22] | value[21] << 8;

		
		rc = ts_read_i2c(ts->client, CY_REG_ACT_INTRVL, &value[30], 3);
		if (rc) {
			CYTTSP_PRINTK(0,"fail to write act intrval to sysinfo mode III \n");
			return rc;
		}
		ts->sysinfo_data.act_intrvl = value[29];
		ts->sysinfo_data.tch_tmout = value[30];
		ts->sysinfo_data.lp_intrvl = value[31];
		mdelay(CY_DLY_SYSINFO);

		
		CYTTSP_PRINTK(2,"SYS_INFO hst mode 0x%x \n", ts->sysinfo_data.hst_mode);
		CYTTSP_PRINTK(2,"SYS_INFO mfg_cmd 0x%x \n", ts->sysinfo_data.mfg_cmd);
		CYTTSP_PRINTK(2,"SYS_INFO mfg_stat 0x%x \n", ts->sysinfo_data.mfg_stat);
		CYTTSP_PRINTK(2,"SYS_INFO cid_0 0x%x \n", ts->sysinfo_data.cid[0]);
		CYTTSP_PRINTK(2,"SYS_INFO cid_1 0x%x \n", ts->sysinfo_data.cid[1]);
		CYTTSP_PRINTK(2,"SYS_INFO cid_2 0x%x \n", ts->sysinfo_data.cid[2]);
		CYTTSP_PRINTK(2,"SYS_INFO bl_ver 0x%x \n", ts->sysinfo_data.bl_ver);
		CYTTSP_PRINTK(2,"SYS_INFO tts_ver 0x%x \n", ts->sysinfo_data.tts_ver);
		CYTTSP_PRINTK(2,"SYS_INFO app_id 0x%x \n", ts->sysinfo_data.app_id);
		CYTTSP_PRINTK(2,"SYS_INFO app_ver 0x%x \n", ts->sysinfo_data.app_ver);
		CYTTSP_PRINTK(2,"SYS_INFO act_intrvl 0x%x \n", ts->sysinfo_data.act_intrvl);
		CYTTSP_PRINTK(2,"SYS_INFO tch_tmout 0x%x \n", ts->sysinfo_data.tch_tmout);
		CYTTSP_PRINTK(2,"SYS_INFO lp_intrvl 0x%x \n", ts->sysinfo_data.lp_intrvl);

		
		CYTTSP_PRINTK(1,"switch back to operational mode \n");
		if (!(rc < 0)) {
			host_reg = CY_OP_MODE;
			rc = ts_write_i2c(ts->client, CY_REG_BASE, &host_reg, sizeof(host_reg));
			
			mdelay(100);
		}
	}


	if (!(rc < 0))
		pdata->power_state = CY_ACTIVE_STATE;
	else
		pdata->power_state = CY_IDLE_STATE;

	CYTTSP_PRINTK(1, "rc=%d Power state is %s\n", rc,
		ts->platform_data->power_state == CY_ACTIVE_STATE ?
		"ACTIVE" : "IDLE");

	return rc;
}
#endif

static int cyttsp_power_on_device(struct cyttsp *g_ts, int on)
{
    int     rc = 0;

	if(on)
	{
		
		g_ts->avdd_1_regulator = regulator_get(NULL, "gp2");
		if (IS_ERR(g_ts->avdd_1_regulator)) {
			pr_err("could not get avdd_L12_touch, rc = %ld\n",
				PTR_ERR(g_ts->avdd_1_regulator));
			regulator_put(g_ts->avdd_1_regulator);
			return -ENODEV;
		}

		
		g_ts->avdd_2_regulator = regulator_get(NULL, "emmc");
		if (IS_ERR(g_ts->avdd_2_regulator)) {
			rc = PTR_ERR(g_ts->avdd_2_regulator);
			pr_err("%s:regulator get avdd L10 voltage failed rc=%d\n",
							__func__, rc);
			
				
			regulator_put(g_ts->avdd_2_regulator);
			return -ENODEV;
		}

		
		g_ts->vdd_regulator = regulator_get(NULL, "msme1");
		if (IS_ERR(g_ts->vdd_regulator)) {
			pr_err("could not get vdd_touch, rc = %ld\n",
				PTR_ERR(g_ts->vdd_regulator));
			regulator_put(g_ts->vdd_regulator);
			return -ENODEV;
		}

		rc = regulator_set_voltage(g_ts->avdd_1_regulator, 2850000, 2850000);
		if (rc) {
			CYTTSP_PRINTK( 1, "TEST_INFO_LEVEL:" "set_voltage avdd_L12_touch failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_set_voltage(g_ts->avdd_2_regulator, 2850000, 2850000);
		if (rc) {
			CYTTSP_PRINTK( 1, "TEST_INFO_LEVEL:" "set_voltage avdd_L10_touch failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_set_voltage(g_ts->vdd_regulator, 1800000, 1800000);
		if (rc) {
			printk("set_voltage vdd_touch failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_enable(g_ts->avdd_1_regulator);
		if (rc) {
			CYTTSP_PRINTK( 1, "TEST_INFO_LEVEL:""enable vdd_L12_touch failed, rc=%d\n", rc);
			regulator_disable(g_ts->avdd_1_regulator);
			return -ENODEV;
		}

		mdelay(5);

		rc = regulator_enable(g_ts->avdd_2_regulator);
		if (rc) {
			CYTTSP_PRINTK( 1, "TEST_INFO_LEVEL:""enable avdd_L110_touch failed, rc=%d\n", rc);
			regulator_disable(g_ts->avdd_2_regulator);
			return -ENODEV;
		}
		mdelay(5);

		rc = regulator_enable(g_ts->vdd_regulator);
		if (rc) {
			CYTTSP_PRINTK( 1, "TEST_INFO_LEVEL:""enable  vdd_s3_touch failed, rc=%d\n", rc);
			regulator_disable(g_ts->vdd_regulator);
			return -ENODEV;
		}
		mdelay(5);

		msleep(ATMEL_POR_DELAY);
		printk("%s %d avdd_touch & vdd_touch have been turn on\n",__func__,__LINE__);
#if 0
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_on(g_ts->pmlog_device);
		if (rc)
			printk("pmlog_device_on fail rc = %d\n", rc);
#endif	
#endif
	}
	else
	{
		printk("%s %d does not support turn onoff vdd&avdd\n",__func__,__LINE__);
#if 0
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_off(g_ts->pmlog_device);
		if (rc)
			printk("pmlog_device_off fail rc = %d\n", rc);
#endif 
#endif
	}

    return rc;
}


#if 0
static int cyttsp_power_on_device(int on)
{
    int rc = 0;
	

	CYTTSP_PRINTK(1,"power on device!!!! \n");

	
	g_ts->vreg_l12 = vreg_get(NULL, "gp2");
	if (IS_ERR(g_ts->vreg_l12)) {
		CYTTSP_PRINTK(0,"vreg_get(%s) failed (%ld)\n",
			"vreg L12", PTR_ERR(g_ts->vreg_l12));
		return PTR_ERR(g_ts->vreg_l12);
	}

	rc = vreg_set_level(g_ts->vreg_l12, 2800);
	if (rc) {
		CYTTSP_PRINTK(0,"vreg L12 set_level failed (%d)\n", rc);
		goto vreg_put_12;
	}
	CYTTSP_PRINTK(1, "success to set L12 to level 2.8v\n");

    
	g_ts->vreg_l10 = vreg_get(NULL, "emmc");
	if (IS_ERR(g_ts->vreg_l10)) {
		CYTTSP_PRINTK(0,"vreg_get(%s) failed (%ld)\n",
			"vreg L10", PTR_ERR(g_ts->vreg_l10));
		goto vreg_put_10;
	}

	rc = vreg_set_level(g_ts->vreg_l10, 2800);
	if (rc) {
		CYTTSP_PRINTK(0,"vreg L10 set_level failed (%d)\n", rc);
		goto vreg_put_10;
	}
    CYTTSP_PRINTK(1, "success to set L10 to level 2.8v\n");
	
	g_ts->vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(g_ts->vreg_s3)) {
		CYTTSP_PRINTK(0,"vreg_get(%s) failed (%ld)\n",
			"vreg S3", PTR_ERR(g_ts->vreg_s3));
		goto vreg_put_s3;
	}

	rc = vreg_set_level(g_ts->vreg_s3, 1800);
	if (rc) {
		CYTTSP_PRINTK(0,"vreg S3 set_level failed (%d)\n",
			rc);
		goto vreg_put_s3;
	}
	CYTTSP_PRINTK(1, "success to set s3 to level 1.8v\n");
	if (on) {
		rc = vreg_enable(g_ts->vreg_l12);
		if (rc) {
			CYTTSP_PRINTK(0,"vreg L12 enable failed (%d)\n", rc);
			return rc;
		}

		rc = vreg_enable(g_ts->vreg_l10);
		if (rc) {
			CYTTSP_PRINTK(0,"vreg L10 enable failed (%d)\n", rc);
			return rc;
		}

		rc = vreg_enable(g_ts->vreg_s3);
		if (rc) {
			CYTTSP_PRINTK(0,"vreg S3 enable failed (%d)\n", rc);
			return rc;
		}

	} else {
		rc = vreg_disable(g_ts->vreg_l12);
		if (rc){
			CYTTSP_PRINTK(0,"vreg L12 disable failed (%d)\n", rc);
			return rc;
        }

		rc = vreg_disable(g_ts->vreg_l10);
		if (rc) {
			CYTTSP_PRINTK(0,"vreg L10 disable failed (%d)\n", rc);
			return rc;
		}

		rc = vreg_disable(g_ts->vreg_s3);
		if (rc) {
			CYTTSP_PRINTK(0,"vreg S3 disable failed (%d)\n", rc);
			return rc;
		}

	}

	return 0;

vreg_put_s3:
	vreg_put(g_ts->vreg_s3);
vreg_put_10:
	vreg_put(g_ts->vreg_l10);
vreg_put_12:
	vreg_put(g_ts->vreg_l12);
	return rc;

#if 0
	for (i = 0; i < CY_MAX_REPORT_ID; i++) {
		cyttsp_vreg[i] = vreg_get(0, cyttsp_vreg_id[i]);

		if (on) {
			rc = vreg_set_level(cyttsp_vreg[i],
			cyttsp_vreg_mV[i]);
			if (rc < 0) {
				CYTTSP_PRINTK(0,"failed to set vreg %s to level %dmv"
					"with :(%d)\n", cyttsp_vreg_id[i], cyttsp_vreg_mV[i], rc);
				goto fail_vreg_set_level;
			}

			rc = vreg_enable(cyttsp_vreg[i]);
			if(rc < 0) {
				CYTTSP_PRINTK(0, "failed to enable %s to %dmv\n",
					cyttsp_vreg_id[i], cyttsp_vreg_mV[i]);
				goto fail_vreg_enable;
			}

			CYTTSP_PRINTK(1, "success to set vdd and avdd %s to level %dmv\n",
			cyttsp_vreg_id[i], cyttsp_vreg_mV[i]);
			msleep(1);
		}
		else {
			if(vreg_disable(cyttsp_vreg[i]) < 0) {
				CYTTSP_PRINTK(0, "failed to disable %s\n",cyttsp_vreg_id[i]);
				rc = -1;
				goto fail_vreg_enable;
			}
		}
	}

	return rc;

fail_vreg_enable:
	if (on) {
		for (; i > 0; i--)
			vreg_disable(cyttsp_vreg[i - 1]);
	} else {
		for (; i > 0; i--)
			vreg_enable(cyttsp_vreg[i - 1]);
	}

fail_vreg_set_level:
	for (; i > 0; i--) {
			vreg_put(cyttsp_vreg[i - 1]);
	}
	return rc;
#endif
}
#endif


static int cyttsp_config_gpio(struct cyttsp *ts)
{
	int rc = 0;

	CYTTSP_PRINTK(1,"configure HW reset!!!! \n");

    
    gpio_set_value(ts->gpio_num[RST_GPIO], 0);
    CYTTSP_PRINTK(1,"set hw reset low, delay 10 ms\n");
    msleep(10);
	
    gpio_set_value(ts->gpio_num[RST_GPIO], 1);
    CYTTSP_PRINTK(1,"set hw reset high, delay 10 ms\n");
	msleep(10);

    return rc;
}


static int cyttsp_release_gpio(struct cyttsp *g_ts)
{
    int i;

	CYTTSP_PRINTK(1, "releasing gpio!!! \n");
	for (i = 0; i < CY_MAX_NUM_GPIO; i++)
			gpio_free(g_ts->gpio_num[i]);

	return 0;
}


static int cyttsp_setup_gpio(struct cyttsp *ts)
{
	int i;
	int rc = 0;

	CYTTSP_PRINTK(1,"Set up GPIO!!!! \n");

	for(i = 0; i < CY_MAX_NUM_GPIO; i++)
	{
		if (i == IRQ_GPIO) {
			
			rc = gpio_tlmm_config( GPIO_CFG(ts->gpio_num[i], 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);
			if( rc )
			{
				CYTTSP_PRINTK(0,"failed to configure gpio_num[%d]:%d (rc=%d)\n",
					i, ts->gpio_num[i], rc);
				goto error_gpio_config;
			}

			rc = gpio_request(ts->gpio_num[i], gpio_name[i]);
			if ( rc )
			{
				CYTTSP_PRINTK(0,"failed to request gpio_num[%d]:%d (rc=%d)\n",
					i, ts->gpio_num[i], rc);
				goto error_gpio_config;
			}

			rc = gpio_direction_input(ts->gpio_num[i]);
			if ( rc )
			{
				CYTTSP_PRINTK(0,"failed to set gpio_num[%d]:%d direction (rc=%d)\n",
					i, ts->gpio_num[i], rc);
				goto error_gpio_config;
			}
		}
		else {
			
			rc = gpio_tlmm_config( GPIO_CFG(ts->gpio_num[i], 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
				GPIO_CFG_ENABLE );
			if( rc )
			{
				CYTTSP_PRINTK(0,"failed to configure gpio_num[%d]:%d (rc=%d)\n",
					i, ts->gpio_num[i], rc);
				goto error_gpio_config;
			}

			rc = gpio_request(ts->gpio_num[i], gpio_name[i]);
			if ( rc )
			{
				CYTTSP_PRINTK(0,"failed to request gpio_num[%d]:%d (rc=%d)\n",
					i, ts->gpio_num[i], rc);
				goto error_gpio_config;
			}

			rc = gpio_direction_output(ts->gpio_num[i], 0);
			if ( rc )
			{
				CYTTSP_PRINTK(0,"failed to set gpio_num[%d]:%d direction (rc=%d)\n",
					i, ts->gpio_num[i], rc);
				goto error_gpio_config;
			}
			udelay(1);
		} 

		
		CYTTSP_PRINTK(1,"success to configure gpio_num[%d]:%d gpio_name:%s\n",
			i , ts->gpio_num[i], gpio_name[i]);
	}

	return 0;

error_gpio_config:
	cyttsp_release_gpio(g_ts);
	return rc;
}

static int cyttsp_keyarray_open(struct input_dev *dev)
{
	int rc = 0;

	mutex_lock(&g_ts->mutex);
	if(g_ts->key_open_count == 0) {
        g_ts->key_open_count++; 
        CYTTSP_PRINTK(1, "key open_count : %d\n", g_ts->key_open_count);
    }
    mutex_unlock(&g_ts->mutex);

    return rc;
}

static void cyttsp_keyarray_close(struct input_dev *dev)
{
	mutex_lock(&g_ts->mutex);
    if(g_ts->key_open_count > 0)
    {
        g_ts->key_open_count--;
        CYTTSP_PRINTK(1, "keyarray input device still opened %d times\n",g_ts->key_open_count );
    }
    mutex_unlock(&g_ts->mutex);
}

static int keyarray_register_input( struct input_dev **input,
                              struct i2c_client *client )
{
	int rc = 0;
	struct input_dev *input_dev;

	input_dev = input_allocate_device();
	if ( !input_dev )
	{
		CYTTSP_PRINTK(0, "failed to input allocate device\n");
		rc = -ENOMEM;
		return rc;
	}

	input_dev->name = CYPRESS_KEY_NAME;
	input_dev->phys = "cypress_keyarray/event0";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &client->dev;
	input_dev->open = cyttsp_keyarray_open;
	input_dev->close = cyttsp_keyarray_close;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);

	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_SEARCH, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);

	rc = input_register_device(input_dev);
	if (rc)
	{
		CYTTSP_PRINTK(0,"%s: Failed to register input device\n", CY_I2C_NAME);
		input_free_device(input_dev);
	}
	else {
		*input = input_dev;
	}

	return rc;
}


static int cyttsp_open(struct input_dev *dev)
{
    int rc = 0;

    mutex_lock(&g_ts->mutex);
    if(g_ts->open_count == 0) {
        g_ts->open_count++; 
        CYTTSP_PRINTK(1, "ts open_count : %d\n", g_ts->open_count);
    }
    mutex_unlock(&g_ts->mutex);
    return rc;
}

static void cyttsp_close(struct input_dev *dev)
{
    mutex_lock(&g_ts->mutex);
    if(g_ts->open_count > 0)
    {
        g_ts->open_count--;
        CYTTSP_PRINTK(1, "ts input device still opened %d times\n",g_ts->open_count );
    }
    mutex_unlock(&g_ts->mutex);
}


static int ts_detect_cypress(struct cyttsp *ts)
{
	int rc = 0;
	uint8_t value[3];
	u8 host_reg;

	mdelay(50);
	#if 0
	CYTTSP_PRINTK(1, "enter soft reset \n");
	host_reg = CY_SOFT_RESET_MODE;
	rc = ts_write_i2c(ts->client, CY_REG_BASE, &host_reg, sizeof(host_reg));
	if (rc)
	{
		CYTTSP_PRINTK(0, "failed to write cyttsp soft reset addr 0x%x : 0x%x (rc=%d)\n", CY_REG_BASE, host_reg, rc);
		return rc;
	}
	CYTTSP_PRINTK(1, "cyttsp soft reset addr 0x%x:0x%x\n", CY_REG_BASE, host_reg);
	mdelay(10);
	#endif
	CYTTSP_PRINTK(1, "switch back to operational mode \n");
	host_reg = CY_OP_MODE;
	rc = ts_write_i2c(ts->client, CY_REG_BASE, &host_reg, sizeof(host_reg));
	if (rc)
	{
		CYTTSP_PRINTK(0, "failed to write cyttsp op mode addr 0x%x : 0x%x (rc=%d)\n", CY_REG_BASE, host_reg, rc);
		return rc;
	}
	CYTTSP_PRINTK(1, "cyttsp op mode 0x%x:0x%x\n", CY_REG_BASE, host_reg);
	mdelay(100);

	
	rc = ts_read_i2c(ts->client, CY_REG_BASE, &value[0], 3);
	if (rc < 0) {
		CYTTSP_PRINTK(0,"Failed to read op mode rc=%d\n", rc);
		return rc;
	}
	CYTTSP_PRINTK(1,"read hst_mode[0x%x] value:0x%x\n", CY_REG_BASE, value[0]);
	CYTTSP_PRINTK(1,"read tt_mode[0x%x] value:0x%x\n", CY_REG_BASE+1, value[1]);
	CYTTSP_PRINTK(1,"read tt_stat[0x%x] value:0x%x\n", CY_REG_BASE+2, value[2]);

	
	rc = ts_read_i2c(ts->client, CY_REG_FW_VER, &ts->fw_ver, 1);
	if (rc)
	{
		CYTTSP_PRINTK(0, "failed to read cyttsp fw version!!! \n");
		return rc;
	}
	CYTTSP_PRINTK(1,"cyttsp fw version addr:0x%x value:0x%x\n", CY_REG_FW_VER, ts->fw_ver);

#if 0
	
	
	if (!(ts->fw_ver == 0x9)) {
		
		
		
	}

#endif
	return 0;
}



static int cyttsp_register_input( struct input_dev **input,
                                    struct cyttsp_platform_data *pdata,
                                    struct i2c_client *client )
{
	struct input_dev *input_dev;
	int rc = 0;

    CYTTSP_PRINTK(1,"enter register input device for cyttsp!!!\n");
	
    input_dev = input_allocate_device();
    if (!input_dev) {
		CYTTSP_PRINTK(0,"failed to input allocate device\n");
        rc = -ENOMEM;
		return rc;
    }

    input_dev->name = CYPRESS_TS_NAME;
    input_dev->phys = "cypress_touchscreen/input0";
    input_dev->id.bustype = BUS_I2C;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0002;
    input_dev->id.version = 0x0100;
    input_dev->dev.parent = &client->dev;
    input_dev->open = cyttsp_open;
    input_dev->close = cyttsp_close;

    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    
    
    input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);

	if(g_ts->cyttsp_x_max== 0)
	    g_ts->cyttsp_x_max = CYTTSP_X_MAX;
	if(g_ts->cyttsp_y_max == 0)
		g_ts->cyttsp_y_max = CYTTSP_Y_MAX;
	CYTTSP_PRINTK(1," cyttsp_x_max : %d\n",(int)g_ts->cyttsp_x_max);
    CYTTSP_PRINTK(1," cyttsp_y_max : %d\n",(int)g_ts->cyttsp_y_max);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,0,DISPLAY_PANEL_X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,0,DISPLAY_PANEL_Y_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, CY_MAXZ, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, CY_LARGE_TOOL_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	CYTTSP_PRINTK(1,"%s: Register input device\n", CY_I2C_NAME);
    rc = input_register_device(input_dev);
    if (rc)
    {
        CYTTSP_PRINTK(0,"%s: Failed to register input device\n", CY_I2C_NAME);
		input_free_device(input_dev);
    }else {
        *input = input_dev;
    }

    return rc;
}


static int cyttsp_calibration(int on)
{

	int rc = 0;
	struct i2c_client *client = g_ts->client;
	uint8_t writeValue = 0,KReadValue = 0;
	uint8_t Kcheck = 0;

	if (on) {  
		
		disable_irq(g_ts->irq);
		CYTTSP_PRINTK(0, "disable irq %d\n",g_ts->irq);

		
		writeValue = 0x90;
		rc = ts_write_i2c(client, CY_REG_BASE, &writeValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "failed to enter system info mode!!!\n");
			CYTTSP_PRINTK(0, "failed to write to HST_MODE[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE, writeValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}

		msleep(1000);

		
		writeValue = 0x10;
		rc = ts_write_i2c(client, CY_REG_BASE, &writeValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "Calibration : CMD Error!!!\n");
			CYTTSP_PRINTK(0, "failed to write to HST_MODE[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE, writeValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}

		writeValue = 0x00;
		rc = ts_write_i2c(client, CY_REG_BASE+1, &writeValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "Calibration : CMD Error!!!\n");
			CYTTSP_PRINTK(0, "failed to write to MFG_CMD[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE+1, writeValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}

		writeValue = 0x20;
		rc = ts_write_i2c(client, CY_REG_BASE+2, &writeValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "Calibration : CMD Error!!!\n");
			CYTTSP_PRINTK(0, "failed to write to MFG_STAT[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE+2, writeValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}

		writeValue = 0x00;
		rc = ts_write_i2c(client, CY_REG_BASE+3, &writeValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "Calibration : CMD Error!!!\n");
			CYTTSP_PRINTK(0, "failed to write to CID_0[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE+3, writeValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}

		writeValue = 0x00;
		rc = ts_write_i2c(client, CY_REG_BASE+4, &writeValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "Calibration : CMD Error!!!\n");
			CYTTSP_PRINTK(0, "failed to write to CID_1[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE+4, writeValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}

		msleep(4000);

		
		rc = ts_read_i2c(client, CY_REG_BASE+1, &KReadValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "failed to read to MFG_CMD[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE+1, KReadValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}

		msleep(1000);

		CYTTSP_PRINTK(0, "read Calibration Value is 0x%x\n", KReadValue);

		Kcheck = KReadValue & 0x02;
		CYTTSP_PRINTK(0, "check Calibration value : 0x%x\n", Kcheck);
		if(Kcheck) {
			CYTTSP_PRINTK(0, "Calibration Success!!!\n");
			g_ts->chk_k_flag = 1;
			mutex_unlock(&g_ts->mutex);
			return rc;
		}
		else {
			CYTTSP_PRINTK(0, "Calibration Failed!!!\n");
			CYTTSP_PRINTK(0, "Please Try Again!!!\n");
			g_ts->chk_k_flag = -1;
		}
		msleep(1000);
	}
	else { 

		msleep(1000);

		
		CYTTSP_PRINTK(0, "Back to operation mode!!!\n");
		writeValue = 0x00;
		rc = ts_write_i2c(client, CY_REG_BASE, &writeValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "failed to write to HST_MODE[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE, writeValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}
		msleep(1000);
		

		
		rc = ts_read_i2c(client, CY_REG_BASE, &KReadValue, 1);
		if (rc) {
			CYTTSP_PRINTK(0, "failed to read to HST_MODE[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE, KReadValue, rc);
			mutex_unlock(&g_ts->mutex);
			return rc;
		}

		CYTTSP_PRINTK(0, "chekc if touch mode is 0x%x!!!\n", KReadValue);

		
		CYTTSP_PRINTK(0, "enter HW RESET!!!\n");
		cyttsp_config_gpio(g_ts);

		
		msleep(1000);
		
		enable_irq(g_ts->irq);
		CYTTSP_PRINTK(0, "enable irq!!!\n");
		CYTTSP_PRINTK(0, "exit k mode goto op mode!!!\n");
	}
	return rc;
}

static ssize_t ts_misc_write( struct file *fp,
                              const char __user *buffer,
                              size_t count,
                              loff_t *ppos )
{
	char echostr[ECHOSTR_SIZE];

	if ( count > ECHOSTR_SIZE )
    {
        CYTTSP_PRINTK(0, "invalid count %d\n", count);
        return -EINVAL;
    }

	if ( copy_from_user(echostr,buffer,count) )
	{
		CYTTSP_PRINTK(0, "failed to copy Echo String from user mode : %s\n", echostr);
		return -EINVAL;
	}
	mutex_lock(&g_ts->mutex);
	echostr[count-1]='\0';
	if ( strcmp(echostr,"start k" ) == 0 )
	{
		CYTTSP_PRINTK(0, "User Input Echo String : %s\n",echostr);
		CYTTSP_PRINTK(0, "Don't touch panel until calibration success!!!\n");
		cyttsp_calibration(1);
	}
	else if ( strcmp(echostr,"exit k" ) == 0 )
	{
		CYTTSP_PRINTK(0, "User Input Echo String : %s\n",echostr);
		cyttsp_calibration(0);
	}
	else
	{
		CYTTSP_PRINTK(0, "touch panel can't calibration!!!\n");
	}
	mutex_unlock(&g_ts->mutex);
    return count;
}

static int ts_misc_release(struct inode *inode, struct file *fp)
{
    int result = 0;

    mutex_lock(&g_ts->mutex);
    if(g_ts->misc_open_count)
        g_ts->misc_open_count--;
    mutex_unlock(&g_ts->mutex);
    return result;
}

static int ts_misc_open(struct inode *inode, struct file *fp)
{
    int result = 0;

    mutex_lock(&g_ts->mutex);
    if( g_ts->misc_open_count <= 1 ) {
        g_ts->misc_open_count++;
        CYTTSP_PRINTK(1, "misc open count : %d\n", g_ts->misc_open_count);
    }
    else {
		result = -EFAULT;
		CYTTSP_PRINTK(0, "failed to open misc count : %d\n", g_ts->misc_open_count);
    }
    mutex_unlock(&g_ts->mutex);
    return result;
}

static struct file_operations ts_misc_fops = {
	.owner 	= THIS_MODULE,
	.open 	= ts_misc_open,
	.release = ts_misc_release,
	.write = ts_misc_write,
	
    
};
static struct miscdevice ts_misc_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "cypress_misc_touch",
	.fops 	= &ts_misc_fops,
};


static void cyttsp_create_kernel_debuglevel(void)
{
	CYTTSP_PRINTK(1, "create kernel debuglevel!!!\n");


	if (kernel_debuglevel_dir!=NULL)
	{
	  debugfs_create_u32("ts_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&CYTTSP_DLL));
	}
	else
	{
		printk(KERN_ERR "kernel_debuglevel_dir ts_dl Fail\n");
	}

}


static void cyttsp_destroy_kernel_debuglevel(void)
{
	CYTTSP_PRINTK(1, "destroy kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}


static int __devinit cyttsp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc = 0;
	struct cyttsp_platform_data *pdata;
	int i;

	CYTTSP_PRINTK(1,"Start Probe!!!\n");

	
	g_ts = kzalloc(sizeof(struct cyttsp), GFP_KERNEL);
	if (g_ts == NULL) {
		CYTTSP_PRINTK(0,"failed to kzalloc g_ts mem!!!\n");
		rc = -ENOMEM;
		return rc;
	}

	
	for (i = 0 ; i < CY_NUM_MT_TCH_ID; i++ )
	{
		g_ts->msg[i].coord.x = 0;
		g_ts->msg[i].coord.y = 0;
		g_ts->msg[i].coord.z = 0;
		g_ts->msg[i].coord.tch_id = 0;
		g_ts->msg[i].state = TS_RELEASE;
		g_ts->msg[i].w = 0;
	}

	
	g_ts->key_msg[0].key_coord.x = 0;
	g_ts->key_msg[0].key_coord.y = 0;
	g_ts->key_msg[0].key_state = KEY_RELEASE;

	
	pdata = client->dev.platform_data;
	g_ts->gpio_num[IRQ_GPIO] = pdata->irq_gpio;
	g_ts->gpio_num[RST_GPIO] = pdata->rst_gpio;
	
	
	g_ts->irq = MSM_GPIO_TO_INT(g_ts->gpio_num[IRQ_GPIO]);
	g_ts->client = client;
	mutex_init(&g_ts->mutex);

	
	rc = cyttsp_power_on_device(g_ts, 1);
	if (rc) {
		CYTTSP_PRINTK(0,"Unable to power device rc=%d\n",rc);
		goto error_alloc_mem;
	}

	
	rc = cyttsp_setup_gpio(g_ts);
	if (rc) {
		CYTTSP_PRINTK(0,"Failed to setup gpio rc=%d\n", rc);
		goto error_power_device;
	}

	
	rc = cyttsp_config_gpio(g_ts);
	if (rc) {
		CYTTSP_PRINTK(0,"Failed to config gpio rc=%d\n", rc);
		goto error_setup_gpio;
	}
	msleep(15);

	
	rc = ts_detect_cypress(g_ts);
    if(rc) {
		CYTTSP_PRINTK(0,"Failed to detect cypress IC rc=%d\n", rc);
		goto error_setup_gpio;
    }
	client->driver = &i2c_ts_driver;
	i2c_set_clientdata(client, g_ts);

	g_ts->power_state = ACTIVE_STATE;
	CYTTSP_PRINTK(1,"Power state is ACTIVE!!!\n");
	PM_LOG_EVENT(PM_LOG_ON, PM_LOG_TOUCH);

#if 0
	msleep(1);
	
	rc = cyttsp_read_sys_info(g_ts);
	if (rc) {
		CYTTSP_PRINTK(0,"Failed to read system info reg map rd=%d\n", rc);
		goto error_setup_gpio;
	}
#endif

	
	rc = cyttsp_register_input(&g_ts->input, pdata, client);
    if(rc)
    {
    	CYTTSP_PRINTK(0, "failed to register cypress ts device\n");
		goto error_setup_gpio;
		return rc;
    }
    input_set_drvdata(g_ts->input, g_ts);

	
    rc = keyarray_register_input(&g_ts->keyarray_input, client);
    if(rc)
    {
		CYTTSP_PRINTK(0, "failed to register cypress keyarray device\n");
		goto error_register_ts_input;
    }
    input_set_drvdata(g_ts->keyarray_input, g_ts);

	
    INIT_DELAYED_WORK(&g_ts->ts_work, cyttsp_irqWorkHandler);
    g_ts->cyttsp_ts_wq = create_singlethread_workqueue("CYTTSP_TS_Wqueue");
    if (!g_ts->cyttsp_ts_wq)
    {
		CYTTSP_PRINTK(0, "failed to create ts singlethread workqueue\n" );
		goto error_register_keyarray_input;
    }

	CYTTSP_PRINTK(1,"Setting up interrupt\n");
	
	rc = request_irq(g_ts->irq, cyttsp_irq,
	IRQF_TRIGGER_FALLING, client->dev.driver->name, g_ts);
	if (rc) {
		CYTTSP_PRINTK(0,"failed to request irq rc=%d\n", rc);
		rc = -ENODEV;
		goto error_create_singlethread_wq;
	}

	
	rc = misc_register(&ts_misc_device);
    if(rc)
    {
       	CYTTSP_PRINTK(0, "failed to register misc driver rc=%d\n", rc);
		goto error_misc_register;
    }

	
	cyttsp_create_kernel_debuglevel();

#ifdef CONFIG_HAS_EARLYSUSPEND
		g_ts->ts_early_suspend.level = 150; 
		g_ts->ts_early_suspend.suspend = cyttsp_suspend;
		g_ts->ts_early_suspend.resume = cyttsp_resume;
		register_early_suspend(&g_ts->ts_early_suspend);
#endif 
	CYTTSP_PRINTK(0,"Start Probe %s\n",
		(rc < 0) ? "FAIL" : "PASS");

	return 0;

error_misc_register:
    misc_deregister(&ts_misc_device);

error_create_singlethread_wq:
	destroy_workqueue(g_ts->cyttsp_ts_wq);

error_register_keyarray_input:
    input_unregister_device(g_ts->keyarray_input);
    input_free_device(g_ts->keyarray_input);
    g_ts->keyarray_input = NULL;

error_register_ts_input:
    input_unregister_device(g_ts->input);
    input_free_device(g_ts->input);
    g_ts->input = NULL;

error_setup_gpio:
	cyttsp_release_gpio(g_ts);

error_power_device:
	cyttsp_power_on_device(g_ts, 0);

error_alloc_mem:
	kfree(g_ts);
	return rc;
}


static int __devexit cyttsp_remove(struct i2c_client *client)
{
	
	struct cyttsp *ts = i2c_get_clientdata(client);
	
	

	CYTTSP_PRINTK(0,"Unregister\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&g_ts->ts_early_suspend);
#endif 

	cyttsp_destroy_kernel_debuglevel();
	misc_deregister(&ts_misc_device);

	
	if (cancel_delayed_work((struct delayed_work *)&ts->work) < 0)
		CYTTSP_PRINTK(0,"error: could not remove work from workqueue\n");
	destroy_workqueue(g_ts->cyttsp_ts_wq);
	free_irq(client->irq, ts);

	input_unregister_device(g_ts->keyarray_input);
    input_free_device(g_ts->keyarray_input);
    g_ts->keyarray_input = NULL;

	input_unregister_device(g_ts->input);
    input_free_device(g_ts->input);
    g_ts->input = NULL;

	cyttsp_release_gpio(g_ts);

	mutex_destroy(&ts->mutex);

	g_ts = NULL;
	kfree(ts);

	CYTTSP_PRINTK(1,"Leaving\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_suspend(struct early_suspend *handler)
{
	struct cyttsp *g_ts = container_of(handler,
	struct cyttsp, ts_early_suspend);
	
	struct i2c_client *client = g_ts->client;
	
	int rc = 0;

	CYTTSP_PRINTK(1, "Enter Sleep mode!!!\n");
	mutex_lock(&g_ts->mutex);
	if (g_ts->is_suspended) {
		CYTTSP_PRINTK(0, "in sleep state\n");
		mutex_unlock(&g_ts->mutex);
		return;
	}

	
	
	disable_irq(g_ts->irq);
	CYTTSP_PRINTK(2,"disable irq %d\n",g_ts->irq );
	g_ts->is_suspended = 1;
	

	rc = cancel_work_sync(&g_ts->work);
	
	if (rc)
		enable_irq(g_ts->client->irq);

	
	g_ts->key_msg[0].key_coord.x = 0x0;
	g_ts->key_msg[0].key_coord.y = 0x0;
	g_ts->key_msg[0].key_state = KEY_RELEASE;
	input_report_key(g_ts->keyarray_input, KEY_HOME, 0);
	input_report_key(g_ts->keyarray_input, KEY_SEARCH, 0);
	input_report_key(g_ts->keyarray_input, KEY_BACK, 0);
	input_report_key(g_ts->keyarray_input, KEY_MENU, 0);

	mdelay(50);
	
	rc = ts_read_i2c(client, CY_REG_BASE, &g_ts->sleep_mode, 1);
	if (rc) {
		CYTTSP_PRINTK(0,"failed to read hst_mode[0x%x] value:0x%x (rc=%d)\n",
		CY_REG_BASE, g_ts->sleep_mode, rc);
		mutex_unlock(&g_ts->mutex);
		return;
	}
	CYTTSP_PRINTK(1,"power state is %d\n", g_ts->power_state);
	CYTTSP_PRINTK(1,"sleep mode is 0x%x\n", g_ts->sleep_mode);
	
	if (g_ts->power_state == ACTIVE_STATE) {
		g_ts->sleep_mode = CY_DEEP_SLEEP_MODE;
		rc = ts_write_i2c(client, CY_REG_BASE, &g_ts->sleep_mode, 1);
		if (rc) {
			CYTTSP_PRINTK(0,"failed to write hst_mode[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE, g_ts->sleep_mode, rc);
			mutex_unlock(&g_ts->mutex);
			return;
		}
		g_ts->power_state = SLEEP_STATE;
		CYTTSP_PRINTK(1,"power state is %d\n", g_ts->power_state);
		CYTTSP_PRINTK(1,"sleep mode is 0x%x\n", g_ts->sleep_mode);
	}
	else if(g_ts->sleep_mode == CY_LOW_PWR_MODE) {
		g_ts->sleep_mode = CY_LOW_PWR_MODE;
		rc = ts_write_i2c(client, CY_REG_BASE, &g_ts->sleep_mode, 1);
		if (rc) {
			CYTTSP_PRINTK(0,"failed to write hst_mode[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE, g_ts->sleep_mode, rc);
			mutex_unlock(&g_ts->mutex);
			return;
		}
		g_ts->power_state = LOW_PWR_STATE;
		CYTTSP_PRINTK(1,"power state is %d\n", g_ts->power_state);
		CYTTSP_PRINTK(1,"sleep mode is 0x%x\n", g_ts->sleep_mode);
	}
	mutex_unlock(&g_ts->mutex);
	PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_TOUCH);
	return;

}

static void cyttsp_resume(struct early_suspend *handler)
{
	struct cyttsp *g_ts = container_of(handler,
		struct cyttsp, ts_early_suspend);
	struct i2c_client *client = g_ts->client;
	int rc = 0;
	int i;

	mutex_lock(&g_ts->mutex);
	CYTTSP_PRINTK(0,"cyttsp resume\n");

	
	#if 0
	for (i = 0; i < CY_NUM_MT_TCH_ID; i++)
	{
		if (g_ts->msg[i].coord.z == -1)
			continue;
		g_ts->msg[i].coord.z = 0;
		g_ts->msg[i].state = TS_RELEASE;
	}
	#endif
	for (i = 0; i < CY_MAX_REPORT_ID; i++) {
		g_ts->msg[i].state = TS_RELEASE;
		g_ts->msg[i].coord.x = 0x0;
		g_ts->msg[i].coord.y = 0x0;
		g_ts->msg[i].coord.z = 0x0;
		g_ts->msg[i].w = 0x0;
		g_ts->msg[i].coord.tch_id = 0x0;
    }
	

	
	rc = cyttsp_config_gpio(g_ts);
	if (rc) {
		CYTTSP_PRINTK(0,"Failed to config gpio rc=%d\n", rc);
		mutex_unlock(&g_ts->mutex);
	    return;
	}
	msleep(50);

	
	if (g_ts->sleep_mode == CY_DEEP_SLEEP_MODE || g_ts->sleep_mode == CY_LOW_PWR_MODE) {
		g_ts->sleep_mode = g_ts->sleep_mode & 0x00;
		rc = ts_write_i2c(client, CY_REG_BASE, &g_ts->sleep_mode, 1);
		if (rc) {
			CYTTSP_PRINTK(0,"failed to write hst_mode[0x%x] value:0x%x (rc=%d)\n",
			CY_REG_BASE, g_ts->sleep_mode, rc);
			mutex_unlock(&g_ts->mutex);
			return;
		}
		g_ts->power_state = ACTIVE_STATE;
		CYTTSP_PRINTK(1,"power state is %d\n", g_ts->power_state);
		CYTTSP_PRINTK(1,"sleep mode is 0x%x\n", g_ts->sleep_mode);
	}

	enable_irq(g_ts->irq);
	g_ts->is_suspended = 0;
	mutex_unlock(&g_ts->mutex);
	PM_LOG_EVENT(PM_LOG_ON, PM_LOG_TOUCH);

	return ;

}
#endif  

static int __init cyttsp_init(void)
{
	int ret = 0;

	CYTTSP_PRINTK(1,"Cyttsp Touchscreen Driver (Built %s @ %s)\n",
		__DATE__, __TIME__);
	i2c_ts_driver.driver.name = CY_I2C_NAME;

	
	CYTTSP_PRINTK(0, "system_rev=0x%x\n", system_rev);
	if (system_rev < CHICAGO_EVT2) {
		ret = i2c_add_driver(&i2c_ts_driver);
		if (ret) {
			CYTTSP_PRINTK(0, " CYTTSP touch driver registration failed\n");
			return ret;
		}
	}
	return ret;
}

static void __exit cyttsp_exit(void)
{
	i2c_del_driver(&i2c_ts_driver);
	CYTTSP_PRINTK(0, " CYTTSP touch driver Exiting\n");
}

module_init(cyttsp_init);
module_exit(cyttsp_exit);
MODULE_FIRMWARE("cyttsp.fw");

