

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pwm.h>
#include <linux/hrtimer.h>
#include <mach/pmic.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include "adp1650_flash.h"
#include <mach/pm_log.h>
#include <linux/delay.h>

#define PERF


extern struct dentry *kernel_debuglevel_dir;
static unsigned int FLASH_ADP1650_DLL=0;
#define FLASH_PRINTK(level, fmt, args...) if(level <= FLASH_ADP1650_DLL) printk("%s:" fmt, __func__, ##args);


static void flash_create_kernel_debuglevel(void)
{
	FLASH_PRINTK(1, "create flash kernel debuglevel!\n");


	if (kernel_debuglevel_dir!=NULL)
	{
	  debugfs_create_u32("flash_adp1650_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&FLASH_ADP1650_DLL));
	}
	else
	{
		printk(KERN_ERR "kernel_debuglevel_dir ts_dl Fail\n");
	}

}


static void flash_destroy_kernel_debuglevel(void)
{
	FLASH_PRINTK(1, "destroy flash kernel debuglevel!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}



enum flash_mode {
	FLASH_OFF,
	FLASH_LOW,
	FLASH_HIGH,
	FLASH_TORCH
};

#ifdef PERF
enum flash_mode preMode;
struct timespec start = {0, 0};
#endif

struct timer_list timer_flash;

enum msm_cam_flash_stat{
	MSM_CAM_FLASH_OFF,
	MSM_CAM_FLASH_ON,
};

static struct adp1650_work_t *adp1650_flash_w;
static struct i2c_client *adp1650_client;
static DECLARE_WAIT_QUEUE_HEAD(adp1650_wait_queue);

struct adp1650_work_t {
	struct work_struct work;
};

static const struct i2c_device_id adp1650_i2c_id[] = {
  { "CAM_FLASH_adp1650", 0 },
  { }
};


static struct dentry *dent;



static int adp1650_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 1,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(adp1650_client->adapter, msgs, 2) < 0) {
		FLASH_PRINTK(0, "adp1650_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}

static int32_t adp1650_i2c_read(uint8_t raddr, unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;

	rc = adp1650_i2c_rxdata(adp1650_client->addr, buf, rlen);
	if (rc < 0) {
		FLASH_PRINTK(0, "CAM:adp1650_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}

	*rdata = buf[0];
	

	return rc;
}

static int32_t adp1650_i2c_txdata(unsigned short saddr,
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
	if (i2c_transfer(adp1650_client->adapter, msg, 1) < 0) {
		FLASH_PRINTK(0, "adp1650_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t adp1650_i2c_write_b_flash(uint8_t waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));
	buf[0] = waddr;
	buf[1] = bdata;

	rc = adp1650_i2c_txdata(adp1650_client->addr, buf, 2);
	if (rc < 0) {
		FLASH_PRINTK(0, "i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
				waddr, bdata);
	}
	return rc;
}

static int adp1650_read_id(uint16_t *flash_id)
{
	int rc = 0;
	uint16_t chipid = 0;

	rc = adp1650_i2c_read(0x00, &chipid, 1);
	if (rc < 0)
		FLASH_PRINTK(0, "CAM:adp1650's id is 0x%x!\n", chipid);

	*flash_id = chipid;

	return rc;
}

static int adp1650_init_client(struct i2c_client *client)
{
	
	init_waitqueue_head(&adp1650_wait_queue);
	return 0;
}

static int adp1650_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	FLASH_PRINTK(0, "CAM: qsd_flash_i2c_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FLASH_PRINTK(0, "i2c_check_functionality failed\n");
		goto probe_failure;
	}

	adp1650_flash_w = kzalloc(sizeof(struct adp1650_work_t), GFP_KERNEL);
	if (!adp1650_flash_w) {
		printk("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, adp1650_flash_w);
	adp1650_init_client(client);
	adp1650_client = client;

	FLASH_PRINTK(0, "qsd_flash_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	FLASH_PRINTK(0, "qsd_flash_probe failed! rc = %d\n", rc);
	return rc;
}

static struct i2c_driver adp1650_i2c_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name = "CAM_FLASH_adp1650",
  },
  .id_table = adp1650_i2c_id,
  .probe    = adp1650_i2c_probe,
};

static int32_t msm_camera_flash_read_led_id(uint16_t *flashid)
{
	uint16_t flash_id;
	int32_t rc = 0;

	rc = adp1650_read_id(&flash_id);
	*flashid = flash_id;

	return rc;
}

#if 1
static void assist_light(uint8_t i_tor)
{

   adp1650_i2c_write_b_flash(CURRENT_SET_REG, i_tor);

   adp1650_i2c_write_b_flash(OUTPUT_MODE_REG, OUTPUT_MODE_DEF |OUTPUT_MODE_ASSIST);


}
#endif

static void flash_light(uint8_t i_fl, uint8_t fl_tim)
{
   adp1650_i2c_write_b_flash(TIMER_REG, fl_tim |TIMER_MODE_DEF);
   adp1650_i2c_write_b_flash(CURRENT_SET_REG, i_fl<<3 |CUR_TOR_100MA);
   adp1650_i2c_write_b_flash(OUTPUT_MODE_REG, OUTPUT_MODE_DEF_EDGE |OUTPUT_MODE_FLASH);
}

static void turn_off(void)
{
   adp1650_i2c_write_b_flash(OUTPUT_MODE_REG, 0);

}


static int adp1650_flash( enum flash_mode mode )
{
    int rc = 0;
    uint16_t errInfo = 0;

    #ifdef PERF
    struct timespec temp;
    #endif

    FLASH_PRINTK(0, "%s, flash mode: %d\n", __func__, mode);

    rc = adp1650_i2c_read(0x05, &errInfo, 1);
    if((rc <0) ||(errInfo!=0) )
    {
        if(errInfo != 0)
        {
             FLASH_PRINTK(0, "%s, error message:0x%x\n", __func__, errInfo);
   #if 0
             gpio_direction_output(13, 0);
	      gpio_direction_output(13, 1);
	      mdelay(1000);
   #endif
        }
        else
        {
             FLASH_PRINTK(0, "%s, rc: %d read reg:0x05 error\n", __func__, rc);
        }

    }

    if (mode != FLASH_OFF)
    {
       PM_LOG_EVENT(PM_LOG_ON, PM_LOG_FLASHLIGHT);
    }
    switch(mode) {
    case FLASH_LOW: 
    	assist_light(CUR_TOR_100MA);

   #ifdef PERF
    if( preMode == FLASH_OFF)
    {
    	temp = timespec_sub(CURRENT_TIME, start);
    	FLASH_PRINTK(1, "%s: the flash off to pre flash on is :%ld ms\n",__func__, ((long)temp.tv_sec*1000)+((long)temp.tv_nsec/1000000));
    }
    #endif

    	#ifdef PERF
    	start = CURRENT_TIME;
    	#endif

    break;

    case FLASH_HIGH:

    #ifdef PERF
    if( preMode == FLASH_OFF)
    {
    	temp = timespec_sub(CURRENT_TIME, start);
    	FLASH_PRINTK(1, "%s: the pre flash off to flash on is :%ld ms\n",__func__, ((long)temp.tv_sec*1000)+((long)temp.tv_nsec/1000000));
    }
    #endif


    flash_light(CUR_FL_300MA,  TIMER_300MS);
    
    break;

    case FLASH_TORCH:
    assist_light(CUR_TOR_100MA);
    
    break;

    case FLASH_OFF:
    #ifdef PERF
    if( preMode == FLASH_LOW)
    {
    	temp = timespec_sub(CURRENT_TIME, start);
    	FLASH_PRINTK(1, "%s: the pre flash duration is :%ld ms\n",__func__, ((long)temp.tv_sec*1000)+((long)temp.tv_nsec/1000000));
    }
    #endif

    #ifdef PERF
    if( preMode == FLASH_HIGH)
    {
    	temp = timespec_sub(CURRENT_TIME, start);
    	FLASH_PRINTK(1, "%s: the flash duration is :%ld ms\n",__func__, ((long)temp.tv_sec*1000)+((long)temp.tv_nsec/1000000));
    }
    #endif
    turn_off();

    	#ifdef PERF
    	start = CURRENT_TIME;
    	#endif

    PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_FLASHLIGHT);

    break;
    default:
    turn_off();
    PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_FLASHLIGHT);
     }

    #ifdef PERF
    preMode = mode;
    #endif
    return rc;
}
#if 0
static int adp1650_flash( enum flash_mode mode )
{
	int rc = 0;
	uint16_t chipid;

#if 0
	rc = gpio_request(13, "FL_EN");
	if (rc < 0) {
		printk("[CAM_FLASH]request gpio failed!\n");
		return -rc;
	}
	gpio_direction_output(13, 1);
	mdelay(1000);

	rc = adp1650_i2c_read(0x00, &chipid, 1);
	printk("CAM:ADP1650_ID=>ID(0x00): %x\n", chipid);
       if((rc < 0) || (chipid != 0x22))
       {
              printk("%s, set flash light mode failed! rc = %d, chipid is 0x%x ", __func__, rc, chipid);
	       return -1;
       }

	
	rc = adp1650_i2c_read(0x02, &chipid, 1);
	if (rc < 0) {
			printk("~~CAM:register 0x02 =>(%d)\n", chipid);
			return -rc;
	}
#endif

      printk("%s, flash mode is %d", __func__, mode);
	switch(mode) {
	case FLASH_LOW: 
		
		adp1650_i2c_write_b_flash(0x03,0x03);	
		adp1650_i2c_write_b_flash(0x04,0xAA);	
		rc = adp1650_i2c_read(0x03, &chipid, 1);
		if (rc < 0) {
			printk("CAM:register 0x03 =>(%d)\n", chipid);
			return -rc;
		}
		break;
	case FLASH_HIGH:
		
	       
		
              adp1650_i2c_write_b_flash(0x02,0xA1);	
		adp1650_i2c_write_b_flash(0x03,0x73); 
		adp1650_i2c_write_b_flash(0x04,0x8F);	
		rc = adp1650_i2c_read(0x03, &chipid, 1);
		if (rc < 0) {
			printk("CAM:register 0x03 =>(%d)\n", chipid);
			return -rc;
		}
		break;
	case FLASH_TORCH:
		
		adp1650_i2c_write_b_flash(0x03,0x03);	
		adp1650_i2c_write_b_flash(0x04,0xAA);	
		rc = adp1650_i2c_read(0x03, &chipid, 1);
			if (rc < 0) {
			printk("CAM:register 0x03 =>(%d)\n", chipid);
			return -rc;
		}
		break;
	case FLASH_OFF:
		
		adp1650_i2c_write_b_flash(0x04, 0x00);	
		
	}
	


#if 0
	rc = adp1650_i2c_read(0x04, &chipid, 1);
       if (rc < 0) {
		printk("CAM:register 0x04 =>(%d)\n", chipid);
		return -rc;
	}

	rc = adp1650_i2c_read(0x05, &chipid, 1);
	if (rc < 0) {
			printk("CAM:register 0x05 =>(%d)\n", chipid);
			return -rc;
	}
#endif


	return rc;


}
#endif
int msm_camera_flash_current_driver(
	struct msm_camera_sensor_flash_current_driver *current_driver,
	unsigned led_state)
{
	int rc = 0;
	
	switch (led_state) {
	case MSM_CAMERA_LED_OFF:

		adp1650_flash(FLASH_OFF);
		break;
	case MSM_CAMERA_LED_LOW:

		adp1650_flash(FLASH_LOW);
		break;
	case MSM_CAMERA_LED_HIGH:

		adp1650_flash(FLASH_HIGH);
		break;
	
	case MSM_CAMERA_LED_VIDEO_TORCH:

		adp1650_flash(FLASH_TORCH);
		break;
	

	case MSM_CAMERA_LED_INIT:
	case MSM_CAMERA_LED_RELEASE:
       	break;

	default:
		rc = -EFAULT;
		break;
	}

	return rc;
}

int msm_camera_flash_i2c_driver(unsigned led_state)
{
		return 0;
}

int32_t msm_camera_flash_set_led_state(
	struct msm_camera_sensor_flash_data *fdata, unsigned led_state)
{
	int rc = 0;

	FLASH_PRINTK(1, "CAM:flash_set_led_state: %d flash_sr_type=%d\n", led_state,
	    fdata->flash_src->flash_sr_type);

	if (fdata->flash_type != MSM_CAMERA_FLASH_LED)
		return -ENODEV;

	switch (fdata->flash_src->flash_sr_type) {
	case MSM_CAMERA_FLASH_SRC_PMIC:
	case MSM_CAMERA_FLASH_SRC_PWM:
		break;

	case MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER:
		rc = msm_camera_flash_current_driver(
			&fdata->flash_src->_fsrc.current_driver_src,
			led_state);
		break;
	
	case MSM_CAMERA_FLASH_SRC_I2C_DRIVER:
		rc = msm_camera_flash_i2c_driver(led_state);
		break;
	

	default:
		rc = -ENODEV;
		break;
	}

	return rc;
}

int msm_flash_ctrl(struct msm_camera_sensor_info *sdata,
	struct flash_ctrl_data *flash_info)
{
	int rc = 0;
	

	FLASH_PRINTK(0, "CAM: msm_flash_ctrl:flash type:(%d) \n",flash_info->flashtype);

	switch (flash_info->flashtype) {
	case LED_FLASH:
		
		
		
		rc = msm_camera_flash_set_led_state(sdata->flash_data,
			flash_info->ctrl_data.led_state);
			break;
	case STROBE_FLASH:
	default:
		FLASH_PRINTK(0, "Invalid Flash MODE\n");
		rc = -EINVAL;
	}
	return rc;
}

int msm_strobe_flash_init(struct msm_sync *sync, uint32_t sftype)
{
	return 0;
}



int adp1650_set_flash(void *data, u64 value)
{

    int rc = 0;
    

    rc = gpio_request(13, "FL_EN");
    if (rc < 0) {
        FLASH_PRINTK(0, "%s, [CAM_FLASH] gpio 13 has been requested!\n", __func__);
    }
    gpio_direction_output(13, 1);
    mdelay(1000);

    switch(value)
    {
    case 0:
        rc = adp1650_flash(FLASH_OFF);
        gpio_direction_output(13, 0);
        break;

    case 1:
    	rc = adp1650_flash(FLASH_TORCH);
       break;

    case 2:
    	rc = adp1650_flash(FLASH_LOW);
       break;

    default:
    	rc = adp1650_flash(FLASH_OFF);
    }

    if(rc <0)
    {
        FLASH_PRINTK(0, "%s, mode is %d, rc is %d", __func__, (int)value, rc);
    }


    gpio_free(13);
    return rc;
}

DEFINE_SIMPLE_ATTRIBUTE(adp1650_flash_enable_fops, NULL , adp1650_set_flash, "%llu\n");

static void adp1650_create_debugfs_entries(void)
{
	FLASH_PRINTK(1, "%s++\n", __func__);

	dent = debugfs_create_dir("flash", NULL);
	if (dent) {
		debugfs_create_file("mode", S_IRUGO | S_IWUGO, dent, NULL, &adp1650_flash_enable_fops);
		
		
		
	}
}

static void adp1650_destroy_debugfs_entries(void)
{
	FLASH_PRINTK(1, "%s++\n", __func__);

	if (dent)
		debugfs_remove_recursive(dent);
}

static int __init adp1650_init(void)
{
    int rc = 0;
	uint16_t flash_id;

       flash_create_kernel_debuglevel();
	rc = i2c_add_driver(&adp1650_i2c_driver);
	if (rc < 0 || adp1650_client == NULL) {
		FLASH_PRINTK(0,"CAM: i2c_add_driver fail \n");
		rc = -EINVAL;
		return rc;
	}

	
	gpio_tlmm_config (GPIO_CFG(13, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	gpio_tlmm_config (GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	gpio_tlmm_config (GPIO_CFG(17, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	gpio_tlmm_config (GPIO_CFG(111, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	

	rc = gpio_request(13, "FL_EN");
	FLASH_PRINTK(0, "%d\n", rc);
	if (rc < 0) {
		FLASH_PRINTK(0, "[CAM_FLASH]request gpio failed!\n");
		
	}

	PM_LOG_EVENT(PM_LOG_ON, PM_LOG_FLASHLIGHT);
	gpio_direction_output(13, 1);

	
	#if 0
	mdelay(1000);
	#else
		mdelay(10);
	#endif
	

	rc = msm_camera_flash_read_led_id(&flash_id);
	FLASH_PRINTK(0, "CAM: Flash ID:(0x%x) \n", flash_id);

	gpio_direction_output(13, 0);
	gpio_free(13);
	PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_FLASHLIGHT);


	adp1650_create_debugfs_entries();
	return rc;
}

void __exit adp1650_exit(void)
{
       adp1650_destroy_debugfs_entries();
	i2c_del_driver(&adp1650_i2c_driver);
	flash_destroy_kernel_debuglevel();
}

MODULE_AUTHOR("Ritchie.Sun");
MODULE_LICENSE("Dual BSD/GPL");
module_init(adp1650_init);
module_exit(adp1650_exit);
