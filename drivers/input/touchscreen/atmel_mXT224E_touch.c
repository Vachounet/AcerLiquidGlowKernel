

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#ifdef   CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/time.h>
#include <mach/gpio.h>
#include <linux/mutex.h>
#include <mach/vreg.h>
#include <mach/atmel_mXT224E_touch.h>
#include "atmel_touch_CMI_obj_V10.h" 
#include <mach/chicago_hwid.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/pm_runtime.h>
#include <mach/pm_log.h>
#define ECHOSTR_SIZE   30
#define ATMEL_POR_DELAY         100 


extern struct dentry *kernel_debuglevel_dir;
static unsigned int ATMEL_DLL=0;
static unsigned int ATMEL_TOUCH_REPORT_DLL=0;
static unsigned int ATMEL_CAPKEY_REPORT_DLL=0;
#define ATMEL_ERR_LEVEL   0
#define ATMEL_FLOW_INFO_LEVEL  1
#define ATMEL_TOUCH_REPORT_LEVEL 1
#define ATMEL_CAPKEY_REPORT_LEVEL 1
#define ATMEL_OTHER_LEVEL 2
#define ATMEL_PRINTK(level, fmt, args...) if(level <= ATMEL_DLL) printk( fmt, ##args);
#define ATMEL_TOUCH_PRINTK(level, fmt, args...) if(level <= ATMEL_TOUCH_REPORT_DLL) printk(fmt, ##args);
#define ATMEL_CAPKEY_PRINTK(level, fmt, args...) if(level <= ATMEL_CAPKEY_REPORT_DLL) printk(fmt, ##args);

struct touchpad_t {
    struct i2c_client        *client;
    struct input_dev         *input;
	struct input_dev         *keyarray_input;
    int                      irq; 
    int                      gpio_irq; 
    int                      gpio_rst; 
    int                      open_count; 
	int                      keyarray_open_count; 
	
	int						 touch_suspended; 
    struct delayed_work      touchpad_work;
    struct workqueue_struct  *touchpad_wqueue;
    uint32_t                 info_block_checksum;
    uint32_t                 T6_config_checksum;
    struct  id_info_t        id_info; 
    struct  obj_t            *init_obj_element[MAX_OBJ_ELEMENT_SIZE];
    struct  touch_point_status_t msg[ATMEL_REPORT_POINTS];
    struct  mutex            mutex;
	struct early_suspend     touch_early_suspend; 
	uint16_t                 atmel_x_max,atmel_y_max;
	int                      fvs_mode_flag;  
	int chk_count;
	struct workqueue_struct *chk_wqueue;
	struct delayed_work      chk_work;
	int backup_emlist_flag;
	struct regulator *vdd_regulator; 
	
	
	
	struct regulator *avdd_2_regulator; 
	
	int selftest_flag;
	struct atmel_selftest_t selftest;
	
	
	int em_key_flag;
	
	
	uint8_t T7_power_mode_value[T7_MAX_SIZE];
	
	int touch_count; 
	
};

static struct touchpad_t         *g_tp;
static int __devinit touchpad_probe(struct i2c_client *client,  const struct i2c_device_id *);
static int __devexit touchpad_remove(struct i2c_client *client);
static int keyarray_register_input( struct input_dev **input,struct i2c_client *client );
static void tp_resume(struct early_suspend *h);
static void tp_suspend(struct early_suspend *h);

#define TOUCH_RETRY_COUNT 5
static int touchpad_write_i2c( struct i2c_client *client,
							   uint16_t			regBuf,
                               uint8_t          *dataBuf,
                               uint8_t          dataLen )
{
    int     result = 0;
	int     retryCnt = TOUCH_RETRY_COUNT;
    uint8_t *buf = NULL;

    struct  i2c_msg msgs[] = {
        [0] = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)buf,
            .len    = 0
        }
    };

    buf = kzalloc( dataLen+sizeof(regBuf), GFP_KERNEL );
    if(NULL == buf)
    {
        ATMEL_PRINTK(0,"alloc memory failed\n");
        return -EFAULT;
    }

    buf[0] = (uint8_t)(regBuf & 0xFF);
    buf[1] = (uint8_t)(regBuf >> 8);
    memcpy( &buf[2], dataBuf, dataLen );
    msgs[0].buf = buf;
    msgs[0].len = dataLen+sizeof(regBuf);

    while(retryCnt)
    {
		result = i2c_transfer(client->adapter, msgs, 1);
		if(result != ARRAY_SIZE(msgs))
		{
			ATMEL_PRINTK(0, "write %Xh %Xh %d bytes return failure, %d\n", buf[0], buf[1], dataLen ,result);
			if(-ETIMEDOUT == result) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
		}
	}

    if((result == 0) && (retryCnt < TOUCH_RETRY_COUNT))
        ATMEL_PRINTK(0, "write %Xh %Xh %d bytes retry at %d\n", buf[0], buf[1], dataLen, TOUCH_RETRY_COUNT-retryCnt);
#if 0
	result = i2c_transfer( client->adapter, msgs, 1 );
    if( result != ARRAY_SIZE(msgs) )
    {
        ATMEL_PRINTK(0, "write %Xh %Xh %d bytes return failure, %d\n", buf[0], buf[1], dataLen ,result);
        result = -EFAULT;
		kfree( buf );
        return result;
    }
#endif
    kfree(buf);
	return result;
}

static int touchpad_read_i2c( struct i2c_client *client,
                            uint16_t           regBuf,
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
            .len    = sizeof(regBuf)
        },
        [1] = {
            .addr   = client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)dataBuf,
            .len    = dataLen
        }
    };

#if 0
	result = touchpad_write_i2c(client,regBuf,NULL,0);
	if(result)
	{
       ATMEL_PRINTK(0, "read %Xh %d bytes return failure, (result=%d)\n", regBuf, dataLen, result);
        return result;
	}
#endif
	while(retryCnt)
    {
		result = i2c_transfer( client->adapter, msgs, ARRAY_SIZE(msgs) );
		if( result != ARRAY_SIZE(msgs) )
		{
			ATMEL_PRINTK(0, "read %Xh %d bytes return failure, (result=%d)\n", regBuf, dataLen, result);
			if(-ETIMEDOUT == result) msleep(10);
            retryCnt--;
		} else {
			result = 0;
			break;
		}
	}
	if((result == 0) && (retryCnt < TOUCH_RETRY_COUNT))
        ATMEL_PRINTK(0, "read %Xh %d bytes retry at %d\n", regBuf, dataLen, TOUCH_RETRY_COUNT-retryCnt);

	return result;
}

static int tp_read_T5_via_i2c( struct i2c_client *client,
                            uint8_t           *dataBuf,
                            uint8_t           dataLen )
{
	int result = 0;
	uint16_t addr = maxTouchCfg_T5_obj.obj_addr;
	uint8_t size = maxTouchCfg_T5_obj.size;


	if(addr == 0 || size == 0) {
		ATMEL_PRINTK(0, "maxTouchCfg_T5_obj is NOT initialized\n");
		result = -ENOMEM;
		return result;
	}
	if(client == NULL || dataBuf == NULL || dataLen != size) {
		ATMEL_PRINTK(0, "no mem for reading message via i2c\n");
		result = -ENOMEM;
		return result;
	}
	result = touchpad_read_i2c(client,addr,dataBuf,dataLen);
	if(result) {
		ATMEL_PRINTK(0, "failed to read addr:0x%x (result=%d)\n", addr, result);
		result = -ENOMEM;
		return result;
	}
	return result;
}


static uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2)
{
	static const uint32_t crcpoly = 0x80001B;
   	uint32_t result;
   	uint16_t data_word;

   	data_word = (uint16_t) ((uint16_t) (byte2 << 8u) | byte1);
   	ATMEL_PRINTK(1,"read dara word: 0x%x\n", data_word);
   	result = ((crc << 1u) ^ (uint32_t) data_word);

   	if (result & 0x1000000) 
		result ^= crcpoly; 

   	return result;
}


static int calculate_config_crc(struct touchpad_t *g_tp)
{
	int result = 0;
   	uint32_t crc = 0;
   	uint16_t crc_area_size = 0;
   	uint16_t remainder;
   	uint8_t  *value = NULL;
    uint8_t  i,j,index;

   	for (i = 0 ; i < g_tp->id_info.num_obj_element ; i++) {
    	if(g_tp->init_obj_element[i]->config_crc == 1) {
    		crc_area_size += (g_tp->init_obj_element[i]->size);
    		ATMEL_PRINTK(1, "g_tp->init_obj_element[i]->size = %x\n", g_tp->init_obj_element[i]->size);
   			ATMEL_PRINTK(1, "crc_area_size = %x\n", crc_area_size);
    	}
    }

    value = kmalloc(crc_area_size, GFP_KERNEL);
   	if(NULL == value) {
        result = -ENOMEM;
		kfree(value);
        return result;
   	}

    index = 0;
   	for (i = 0 ; i < g_tp->id_info.num_obj_element ; i++) {
    	if(g_tp->init_obj_element[i]->config_crc == 1) {
    		memcpy(&value[index], g_tp->init_obj_element[i]->value_array, g_tp->init_obj_element[i]->size);
    		index += g_tp->init_obj_element[i]->size;
			ATMEL_PRINTK(1, "index = %x\n", index);
    	}
    }

    for (j = 0 ; j < crc_area_size ; j++) {
        ATMEL_PRINTK(1, "value_array[%d]:0x%x\n", j, value[j]);
    }

	
    j = 0;
    remainder = crc_area_size%2;
    if (remainder==0) {
		
   		while (j < (crc_area_size-1)) {
			crc = CRC_24(crc, (value[j]),(value[j+1]));
			ATMEL_PRINTK(1, "even:calculate config CRC:0x%x\n", crc);
      		j += 2;
      	}
    }else {
     	
   		while (j < (crc_area_size-1)) {
   			crc = CRC_24(crc, (value[j]), (value[j+1]));
   			ATMEL_PRINTK(1, "odd:calculate config CRC:0x%x\n", crc);
      		j += 2;
      	}
      	crc = CRC_24(crc, (value[j]), 0);
        ATMEL_PRINTK(1, "calculate config CRC:0x%x\n", crc);
    }

   	
   	crc = (crc & 0x00FFFFFF);
   	ATMEL_PRINTK(0, "calculate config CRC:0x%x\n", crc);
    ATMEL_PRINTK(0, "g_tp->T6_config_checksum:0x%x\n", g_tp->T6_config_checksum);

   	if (crc != g_tp->T6_config_checksum) {
   		ATMEL_PRINTK(0, "failed to calculate config crc:0x%x\n", crc);
   		result = -EFAULT;
		kfree(value);
        return result;
   	}
	kfree(value);
	return result;
}


static int calculate_infoblock_crc(struct touchpad_t *g_tp)
{
	int result = 0;
   	int INFO_BLOCK_SIZE = (NUM_OF_ID_INFO_BLOCK +
		OBJECT_TABLE_ELEMENT_SIZE * g_tp->id_info.num_obj_element) + INFO_BLOCK_CHECKSUM_SIZE;
   	uint32_t crc = 0;
   	uint16_t crc_area_size,regbuf;
   	uint8_t  *value = NULL;
    uint8_t  i;
   	int	j = 0;

   	value = kmalloc(INFO_BLOCK_SIZE, GFP_KERNEL);
   	if(NULL == value) {
        result = -ENOMEM;
        return result;
   	}

   	
   	crc_area_size = NUM_OF_ID_INFO_BLOCK + OBJECT_TABLE_ELEMENT_SIZE*g_tp->id_info.num_obj_element;

   	
   	result = touchpad_read_i2c(g_tp->client, crc_area_size, &value[0], INFO_BLOCK_CHECKSUM_SIZE);
   	for (regbuf = crc_area_size; regbuf < crc_area_size+INFO_BLOCK_CHECKSUM_SIZE; regbuf++) {
    	ATMEL_PRINTK(2, "read 0x%xh = 0x%x\n", regbuf, value[j]);
    	j++;
   	}
   	g_tp->info_block_checksum = value[0] | value[1]<< 8 | value[2]<<16;
   	ATMEL_PRINTK(1, "info block checksum : 0x%x\n", g_tp->info_block_checksum);

   	result = touchpad_read_i2c(g_tp->client, ADD_ID_INFO_BLOCK, &value[0], crc_area_size);
  	ATMEL_PRINTK(2, "read info block checksum: 0x%x\n", value[0]);
   	if (result < 0) {
      	ATMEL_PRINTK(0, "unable to read reg 0x%x and value failed 0x%x, return %d\n", crc_area_size,value[0], result);
      	kfree(value);
      	return result;
   	}

   	i = 0;
   	while (i < (crc_area_size - 1)) {
      	crc = CRC_24(crc, (value[i]), (value[i+1]));
      	i += 2;
   	}

   	crc = CRC_24(crc, (value[i]), 0);
   	ATMEL_PRINTK(1, "read info block checksum: 0x%x:\n", crc);

   	
   	crc = (crc & 0x00FFFFFF);
   	ATMEL_PRINTK(1, "read info block CRC: 0x%x:\n", crc);

   	if (crc != g_tp->info_block_checksum) {
   		ATMEL_PRINTK(0, "failed to calculate infoblock crc: 0x%x:\n", crc);
   		kfree(value);
        return result;
   	}
   	return result;
}

static void save_multi_touch_struct(uint16_t x_coord,uint16_t y_coord,uint8_t touch_status, uint8_t report_id, uint8_t w, uint8_t z)
{

	g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].coord.x = (uint)x_coord;
	g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].coord.y = (uint)y_coord;
	if ((touch_status & TOUCH_PRESS_RELEASE_MASK) == TOUCH_PRESS_RELEASE_MASK) 
    {
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].state = RELEASE;
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].w = w;
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].z = 0;
	}
	else if	(( touch_status & TOUCH_RELEASE_MASK) == TOUCH_RELEASE_MASK) 
	{
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].state = RELEASE;
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].w = w;
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].z = 0;
	}
	else if((touch_status & TOUCH_PRESS_MASK) == TOUCH_PRESS_MASK) 
    {
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].state = PRESS;
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].w = w;
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].z = z;
	}
	else if ((touch_status & TOUCH_MOVE_MASK) == TOUCH_MOVE_MASK) 
    {
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].state = MOVE;
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].w = w;
		g_tp->msg[report_id - maxTouchCfg_T9_obj.reportid_ub].z = z;
	}
	else if ((touch_status & TOUCH_DETECT_MASK ) != 0x0)
	{
		
	}
	else
    {
    	ATMEL_PRINTK(0, "error touch_state = %x\n",touch_status);
    }

	return;
}

static void tp_report_coord_via_mt_protocol(void)
{
	int i;

	for(i=0;i<ATMEL_REPORT_POINTS;i++)
	{
		if (g_tp->msg[i].z == -1)
			continue;
		input_report_key(g_tp->input, BTN_TOUCH, 1);
		input_report_abs(g_tp->input, ABS_MT_TOUCH_MAJOR, g_tp->msg[i].z);
		input_report_abs(g_tp->input, ABS_MT_WIDTH_MAJOR, g_tp->msg[i].w);
		input_report_abs(g_tp->input, ABS_MT_POSITION_X, g_tp->msg[i].coord.x);
		input_report_abs(g_tp->input, ABS_MT_POSITION_Y, g_tp->msg[i].coord.y);
		input_report_abs(g_tp->input, ABS_MT_TRACKING_ID, i);
		input_report_abs(g_tp->input, ABS_MT_PRESSURE, g_tp->msg[i].z);
		ATMEL_TOUCH_PRINTK(1, "touch: id=%d, (x,y)=(%d,%d), (z,w)=(%d,%d)\n",
			i , g_tp->msg[i].coord.x, g_tp->msg[i].coord.y, g_tp->msg[i].z, g_tp->msg[i].w);
		input_mt_sync(g_tp->input);

		if (g_tp->msg[i].z == 0)
			g_tp->msg[i].z = -1;
	}
	input_sync(g_tp->input);
}

static void touchpad_report_coord(uint16_t x_coord, uint16_t y_coord, uint8_t touch_status,
	uint8_t report_id, uint8_t w, uint8_t z)
{

	save_multi_touch_struct(x_coord, y_coord, touch_status, report_id, w, z);
	tp_report_coord_via_mt_protocol();
	if ( report_id == maxTouchCfg_T9_obj.reportid_ub )
	{
		
	}
	return;
}


int T15_msg_handler (uint8_t *value)
{

	ATMEL_CAPKEY_PRINTK(1, "keyarray T15 report id:%d\n", value[0]);
	ATMEL_CAPKEY_PRINTK(1, "Keyarray T15 value[0]:%d value[1]:%d value[2]:%d value[3]:%d\n",
		value[0], value[1], value[2], value[3]);

	if (g_tp->em_key_flag == 1) {
		
		if(value[2] & 0x01) {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] MENU KEY is pressed\n",KEY_F2);
			input_report_key(g_tp->keyarray_input, KEY_F2, 1);
		} else {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] MENU KEY is released\n",KEY_F2);
			input_report_key(g_tp->keyarray_input, KEY_F2, 0);
		} if(value[2] & 0x02) {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] BACK KEY is pressed\n",KEY_F5);
			input_report_key(g_tp->keyarray_input, KEY_F5, 1);
		} else {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] BACK KEY is released\n",KEY_F5);
			input_report_key(g_tp->keyarray_input, KEY_F5, 0);
		} if(value[2] & 0x04) {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] SEARCH KEY is pressed\n",KEY_F6);
			input_report_key(g_tp->keyarray_input, KEY_F6, 1);
		} else {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] SEARCH KEY is released\n",KEY_F6);
			input_report_key(g_tp->keyarray_input, KEY_F6, 0);
		}
		if(value[2] & 0x08) {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] HOME KEY is pressed\n", KEY_F7);
			input_report_key(g_tp->keyarray_input, KEY_F7, 1);
		} else {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] HOME KEY is released\n", KEY_F7);
			input_report_key(g_tp->keyarray_input, KEY_F7, 0);
		}
	} else { 
		
		if(value[2] & 0x08) {
			ATMEL_CAPKEY_PRINTK(1, " k[%d] HOME KEY is pressed\n", KEY_HOME);
			input_report_key(g_tp->keyarray_input, KEY_HOME, 1);
		} else {
			ATMEL_CAPKEY_PRINTK(1,"k[%d] HOME KEY is released\n", KEY_HOME);
			input_report_key(g_tp->keyarray_input, KEY_HOME, 0);
		}
		if(value[2] & 0x04) {
			ATMEL_CAPKEY_PRINTK(1, "k[%d] SEARCH KEY is pressed\n", KEY_SEARCH);
			input_report_key(g_tp->keyarray_input, KEY_SEARCH, 1);
		} else {
			ATMEL_CAPKEY_PRINTK(1, "k[%d] SEARCH KEY is released\n", KEY_SEARCH);
			input_report_key(g_tp->keyarray_input, KEY_SEARCH, 0);
		}
		if(value[2] & 0x02) {
			ATMEL_CAPKEY_PRINTK(1, "k[%d] BACK KEY is pressed\n", KEY_BACK);
			input_report_key(g_tp->keyarray_input, KEY_BACK, 1);
		} else {
			ATMEL_CAPKEY_PRINTK(1, "k[%d] BACK KEY is released\n", KEY_BACK);
			input_report_key(g_tp->keyarray_input, KEY_BACK, 0);
		}
		if(value[2] & 0x01) {
			ATMEL_CAPKEY_PRINTK(1, "k[%d] MENU KEY is pressed\n", KEY_MENU);
			input_report_key(g_tp->keyarray_input, KEY_MENU, 1);
		} else {
			ATMEL_CAPKEY_PRINTK(1, "k[%d] MENU KEY is released\n", KEY_MENU);
			input_report_key(g_tp->keyarray_input, KEY_MENU, 0);
		}
	}	
	input_sync(g_tp->keyarray_input);
    return 0;
}


int T6_msg_handler (uint8_t *value)
{
	if (value[1] == 0x40) {

	} else if (value[1] == 0x10) {
		
		g_tp->touch_count = 0;
		
	} else if (value[1] == 0x90) {

	}
	else {

	}
    return 0;
}


void roll_back_to_normal_T8(void)
{
	int result = 0;
	
	uint8_t T8_value[4] = {0x5,0x0,0xF,0x81};
	
	
	result = touchpad_write_i2c(g_tp->client, maxTouchCfg_T8_obj.obj_addr+6, 
		&T8_value[0], 4);

}


int T9_msg_handler (uint8_t *value)
{
	uint16_t x_coord,y_coord;

    
    x_coord = value[2] << 2 | (value[4] & 0xc0) >> 6;
    y_coord = value[3] << 2 | (value[4] & 0x0c) >> 2;

    touchpad_report_coord(x_coord, y_coord, value[1], value[0], value[5], value[6]);
	
	if(g_tp->touch_count != -1) {
		g_tp->touch_count++;

	}	
	if(g_tp->touch_count >= TOTAL_TP_COUNT)
	{ 
		roll_back_to_normal_T8();
		g_tp->touch_count=-1;
 
	}
	
    return 0;
}


int T25_msg_handler ( uint8_t *value )
{

	if (g_tp->selftest_flag == 1)
		g_tp->selftest.value = value[1];

    return 0;
}

static irqreturn_t mxt224E_irq_thread(int irq, void *dev_id)

{
	struct i2c_client	*client = g_tp->client;
	struct touchpad_t *g_tp = dev_id;
	uint8_t value[maxTouchCfg_T5_obj.size];
	uint8_t i;


    mutex_lock(&g_tp->mutex);
	disable_irq_nosync(g_tp->irq);
    do {
		if ((tp_read_T5_via_i2c(client , &value[0], maxTouchCfg_T5_obj.size)) < 0 ) {
			enable_irq(g_tp->irq);
			mutex_unlock(&g_tp->mutex);
			return IRQ_HANDLED;
		}
    	
    	for (i = 0 ; i < g_tp->id_info.num_obj_element ; i++) {
			if (value[0] >= g_tp->init_obj_element[i]->reportid_ub && value[0] <=  g_tp->init_obj_element[i]->reportid_lb) {
				if(g_tp->init_obj_element[i]->msg_handler) {
					g_tp->init_obj_element[i]->msg_handler(value);
				}
				break;
			}
		}
    }while(gpio_get_value(g_tp->gpio_irq) == 0);
	enable_irq(g_tp->irq);
	mutex_unlock(&g_tp->mutex);
    return IRQ_HANDLED;
}

static int touchpad_power_on_device(struct touchpad_t *g_tp, int on)
{
	int rc = 0;
    static int prev_on = 0;

    if (on == prev_on) {
		return 0;
	}
	
	if(on) {
	
		g_tp->avdd_2_regulator = regulator_get(NULL, "emmc");
		if (IS_ERR(g_tp->avdd_2_regulator)) {
			rc = PTR_ERR(g_tp->avdd_2_regulator);
			pr_err("%s:regulator get avdd L10 voltage failed rc=%d\n",
							__func__, rc);
			rc = -ENODEV;
			goto exit;
		}

		
		g_tp->vdd_regulator = regulator_get(NULL, "msme1");
		if (IS_ERR(g_tp->vdd_regulator)) {
			pr_err("could not get vdd_touch, rc = %ld\n",
				PTR_ERR(g_tp->vdd_regulator));
			rc = -ENODEV;	
			goto get_vdd_fail;
		}

			rc = regulator_set_voltage(g_tp->avdd_2_regulator, 2800000, 2800000);
		if (rc) {
			ATMEL_PRINTK( 1, "TEST_INFO_LEVEL:" "set_voltage avdd_L10_touch failed, rc=%d\n", rc);
			goto set_avdd_fail;
		}

		rc = regulator_set_voltage(g_tp->vdd_regulator, 1800000, 1800000);
		if (rc) {
			printk("set_voltage vdd_touch failed, rc=%d\n", rc);
			goto set_vdd_fail;
		}


		rc = regulator_enable(g_tp->avdd_2_regulator);
		if (rc) {
			ATMEL_PRINTK( 1, "TEST_INFO_LEVEL:""enable avdd_L110_touch failed, rc=%d\n", rc);
			goto set_avdd_fail;
		}
		mdelay(5);

		rc = regulator_enable(g_tp->vdd_regulator);
		if (rc) {
			ATMEL_PRINTK( 1, "TEST_INFO_LEVEL:""enable  vdd_s3_touch failed, rc=%d\n", rc);
			goto set_vdd_fail;
		}
		mdelay(5);

		msleep(ATMEL_POR_DELAY);
		printk("%s %d avdd_touch & vdd_touch have been turn on\n",__func__,__LINE__);

		prev_on = on;
		return 0;	
	}
	else
	{
		printk("%s %d does not support turn onoff vdd&avdd\n",__func__,__LINE__);

		prev_on = on;
	}

set_vdd_fail:
	regulator_disable(g_tp->avdd_2_regulator);
set_avdd_fail:
	regulator_put(g_tp->vdd_regulator);
get_vdd_fail:
	regulator_put(g_tp->avdd_2_regulator);
exit:
	return rc;
}

static int touchpad_release_gpio(struct touchpad_t *g_tp)
{
    gpio_free(g_tp->gpio_irq);
    gpio_free(g_tp->gpio_rst);
    return 0;
}


static int touchpad_setup_gpio(struct touchpad_t *g_tp)
{
	int rc;

    
    rc = gpio_tlmm_config( GPIO_CFG(g_tp->gpio_irq, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);
    if(rc) {
		goto error_gpio_config;
    }

	
    rc = gpio_request(g_tp->gpio_irq, "mXT224E_ts_irq");
    if (rc) {
		goto error_gpio_config;
    }

	
    rc = gpio_direction_input(g_tp->gpio_irq);
    if (rc) {
 		goto error_gpio_config;
    }

    rc = gpio_tlmm_config( GPIO_CFG(g_tp->gpio_rst, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
                GPIO_CFG_ENABLE);
    if(rc) {

		goto error_gpio_config;
    }

	
    rc = gpio_request(g_tp->gpio_rst, "mXT224E_ts_rst");
	if (rc) {

		goto error_gpio_config;
    }

	
    rc = gpio_direction_output(g_tp->gpio_rst, 0);
    if ( rc ) {

        goto error_gpio_config;
    }
	return 0;

error_gpio_config:
	touchpad_release_gpio(g_tp);
	return rc;
}


static int touchpad_config_gpio(struct touchpad_t *g_tp)
{
	int rc = 0;

    
    gpio_set_value(g_tp->gpio_rst, 0);

    msleep(1);
	
    gpio_set_value(g_tp->gpio_rst, 1);

	msleep(100);
    return rc;
}


#define TOUCH_CHG_RETRY_COUNT 5
static int touchpad_check_chg_pin_low(struct touchpad_t *g_tp)
{
	int rc;
	int retryCnt = TOUCH_CHG_RETRY_COUNT;

	while(retryCnt) {
		rc = gpio_get_value(g_tp->gpio_irq);
		if(rc) {

			if(-ETIMEDOUT == rc) msleep(1);
            retryCnt--;
		}else {
            rc = 0;
            break;
        }
	}

	if((rc == 0) && (retryCnt < TOUCH_CHG_RETRY_COUNT)) {

	}
	return rc;
}


static int fillin_object_table(struct obj_t *obj, uint16_t addr,uint8_t *value )
{

	obj->obj_addr = value[1] | value[2] << 8;
	obj->size = value[3] + 1;
	obj->instance = value[4] + 1;
	obj->num_reportid = value[5];

	return 0;
}


static int fillin_init_obj_table(uint8_t match_type_id, struct obj_t *ptr_obj_struct, uint8_t *obj_value,
	unsigned char *value_ub, uint16_t obj_addr, uint8_t array_size, int *index)
{
	int j;
	int rc = 0;
	uint8_t read_type_id = obj_value[0];
	uint8_t num_of_reportid = obj_value[5];
	uint8_t num_of_instance = obj_value[4]+1;
	uint8_t reg_addr = obj_addr;


	if (read_type_id == match_type_id) {
		
    	for (j = 0; j< OBJECT_TABLE_ELEMENT_SIZE; j++) {

			reg_addr++;
    	}
    	fillin_object_table(ptr_obj_struct, obj_addr, obj_value);
    	
    	ptr_obj_struct->reportid_ub = *value_ub;
    	ptr_obj_struct->reportid_lb = *value_ub+(num_of_reportid*num_of_instance)-1;
    	*value_ub += (num_of_reportid*num_of_instance);
    	
		

		
    	if ((uint8_t)array_size == 0 || ptr_obj_struct->size == (uint8_t)array_size) {
       		g_tp->init_obj_element[*index] = ptr_obj_struct;
            *index += 1;
       	} else {
			rc = -EFAULT;

    		return rc;
       	}
    }
    return rc;
}


static int touchpad_config_mXT224E(struct touchpad_t *g_tp)
{
	int result;
	struct i2c_client *client = g_tp->client;
	uint8_t value;
	int i;
	uint16_t regbuf;


    value = T6_BACKUPNV_VALUE;
    for (i = 0 ; i < g_tp->id_info.num_obj_element ; i++)
    {
    	if(g_tp->init_obj_element[i]->value_array != NULL)
    	{
			result = touchpad_write_i2c(client,g_tp->init_obj_element[i]->obj_addr,g_tp->init_obj_element[i]->value_array,g_tp->init_obj_element[i]->size);
			ATMEL_PRINTK(1, "write reg_check 0x%x = 0x%x\n",
					g_tp->init_obj_element[i]->obj_addr, g_tp->init_obj_element[i]->value_array[0]);
    		if(result < 0)
    		{
    	  	 	ATMEL_PRINTK(0, "Unable to read reg_check 0x%x = 0x%x\n",
					g_tp->init_obj_element[i]->obj_addr, g_tp->init_obj_element[i]->value_array[0]);
    	   		return result;
    		}
    	}
    }

    
    regbuf = maxTouchCfg_T6_obj.obj_addr+1;
    result = touchpad_write_i2c(client,regbuf,&value,WRITE_T6_SIZE);

    if(result < 0)
    {

    	return result;
    }

    
    value = T6_RESET_VALUE;
    result = touchpad_write_i2c(client,maxTouchCfg_T6_obj.obj_addr,&value ,WRITE_T6_SIZE);

    if(result < 0)
    {

    	return result;
    }
	msleep(100);
    return result;
}

static int touchpad_detect_mXT224E(struct touchpad_t *g_tp)
{
	int result = 0;
    uint8_t value[NUM_OF_ID_INFO_BLOCK];
    uint16_t regbuf,obj_addr;
    int t = 0;
    uint8_t reportid_ub = 1;

	
    result = touchpad_read_i2c(g_tp->client, ADD_ID_INFO_BLOCK, &value[0], NUM_OF_ID_INFO_BLOCK);
    if(result < 0 || !(value[0] == 0x81 && value[2] == 0x10 &&
		(value[3] == 0xAA || value[3] == 0xAB))) {
   		return result;
    }

    
    g_tp->id_info.family_id = value[0];
    g_tp->id_info.variant_id = value[1];
    g_tp->id_info.version = value[2];
    g_tp->id_info.build = value[3];
    g_tp->id_info.matrix_x_size = value[4];
    g_tp->id_info.matrix_y_size = value[5];
    g_tp->id_info.num_obj_element = value[6];

	
    for (obj_addr = OBJECT_TABLE_ELEMENT_1 ; obj_addr < OBJECT_TABLE_ELEMENT_1+OBJECT_TABLE_ELEMENT_SIZE*g_tp->id_info.num_obj_element;
		obj_addr += OBJECT_TABLE_ELEMENT_SIZE) {
    	result = touchpad_read_i2c(g_tp->client, obj_addr, &value[0], OBJECT_TABLE_ELEMENT_SIZE);

    	if (t < MAX_OBJ_ELEMENT_SIZE) {
        	
        	fillin_init_obj_table(GEN_MESSAGEPROCESOR_T5, &maxTouchCfg_T5_obj, value, &reportid_ub,
        	                       	obj_addr, 0, &t);
        	
        	fillin_init_obj_table(GEN_COMMANDPROCESSOR_T6, &maxTouchCfg_T6_obj, value, &reportid_ub,
        	                        obj_addr, 0, &t);
			
        	fillin_init_obj_table(SPT_USERDATA_T38, &maxTouchCfg_T38_obj, value, &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T38_CMI_V10), &t);
        	
        	fillin_init_obj_table(GEN_POWERCONFIG_T7, &maxTouchCfg_T7_obj, value, &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T7_CMI_V10), &t);
			
			fillin_init_obj_table(GEN_ACQUISITIONCONFIG_T8, &maxTouchCfg_T8_obj, value, &reportid_ub,
										obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T8_CMI_V10), &t);
			
			fillin_init_obj_table(TOUCH_MULTITOUCHSCREEN_T9, &maxTouchCfg_T9_obj, value, &reportid_ub,
											obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T9_CMI_V10), &t);
        	
        	fillin_init_obj_table(TOUCH_KEYARRAY_T15, &maxTouchCfg_T15_obj, value, &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T15_CMI_V10), &t);
        	
        	fillin_init_obj_table(SPT_COMMSCONFIG_T18, &maxTouchCfg_T18_obj, value,  &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T18_CMI_V10), &t);
        	
        	fillin_init_obj_table(SPT_GPIOPWM_T19, &maxTouchCfg_T19_obj, value, &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T19_CMI_V10), &t);
        	
        	fillin_init_obj_table(TOUCH_PROXIMITY_T23, &maxTouchCfg_T23_obj, value, &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T23_CMI_V10), &t);
        	
        	fillin_init_obj_table(SPT_SELFTEST_T25, &maxTouchCfg_T25_obj, value, &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T25_CMI_V10), &t);
			
        	fillin_init_obj_table(DEBUG_DIAGNOSTIC_T37, &maxTouchCfg_T37_obj, value, &reportid_ub,
        	                        obj_addr, 0, &t );
			
        	fillin_init_obj_table(SPT_MESSAGECOUNT_T44, &maxTouchCfg_T44_obj, value, &reportid_ub,
        		                    obj_addr, 0, &t);
        	
        	fillin_init_obj_table(PROCI_GRIPSUPPRESSION_T40, &maxTouchCfg_T40_obj, value, &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T40_CMI_V10), &t);
        	
        	fillin_init_obj_table(PROCI_TOUCHSUPPRESSION_T42, &maxTouchCfg_T42_obj, value, &reportid_ub,
        		                    obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T42_CMI_V10), &t);
        	
        	fillin_init_obj_table(SPT_CTECONFIG_T46, &maxTouchCfg_T46_obj, value, &reportid_ub,
        	                        obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T46_CMI_V10), &t);
			
			fillin_init_obj_table(PROCI_STYLUS_T47, &maxTouchCfg_T47_obj, value, &reportid_ub,
        	                        obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T47_CMI_V10), &t);
			
			fillin_init_obj_table(PROCI_NOISESUPPRESSION_T48, &maxTouchCfg_T48_obj, value, &reportid_ub,
        	                        obj_addr, (uint8_t)ARRAY_SIZE(maxTouchCfg_T48_CMI_V10), &t);
        }

	}
	
    return 0;
}

static int power_on_flow_another_way(struct touchpad_t *g_tp)
{
	int result = 0;
	int obj_size = (int)(maxTouchCfg_T5_obj.size);
	uint8_t value[obj_size];
	uint16_t regbuf;
	int i;
	int j = 0;

	
	result = touchpad_config_mXT224E(g_tp);
	if(result < 0)
    {

        return result;
    }

	
	result = touchpad_check_chg_pin_low(g_tp);
    if(result)
    {

		return result;
    }

    for (;;)
	{
		
		touchpad_read_i2c(g_tp->client, maxTouchCfg_T5_obj.obj_addr, &value[0],obj_size);
    	i = 0;
    	
    	for (regbuf = maxTouchCfg_T5_obj.obj_addr; regbuf < maxTouchCfg_T5_obj.obj_addr+maxTouchCfg_T5_obj.size; regbuf++)
    	{

    		i++;
    	}
    	
    	if(value[0] >= maxTouchCfg_T6_obj.reportid_ub && value[0] <= maxTouchCfg_T6_obj.reportid_lb)
    	{
    		
    		if((value[1] & CFGERR_MASK) != 0x0) {
    			ATMEL_PRINTK(0, "Failed to CFGERR bit:0x%x\n", value[1]);
    			return -EFAULT;
        	}
        	else {
        		ATMEL_PRINTK(2, "check CFGERR:0x%x\n", value[1]);
    			g_tp->T6_config_checksum = value[2] | value[3]<< 8 | value[4]<<16;
    			ATMEL_PRINTK(2, "T6 config checksum:0x%x\n", g_tp->T6_config_checksum);
    			result = calculate_config_crc(g_tp);
    			if (result) {

    				return -EFAULT;
    			}
    			break;
        	}
    	}
    	else {
    		if (j > 10) {

    			return -EFAULT;
    		}
    		j++;
    	}
	}
    return 0;
}


static int power_on_flow_one_way(struct touchpad_t *g_tp)
{
	int result;
	int obj_size = (int)( maxTouchCfg_T5_obj.size );
	uint8_t value[obj_size];
	uint16_t regbuf;
	int i;
	int j = 0;

	ATMEL_PRINTK(2, "read obj size:%d\n", obj_size)
	for (;;)
	{
		
		touchpad_read_i2c(g_tp->client, maxTouchCfg_T5_obj.obj_addr, &value[0],obj_size);
    	i = 0;

		
    	for (regbuf = maxTouchCfg_T5_obj.obj_addr; regbuf < maxTouchCfg_T5_obj.obj_addr+maxTouchCfg_T5_obj.size; regbuf++)
    	{

    		i++;
    	}
    	
    	if(value[0] >= maxTouchCfg_T6_obj.reportid_ub && value[0] <= maxTouchCfg_T6_obj.reportid_lb)
    	{
    		
    		if((value[1] & CFGERR_MASK) != 0x0)
    		{
    			ATMEL_PRINTK(0, "Failed to  CFGERR bit:0x%x\n", value[1]);
        		result = power_on_flow_another_way(g_tp);
        		if (result)
    			{
    				ATMEL_PRINTK(0, "Failed to power on flow another way (result=%d)\n", result);
    				return result;
    			}
    			return result;
        	}
        	else
        	{
        		ATMEL_PRINTK(1, "check CFGERR:0x%x\n", value[1]);
    			g_tp->T6_config_checksum = value[2] | value[3]<< 8 | value[4]<<16;
    			ATMEL_PRINTK(1, "T6 config checksum:0x%x\n",g_tp->T6_config_checksum);
    			result = calculate_config_crc(g_tp);
    			if (result)
    			{
    				ATMEL_PRINTK(0, "Failed to calculate config crc (result=%d)\n", result);
    				result = power_on_flow_another_way(g_tp);
    				if (result)
    				{
						ATMEL_PRINTK(0, "Failed to power on flow another way (result=%d)\n", result);
    				    return result;
    				}
    				return result;
    			}
    			break;
        	}
    	}
    	else
    	{
    		if (j > 10)
    		{
    			ATMEL_PRINTK(0, "Failed to read T6 message\n");
    			return -EFAULT;
    		}
    		j++;
    	}
	}
    return 0;
}


static uint16_t get_ref_value(int x,int y,uint8_t data_buffer[][SIZE_OF_REF_MODE_PAGE])
{
	int page_idx,element_idx;

	page_idx = (x*y_channel+y)*2 / (SIZE_OF_REF_MODE_PAGE-2);
	element_idx = ((x*y_channel+y)*2 % (SIZE_OF_REF_MODE_PAGE-2))+2;
	if(page_idx >= NUM_OF_REF_MODE_PAGE || element_idx-1 >= SIZE_OF_REF_MODE_PAGE) {
		ATMEL_PRINTK( 0,"ERROR_LEVEL:" "page_idx[%d] or elemenet_idx[%d] is not correct\n",page_idx,element_idx );
		return 0;
	}
	else
		return (uint16_t)(data_buffer[page_idx][element_idx] | data_buffer[page_idx][element_idx+1] << 8);
}


static int read_T37(struct touchpad_t *g_tp, uint8_t data_buffer[][SIZE_OF_REF_MODE_PAGE],
	uint8_t command)
{
	struct i2c_client *client = g_tp->client;
	int 		result = 0;
	uint8_t 	try_ctr;
	uint8_t 	value;
	uint16_t	addr_T6;
	uint16_t 	addr_T37;
	int i;

	
	value = command; 
	addr_T6 = maxTouchCfg_T6_obj.obj_addr+5;
	result = touchpad_write_i2c(client, addr_T6, &value, 1);
	if (result)
	{
		ATMEL_PRINTK(0, "failed to write References Mode!\n");
        result = -EFAULT;
		return result;
	}

	for(i=0;i<NUM_OF_REF_MODE_PAGE;i++)
		memset(data_buffer[i], 0xFF, SIZE_OF_REF_MODE_PAGE);

	for(i=0; i<NUM_OF_REF_MODE_PAGE; i++)
	{
		
		addr_T37 = maxTouchCfg_T37_obj.obj_addr;
		try_ctr = 0;
		while(!((data_buffer[i][0] == command) && (data_buffer[i][1] == i)))
		{
			
			if(try_ctr > 100) {
				
				ATMEL_PRINTK(0, "failed to read T37!\n");
				result = -EFAULT;
				return result;
			}
			msleep(5);
			try_ctr++; 
			result = touchpad_read_i2c( client, addr_T37, &data_buffer[i][0], 2);
			if (result) {
				ATMEL_PRINTK( 0,"ERROR_LEVEL:" "failed to read T37!\n" );
				result = -EFAULT;
				return result;
			}
			ATMEL_PRINTK(0, "Mode = 0x%x\n",data_buffer[i][0] );
			ATMEL_PRINTK(0 , "Page = 0x%x\n",data_buffer[i][1] );
		}

		result = touchpad_read_i2c( client, addr_T37, &data_buffer[i][0], SIZE_OF_REF_MODE_PAGE);
		if (result) {
			ATMEL_PRINTK(0, "failed to read T37!\n");
			result = -EFAULT;
			return result;
		}

		if(i != NUM_OF_REF_MODE_PAGE -1) {
			
			value = 0x01;
			result = touchpad_write_i2c(client, addr_T6, &value, 1);
			if (result) {
				ATMEL_PRINTK(0, "failed to write page up\n");
				result = -EFAULT;
				return result;
			}
		}
	}

    return result;
}


static int touchpad_selftest_fvs(struct touchpad_t *g_tp)
{
	int  result = 0;
	struct i2c_client *client = g_tp->client;
	uint8_t value;

	if (g_tp->selftest_flag == 1) {
		value = 0xFE; 
		result = touchpad_write_i2c(client, maxTouchCfg_T25_obj.obj_addr+1, &value, 1);
		if (result) {
			ATMEL_PRINTK(0, "%s failed to write self test!\n", __func__);
			result = -EFAULT;
			return result;
		}
		msleep(2000);
	} else {
		
		value = maxTouchCfg_T25_obj.value_array[1];
		ATMEL_PRINTK(0, "%s read maxTouchCfg_T25_obj.value_array=%x\n", __func__, maxTouchCfg_T25_obj.value_array[1]);
		result = touchpad_write_i2c(client, maxTouchCfg_T25_obj.obj_addr+1, &value, 1);
		if (result) {
			ATMEL_PRINTK(0, "%s failed to write self test!\n", __func__);
			result = -EFAULT;
			return result;
		}
	}

    return result;

}


static long tp_misc_ioctl( struct file *fp,
                           unsigned int cmd,
                           unsigned long arg )
{
	struct i2c_client *client = g_tp->client;
	struct atmel_power_T7_t power_mode;
	struct atmel_acquisition_T8_t acquisition;
	struct atmel_multitouchscreen_T9_t multitouchscreen;
	struct atmel_keyarray_T15_t keyarray;
	struct atmel_grip_suppression_T40_t grip_suppression;
	struct atmel_touch_suppression_T42_t touch_suppression;
	struct atmel_cte_T46_t cte;
	struct atmel_stylus_T47_t stylus;
	struct atmel_noise_suppression_T48_t noise_suppression;
	struct atmel_power_switch_t power_switch;
	struct atmel_references_mode_t references_mode;
	struct atmel_deltas_mode_t deltas_mode;
	struct atmel_selftest_t local_selftest;
	struct atmel_EM_key_delta_t key_delta;

	int  result = 0;
	int i = 0, j = 0;
	uint8_t *pData = NULL;
    uint    length = 0;
	uint8_t value[54];
	uint8_t value_T9[35];
	uint8_t get_value_T9[33];
	uint8_t value_T46[9];
	uint8_t get_value_T46[8];
	uint8_t value_T48[39];
	uint8_t get_value_T48[36];
	uint8_t data_buffer[NUM_OF_REF_MODE_PAGE][SIZE_OF_REF_MODE_PAGE];
	uint16_t addr;
	uint8_t bkp_value_T6;
	uint16_t regbuf;

	ATMEL_PRINTK(1, "cmd number=%d\n", _IOC_NR(cmd));
	memset(value,0,54);
	memset(value_T9,0,35);
	memset(get_value_T9,0,33);
	memset(value_T46,0,9);
	memset(get_value_T46,0,8);
	memset(value_T48,0,39);
	memset(get_value_T48,0,36);
	memset(data_buffer,0,NUM_OF_REF_MODE_PAGE);
	switch(cmd)
    {
		case ATMEL_TOUCH_SET_POWER_MODE:
            pData = (void *)&power_mode;
            length = sizeof(power_mode);
            if( copy_from_user( (void *)pData,
                                (void *)arg,
                                length) ) {
				ATMEL_PRINTK(0, "copy power mode from user failed\n");
                goto error_ioctl;
            } else {
                g_tp->T7_power_mode_value[0] = power_mode.idleacqint;
				g_tp->T7_power_mode_value[1] = power_mode.actvacqint;
				g_tp->T7_power_mode_value[2] = power_mode.actv2idleto;
				result = touchpad_write_i2c(client, maxTouchCfg_T7_obj.obj_addr, &g_tp->T7_power_mode_value[0], length);
				if (result) {
					ATMEL_PRINTK(0, "failed to write power mode\n");
					goto error_ioctl;
				}
				
				for (i = 0 ; i < length; i++)
					ATMEL_PRINTK(1, "Set power mode value[%d]:%x\n", i, g_tp->T7_power_mode_value[i]);
            }
			
			g_tp->backup_emlist_flag = 1;
			
            break;
		case ATMEL_TOUCH_GET_POWER_MODE:
            length = sizeof(power_mode);
            result = touchpad_read_i2c(client, maxTouchCfg_T7_obj.obj_addr, &g_tp->T7_power_mode_value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "failed to read power mode_T7\n");
               goto error_ioctl;
			}
			result = copy_to_user((void *)arg, &g_tp->T7_power_mode_value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy power mode_T7 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < 3; i++)
				ATMEL_PRINTK(1, "Get power mode_T7 value[%d]:%x\n", i, g_tp->T7_power_mode_value[i]);
            break;
		case ATMEL_TOUCH_SET_ACQUISITION:
            pData = (void *)&acquisition;
            length = sizeof(acquisition);
            if( copy_from_user( (void *)pData,
                                (void *)arg,
                                length) ) {
				ATMEL_PRINTK(0, "copy acquisition_T8 from user failed\n");
				goto error_ioctl;
            } else {
				value[0] = acquisition.chrgtime;
				value[1] = acquisition.tchdrift;
				value[2] = acquisition.driftst;
				value[3] = acquisition.tchautocal;
				value[4] = acquisition.sync;
				value[5] = acquisition.atchcalst;
				value[6] = acquisition.atchcalsthr;
				value[7] = acquisition.atchfrccalthr;
				value[8] = acquisition.atchfrccalratio;
				result = touchpad_write_i2c(client, maxTouchCfg_T8_obj.obj_addr, &value[0], 1);
				if (result) {
					ATMEL_PRINTK(0, "failed to write Acquisition_T8\n");
					goto error_ioctl;
				}
				result = touchpad_write_i2c(client, maxTouchCfg_T8_obj.obj_addr+2, &value[1], 8);
				if (result) {
					ATMEL_PRINTK(0, "failed to write Acquisition_T8\n");
					goto error_ioctl;
				}
				
				for (i = 0 ; i < length; i++)
					ATMEL_PRINTK(1, "Set Acquisition_T8 value[%d]:%x\n", i, value[i]);
			}
			
			g_tp->backup_emlist_flag = 1;
			
            break;
        case ATMEL_TOUCH_GET_ACQUISITION:
            length = sizeof(acquisition);
            result = touchpad_read_i2c(client, maxTouchCfg_T8_obj.obj_addr, &value[0], 1);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Acquisition_T8\n");
                goto error_ioctl;
			}
			result = touchpad_read_i2c(client, maxTouchCfg_T8_obj.obj_addr+2, &value[1], 8);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Acquisition_T8\n");
                goto error_ioctl;
			}
			result = copy_to_user((void *)arg, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy Acquisition_T8 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < 9; i++)
				ATMEL_PRINTK(1, "Get Acquisition_T8 value[%d]:%x\n", i, value[i]);
            break;
		case ATMEL_TOUCH_SET_MULTITOUCHSCREEN:
            pData = (void *)&multitouchscreen;
            length = sizeof(multitouchscreen);
            if( copy_from_user( (void *)pData,
                                (void *)arg,
                                length) ) {
                ATMEL_PRINTK(0, "copy Multitouchscreen_T9 from user failed\n");
                goto error_ioctl;
            } else {
                value_T9[0] = multitouchscreen.ctrl;
				value_T9[1] = multitouchscreen.xorigin;
				value_T9[2] = multitouchscreen.yorigin;
				value_T9[3] = multitouchscreen.xsize;
				value_T9[4] = multitouchscreen.ysize;
				value_T9[5] = multitouchscreen.akscfg;
				value_T9[6] = multitouchscreen.blen;
				value_T9[7] = multitouchscreen.tchthr;
				value_T9[8] = multitouchscreen.tchdi;
				value_T9[9] = multitouchscreen.orient;
				value_T9[10] = multitouchscreen.mrgtimeout;
				value_T9[11] = multitouchscreen.movhysti;
				value_T9[12] = multitouchscreen.movhystn;
				value_T9[13] = multitouchscreen.movfilter;
				value_T9[14] = multitouchscreen.numtouch;
				value_T9[15] = multitouchscreen.mrghyst;
				value_T9[16] = multitouchscreen.mrgthr;
				value_T9[17] = multitouchscreen.amphyst;
				value_T9[18] = multitouchscreen.xrange & 0xFF00 >> 8;
				value_T9[19] = multitouchscreen.xrange & 0x00FF;
				value_T9[20] = multitouchscreen.yrange & 0xFF00 >> 8;
				value_T9[21] = multitouchscreen.yrange & 0x00FF;
				value_T9[22] = multitouchscreen.xloclip;
				value_T9[23] = multitouchscreen.xhiclip;
				value_T9[24] = multitouchscreen.yloclip;
				value_T9[25] = multitouchscreen.yhiclip;
				value_T9[26] = multitouchscreen.xedgectrl;
				value_T9[27] = multitouchscreen.xedgedist;
				value_T9[28] = multitouchscreen.yedgectrl;
				value_T9[29] = multitouchscreen.yedgedist;
				value_T9[30] = multitouchscreen.jumplimit;
				value_T9[31] = multitouchscreen.tchhyst;
				value_T9[32] = multitouchscreen.xpitch;
				value_T9[33] = multitouchscreen.ypitch;
				value_T9[34] = multitouchscreen.nexttchdi;
				result = touchpad_write_i2c(client, maxTouchCfg_T9_obj.obj_addr, &value_T9[0], length+2);
				if (result) {
					ATMEL_PRINTK(0, "failed to write Multitouchscreen_T9\n");
					goto error_ioctl;
				}
            }
			
			g_tp->backup_emlist_flag = 1;
			
            break;
        case ATMEL_TOUCH_GET_MULTITOUCHSCREEN:
            length = sizeof(multitouchscreen);
            result = touchpad_read_i2c(client, maxTouchCfg_T9_obj.obj_addr, &value_T9[0], length+2);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Multitouchscreen_T9\n");
                goto error_ioctl;
			}
			for (i = 0; i < length; i++) {
				if (i < 18) {
					get_value_T9[i] = value_T9[i];
				} else if (i == 18) {
					get_value_T9[i]  = value_T9[i] | value_T9[i+1] << 8;
				} else if (i == 19) {
					get_value_T9[i]  = value_T9[i+1] | value_T9[i+2] << 8;
				} else if (i >= 20) {
					get_value_T9[i] = value_T9[i+2];
				}
			}
			result = copy_to_user((void *)arg, &get_value_T9[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy Multitouchscreen_T9 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < length; i++)
				ATMEL_PRINTK(1, "Get Multitouchscreen_T9 get_value_T9[%d]:%x\n", i, get_value_T9[i]);
            break;
		case ATMEL_TOUCH_SET_KEYARRAY:
            pData = (void *)&keyarray;
            length = sizeof(keyarray);
            if(copy_from_user( (void *)pData,
                                (void *)arg,
                                length)) {
                ATMEL_PRINTK(0, "copy Keyarray_T15 from user failed\n");
				goto error_ioctl;
            } else {
                value[0] = keyarray.ctrl;
				value[1] = keyarray.xorigin;
				value[2] = keyarray.yorigin;
				value[3] = keyarray.xsize;
				value[4] = keyarray.ysize;
				value[5] = keyarray.akscfg;
				value[6] = keyarray.blen;
				value[7] = keyarray.tchthr;
				value[8] = keyarray.tchdi;
				result = touchpad_write_i2c(client, maxTouchCfg_T15_obj.obj_addr, &value[0], length);
				if (result) {
					ATMEL_PRINTK(0, "failed to write keyarray_T15\n");
					goto error_ioctl;
				}
            }
			
			g_tp->backup_emlist_flag = 1;
			
            break;
        case ATMEL_TOUCH_GET_KEYARRAY:
            length = sizeof(keyarray);
            result = touchpad_read_i2c(client, maxTouchCfg_T15_obj.obj_addr, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Keyarray_T15\n");
				goto error_ioctl;
			}
			result = copy_to_user((void *)arg, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy get Keyarray_T15 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < length; i++)
				ATMEL_PRINTK(1, "Get Keyarray_T15 value[%d]:%x\n", i, value[i]);
            break;
		case ATMEL_TOUCH_SET_GRIPSUPPRESSION:
            pData = (void *)&grip_suppression;
            length = sizeof(grip_suppression);
            if( copy_from_user( (void *)pData,
                                (void *)arg,
                                length) ) {
                ATMEL_PRINTK(0, "copy grip_suppression_T40 from user failed\n");
				goto error_ioctl;
            } else {
                value[0] = grip_suppression.ctrl;
				value[1] = grip_suppression.xlogrip;
				value[2] = grip_suppression.xhigrip;
				value[3] = grip_suppression.ylogrip;
				value[4] = grip_suppression.yhigrip;
				result = touchpad_write_i2c(client, maxTouchCfg_T40_obj.obj_addr, &value[0], length);
				if (result) {
					ATMEL_PRINTK(0, "failed to write GRIP_SUPPRESSION_T40\n");
					goto error_ioctl;
				}
            }
			
			g_tp->backup_emlist_flag = 1;
			
            break;
        case ATMEL_TOUCH_GET_GRIPSUPPRESSION:
            length = sizeof(grip_suppression);
            result = touchpad_read_i2c(client, maxTouchCfg_T40_obj.obj_addr, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "failed to read GRIP_SUPPRESSION_T40\n");
                goto error_ioctl;
			}
			result = copy_to_user((void *)arg, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy GRIP_SUPPRESSION_T40 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < length; i++)
				ATMEL_PRINTK(1, "Get GRIP_SUPPRESSION_T40 value[%d]:%x\n", i, value[i]);
            break;
		case ATMEL_TOUCH_SET_TOUCHSUPPRESSION:
			pData = (void *)&touch_suppression;
            length = sizeof(touch_suppression);
            if( copy_from_user( (void *)pData,
                                (void *)arg,
                                length) ) {
                ATMEL_PRINTK(0, "copy touch_suppression_T42 from user failed\n");
				goto error_ioctl;
            } else {
                value[0] = touch_suppression.ctrl;
				value[1] = touch_suppression.apprthr;
				value[2] = touch_suppression.maxapprarea;
				value[3] = touch_suppression.maxtcharea;
				value[4] = touch_suppression.supstrength;
				value[5] = touch_suppression.supextto;
				value[6] = touch_suppression.maxnumtchs;
				value[7] = touch_suppression.shapestrength ;
				value[7] = touch_suppression.shapestrength;
				result = touchpad_write_i2c(client, maxTouchCfg_T42_obj.obj_addr, &value[0], length);
				if (result) {
					ATMEL_PRINTK(0, "failed to write touch_suppression_T42\n");
					goto error_ioctl;
				}
            }
			
			g_tp->backup_emlist_flag = 1;
			
            break;
		case ATMEL_TOUCH_GET_TOUCHSUPPRESSION:
			length = sizeof(touch_suppression);
            result = touchpad_read_i2c(client, maxTouchCfg_T42_obj.obj_addr, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "failed to read TOUCH_SUPPRESSION_T42\n");
                goto error_ioctl;
			}
			result = copy_to_user((void *)arg, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy TOUCH_SUPPRESSION_T42 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < length; i++)
				ATMEL_PRINTK(1, "Get TOUCH_SUPPRESSION_T42 value[%d]:%x\n", i, value[i]);
            break;
		case ATMEL_TOUCH_SET_CTE:
			pData = (void *)&cte;
            length = sizeof(cte);
            if( copy_from_user( (void *)pData,
                                (void *)arg,
                                length) ) {
                ATMEL_PRINTK(0, "copy CTE_MODE_T46 from user failed\n");
				goto error_ioctl;
            } else {
                value_T46[0] = cte.ctrl;
				value_T46[1] = cte.mode;
				value_T46[2] = cte.idlesyncsperx;
				value_T46[3] = cte.actvsyncsperx;
				value_T46[4] = cte.adcspersync;
				value_T46[5] = cte.pulsesperadc;
				value_T46[6] = cte.xslew;
				value_T46[7] = cte.syncdelay & 0xFF00 >> 8;
				value_T46[8] = cte.syncdelay & 0x00FF;
				result = touchpad_write_i2c(client, maxTouchCfg_T46_obj.obj_addr, &value_T46[0], length+1);
				if (result) {
					ATMEL_PRINTK(0, "failed to write CTE_MODE_T46\n");
					goto error_ioctl;
				}
            }
			
			g_tp->backup_emlist_flag = 1;
			
            break;
		case ATMEL_TOUCH_GET_CTE:
			length = sizeof(cte);
            result = touchpad_read_i2c(client, maxTouchCfg_T46_obj.obj_addr, &value_T46[0], length+1);
			if (result) {
				ATMEL_PRINTK(0, "failed to read CTE_MODE_T46\n");
                goto error_ioctl;
			}
			for (i = 0; i < length; i++) {
				if(i < 7) {
					get_value_T46[i] = value_T46[i];
				} else {
					get_value_T46[i] = value_T46[i] | value_T46[i+1] << 8;
				}
			}
			result = copy_to_user((void *)arg, &get_value_T46[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy CTE_MODE_T46 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < length; i++)
				ATMEL_PRINTK(1, "Get CTE_MODE_T46 value[%d]:%x\n", i, get_value_T46[i]);
            break;
		case ATMEL_TOUCH_SET_STYLUS:
			pData = (void *)&stylus;
            length = sizeof(stylus);
            if( copy_from_user( (void *)pData,
                                (void *)arg,
                                length) ) {
                ATMEL_PRINTK(0, "copy STYLUS_T47 from user failed\n");
				goto error_ioctl;
            } else {
                value[0] = stylus.ctrl;
				value[1] = stylus.contmin;
				value[2] = stylus.contmax;
				value[3] = stylus.stability;
				value[4] = stylus.maxtcharea;
				value[5] = stylus.amplthr;
				value[6] = stylus.styshape;
				value[7] = stylus.hoversup;
				value[8] = stylus.confthr;
				value[9] = stylus.syncsperx;
				result = touchpad_write_i2c(client, maxTouchCfg_T47_obj.obj_addr, &value[0], length);
				if (result) {
					ATMEL_PRINTK(0, "failed to write STYLUS_T47\n");
					goto error_ioctl;
				}
            }
			
			g_tp->backup_emlist_flag = 1;
			
            break;
		case ATMEL_TOUCH_GET_STYLUS:
			length = sizeof(stylus);
            result = touchpad_read_i2c(client, maxTouchCfg_T47_obj.obj_addr, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "failed to read STYLUS_T47\n");
                goto error_ioctl;
			}
			result = copy_to_user((void *)arg, &value[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy STYLUS_T47 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < length; i++)
				ATMEL_PRINTK(1, "Get STYLUS_T47 value[%d]:%x\n", i, value[i]);
            break;
		case ATMEL_TOUCH_SET_NOISESUPPRESSION:
			pData = (void *)&noise_suppression;
            length = sizeof(noise_suppression);
            if( copy_from_user( (void *)pData,
                                (void *)arg,
                                length) ) {
                ATMEL_PRINTK(0, "copy Noise_Suppression_T48 from user failed\n");
				goto error_ioctl;
            } else {
                value_T48[0] = noise_suppression.ctrl;
				value_T48[1] = noise_suppression.cfg;
				value_T48[2] = noise_suppression.calcfg;
				value_T48[3] = noise_suppression.basefreq;
				value_T48[4] = noise_suppression.mffreq_0;
				value_T48[5] = noise_suppression.mffreq_1;
				value_T48[6] = noise_suppression.gcactvinvldadcs;
				value_T48[7] = noise_suppression.gcidleinvldadcs;
				value_T48[8] = noise_suppression.gcmaxadcsperx;
				value_T48[9] = noise_suppression.gclimitmin;
				value_T48[10] = noise_suppression.gclimitmax;
				value_T48[11] = noise_suppression.gccountmintgt & 0xFF00 >> 8;
				value_T48[12] = noise_suppression.gccountmintgt & 0x00FF;
				value_T48[13] = noise_suppression.mfinvlddiffthr;
				value_T48[14] = noise_suppression.mfincadcspxthr & 0xFF00 >> 8;
				value_T48[15] = noise_suppression.mfincadcspxthr & 0x00FF;
				value_T48[16] = noise_suppression.mferrorthr & 0xFF00 >> 8;
				value_T48[17] = noise_suppression.mferrorthr & 0x00FF;
				value_T48[18] = noise_suppression.selfreqmax;
				value_T48[19] = noise_suppression.blen;
				value_T48[20] = noise_suppression.tchthr;
				value_T48[21] = noise_suppression.tchdi;
				value_T48[22] = noise_suppression.movhysti;
				value_T48[23] = noise_suppression.movhystn;
				value_T48[24] = noise_suppression.movfilter;
				value_T48[25] = noise_suppression.numtouch;
				value_T48[26] = noise_suppression.mrghyst;
				value_T48[27] = noise_suppression.mrgthr;
				value_T48[28] = noise_suppression.xloclip;
				value_T48[29] = noise_suppression.xhiclip;
				value_T48[30] = noise_suppression.yloclip;
				value_T48[31] = noise_suppression.yhiclip;
				value_T48[32] = noise_suppression.xedgectrl;
				value_T48[33] = noise_suppression.xedgedist;
				value_T48[34] = noise_suppression.yedgectrl;
				value_T48[35] = noise_suppression.yedgedist;
				value_T48[36] = noise_suppression.jumplimit;
				value_T48[37] = noise_suppression.tchhyst;
				value_T48[38] = noise_suppression.nexttchdi;
				result = touchpad_write_i2c(client, maxTouchCfg_T48_obj.obj_addr, &value_T48[0], 4);
				if (result) {
					ATMEL_PRINTK(0, "failed to write Noise_Suppression_T48\n");
					goto error_ioctl;
				}
				result = touchpad_write_i2c(client, maxTouchCfg_T48_obj.obj_addr+8, &value_T48[4], 2);
				if (result) {
					ATMEL_PRINTK(0, "failed to write Noise_Suppression_T48\n");
					goto error_ioctl;
				}
				result = touchpad_write_i2c(client, maxTouchCfg_T48_obj.obj_addr+13, &value_T48[6], 2);
				if (result) {
					ATMEL_PRINTK(0, "failed to write Noise_Suppression_T48\n");
					goto error_ioctl;
				}
				result = touchpad_write_i2c(client, maxTouchCfg_T48_obj.obj_addr+17, &value_T48[8], 11);
				if (result) {
					ATMEL_PRINTK(0, "failed to write Noise_Suppression_T48\n");
					goto error_ioctl;
				}
				result = touchpad_write_i2c(client, maxTouchCfg_T48_obj.obj_addr+34, &value_T48[19], 20);
				if (result) {
					ATMEL_PRINTK(0, "failed to write Noise_Suppression_T48\n");
					goto error_ioctl;
				}
            }
			
			g_tp->backup_emlist_flag = 1;
			
            break;
		case ATMEL_TOUCH_GET_NOISESUPPRESSION:
			length = sizeof(noise_suppression);
            result = touchpad_read_i2c(client, maxTouchCfg_T48_obj.obj_addr, &value_T48[0], 4);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Noise_Suppression_T48\n");
                goto error_ioctl;
			}
			result = touchpad_read_i2c(client, maxTouchCfg_T48_obj.obj_addr+8, &value_T48[4], 2);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Noise_Suppression_T48\n");
                goto error_ioctl;
			}
			result = touchpad_read_i2c(client, maxTouchCfg_T48_obj.obj_addr+13, &value_T48[6], 2);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Noise_Suppression_T48\n");
                goto error_ioctl;
			}
			result = touchpad_read_i2c(client, maxTouchCfg_T48_obj.obj_addr+17, &value_T48[8], 11);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Noise_Suppression_T48\n");
                goto error_ioctl;
			}
			result = touchpad_read_i2c(client, maxTouchCfg_T48_obj.obj_addr+34, &value_T48[19], 20);
			if (result) {
				ATMEL_PRINTK(0, "failed to read Noise_Suppression_T48\n");
                goto error_ioctl;
			}
			for (i = 0; i < length; i++) {
				if (i <= 10) {
					get_value_T48[i] = value_T48[i];
				} else if (i == 11) {
					get_value_T48[i] = value_T48[i] | value_T48[i+1] << 8;
				} else if (i == 12) {
					get_value_T48[i] = value_T48[i+1];
				} else if (i == 13) {
					get_value_T48[i] = value_T48[i+1] | value_T48[i+2] << 8;
				} else if (i == 14) {
					get_value_T48[i] = value_T48[i+2] | value_T48[i+3] << 8;
				} else if (i >= 15) {
					get_value_T48[i] = value_T48[i+3];
				}
			}
			result = copy_to_user((void *)arg, &get_value_T48[0], length);
			if (result) {
				ATMEL_PRINTK(0, "copy Noise_Suppression_T48 to user failed\n");
                goto error_ioctl;
			}
			
			for (i = 0 ; i < length; i++)
				ATMEL_PRINTK(1, "Get Noise_Suppression_T48 value[%d]:%x\n", i, get_value_T48[i]);
            break;
		case ATMEL_TOUCH_SET_POWER_SWITCH:
			if(copy_from_user( (void *)&power_switch,
                                (void *)arg,
                                sizeof(power_switch) ) ) {
                ATMEL_PRINTK(0, "copy POWER_SWITCH from user failed!\n");
				goto error_ioctl;
            }
            if(power_switch.on)
                tp_resume(&g_tp->touch_early_suspend);
            else
                tp_suspend(&g_tp->touch_early_suspend);
            break;
		case ATMEL_TOUCH_GET_REFERENCES_MODE:
            pData = (void *)&references_mode;
            length = sizeof(references_mode);
			
        	mutex_lock(&g_tp->mutex);
            read_T37(g_tp,data_buffer,0x11);
			mutex_unlock(&g_tp->mutex);

			for(i=0;i<x_channel;i++)
			{
				for(j=0;j<y_channel;j++)
				{
					references_mode.data[i][j] = (int16_t)get_ref_value(i,j,data_buffer);
					ATMEL_PRINTK(0, "x[%d]y[%d] = %d\n", i, j, references_mode.data[i][j]);
				}
			}
			result = copy_to_user( (void *)arg,pData, length );
			if (result) {
				ATMEL_PRINTK(0, "copy REFERENCES_MODE_T37 to user failed!\n");
				goto error_ioctl;
			}
            break;
		case ATMEL_TOUCH_GET_DELTAS_MODE:
			pData = (void *)&deltas_mode;
            length = sizeof(deltas_mode);
			
        	mutex_lock(&g_tp->mutex);
            read_T37(g_tp,data_buffer,0x10);
			mutex_unlock(&g_tp->mutex);
			for(i=0;i<x_channel;i++){
				for(j=0;j<y_channel;j++) {
					deltas_mode.deltas[i][j] = (int16_t)get_ref_value(i,j,data_buffer);
					ATMEL_PRINTK(1, "x[%d]y[%d] = %d\n",i, j, deltas_mode.deltas[i][j]);
				}
			}
			result = copy_to_user( (void *)arg,pData, length );
			if (result) {
				ATMEL_PRINTK(0, "copy ATMEL_TOUCH_IOCTL_GET_DELTAS_MODE to user failed!\n" );
				goto error_ioctl;
			}
            break;
		case ATMEL_TOUCH_GET_VERSION:
			addr = ADD_ID_INFO_BLOCK+2;
            result = touchpad_read_i2c(client, addr, &value[0], 1);
			if (result) {
				ATMEL_PRINTK(0, "failed to read mXT224E FW Version!\n");
                goto error_ioctl;
			}
			result = touchpad_read_i2c(client, ADD_ID_INFO_BLOCK, &value[1], 1);
			if (result) {
				ATMEL_PRINTK(0, "failed to read mXT224E chip ID!\n");
                goto error_ioctl;
			}
			result = copy_to_user( (void *)arg, &value[0], 2);
			if (result) {
				ATMEL_PRINTK(0, "copy mXT224E FW Version to user failed!\n");
                goto error_ioctl;
			}
			ATMEL_PRINTK(0, "mXT224E FW Version:0x%x\n", value[0]);
			ATMEL_PRINTK(0, "mXT224E chip ID:0x%x\n", value[1]);
            break;
		case ATMEL_TOUCH_SET_SELFTEST_FVS_MODE:
        {
            if( copy_from_user( (void *)&local_selftest,
                                (void *)arg,
                                sizeof(local_selftest) ) )
            {
               ATMEL_PRINTK(0, "copy SELFTEST from user failed!\n");
               goto error_ioctl;
            }
            if (local_selftest.on) {
				g_tp->selftest.value = 0;
                g_tp->selftest_flag = 1;
				touchpad_selftest_fvs(g_tp);
			} else {
                g_tp->selftest_flag = 0;
				touchpad_selftest_fvs(g_tp);
				result = copy_to_user((void *)arg, &g_tp->selftest, sizeof(g_tp->selftest));
				if (result) {
					ATMEL_PRINTK(0, "copy Selftest Result to user failed!\n" );
					goto error_ioctl;
				}
			}
            break;
        }
		case ATMEL_TOUCH_SET_EM_KEY_DELTA:
			if( copy_from_user( (void *)&key_delta,
                                (void *)arg,
                                sizeof(key_delta) ) )
            {
                ATMEL_PRINTK(0, "copy EM_KEY_DELTA from user failed!\n");
                goto error_ioctl;
            }
            if(key_delta.onoff)
                g_tp->em_key_flag = 1; 
            else
               g_tp->em_key_flag = 0; 
            break;
	} 

	
	if (g_tp->backup_emlist_flag == 1) {
		regbuf = maxTouchCfg_T6_obj.obj_addr+1;
		bkp_value_T6 = T6_BACKUPNV_VALUE;
		result = touchpad_write_i2c(client, regbuf, &bkp_value_T6, 1);
		ATMEL_PRINTK(1, "software backup cmd[0x%x]=0x%x\n", regbuf, bkp_value_T6);
		if(result < 0)
		{
			ATMEL_PRINTK(0, "Unable to write T6 backup cmd[0x%x]=0x%x\n", regbuf, bkp_value_T6);
			goto error_ioctl;
		}
		msleep(10);
#if 0		
		
		reset_value_T6 = 0x01;
		result = touchpad_write_i2c(client, maxTouchCfg_T6_obj.obj_addr, &reset_value_T6, 1);
		ATMEL_PRINTK(1, "software reset[0x%x]=0x%x\n",
			maxTouchCfg_T6_obj.obj_addr, reset_value_T6);
		if(result < 0)
		{
			ATMEL_PRINTK(0, "software reset[0x%x]= 0x%x failed \n", maxTouchCfg_T6_obj.obj_addr, reset_value_T6);
			goto error_ioctl;
		}
		msleep(100);
#endif		
	}
	

    return 0;

error_ioctl:
	result = -EFAULT;
	return result;
}

static int tp_misc_release(struct inode *inode, struct file *fp)
{
    
    ATMEL_PRINTK(1, "tp_misc_release\n");
	#if 0
    mutex_lock(&g_tp->mutex);

    if(g_tp->misc_open_count < 0) {
        g_tp->misc_open_count--;
		ATMEL_PRINTK(0, "still opened touch %d times\n", g_tp->misc_open_count);
    }
    mutex_unlock(&g_tp->mutex);
	#endif
    return 0;
}

static int tp_misc_open(struct inode *inode, struct file *fp)
{
    
    ATMEL_PRINTK(1, "tp_misc_open\n");
	#if 0
    mutex_lock(&g_tp->mutex);
    if(g_tp->misc_open_count <= 1) {
        g_tp->misc_open_count++;
        ATMEL_PRINTK(1, "opened touch %d times\n", g_tp->misc_open_count);
    } else {
		result = -EFAULT;
		ATMEL_PRINTK( 0, "failed to open misc count:%d\n", g_tp->misc_open_count);
    }
    mutex_unlock(&g_tp->mutex);
    #endif
    return 0;
}

static struct file_operations tp_misc_fops = {
	.owner 	= THIS_MODULE,
	.open 	= tp_misc_open,
	.release = tp_misc_release,
	
	
    .unlocked_ioctl = tp_misc_ioctl,
};

static struct miscdevice tp_misc_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "atmel_mXT224E_touch",
	.fops 	= &tp_misc_fops,
};


static void tp_suspend(struct early_suspend *h)
{
    int ret = 0;
    struct touchpad_t *tp = container_of( h,struct touchpad_t,touch_early_suspend);
    struct i2c_client *client = tp->client;
    int obj_size = 2;
	uint8_t value[maxTouchCfg_T7_obj.size];

    ATMEL_PRINTK(1, "tp_suspend\n");
	memset(value,0,maxTouchCfg_T7_obj.size);
	mutex_lock(&g_tp->mutex);
	if(g_tp->touch_suspended)
	{
		ATMEL_PRINTK(0, "in sleep state\n");
		mutex_unlock(&g_tp->mutex);
		return;
	}
    disable_irq(tp->irq);
	ATMEL_PRINTK(2, "disable irq %d\n", tp->irq);
    g_tp->touch_suspended = 1;
	
	mutex_unlock(&g_tp->mutex);
    ret = cancel_work_sync(&tp->touchpad_work.work);
	mutex_lock(&g_tp->mutex);
	
    if (ret) 
    {
		enable_irq(tp->irq);
	}
    
	
	input_report_key(g_tp->keyarray_input, KEY_HOME, 0);
	input_report_key(g_tp->keyarray_input, KEY_MENU, 0);
	input_report_key(g_tp->keyarray_input, KEY_BACK, 0);
	input_report_key(g_tp->keyarray_input, KEY_SEARCH, 0);

	
	
	value[0]= 0x0;
	value[1] = 0x0;
	ret = touchpad_write_i2c(client, maxTouchCfg_T7_obj.obj_addr, &value[0], obj_size);
	if (ret)
	{
		ATMEL_PRINTK(0, "cannot enter real sleep mode (ret=%d)\n", ret);
		mutex_unlock(&g_tp->mutex);
		return;
	}
	mutex_unlock(&g_tp->mutex);
	PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_TOUCH);

	return;
}


static void tp_resume(struct early_suspend *h)
{
    int result = 0;
    struct touchpad_t *tp = container_of( h,struct touchpad_t,touch_early_suspend);
	int obj_size = (int)( maxTouchCfg_T5_obj.size );
	uint8_t value[obj_size];
	uint8_t reset_value_T6;
	
	uint8_t T8_value[4] = {0x1,0x0,0x4,0x81};
	
	int i;

    ATMEL_PRINTK(1, "tp resume\n" );
	mutex_lock(&g_tp->mutex);
    if(0 == g_tp->touch_suspended)
	{
		mutex_unlock(&g_tp->mutex);
		return;
	}

    
	for (i = 0 ; i < ATMEL_REPORT_POINTS ; i++)
	{
		if (g_tp->msg[i].z == -1)
			continue;
		g_tp->msg[i].z = 0;
		g_tp->msg[i].state = RELEASE;
	}
	tp_report_coord_via_mt_protocol();

	if (g_tp->backup_emlist_flag == 0) {
		
		result = touchpad_config_gpio(g_tp);
		if(result) {
			ATMEL_PRINTK(0, "failed to config gpio\n");
			mutex_unlock(&g_tp->mutex);
			return;
		}
    } else {
		
		
		reset_value_T6 = 0x01;
		result = touchpad_write_i2c(tp->client, maxTouchCfg_T6_obj.obj_addr, &reset_value_T6, 1);
		ATMEL_PRINTK(1, "software reset[0x%x]=0x%x\n",
			maxTouchCfg_T6_obj.obj_addr, reset_value_T6);
		if(result < 0)
		{
			ATMEL_PRINTK(0, "software reset[0x%x]= 0x%x failed \n", maxTouchCfg_T6_obj.obj_addr, reset_value_T6);
			mutex_unlock(&g_tp->mutex);
			return;
		}
		msleep(100);
		
	}
	
	result = touchpad_check_chg_pin_low(g_tp);
	if(result)
	{
		ATMEL_PRINTK(0, "Failed to get CHG pin value:%d (result=%d)\n", g_tp->gpio_irq, result);
		mutex_unlock(&g_tp->mutex);
		return;
	}

	
	result = touchpad_read_i2c(tp->client, maxTouchCfg_T5_obj.obj_addr, &value[0],obj_size);
	if (result) {
		ATMEL_PRINTK(0, "Failed to read message_T5 adddr[0x%x]:0x%x (result=%d)\n",
			maxTouchCfg_T5_obj.obj_addr, value[0], result);
		mutex_unlock(&g_tp->mutex);
		return;
	}

    
    if(value[0] >= maxTouchCfg_T6_obj.reportid_ub && value[0] <= maxTouchCfg_T6_obj.reportid_lb)
    {
    	
    	if((value[1] & CFGERR_MASK) != 0x0)
    	{
    		ATMEL_PRINTK(0, "failed to CFGERR bit:0x%x\n", value[1]);
			mutex_unlock(&g_tp->mutex);
			return;
		}
    }
	else
		ATMEL_PRINTK(0, "report id is not 1, value:0x%x\n", value[0]);

	
	
    if (g_tp->backup_emlist_flag == 1) { 
		value[0] = g_tp->T7_power_mode_value[0];
		value[1] = g_tp->T7_power_mode_value[1];
		result = touchpad_write_i2c(tp->client, maxTouchCfg_T7_obj.obj_addr, &g_tp->T7_power_mode_value[0], 2);
		if (result) {
			ATMEL_PRINTK(0, "cannot read normal mode\n");
			mutex_unlock(&g_tp->mutex);
			return;
		}
	} else if (g_tp->backup_emlist_flag == 0) {
		value[0]= maxTouchCfg_T7_obj.value_array[0];
		value[1] = maxTouchCfg_T7_obj.value_array[1];
		result = touchpad_write_i2c(tp->client, maxTouchCfg_T7_obj.obj_addr, &value[0], 2);
		if (result) {
			ATMEL_PRINTK(0, "cannot read normal mode\n");
			mutex_unlock(&g_tp->mutex);
			return;
		}
    } else {
		ATMEL_PRINTK(0, "invalided command\n");
		mutex_unlock(&g_tp->mutex);
		return;
	}
	
	result = touchpad_write_i2c(g_tp->client, maxTouchCfg_T8_obj.obj_addr+6, &T8_value[0], 4);
	if (result) {
		ATMEL_PRINTK(0, "ERROR_LEVEL:" "failed to write recalibration parameters in T8 object!!!\n");
	}
    
	result = touchpad_read_i2c(g_tp->client, maxTouchCfg_T8_obj.obj_addr+6, &T8_value[0], 4);
	if (result) {
		ATMEL_PRINTK(0, "ERROR_LEVEL:" "failed to read recalibration parameters in T8 object!!!\n");
	}
    for (i = 0; i < 4; i++) {	
		ATMEL_PRINTK(1,"%s, T8_value[0x%x]=0x%x\n", __func__,maxTouchCfg_T8_obj.obj_addr+6+i,T8_value[i]);
    }
	
	enable_irq(tp->irq);
	g_tp->backup_emlist_flag = 0;
	g_tp->touch_suspended = 0;
	
	g_tp->touch_count = 0;
	
    mutex_unlock(&g_tp->mutex);
	PM_LOG_EVENT(PM_LOG_ON, PM_LOG_TOUCH);

	return;
}


static int touchpad_keyarray_event(struct input_dev *dev, unsigned int type,
             unsigned int code, int value)
{
  return 0;
}

static int touchpad_keyarray_open(struct input_dev *dev)
{
	int rc = 0;

	ATMEL_PRINTK(1, "open keyarray input class\n");
    mutex_lock(&g_tp->mutex);
    if(g_tp->keyarray_open_count == 0) {
        g_tp->keyarray_open_count++; 
        ATMEL_PRINTK(1, "keyarray opened %d times\n", g_tp->keyarray_open_count);
    }
    mutex_unlock(&g_tp->mutex);

    return rc;
}

static void touchpad_keyarray_close(struct input_dev *dev)
{

	ATMEL_PRINTK(1,"close keyarray input class\n");
	mutex_lock(&g_tp->mutex);
    if(g_tp->keyarray_open_count > 0) {
        g_tp->keyarray_open_count--;
        ATMEL_PRINTK(1, "still opened keyarray %d times\n", g_tp->keyarray_open_count);
    }
    mutex_unlock(&g_tp->mutex);

}

static int keyarray_register_input( struct input_dev **input,
                              struct i2c_client *client )
{
	int rc = 0;
	struct input_dev *input_dev;

	input_dev = input_allocate_device();
	if (!input_dev)
	{
		rc = -ENOMEM;
		return rc;
	}

	input_dev->name = ATMEL_KEYARRAY_NAME;
	input_dev->phys = "atmel_mXT224E_keyarray/event0";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &client->dev;
	input_dev->open = touchpad_keyarray_open;
	input_dev->close = touchpad_keyarray_close;
	input_dev->event = touchpad_keyarray_event;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);

	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_SEARCH, input_dev->keybit);
    set_bit(KEY_MENU, input_dev->keybit);
	
	
	set_bit(KEY_F2, input_dev->keybit);
	set_bit(KEY_F5, input_dev->keybit);
	set_bit(KEY_F6, input_dev->keybit);
	set_bit(KEY_F7, input_dev->keybit);

	ATMEL_PRINTK(1, "%s: Register input device\n", ATMEL_KEYARRAY_NAME);
	rc = input_register_device(input_dev);
	if (rc) {
		ATMEL_PRINTK(0, "%s: Failed to register keyarray input device\n", ATMEL_KEYARRAY_NAME);
		input_free_device( input_dev );
	}
	else {
		*input = input_dev;
	}

  return rc;
}

static int touchpad_open(struct input_dev *dev)
{
    int rc = 0;

    ATMEL_PRINTK(1, "open touch input class\n");
    mutex_lock(&g_tp->mutex);
    if(g_tp->open_count == 0)
    {
        g_tp->open_count++;
        ATMEL_PRINTK(1, "opened touch %d times\n", g_tp->open_count);
    }
    mutex_unlock(&g_tp->mutex);

    return rc;
}

static void touchpad_close(struct input_dev *dev)
{
    ATMEL_PRINTK(1, "close touch input class\n");
    mutex_lock(&g_tp->mutex);
    if(g_tp->open_count > 0)
    {
        g_tp->open_count--;
        ATMEL_PRINTK(1, "still opened touch %d times\n", g_tp->open_count);
    }
    mutex_unlock(&g_tp->mutex);

}


static int touchpad_register_input( struct input_dev **input,
                                    struct atmel_platform_data *pdata,
                                    struct i2c_client *client )
{
    int rc = 0;
    struct input_dev *input_dev;
    int i;

    i = 0;
    input_dev = input_allocate_device();
    if ( !input_dev ) {
        rc = -ENOMEM;
        return rc;
    }
    input_dev->name = ATMEL_TS_NAME;
    input_dev->phys = "atmel_mXT224E_touchscreen/input0";
    input_dev->id.bustype = BUS_I2C;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0002;
    input_dev->id.version = 0x0100;
    input_dev->dev.parent = &client->dev;
    input_dev->open = touchpad_open;
    input_dev->close = touchpad_close;
    

    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);

	g_tp->atmel_x_max = maxTouchCfg_T9_obj.value_array[18] | maxTouchCfg_T9_obj.value_array[19] << 8;
	g_tp->atmel_y_max = maxTouchCfg_T9_obj.value_array[20] | maxTouchCfg_T9_obj.value_array[21] << 8;
	if(g_tp->atmel_y_max == 0)
	    g_tp->atmel_y_max = ATMEL_Y_MAX;
	if(g_tp->atmel_x_max == 0)
		g_tp->atmel_x_max = ATMEL_X_MAX;
    ATMEL_PRINTK(2, "atmel_x_max:%d\n", (int)g_tp->atmel_x_max);
    ATMEL_PRINTK(2, "atmel_y_max:%d\n", (int)g_tp->atmel_y_max);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ATMEL_TOUCAN_PANEL_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ATMEL_TOUCAN_PANEL_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,  0, ATMEL_TOUCAN_PANEL_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,  0, ATMEL_TOUCAN_PANEL_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	ATMEL_PRINTK(1,"%s: Register input device\n", ATMEL_TS_NAME);
    rc = input_register_device(input_dev);
    if (rc) {
        ATMEL_PRINTK(0, "%s: Failed to register input device\n", ATMEL_TS_NAME);
        input_free_device(input_dev);
    }else {
        *input = input_dev;
    }

    return rc;
}


static void touchpad_create_kernel_debuglevel(void)
{
	ATMEL_PRINTK(1, "create kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir!=NULL) {
		debugfs_create_u32("ts_flow_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&ATMEL_DLL));
		debugfs_create_u32("ts_report_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&ATMEL_TOUCH_REPORT_DLL));
		debugfs_create_u32("cap_report_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&ATMEL_CAPKEY_REPORT_DLL));
	} else {
		printk(KERN_ERR "failed to create ATMEL mXT224E touch dll in kernel_debuglevel_dir!!!\n");
	}
}


static void touchpad_destroy_kernel_debuglevel(void)
{
	ATMEL_PRINTK(1, "destroy kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}

static const struct i2c_device_id i2cAtmelTouch_idtable[] = {
	{ ATMEL_TS_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, i2cAtmelTouch_idtable);

static struct i2c_driver i2c_touchpad_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = ATMEL_TS_NAME,
	},
	.probe	 = touchpad_probe,
	.remove	 = touchpad_remove,


	.id_table = i2cAtmelTouch_idtable,
};

static int __devinit touchpad_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int      result = 0;
	struct   atmel_platform_data *pdata;
    int i;
	printk("%s %d\n",__func__,__LINE__);
	
	g_tp = kzalloc(sizeof(struct touchpad_t), GFP_KERNEL);
    if(!g_tp) {
        result = -ENOMEM;
        return result;
    }

	
    for (i = 0 ; i < ATMEL_REPORT_POINTS ; i++) {
		g_tp->msg[i].coord.x = 0;
		g_tp->msg[i].coord.y = 0;
		g_tp->msg[i].state = RELEASE;
	}

	
    pdata = client->dev.platform_data;
    g_tp->gpio_irq = pdata->irq_gpio;;
    g_tp->gpio_rst = pdata->rst_gpio;;
    g_tp->irq = MSM_GPIO_TO_INT(g_tp->gpio_irq);
    g_tp->client = client;
	
	g_tp->backup_emlist_flag = 0;
	
    mutex_init(&g_tp->mutex);

	
	touchpad_create_kernel_debuglevel();

    
    result = touchpad_power_on_device(g_tp, 1);
    if(result) {
    	ATMEL_PRINTK(0, "Unable to power device result=%d\n", result);
		goto error_alloc_mem;
    }

    
    result = touchpad_setup_gpio(g_tp);
    if(result) {
        ATMEL_PRINTK(0, "Failed to setup gpio result=%d\n", result);
		goto error_power_device;
    }

    
    result = touchpad_config_gpio(g_tp);
    if(result) {
        ATMEL_PRINTK(0, "Failed to config gpio\n" );
		goto error_setup_gpio;
    }

    
	result = touchpad_check_chg_pin_low(g_tp);
    if(result) {
        ATMEL_PRINTK(0, "Failed to get CHG pin value:%d (result=%d)\n", g_tp->gpio_irq, result);
		goto error_setup_gpio;
    }

    
    result = touchpad_detect_mXT224E(g_tp);
    if(result) {
        ATMEL_PRINTK(0, "Failed to detect ATMEL mXT224E touch IC (result=%d)\n", result);
		goto error_setup_gpio;
    }
   	client->driver = &i2c_touchpad_driver;
    i2c_set_clientdata(client, g_tp);

    
    result = calculate_infoblock_crc(g_tp);
    if(result < 0) {
    	ATMEL_PRINTK(0, "Failed to calculate infoblock crc (result=%d)\n", result);
		goto error_setup_gpio;
    }

    
    result = power_on_flow_one_way(g_tp);
    if(result) {
    	ATMEL_PRINTK(0, "Failed to power_on_flow_one_way\n");
		goto error_setup_gpio;
    }

	
    result = touchpad_register_input(&g_tp->input, pdata, client);
    if(result) {
    	ATMEL_PRINTK(0, "Failed to register mXT224E ts input device (result=%d)\n", result);
		goto error_setup_gpio;
    }
    input_set_drvdata(g_tp->input, g_tp);

    
    result = keyarray_register_input(&g_tp->keyarray_input, client);
    if(result) {
		ATMEL_PRINTK(0, "Failed to register mXT224E keyarray input device (result=%d)\n", result);
		goto error_register_ts_input;
    }
    input_set_drvdata(g_tp->keyarray_input, g_tp);

#if 0
	
    INIT_DELAYED_WORK(&g_tp->touchpad_work, touchpad_irqWorkHandler);
    g_tp->touchpad_wqueue = create_singlethread_workqueue("ATMEL_Touchpad_Wqueue");
    if (!g_tp->touchpad_wqueue) {
		ATMEL_PRINTK(0, "Failed to create singlethread workqueue (result=%d)\n", result);
		goto error_register_keyarray_input;
    }

	ATMEL_PRINTK(1, "Setting up interrupt\n");
    result = request_irq(g_tp->irq, touchpad_irqHandler, IRQF_TRIGGER_LOW,"ATMEL_Touchpad_IRQ", g_tp);
    if (result) {
		ATMEL_PRINTK(0, "failed to request irq:%d (result=%d)\n", g_tp->irq, result);
		goto error_create_singlethread_wq;
    }
#endif
	
	result = request_threaded_irq(g_tp->irq, NULL, mxt224E_irq_thread,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, "mxt224E_ts", g_tp);
	if (result < 0) {
		ATMEL_PRINTK(0, "failed to request irq:%d (result=%d)\n", g_tp->irq, result);
		goto error_register_keyarray_input;
	}

	
    result = misc_register(&tp_misc_device);
    if(result) {
       	ATMEL_PRINTK(0, "Failed to register mXT224E touch misc driver (result=%d)\n", result);
		goto error_irq;
    }

	
    g_tp->touch_early_suspend.level = 150; 
    g_tp->touch_early_suspend.suspend = tp_suspend;
    g_tp->touch_early_suspend.resume = tp_resume;
    register_early_suspend(&g_tp->touch_early_suspend);

	ATMEL_PRINTK(0,"Start Probe %s\n",
		(result < 0) ? "FAIL" : "PASS");
    return 0;
#if 0
error_misc_register:
    misc_deregister(&tp_misc_device);
#endif
#if 0
error_create_singlethread_wq:
	destroy_workqueue(g_tp->touchpad_wqueue);
#endif
error_irq:
	free_irq(g_tp->irq, g_tp);

error_register_keyarray_input:
    input_unregister_device(g_tp->keyarray_input);
    input_free_device(g_tp->keyarray_input);
    g_tp->keyarray_input = NULL;

error_register_ts_input:
    input_unregister_device(g_tp->input);
    input_free_device(g_tp->input);
    g_tp->input = NULL;

error_setup_gpio:
	touchpad_release_gpio(g_tp);

error_power_device:
	touchpad_power_on_device(g_tp, 0);

error_alloc_mem:
	kfree(g_tp);
	result = -EFAULT;
	return result;
}


static int __devexit touchpad_remove(struct i2c_client *client)
{
	
	struct touchpad_t *g_tp = i2c_get_clientdata(client);

	ATMEL_PRINTK(0, "Unregister\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	
#endif 

	

	
	if (cancel_delayed_work((struct delayed_work *)&g_tp->touchpad_work) < 0)
		ATMEL_PRINTK(0, "error: could not remove work from workqueue\n");
	destroy_workqueue(g_tp->touchpad_wqueue);
	free_irq(g_tp->irq, g_tp);

	input_unregister_device(g_tp->keyarray_input);
    input_free_device(g_tp->keyarray_input);
    g_tp->keyarray_input = NULL;

	input_unregister_device(g_tp->input);
    input_free_device(g_tp->input);
    g_tp->input = NULL;
	touchpad_release_gpio(g_tp);
	touchpad_destroy_kernel_debuglevel();
	mutex_destroy(&g_tp->mutex);

	g_tp = NULL;
	kfree(g_tp);

	ATMEL_PRINTK(0, "Leaving\n");

	return 0;
}

static int __init touchpad_init(void)
{
	int rc = 0;

	ATMEL_PRINTK(0, "ATMEL mXT224E Touchscreen Driver (Built %s @ %s)\n",
		__DATE__, __TIME__);
	
	ATMEL_PRINTK(0, "system_rev=0x%x\n", system_rev);
	printk("%s %d\n",__func__,__LINE__);
	if (system_rev >=  CHICAGO_EVT2) {
		i2c_touchpad_driver.driver.name = ATMEL_TS_NAME;
		rc = i2c_add_driver(&i2c_touchpad_driver);
		if (rc) {
			ATMEL_PRINTK(0, "ATMEL mXT224E touch driver registration failed\n");
			return rc;
		}
		printk("%s %d\n",__func__,__LINE__);
	}
    return rc;
}
module_init(touchpad_init);

static void __exit touchpad_exit(void)
{
	i2c_del_driver(&i2c_touchpad_driver);
	ATMEL_PRINTK(0, " ATMEL mXT224E touch driver Exiting\n");

}
module_exit(touchpad_exit);

MODULE_DESCRIPTION("ATMEL xMT224E touchpad driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Emily Jiang");
MODULE_ALIAS("platform:atmel_xMT224E_touch");
