#ifndef _LSENSOR_H_
#define _LSENSOR_H_
#include <asm/ioctl.h>
#include <linux/input.h>
#include <linux/kernel.h>

#define MYBIT(b)        (1<<b)
#define TST_BIT(x,b)    ((x & (1<<b)) ? 1 : 0)
#define CLR_BIT(x,b)    (x &= (~(1<<b)))
#define SET_BIT(x,b)    (x |= (1<<b))

#define LSENSOR_ADDR      (0x44)

#define LSENSOR_GPIO_INT  20   
#define LSENSOR_GPIO_PWR_SW 42 
#define LSENSOR_GPIO_PWR_SW1 96 

#define LSENSOR_EN_AP     0x01  
#define LSENSOR_EN_LCD    0x02  
#define LSENSOR_EN_ENG    0x04  
#define PSENSOR_EN_AP     0x10  

#define LSENSOR_CALIBRATION_LOOP 20 

struct lsensor_info_data {  
  
  unsigned short adata;   
  unsigned short pdata;   

  unsigned int lux;
  unsigned char dist;  

  unsigned short bkl_level;
};

struct lux_bkl_table {
  short level;  
  short now;    
  short high;   
  short low;    
};
#define LSENSOR_BKL_TABLE_SIZE 12 
struct lsensor_drv_data {
  int inited;
  int enable;         
  int enable_in_work; 
  int opened;   
  int eng_mode;   
  int irq_enabled;
  int irq_active_high; 
  int in_early_suspend;
  int in_suspend;
  int intr_gpio;
  int pwr_gpio;
  int i2c_err;    
  int i2c_addr;

  struct i2c_client *client;
  struct wake_lock wlock;
  struct work_struct work;
  struct work_struct work_bkl_fading;
  struct workqueue_struct *wqueue;
  struct input_dev *input_als;
  struct input_dev *input_prox;

  struct completion info_comp;
  int info_waiting;

  unsigned int m_ga;    

  unsigned int lux_history[3];  
  unsigned int lux;       
  unsigned int lux_old;   
  unsigned int als_range;  
  unsigned int als_range_old;  

  unsigned char prox_threshold;     
  unsigned char prox_threshold_far; 
  unsigned char dist;       
  unsigned char dist_old;   
  unsigned char ps_hgap;	
  unsigned char ps_lgap;	
  unsigned char pdata_min;  

  unsigned int als_nv;   
  unsigned int prox_nv;  

  short bkl_now;
  short bkl_end;
  unsigned int bkl_idx;         
  unsigned int bkl_idx_old;     
  struct lux_bkl_table bkl_table[LSENSOR_BKL_TABLE_SIZE]; 
  unsigned long jiff_update_bkl_wait_time;    
  unsigned long jiff_resume_fast_update_time; 

  unsigned long jiff_em_polling_interval;  

  #if defined(CONFIG_DEBUG_FS)
  
  struct dentry		*dent;
  char debug_write_buf[128];
  char debug_read_buf[128];
  #endif
};

struct lsensor_eng_data { 

	unsigned char als_en;	
	unsigned char als_range;
	unsigned char als_mode;	
	unsigned int  m_ga;   
	unsigned char ps_en;	
	unsigned char ps_drv;	
	unsigned char ps_sleep;	

	uint16_t als_lt;		
	uint16_t als_ht;		
	uint16_t ps_lt;			
	uint16_t ps_ht;			
	unsigned char als_pers;	
	unsigned char ps_pers;	
};

struct lsensor_cal_data {
   unsigned int als;
  struct {
    unsigned short prox_far;
    unsigned short prox_near;
  } prox;
};

#define LSENSOR_IOC_MAGIC       'l' 

#define LIGHTSENSOR_IOCTL_ENABLE _IO(LSENSOR_IOC_MAGIC, 1)
#define LIGHTSENSOR_IOCTL_DISABLE _IO(LSENSOR_IOC_MAGIC, 2)
#define LIGHTSENSOR_IOCTL_GET_ADC _IOR(LSENSOR_IOC_MAGIC, 3, int *)
#define LIGHTSENSOR_IOCTL_GET_CHIPID _IOR(LSENSOR_IOC_MAGIC, 4, char *)
#define LIGHTSENSOR_IOCTL_CALIBRATE	_IOW(LSENSOR_IOC_MAGIC, 5, struct lsensor_cal_data)
#define LIGHTSENSOR_IOCTL_CAL_ALS _IOW(LSENSOR_IOC_MAGIC, 6, struct lsensor_cal_data)
#define LIGHTSENSOR_IOCTL_CAL_FAR	_IOW(LSENSOR_IOC_MAGIC, 7, struct lsensor_cal_data)
#define LIGHTSENSOR_IOCTL_CAL_NEAR	_IOW(LSENSOR_IOC_MAGIC, 8, struct lsensor_cal_data)

#define LSENSOR_IOC_UPDATE_CAL_DATA _IOW(LSENSOR_IOC_MAGIC, 9, struct lsensor_cal_data)

#define PROXIMITY_SENSOR_IOCTL_ENABLE 		_IO(LSENSOR_IOC_MAGIC, 10)
#define PROXIMITY_SENSOR_IOCTL_DISABLE 		_IO(LSENSOR_IOC_MAGIC, 11)
#define PROXIMITY_SENSOR_IOCTL_GET_ADC 		_IOR(LSENSOR_IOC_MAGIC, 12, int *)
#define PROXIMITY_SENSOR_IOCTL_GET_CHIPID _IOR(LSENSOR_IOC_MAGIC, 13, char *)

#define LPSENSOR_IOCTL_ENG_ENABLE 	_IO(LSENSOR_IOC_MAGIC, 14)
#define LPSENSOR_IOCTL_ENG_DISABLE 	_IO(LSENSOR_IOC_MAGIC, 15)
#define LPSENSOR_IOCTL_ENG_CTL_R 	_IOR(LSENSOR_IOC_MAGIC, 16, struct lsensor_eng_data)
#define LPSENSOR_IOCTL_ENG_CTL_W 	_IOW(LSENSOR_IOC_MAGIC, 17, struct lsensor_eng_data)
#define LPSENSOR_IOCTL_ENG_INFO 	_IOR(LSENSOR_IOC_MAGIC, 18, struct lsensor_info_data)

#define LIGHTSENSOR_IOCTL_ENABLE_AUTO_BKL   _IOW(LSENSOR_IOC_MAGIC, 20, unsigned char *)
#define LIGHTSENSOR_IOCTL_DISABLE_AUTO_BKL  _IOW(LSENSOR_IOC_MAGIC, 21, unsigned char *)

#endif

