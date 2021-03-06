#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <mach/pm_log.h>
#include <mach/chicago_hwid.h>

#include "../../arch/arm/mach-msm/smd_private.h"
#include <oem/oem_smem_struct.h>

#include "isl29028.h"

#if 0
static int lsensor_log_on = 0;
static int lsensor_log3_on = 0;

#define MSG(format, arg...) {if(lsensor_log_on)  printk(KERN_INFO "[ALS]" format "\n", ## arg);}
#define MSG2(format, arg...) printk(KERN_INFO "[ALS]" format "\n", ## arg) 
#define MSG3(format, arg...) {if(lsensor_log3_on)  printk(KERN_INFO "[ALS]" format "\n", ## arg);}
#endif
#define LSENSOR_DEBUG_ENG_MODE 0  


extern struct dentry *kernel_debuglevel_dir;
static unsigned int ALS_DLL=0;  
#define ERR_LEVEL		0
#define INFO_LEVEL		1
#define DEBUG1_LEVEL	2
#define DEBUG2_LEVEL	3
#define ALS_PRINTK(level, fmt, args...) if (level <= ALS_DLL) printk("[ALS]"fmt"\n",  ##args);

static DEFINE_MUTEX(lsensor_enable_lock);
static DEFINE_SPINLOCK(lsensor_irq_lock);

static DEFINE_SPINLOCK(lsensor_pstate_lock);

static struct lsensor_info_data ls_info;
static struct lsensor_drv_data  ls_drv =
{
  .bkl_now = 130,
  .bkl_idx = (LSENSOR_BKL_TABLE_SIZE>>1), 
  .bkl_table = {  
  
    {30 ,  0    , 30    , 0 }, 
    {50 ,  25   , 60    , 15 },
    {70 ,  52   , 100   , 44 },
    {90 ,  87   , 180   , 75 },
    {110,  160  , 350   , 98 },
    {130,  330  , 460   , 160 },
    {150,  450  , 800   , 320 },
    {170,  750  , 1350  , 440},
    {190,  1100 , 2400  , 700},
    {211,  2100 , 3500  , 1000},
    {233,  3350 , 4600  , 1500},
    {255,  4400 , 5210  , 2200}, 
    },
  .lux_history = {75,75,75},
  .lux = 75,
  .lux_old = 75,
  .m_ga = 4096,  
  .prox_threshold = 65,
  .prox_threshold_far = 45,
  .dist = 1,
  .dist_old  = 1,
  .jiff_em_polling_interval = HZ/5,
  .ps_hgap = 25,
  .ps_lgap = 15,
  .pdata_min = 0,
  .pwr_gpio = 0,
  .als_range = 1,
  .als_range_old = 1,
};
static struct lsensor_eng_data  ls_eng;
static struct timer_list ls_timer;
static smem_vendor_id1_apps_data *smem_vendor1_data = NULL;

static int lsensor_read_i2c(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
{
  int ret;
  struct i2c_msg msgs[] = {
    [0] = {
      .addr   = addr,
      .flags  = 0,
      .buf    = (void *)&reg,
      .len    = 1
    },
    [1] = {
      .addr   = addr,
      .flags  = I2C_M_RD,
      .buf    = (void *)buf,
      .len    = len
    }
  };
  if(!ls_drv.client)
    return -ENODEV;
  ret = i2c_transfer(ls_drv.client->adapter, msgs, 2);
  if(ret == 2)
  {
    if(ls_drv.i2c_err)
      ALS_PRINTK(1,"%s, ret = 2, i2c_err = 0",__func__);
    ls_drv.i2c_err = 0;
  }
  else
  {
    ls_drv.i2c_err ++;
    if(ls_drv.i2c_err < 20)
      ALS_PRINTK(1,"%s, ret = %d, i2c_err = %d",__func__,ret,ls_drv.i2c_err);
  }
  return ret;
}
static int lsensor_write_i2c(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
{
  int i, ret;
  unsigned char buf_w[64];
  struct i2c_msg msgs[] = {
    [0] = {
      .addr   = addr,
      .flags  = 0,
      .buf    = (void *)buf_w,
      .len    = len+1
    }
  };
  if(len >= sizeof(buf_w))  
    return -ENOMEM;
  if(!ls_drv.client)
    return -ENODEV;
  buf_w[0] = reg;
  for(i=0; i<len; i++)
    buf_w[i+1] = buf[i];
  ret = i2c_transfer(ls_drv.client->adapter, msgs, 1);
  if(ret == 1)
  {
    if(ls_drv.i2c_err)
      ALS_PRINTK(1,"%s, ret = 1, i2c_err = 0",__func__);
    ls_drv.i2c_err = 0;
  }
  else
  {
    ls_drv.i2c_err ++;
    if(ls_drv.i2c_err < 20)
      ALS_PRINTK(1,"%s, ret = %d, i2c_err = %d",__func__,ret,ls_drv.i2c_err);
  }
  return ret;
}

void lsensor_info_data_log(void)
{
  #if LSENSOR_DEBUG_ENG_MODE
    ALS_PRINTK(1,"data=%5d %5d %5d %5d %5d",ls_info.adata, 
    	ls_info.pdata,ls_info.lux,ls_info.dist,ls_info.bkl_level);
  #endif
}
void lsensor_eng_data_log(void)
{
  #if LSENSOR_DEBUG_ENG_MODE
    ALS_PRINTK(2,"als_en m_ga = %d %d",
      ls_eng.als_en, ls_eng.m_ga);
    ALS_PRINTK(2,"als_pers als_range als_mode = %d %d %d ",
      ls_eng.als_pers, ls_eng.als_range, ls_eng.als_mode);
    ALS_PRINTK(2,"ps_en ps_drv ps_sleep ps_pers= %d %d %d %d",
      ls_eng.ps_en, ls_eng.ps_drv, ls_eng.ps_sleep, ls_eng.ps_pers);
  #endif
}

static inline void lsensor_irq_onOff(unsigned int onOff)
{
  unsigned long flags;
  spin_lock_irqsave(&lsensor_irq_lock, flags);
  if(onOff)
  {
    if(! ls_drv.irq_enabled)
    {
      
      ls_drv.irq_enabled = 1;
      irq_set_irq_wake(MSM_GPIO_TO_INT(ls_drv.intr_gpio), 1);
      enable_irq(MSM_GPIO_TO_INT(ls_drv.intr_gpio));
    }
  }
  else
  {
    if(ls_drv.irq_enabled)
    {
      disable_irq_nosync(MSM_GPIO_TO_INT(ls_drv.intr_gpio));
      irq_set_irq_wake(MSM_GPIO_TO_INT(ls_drv.intr_gpio), 0);
      ls_drv.irq_enabled = 0;
      
    }
  }
  spin_unlock_irqrestore(&lsensor_irq_lock, flags);
}

static void lsensor_update_lux(unsigned int adata)
{
  static u8 middle[] = {1,0,2,0,0,2,0,1};
  int index;
  unsigned int als_kdata;
  unsigned int in_lux;
  
  if (ls_drv.als_range) {
    als_kdata = (adata * 534 / 1024) * ls_drv.m_ga / 1024;  
    in_lux = adata * 534 / 1024;
  } else {
    als_kdata = (adata * 33 / 1024) * ls_drv.m_ga / 1024; 
    in_lux = adata * 33 / 1024;
  }
  
  ls_drv.lux_history[2] = ls_drv.lux_history[1];
  ls_drv.lux_history[1] = ls_drv.lux_history[0];
  ls_drv.lux_history[0] = als_kdata;
  {
    index = 0;
    if( ls_drv.lux_history[0] > ls_drv.lux_history[1] ) index += 4;
    if( ls_drv.lux_history[1] > ls_drv.lux_history[2] ) index += 2;
    if( ls_drv.lux_history[0] > ls_drv.lux_history[2] ) index++;
    ls_drv.lux = ls_drv.lux_history[middle[index]];
  }
}
static void lsensor_update_dist(unsigned int pdata)
{
  
  if (pdata > ls_drv.prox_threshold) 
  {
    ls_drv.dist = 0;  
  } else if (pdata <= ls_drv.prox_threshold_far)  
  {
    ls_drv.dist = 1;  
  }
  
}

static void lsensor_update_threshold(void)
{
  unsigned char r03,r04,r08;
  unsigned int prox_ht,prox_lt,prox1;
  
  msleep(10);
  lsensor_read_i2c(0x44,0x08,&r08,sizeof(r08));
  prox1 = r08;
  prox_ht = prox1 + ls_drv.ps_hgap;
  prox_lt = prox1 + ls_drv.ps_lgap;

  
  ls_drv.pdata_min = prox1;
  
  if((prox_ht > 200) || (prox_lt > 200) || (prox_lt >= prox_ht )) 
  {
    ls_drv.prox_threshold  = 175;     
    ls_drv.prox_threshold_far = 150;  
  }
  
  else 
  {
    ls_drv.prox_threshold  = prox_ht;
    ls_drv.prox_threshold_far = prox_lt;
  }
  r03 = ls_drv.prox_threshold_far;
  r04 = ls_drv.prox_threshold;
  lsensor_write_i2c(0x44,0x03,&r03,sizeof(r03));
  lsensor_write_i2c(0x44,0x04,&r04,sizeof(r04));
  ALS_PRINTK(0,"psensor enable :start_pdata=%d,ls_drv.pdata_min=%d,ps_ht = %d,ps_lt = %d\n",	\
  			prox1,ls_drv.pdata_min,ls_drv.prox_threshold,ls_drv.prox_threshold_far);
  
}

extern void lsensor_update_backlight(int brightness_als);
static void lsensor_set_backlight(int level)
{
    ALS_PRINTK(2,"%s = %d", __func__, level);
    lsensor_update_backlight(level);
}
static void lsensor_bkl_fading(int start, int end)
{
  int count = 0,dist, stepSize;
  unsigned long jiff_start = jiffies;
  
  dist = start >= end ? (start-end) : (end - start);
  
  
  stepSize = dist / 3;

  if(stepSize < 1)
  {
    ALS_PRINTK(1,"start=%d end=%d dist=%d\n",start,end,dist);
    return;
  }

  while(1)
  {
    if(start == end)
      break;
    if(start < end) 
    {
      start += stepSize;
      if(start > end)
        start = end;
    }
    else  
    {
      start -= stepSize;
      if(start < end)
        start = end;
    }
    lsensor_set_backlight(start);
    count ++;
  }
  ALS_PRINTK(2,"%s (%3d~%3d) (%d in %d ms)", __func__, start, end, count, jiffies_to_msecs(jiffies-jiff_start));
}
static void lsensor_bkl_fading_work_func(struct work_struct *work)
{
  int start, count = 0;
  unsigned long jiff_start = jiffies;
  
  start = ls_drv.bkl_now;
  while(1)
  {
    if(ls_drv.in_early_suspend || ls_drv.bkl_now == ls_drv.bkl_end)
      break;
    if(ls_drv.bkl_now < ls_drv.bkl_end) 
    {
      if(ls_drv.bkl_now < 50)       ls_drv.bkl_now += 1;
      else if(ls_drv.bkl_now < 130) ls_drv.bkl_now += 2;
      else if(ls_drv.bkl_now < 200) ls_drv.bkl_now += 3;
      else                          ls_drv.bkl_now += 4;
      if(ls_drv.bkl_now > ls_drv.bkl_end)
        ls_drv.bkl_now = ls_drv.bkl_end;
    }
    else  
    {
      if(ls_drv.bkl_now < 50)       ls_drv.bkl_now -= 1;
      else if(ls_drv.bkl_now < 130) ls_drv.bkl_now -= 2;
      else if(ls_drv.bkl_now < 200) ls_drv.bkl_now -= 3;
      else                          ls_drv.bkl_now -= 4;
      if(ls_drv.bkl_now < ls_drv.bkl_end)
        ls_drv.bkl_now = ls_drv.bkl_end;
    }
    lsensor_set_backlight(ls_drv.bkl_now);

	
	ls_info.bkl_level = ls_drv.bkl_now;
	
    if(ls_drv.in_suspend || ls_drv.bkl_now == ls_drv.bkl_end)
      break;
    
    msleep(50); 
    count ++;
  }
  ALS_PRINTK(3,"%s (%3d~%3d) (%d in %d ms)", __func__, start, ls_drv.bkl_end, count, jiffies_to_msecs(jiffies-jiff_start));

}
static void lsenosr_update_bkl(void)
{
  int high, low, dd, lux;
  unsigned int i, wait;

  
  if(time_before(jiffies, ls_drv.jiff_update_bkl_wait_time)) 
    return;
  if(!(ls_drv.enable & LSENSOR_EN_LCD)) 
    return;
  if(ls_drv.bkl_idx > (LSENSOR_BKL_TABLE_SIZE-1))
  {
    ALS_PRINTK(1,"ERROR: %s bkl_idx=%d (OVERFLOW!)",__func__,ls_drv.bkl_idx);
    ls_drv.bkl_idx = (LSENSOR_BKL_TABLE_SIZE-1);
  }
  lux   = ls_drv.lux;
  high  = ls_drv.bkl_table[ls_drv.bkl_idx].high;
  low   = ls_drv.bkl_table[ls_drv.bkl_idx].low;
  if(lux <= high && lux >= low)
  {
    
  }
  else if(lux > high) 
  {
    if(ls_drv.bkl_idx < (LSENSOR_BKL_TABLE_SIZE-1))
      ls_drv.bkl_idx ++;
  }
  else if(lux < low) 
  {
    if(ls_drv.bkl_idx > 0)
      ls_drv.bkl_idx --;
  }

  if(ls_drv.bkl_idx_old != ls_drv.bkl_idx)
  {
    for(i=LSENSOR_BKL_TABLE_SIZE-1; i>0; i--)
    {
      if(lux > ls_drv.bkl_table[i].now)
        break;
    }
    if(i > ls_drv.bkl_idx) 
    {
      dd = i - ls_drv.bkl_idx;
      wait = dd > 4? 150  :
             dd > 3? 250  :
             dd > 2? 550  : 850;
    }
    else 
    {
      dd = ls_drv.bkl_idx - i;
      wait = dd > 4? 250  :
             dd > 3? 550  :
             dd > 2? 850 : 1250;
    }
    ALS_PRINTK(2,"idx=%02d i=%02d, now=%04d, lux=%04d, wait=%04d", ls_drv.bkl_idx, i, ls_drv.bkl_table[ls_drv.bkl_idx].now, lux, wait);
    
    ls_drv.bkl_end = ls_drv.bkl_table[ls_drv.bkl_idx].level;
    if(!ls_drv.in_early_suspend)
    {
      queue_work(ls_drv.wqueue, &ls_drv.work_bkl_fading);
    }
    
    if(time_before(jiffies, ls_drv.jiff_resume_fast_update_time)) 
      ls_drv.jiff_update_bkl_wait_time = jiffies;
    else
      ls_drv.jiff_update_bkl_wait_time = jiffies + HZ*wait/1024;
	ALS_PRINTK(2,"%s  %d ms", __func__, jiffies_to_msecs(ls_drv.jiff_update_bkl_wait_time-jiffies));
  }
  ls_drv.bkl_idx_old = ls_drv.bkl_idx;
}
static int lsenosr_update_idx(short bkl)
{
  unsigned int i;
  short level;
  for(i=LSENSOR_BKL_TABLE_SIZE-1; i>0; i--)
  {
    level = ls_drv.bkl_table[i].level;
    if(bkl >= level)
    {
      ls_drv.bkl_idx = i;
      return level;
    }
  }
  ls_drv.bkl_idx = 0;
  return ls_drv.bkl_table[0].level;   
}


static void lsensor_eng_set_reg(void)
{
  unsigned char r00_0A[11];
  ALS_PRINTK(2,"%s", __func__);

  lsensor_read_i2c(0x44,0x00,&r00_0A[0],sizeof(r00_0A));

  
  r00_0A[1] =
    ((ls_eng.ps_en    & 0x1) << 7) |
    ((ls_eng.ps_sleep & 0x7) << 4) |
    ((ls_eng.ps_drv   & 0x1) << 3) |
    ((ls_eng.als_en   & 0x1) << 2) |
    ((ls_eng.als_range& 0x1) << 1) |
    ((ls_eng.als_mode & 0x1) << 0);
  
  r00_0A[2] =
    ((ls_eng.ps_pers  & 0x3) << 5) |
    ((ls_eng.als_pers & 0x3) << 1) ;  
  
  ls_drv.ps_lgap = ls_eng.ps_lt  & 0xFF;
  ls_drv.ps_hgap = ls_eng.ps_ht  & 0xFF;
  r00_0A[5] = ls_eng.als_lt & 0xFF;
  r00_0A[6] = ((ls_eng.als_lt & 0xF00) >> 8) + ((ls_eng.als_ht & 0x00F) << 4);
  r00_0A[7] = ((ls_eng.als_ht & 0xFF0) >> 4);

  ls_drv.m_ga = ls_eng.m_ga;

  lsensor_write_i2c(0x44,0x00,&r00_0A[0],sizeof(r00_0A));
}

static void lsensor_eng_get_reg(void)
{
  unsigned char r00_0A[11];
  ALS_PRINTK(2,"%s", __func__);

  lsensor_read_i2c(0x44,0x00,&r00_0A[0],sizeof(r00_0A));
  
  ls_eng.ps_en = (r00_0A[1] & 0x80) >> 7;
  ls_eng.ps_sleep = (r00_0A[1] & 0x70) >> 4;
  ls_eng.ps_drv = (r00_0A[1] & 0x08)  >> 3;
  ls_eng.als_en = (r00_0A[1] & 0x04) >> 2;
  ls_eng.als_range = (r00_0A[1] & 0x02) >> 1;
  ls_eng.als_mode = (r00_0A[1] & 0x01);
  
  ls_eng.ps_pers = (r00_0A[2] & 0x60) >> 5;
  ls_eng.als_pers = (r00_0A[2] & 0x06) >> 1;
  
  ls_eng.ps_lt = ls_drv.ps_lgap;
  ls_eng.ps_ht = ls_drv.ps_hgap;
  ls_eng.als_lt = r00_0A[5] + ((r00_0A[6] & 0x0F) << 8);
  ls_eng.als_ht = (r00_0A[7] << 4) + ((r00_0A[6] & 0xF0) >> 4);

  ls_eng.m_ga = ls_drv.m_ga;
}

static void lsensor_update_equation_parameter(struct lsensor_cal_data *lp_cal_data)
{
  unsigned int als,prox, prox_far;

  {
    
    if(lp_cal_data->als < 1000 ) 
    {
      als = lp_cal_data->als;
    }
    else
    {
      als = 240;  
    }
    ls_drv.m_ga = 958 * 1024 / als; 
    ALS_PRINTK(3,"%s: m_ga = %d",__func__, ls_drv.m_ga);
    
    
    if((lp_cal_data->prox.prox_far > 100) ||  
      (lp_cal_data->prox.prox_near > 200) ||  
      (lp_cal_data->prox.prox_far >= lp_cal_data->prox.prox_near))  
    {
      prox      = 25;   
      prox_far  = 15;   
    }
    else  
    {
      
      prox      = (lp_cal_data->prox.prox_near -lp_cal_data->prox.prox_far) * 1 / 2;
      prox_far  = (lp_cal_data->prox.prox_near -lp_cal_data->prox.prox_far) * 3 / 10;
      
    }
    if(prox <= 15) 
    {
      prox      = 15;
      prox_far  = 10;
    }
	
    ls_drv.ps_hgap = prox; 
    ls_drv.ps_lgap = prox_far & 0xFF;  
    
    ALS_PRINTK(1,"%s: prox_hgap= %d, prox_lgap = %d",__func__,ls_drv.ps_hgap,ls_drv.ps_lgap);
  }
}

static void lsensor_calibration(struct lsensor_cal_data *lp_cal_data)
{
  
  unsigned int m_ga_old, irq_old;
  int i, lux_count , dist_count ;
  unsigned int lux_average, dist_average, lux_sum, dist_sum;
  unsigned int lux[LSENSOR_CALIBRATION_LOOP], dist[LSENSOR_CALIBRATION_LOOP];
  unsigned int lux_high, lux_low, dist_high, dist_low;
  unsigned int pdata, adata;
  unsigned char r00_0A[11], r00_0A_old[11];

  irq_old = ls_drv.irq_enabled;
  lsensor_irq_onOff(0);

  lsensor_read_i2c(0x44,0x00,&r00_0A[0],sizeof(r00_0A));
  m_ga_old = ls_drv.m_ga;
  r00_0A_old[1] = r00_0A[1];
  r00_0A_old[2] = r00_0A[2];

  r00_0A[1] = 0xB6;
  r00_0A[2] = 0x04;
  lsensor_write_i2c(0x44,0x01,&r00_0A[1],sizeof(r00_0A[1]));
  lsensor_write_i2c(0x44,0x02,&r00_0A[2],sizeof(r00_0A[2]));
  msleep(100);

  ls_drv.m_ga = 1024;
 
  lux_average = 0;
  dist_average = 0;
  lux_high  = 0;
  lux_low = 0xFFF;
  dist_high = 0;
  dist_low  = 0xFF;
  for(i=0;i<LSENSOR_CALIBRATION_LOOP;i++)
  {
    lsensor_read_i2c(0x44,0x00,&r00_0A[0],sizeof(r00_0A));
    pdata = r00_0A[8];
    adata = r00_0A[9] + (r00_0A[10] << 8);
    msleep(105);

    lux[i]  = adata;
    dist[i] = pdata;
    lux_average += adata;
    dist_average  += pdata;
    ALS_PRINTK(1,"%02d = %04d %04d", i, adata, pdata);
    if(adata >= lux_high)     lux_high = adata;
    if(adata <= lux_low)      lux_low = adata;
    if(pdata >= dist_high)    dist_high = pdata;
    if(pdata <= dist_low)   dist_low = pdata;
  }
  lux_average /= LSENSOR_CALIBRATION_LOOP;
  dist_average /= LSENSOR_CALIBRATION_LOOP;
  ALS_PRINTK(1,"ls value= %d~%d~%d,Pdata = %d~%d~%d (Low~Average~High)",
    lux_low,lux_average,lux_high,dist_low,dist_average,dist_high);
  
  lux_count = 0;
  dist_count = 0;
  lux_sum = 0;
  dist_sum = 0;
  for (i=0;i<LSENSOR_CALIBRATION_LOOP;i++)
  {
    if (lux[i] != lux_high && lux[i] != lux_low)
    {
      lux_count ++;
      lux_sum += lux[i];
    }
    if (dist[i] != dist_high && dist[i] != dist_low)
    {
      dist_count ++;
      dist_sum += dist[i];
    }
  }
  if (lux_count)
  {
    lux_sum /= lux_count;
  }
  else
  {
    ALS_PRINTK(1,"ls value has no middle value!/n");
    lux_sum = lux_average;
  }
  if(dist_count)
  {
    dist_sum /= dist_count;
  }
  else
  {
    ALS_PRINTK(1,"ps value has no middle value!/n");
    dist_sum = dist_average;
  }
  ALS_PRINTK(1,"Calibration = %d %d, count = %d %d",
    lux_sum, dist_sum, lux_count, dist_count);
 
    lp_cal_data->als = lux_sum;

    lp_cal_data->prox.prox_far  = dist_sum;
    lp_cal_data->prox.prox_near = 0xFF;

  ls_drv.m_ga = m_ga_old;
  r00_0A[1] = r00_0A_old[1];
  r00_0A[2] = r00_0A_old[2];
  lsensor_write_i2c(0x44,0x01,&r00_0A[1],sizeof(r00_0A[1]));
  lsensor_write_i2c(0x44,0x02,&r00_0A[2],sizeof(r00_0A[2]));

  if(irq_old)
    lsensor_irq_onOff(1);
}

static void lsensor_work_func(struct work_struct *work)
{
  unsigned char r00_0A[11], r00_0A_old[11];
  char a_flag, p_flag, irq_active_high = 1, a_en = 0, p_en = 0;
  unsigned int prox_ht,prox_lt;
  unsigned long flags;
  

  if(!ls_drv.inited)
    return;
  if(ls_drv.in_suspend) 
    return;
 
  mutex_lock(&lsensor_enable_lock);
  ls_drv.enable_in_work = ls_drv.enable;
  mutex_unlock(&lsensor_enable_lock);

  lsensor_read_i2c(0x44,0x00,&r00_0A[0],sizeof(r00_0A));
  memcpy(r00_0A_old, r00_0A, sizeof(r00_0A_old));
  a_flag = TST_BIT(r00_0A[2],3);  
  p_flag = TST_BIT(r00_0A[2],7);  
  ls_drv.als_range = TST_BIT(r00_0A[1],1); 
  ls_info.pdata = r00_0A[8];
  ls_info.adata = r00_0A[9] + (r00_0A[10] << 8);

  if(ls_drv.enable_in_work & PSENSOR_EN_AP || (ls_drv.enable_in_work & LSENSOR_EN_ENG && TST_BIT(r00_0A[1],7)) ) 
  {
    if (ls_info.pdata < ls_drv.pdata_min)
    {
      ls_drv.pdata_min = ls_info.pdata;
	  prox_ht  = ls_drv.pdata_min + ls_drv.ps_hgap;
	  prox_lt  = ls_drv.pdata_min + ls_drv.ps_lgap;
	  if ((prox_ht < ls_drv.prox_threshold) & (prox_lt < ls_drv.prox_threshold_far)) 
	  {
	    ls_drv.prox_threshold = prox_ht;
	    ls_drv.prox_threshold_far = prox_lt;
		
	    r00_0A[3] = ls_drv.prox_threshold_far;
        r00_0A[4] = ls_drv.prox_threshold;
	
        lsensor_write_i2c(0x44,0x03,&r00_0A[3],2);
		ALS_PRINTK(0,"pdata=%d,ls_drv.pdata_min=%d,ps_ht = %d,ps_lt = %d,ps_dist = %d",ls_info.pdata,	\
	 	   ls_drv.pdata_min,ls_drv.prox_threshold,ls_drv.prox_threshold_far,ls_drv.dist);
	  }
	  else
	  {
        
	  }
    }
  }

  if(ls_drv.enable_in_work & LSENSOR_EN_AP || 
    ls_drv.enable_in_work & LSENSOR_EN_LCD || 
    (ls_drv.enable_in_work & LSENSOR_EN_ENG && TST_BIT(r00_0A[1],2))) 
  {
    
    lsensor_update_lux(ls_info.adata);

    if (ls_drv.als_range) 
    {
      if (ls_info.adata < 100)   
	    {
        ls_drv.als_range = 0;
        CLR_BIT(r00_0A[1],1);
        
        lsensor_write_i2c(0x44,0x01,&r00_0A[1],1);

        ALS_PRINTK(1,"%s, high->low:als_range=%d, adata=%d",__func__,ls_drv.als_range,ls_info.adata);
      }
    } 
    else
    { 
      if (ls_info.adata > 2048)   
      {
        ls_drv.als_range = 1;
        SET_BIT(r00_0A[1],1);
        
        lsensor_write_i2c(0x44,0x01,&r00_0A[1],1);
        
        ALS_PRINTK(1,"%s, low->high:als_range=%d, adata=%d",__func__,ls_drv.als_range,ls_info.adata);
      }
    }

    if (ls_drv.als_range_old != ls_drv.als_range)
    {
      ls_drv.als_range_old = ls_drv.als_range;
      ALS_PRINTK(0,"%s, als_range=%d, adata=%d,lux=%d",__func__,ls_drv.als_range,ls_info.adata,ls_drv.lux);
    }

    if(ls_drv.enable_in_work & LSENSOR_EN_AP)
    {
      if(ls_drv.lux_old != ls_drv.lux)
      {
        ls_drv.lux_old = ls_drv.lux;
		if (ls_drv.lux <= 10000) {
        input_event(ls_drv.input_als, EV_MSC, MSC_RAW, ls_drv.lux);
        input_sync(ls_drv.input_als);
        
	   }
	   else 
	   {
		input_event(ls_drv.input_als, EV_MSC, MSC_RAW, 10000);
        input_sync(ls_drv.input_als);
	   }
      }
    }
  }
    
  if(ls_drv.enable_in_work & PSENSOR_EN_AP) 
  {
    
    lsensor_update_dist(ls_info.pdata);
    
    if(ls_drv.dist_old != ls_drv.dist)
    {
      if(ls_drv.dist_old == 1)
      {
        ALS_PRINTK(1,"prox 1->0");
 
        spin_lock_irqsave(&lsensor_pstate_lock, flags);
        if (!smem_vendor1_data) {
          ALS_PRINTK(0, "Can't find ps_state.\n");
        } 
        else
        {
          smem_vendor1_data->p_state = 0;
          ALS_PRINTK(1, "far->near:p_state= %d",smem_vendor1_data->p_state);
        }
        spin_unlock_irqrestore(&lsensor_pstate_lock, flags);
        
      }
      else
      {
        ALS_PRINTK(1,"prox 0->1");

        spin_lock_irqsave(&lsensor_pstate_lock, flags);
        if (!smem_vendor1_data) {
          ALS_PRINTK(0, "Can't find ps_state.");
        }
        else
        {
          smem_vendor1_data->p_state = 1;
          ALS_PRINTK(1, "near->far:p_state= %d",smem_vendor1_data->p_state);
        }
        spin_unlock_irqrestore(&lsensor_pstate_lock, flags);
        
      }
      ls_drv.dist_old = ls_drv.dist;
      input_report_abs(ls_drv.input_prox, ABS_DISTANCE, ls_drv.dist);
      input_sync(ls_drv.input_prox);
      
    }
  }
  ALS_PRINTK(3,"%c%3d(%d), %c%4d(%d)",p_flag?' ':'x',ls_info.pdata,ls_drv.dist,a_flag?' ':'x',ls_info.adata,ls_drv.lux);
  
  if(ls_drv.enable_in_work & LSENSOR_EN_ENG)
  {
    if(ls_drv.in_early_suspend)
    {
      
      del_timer_sync(&ls_timer);
    }
    else
    {

      ls_info.lux = ls_drv.lux;

	  lsensor_update_dist(ls_info.pdata);
      ls_info.dist = ls_drv.dist;

	  if(ls_drv.enable_in_work & LSENSOR_EN_LCD)
	  {
		lsenosr_update_bkl();
	  }
	  
      mod_timer(&ls_timer, jiffies + ls_drv.jiff_em_polling_interval);
    }
    goto exit;
  }

  if(ls_drv.in_early_suspend) 
    a_en = 0;
  else if(ls_drv.enable_in_work & LSENSOR_EN_AP ||
    ls_drv.enable_in_work & LSENSOR_EN_LCD)
    a_en = 1;
  else
    a_en = 0;  
  
  if(ls_drv.enable_in_work & PSENSOR_EN_AP)
    p_en = 1;
  else
    p_en = 0;

  
  if(a_en)  SET_BIT(r00_0A[1],2);
  else      CLR_BIT(r00_0A[1],2);
  if(p_en)  SET_BIT(r00_0A[1],7);
  else      CLR_BIT(r00_0A[1],7);

  
  if(a_en && p_en)    
  {
    if(ls_drv.dist)   
    {
      
      irq_active_high = 0;
      if(a_flag)  CLR_BIT(r00_0A[2],3); 
      else        SET_BIT(r00_0A[2],3);
      if(p_flag)  CLR_BIT(r00_0A[2],7); 
      else        SET_BIT(r00_0A[2],7);
      if(time_before(jiffies, ls_drv.jiff_resume_fast_update_time)) 
      {
        
        CLR_BIT(r00_0A[2],1); 
        CLR_BIT(r00_0A[2],2);
      }
      else  
      {
        
        CLR_BIT(r00_0A[2],1); 
        SET_BIT(r00_0A[2],2);
      }
      r00_0A[5] = 0x00; 
      r00_0A[6] = 0x08; 
      r00_0A[7] = 0x80; 
    }
    else  
    {
      
      
      irq_active_high = 0;
      if(a_flag)  CLR_BIT(r00_0A[2],3); 
      else        SET_BIT(r00_0A[2],3);
      if(p_flag)  CLR_BIT(r00_0A[2],7); 
      else        SET_BIT(r00_0A[2],7);
      
      CLR_BIT(r00_0A[2],1); 
      CLR_BIT(r00_0A[2],2);
      r00_0A[5] = 0x00; 
      r00_0A[6] = 0x08; 
      r00_0A[7] = 0x80; 
    }
  }
  else if(a_en) 
  {
    
    irq_active_high = 0;
    if(a_flag)  CLR_BIT(r00_0A[2],3); 
    else        SET_BIT(r00_0A[2],3);
    if(p_flag)  CLR_BIT(r00_0A[2],7); 
    else        SET_BIT(r00_0A[2],7);
    if(time_before(jiffies, ls_drv.jiff_resume_fast_update_time)) 
    {
      
      CLR_BIT(r00_0A[2],1); 
      CLR_BIT(r00_0A[2],2);
    }
    else  
    {
      
      CLR_BIT(r00_0A[2],1); 
      SET_BIT(r00_0A[2],2);
    }
    r00_0A[5] = 0x00; 
    r00_0A[6] = 0x08; 
    r00_0A[7] = 0x80; 
  }
  else if(p_en) 
  {
    if(ls_drv.dist)  
    {
      
      irq_active_high = 0;
      if(a_flag)  CLR_BIT(r00_0A[2],3); 
      else        SET_BIT(r00_0A[2],3);
      if(p_flag)  CLR_BIT(r00_0A[2],7); 
      else        SET_BIT(r00_0A[2],7);
    }
    else  
    {
      
      irq_active_high = 1;
      if(a_flag)  CLR_BIT(r00_0A[2],3); 
      else        SET_BIT(r00_0A[2],3);
      
    }
  }
  
  if(r00_0A[5] != r00_0A_old[5] ||
     r00_0A[6] != r00_0A_old[6] ||
     r00_0A[7] != r00_0A_old[7])
  {
    lsensor_write_i2c(0x44,0x05,&r00_0A[5],3);
  }

  
  lsensor_write_i2c(0x44,0x01,&r00_0A[1],2);

  if(ls_drv.irq_active_high != irq_active_high)
  {
    ls_drv.irq_active_high = irq_active_high;
    if(ls_drv.irq_active_high)
    {
      irq_set_irq_type(MSM_GPIO_TO_INT(ls_drv.intr_gpio), IRQF_TRIGGER_HIGH);
      ALS_PRINTK(2,"IRQF_HIGH");
    }
    else
    {
      irq_set_irq_type(MSM_GPIO_TO_INT(ls_drv.intr_gpio), IRQF_TRIGGER_LOW);
      ALS_PRINTK(2,"IRQF_LOW");
    }
  }

  
  
  if(a_en && (ls_drv.enable_in_work & LSENSOR_EN_LCD))
    lsenosr_update_bkl();

  
  
  if(a_en || p_en)
    lsensor_irq_onOff(1);

exit:
  
  
  if(ls_drv.info_waiting)
  {
    ls_drv.info_waiting = 0;
    complete(&(ls_drv.info_comp));
  }
  
  
  mutex_lock(&lsensor_enable_lock);
  if(ls_drv.enable_in_work != ls_drv.enable)
  {
    ALS_PRINTK(1,"%s, enable_in_work=%02X, enable=%02X (not sync!)",__func__,ls_drv.enable_in_work,ls_drv.enable);
    queue_work(ls_drv.wqueue, &ls_drv.work);
  }
  mutex_unlock(&lsensor_enable_lock);
  
  return;
}




static irqreturn_t lsensor_irqhandler(int irq, void *dev_id)
{
  lsensor_irq_onOff(0);
  if(!ls_drv.inited)
  {
    ALS_PRINTK(1,"%s, lsensor not inited! Disable irq!", __func__);
    disable_irq_nosync(MSM_GPIO_TO_INT(ls_drv.intr_gpio));
    goto exit;
  }
  if(ls_drv.in_suspend) {
    ALS_PRINTK(1,"%s, in_suspend", __func__); 
  }
  else
    queue_work(ls_drv.wqueue, &ls_drv.work);
exit:
  return IRQ_HANDLED;
}

static void lsensor_timer_func(unsigned long temp)
{
  if(!ls_drv.inited)
    return;
  if(ls_drv.in_suspend || ls_drv.in_early_suspend)  
  {
    
    
  }
  else
  {
    
    queue_work(ls_drv.wqueue, &ls_drv.work);
  }
}



void lsensor_enable_onOff(unsigned int mode, unsigned int onOff)  
{
  static unsigned int als_pm_on = 0, prox_pm_on = 0;
  
  
  mutex_lock(&lsensor_enable_lock);
  if(mode==LSENSOR_EN_AP || mode==LSENSOR_EN_LCD ||
    mode==LSENSOR_EN_ENG || mode==PSENSOR_EN_AP )
  {
    if(onOff) ls_drv.enable |=   mode;
    else      ls_drv.enable &= (~mode);
  }
  
  
  if(ls_drv.enable & LSENSOR_EN_AP || ls_drv.enable & LSENSOR_EN_LCD) 
  {
    if(!als_pm_on)
    {
      als_pm_on = 1;
      PM_LOG_EVENT(PM_LOG_ON, PM_LOG_SENSOR_ALS);
    }
  }
  else  
  {
    if(als_pm_on)
    {
      als_pm_on = 0;
      PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_SENSOR_ALS);
    }
  }
  if(ls_drv.enable & PSENSOR_EN_AP) 
  {
    if(!prox_pm_on)
    {
      prox_pm_on = 1;
	  wake_lock(&ls_drv.wlock);
      PM_LOG_EVENT(PM_LOG_ON, PM_LOG_SENSOR_PROXIMITY);
	  
    }
  }
  else  
  {
    if(prox_pm_on)
    {
      prox_pm_on = 0;
	  wake_unlock(&ls_drv.wlock);
      PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_SENSOR_PROXIMITY);
    }
  }

  mutex_unlock(&lsensor_enable_lock);

  if(ls_drv.in_suspend) 
    return;
  if(ls_drv.enable)
    queue_work(ls_drv.wqueue, &ls_drv.work);

  
}


static int isl29028_smbus_read_byte(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data)
{
  uint8_t dummy;
  dummy = i2c_smbus_read_byte_data(client, reg_addr);
  if (dummy < 0)
    return -1;
  *data = dummy & 0x000000ff;

  return 0;
}

static int isl29028_smbus_write_byte(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data)
{
  s32 dummy;
  dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
  if (dummy < 0)
    return -1;
  return 0;
}

static int isl29028_write_reg(struct i2c_client *client, unsigned char addr, unsigned char *data)
{
  int comres = 0 ;
  comres = isl29028_smbus_write_byte(client, addr, data);

  return comres;
}

static void a2h(char *in, char *out) 
{
  int i;
  char a, h[2];
  for(i=0; i<2; i++)
  {
    a = *in++;
    if(a <= '9')        h[i] = a - '0';
    else if (a <= 'F')  h[i] = a - 'A' + 10;
    else if (a <= 'f')  h[i] = a - 'a' + 10;
    else                h[i] = 0;
  }
  *out = (h[0]<<4) + h[1];
}
static const char my_ascii[] = "0123456789ABCDEF";
static void h2a(char *in, char *out) 
{
  char c = *in;
  *out++ =  my_ascii[c >> 4];
  *out =    my_ascii[c & 0xF];
}
#define QSD_BAT_BUF_LENGTH  256
void lsensor_i2c_test(unsigned char *bufLocal, size_t count)
{
  struct i2c_msg msgs[2];
  int i2c_ret, i, j;
  char id, reg[2], len, dat[QSD_BAT_BUF_LENGTH/4];

  printk(KERN_INFO "\n");

  if(bufLocal[1]=='r' && count>=9)
  {
    a2h(&bufLocal[2], &id);     
    a2h(&bufLocal[4], &reg[0]); 
    a2h(&bufLocal[6], &len);    
    if(len >= sizeof(dat))
    {
      ALS_PRINTK(1,"R %02X:%02X(%02d) Fail: max length=%d", id,reg[0],len,sizeof(dat));
      return;
    }

    msgs[0].addr = id;
    msgs[0].flags = 0;
    msgs[0].buf = &reg[0];
    msgs[0].len = 1;

    msgs[1].addr = id;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = &dat[0];
    msgs[1].len = len;

    i2c_ret = i2c_transfer(ls_drv.client->adapter, msgs,2);
    if(i2c_ret != 2)
    {
      ALS_PRINTK(1,"R %02X:%02X(%02d) Fail: ret=%d", id,reg[0],len,i2c_ret);
      return;
    }

    j = 0;
    for(i=0; i<len; i++)
    {
      h2a(&dat[i], &bufLocal[j]);
      bufLocal[j+2] = ' ';
      j = j + 3;
    }
    bufLocal[j] = '\0';
    ALS_PRINTK(1,"R %02X:%02X(%02d) = %s", id,reg[0],len,bufLocal);
  }

  else if(bufLocal[1]=='R' && count>=11)
  {
    a2h(&bufLocal[2], &id);     
    a2h(&bufLocal[4], &reg[0]); 
    a2h(&bufLocal[6], &reg[1]); 
    a2h(&bufLocal[8], &len);    
    if(len >= sizeof(dat))
    {
      ALS_PRINTK(1,"R %02X:%02X%02X(%02d) Fail (max length=%d)", id,reg[0],reg[1],len,sizeof(dat));
      return;
    }

    msgs[0].addr = id;
    msgs[0].flags = 0;
    msgs[0].buf = &reg[0];
    msgs[0].len = 2;

    msgs[1].addr = id;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = &dat[0];
    msgs[1].len = len;

    i2c_ret = i2c_transfer(ls_drv.client->adapter, msgs,2);
    if(i2c_ret != 2)
    {
      ALS_PRINTK(1,"R %02X:%02X%02X(%02d) Fail (ret=%d)", id,reg[0],reg[1],len,i2c_ret);
      return;
    }
    j = 0;
    for(i=0; i<len; i++)
    {
      h2a(&dat[i], &bufLocal[j]);
      bufLocal[j+2] = ' ';
      j = j + 3;
    }
    bufLocal[j] = '\0';
    ALS_PRINTK(1,"R %02X:%02X%02X(%02d) = %s", id,reg[0],reg[1],len,bufLocal);
  }

  else if(bufLocal[1]=='w' && count>=9)
  {
    a2h(&bufLocal[2], &id);     
    len = count - 5;
    if(len & 1)
    {
      ALS_PRINTK(1,"W %02X Fail (invalid data) len=%d", id,len);
      return;
    }
    len = len/2;
    if(len >= sizeof(dat))
    {
      ALS_PRINTK(1,"W %02X Fail (too many data)", id);
      return;
    }

    j = 4;
    for(i=0; i<len; i++)
    {
      a2h(&bufLocal[j], &dat[i]);
      j = j + 2;
    }

    msgs[0].addr = id;
    msgs[0].flags = 0;
    msgs[0].buf = &dat[0];
    msgs[0].len = len;

    i2c_ret = i2c_transfer(ls_drv.client->adapter, msgs,1);
    
    ALS_PRINTK(1,"W %02X = %s", id, i2c_ret==1 ? "Pass":"Fail");
  }
  else
  {
    ALS_PRINTK(1,"rd: r40000B   (addr=40(7bit), reg=00, read count=11");
    ALS_PRINTK(1,"Rd: R2C010902 (addr=2C(7bit), reg=0109, read count=2");
    ALS_PRINTK(1,"wr: w40009265CA (addr=40(7bit), reg & data=00,92,65,CA...");
  }
}

static ssize_t lsensor_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  
  return 0;
}
static ssize_t lsensor_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  char bufLocal[QSD_BAT_BUF_LENGTH];
  char reg_00_0A[11];
  int Pinvalue;
  static struct lsensor_cal_data cal_data;

  if(!ls_drv.inited)
    goto exit;

  printk(KERN_INFO "\n");
  if(count >= sizeof(bufLocal))
  {
    ALS_PRINTK(1,"%s input invalid, count = %d", __func__, count);
    return count;
  }
  memcpy(bufLocal,buf,count);

  switch(bufLocal[0])
  {

    case 'z':
      if(bufLocal[1]=='0')
      {
        ALS_PRINTK(1,"Dynamic Log Off");
        ALS_DLL = 0;

      }
      else if(bufLocal[1]=='1')
      {
        ALS_PRINTK(1,"Dynamic Log On");
        ALS_DLL = 1;
      }
      else if(bufLocal[1]=='2')
      {
        ALS_PRINTK(1,"Dynamic Log On");
        ALS_DLL = 2;
      }
	  else if(bufLocal[1]=='3')
      {
        ALS_PRINTK(1,"Dynamic Log On");
        ALS_DLL = 3;
      }
      break;

    case 'a': 
      if(bufLocal[1]=='0')
      {
        ALS_PRINTK(1,"ALS -");
        lsensor_enable_onOff(LSENSOR_EN_AP,0);
      }
      else if(bufLocal[1]=='1')
      {
        ALS_PRINTK(1,"ALS +");
        lsensor_enable_onOff(LSENSOR_EN_AP,1);
      }
      else if(bufLocal[1]=='2')
      {
        ALS_PRINTK(1,"Prox -");
        lsensor_enable_onOff(PSENSOR_EN_AP,0);
      }
      else if(bufLocal[1]=='3')
      {
        ALS_PRINTK(1,"Prox +");
        lsensor_enable_onOff(PSENSOR_EN_AP,1);
      }
      break;

  case 'b':
    switch(bufLocal[1])
      {
        case '0':
          ALS_PRINTK(1,"Calibration");
          lsensor_calibration(&cal_data);
          break;
    case '1':
          ALS_PRINTK(1,"Write ALS  NV");
           ls_drv.als_nv  = cal_data.als;
           ALS_PRINTK(1,"als_nv=0x%X", ls_drv.als_nv);
          break;
        case '2':
          ALS_PRINTK(1,"Write PROX FAR NV");
          cal_data.prox.prox_near = (ls_drv.prox_nv>>8) & 0xFF;
          cal_data.prox.prox_far  = cal_data.prox.prox_far;
          ls_drv.prox_nv = cal_data.prox.prox_far + (cal_data.prox.prox_near<<8);
          ALS_PRINTK(1,"prox_nv=0x%X", ls_drv.prox_nv);
          break;
        case '3':
          ALS_PRINTK(1,"Write ALS + PROX FAR NV");
          cal_data.prox.prox_near = (ls_drv.prox_nv>>8) & 0xFF;
          cal_data.prox.prox_far  = cal_data.prox.prox_far;
          ls_drv.als_nv  = cal_data.als;
          ls_drv.prox_nv = cal_data.prox.prox_far + (cal_data.prox.prox_near<<8);
          ALS_PRINTK(1,"als_nv=0x%X prox_nv=0x%X", ls_drv.als_nv, ls_drv.prox_nv);
          break;
        case '4':
          ALS_PRINTK(1,"Clear ALS + PROX NV");
          cal_data.als = 0xFFF;
          cal_data.prox.prox_near = 0xFF;
          cal_data.prox.prox_far  = 0xFF;
          ls_drv.als_nv  = 0xFFF;
          ls_drv.prox_nv = 0xFFFF;
          ALS_PRINTK(1,"als_nv=0xFFF prox_nv=0xFFFF");
          break;
        case '5':
          ALS_PRINTK(1,"Read NV");
          ls_drv.als_nv   = cal_data.als;
          ls_drv.prox_nv  = cal_data.prox.prox_far + (cal_data.prox.prox_near<<8);
          ALS_PRINTK(1,"%s ALS=%d, PROX_FAR=%d, PROX_NEAR=%d",
            __func__, cal_data.als, cal_data.prox.prox_far, cal_data.prox.prox_near);
          break;
    case '6':
          ALS_PRINTK(1,"Update m_ga");
          lsensor_update_equation_parameter(&cal_data);
          break;
        case '7':
          ALS_PRINTK(1,"Update prox_threshold");
          lsensor_update_equation_parameter(&cal_data);
          break;
    case '8':
          ALS_PRINTK(1,"Write PROX NEAR NV");
          cal_data.prox.prox_near = cal_data.prox.prox_far; 
          cal_data.prox.prox_far  = ls_drv.prox_nv & 0xFF;
          ls_drv.prox_nv = cal_data.prox.prox_far + (cal_data.prox.prox_near<<8);
          ALS_PRINTK(1,"prox_nv=0x%X", ls_drv.prox_nv);
          break;
    default:
          ALS_PRINTK(1,"[0]cal, [1]write als, [2]write prox far, [3]write als+prox far, \n[4]clear nv,"   \
            "[5]read nv, [6]update m_ga, [7]update prox_threshold [8]write prox near");
   }
   break;

    case 'd': 
      switch(bufLocal[1])
      {
        case '0':
          ALS_PRINTK(1,"Disable Irq");
          disable_irq(MSM_GPIO_TO_INT(ls_drv.intr_gpio));
          break;
        case '1':
          ALS_PRINTK(1,"Enable Irq");
          enable_irq(MSM_GPIO_TO_INT(ls_drv.intr_gpio));
          break;
        case '2':
          ALS_PRINTK(1,"Disable Wake");
          irq_set_irq_wake(MSM_GPIO_TO_INT(ls_drv.intr_gpio), 0);
          break;
        case '3':
          ALS_PRINTK(1,"Enable Wake");
          irq_set_irq_wake(MSM_GPIO_TO_INT(ls_drv.intr_gpio), 1);
          break;
      }
      break;
	  
	case 'g': 
	  switch(bufLocal[1])
	  {
	    case '0':
		  ALS_PRINTK(1,"Set gpio 42 Low");
		  gpio_direction_output(ls_drv.pwr_gpio, 0);
		  break;
		case '1':
		  ALS_PRINTK(1,"Set gpio 42 High");
		  gpio_direction_output(ls_drv.pwr_gpio, 1);
		  break;
		case '2':
		  ALS_PRINTK(1,"Read gpio 42 value");
		  Pinvalue = gpio_get_value(ls_drv.pwr_gpio);
		  ALS_PRINTK(1,"[ALS] gpio 42 value= %d\n",Pinvalue);
		  break;
	  }
	  break;

    case 'e':
      break;

    case 'y': 
      lsensor_read_i2c(0x44,0x00,&reg_00_0A[0],sizeof(reg_00_0A));
      ALS_PRINTK(1,"[01-07]%02X %02X P:%02X %02X  A:%02X %02X %02X (%03X~%03X)",
        reg_00_0A[1],reg_00_0A[2],reg_00_0A[3],reg_00_0A[4],
        reg_00_0A[5],reg_00_0A[6],reg_00_0A[7],
        reg_00_0A[5] + ((reg_00_0A[6] & 0x0F)<<8),
        (reg_00_0A[6]>>4) + (reg_00_0A[7]<<4));
      ALS_PRINTK(1,"[08-0A]%03d %04d",reg_00_0A[8], reg_00_0A[9] + (reg_00_0A[10]<<8));
      ALS_PRINTK(1,"INT = %d",gpio_get_value(ls_drv.intr_gpio));
      break;

    case 'i':
      lsensor_i2c_test(bufLocal, count);
      break;

    default:
      break;
  }
exit:
  return count;
}

static ssize_t lsensor_ftd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  unsigned char r00;
  if(lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00)) != 2)
    r00 = 0;
  
  return sprintf(buf, "%02X\n", r00);
}


static ssize_t lsensor_ftd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  char bufLocal[128];

  if(!ls_drv.inited)
    goto exit;

  printk(KERN_INFO "\n");
  if(count >= sizeof(bufLocal))
  {
    ALS_PRINTK(1,"%s input invalid, count = %d", __func__, count);
    return count;
  }
  memcpy(bufLocal,buf,count);

  switch(bufLocal[0])
  {
    
    case 'm':
      if(bufLocal[1]=='1')
      {
        ALS_PRINTK(1,"Test Mode = ON");

      }
      else
      {
        ALS_PRINTK(1,"Test Mode = OFF");

      }
      break;

    case 'd':
      if(bufLocal[1] == '0')
      {
        ALS_PRINTK(1,"Stop demo");
      }
      else if(bufLocal[1] == '1')
      {
        ALS_PRINTK(1,"Start demo");
      }
      else if(bufLocal[1] == '2')
      {
        ALS_PRINTK(1,"red, orange, yellow, green");
      }
      else if(bufLocal[1] == '3')
      {
        ALS_PRINTK(1,"cherry, turquoise, blue, purple1");
      }
      else if(bufLocal[1] == '4')
      {
        ALS_PRINTK(1,"purple2, white, red, orange");
      }
      break;

    default:
      break;
  }
exit:
  return count;
}

static ssize_t isl29028_reg_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
  size_t count = 0;
  uint8_t reg[11];
  int i;

  for (i = 0; i < 11; i++) {
    isl29028_smbus_read_byte(ls_drv.client ,i,reg+i);
    count += sprintf(&buf[count], "0x%x: %02X\n", i, reg[i]);
  }

  return count;
}

static ssize_t isl29028_register_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
  int address, value;

  sscanf(buf, "%d %02X", &address, &value);

  if (isl29028_write_reg(ls_drv.client , (unsigned char)address, (unsigned char *)&value) < 0)
    return -EINVAL;

  return count;
}

static struct device_attribute lsensor_ctrl_attrs[] = {
  __ATTR(ctrl, 0664, lsensor_ctrl_show, lsensor_ctrl_store),
  __ATTR(ftd, 0664, lsensor_ftd_show, lsensor_ftd_store),   
  __ATTR(isl_reg, 0664,isl29028_reg_show, isl29028_register_store),
};


static void lsensor_early_suspend_func(struct early_suspend *h)
{
  ALS_PRINTK(2,"%s+", __func__);
  ls_drv.in_early_suspend = 1;
  del_timer_sync(&ls_timer);
  lsensor_enable_onOff(0,0);
  flush_workqueue(ls_drv.wqueue);

  if(ls_drv.enable & LSENSOR_EN_LCD)  
      lsensor_bkl_fading(ls_drv.bkl_now, 0);
  ALS_PRINTK(2,"%s-", __func__);
}
static void lsensor_late_resume_func(struct early_suspend *h)
{
  ALS_PRINTK(2,"%s+", __func__);
  ls_drv.in_early_suspend = 0;

  if(ls_drv.enable & LSENSOR_EN_LCD)  
      lsensor_bkl_fading(0, ls_drv.bkl_now);
  ls_drv.jiff_update_bkl_wait_time    = jiffies + 2*HZ;
  ls_drv.jiff_resume_fast_update_time = jiffies + 7*HZ;
  lsensor_enable_onOff(0,0);
  ALS_PRINTK(2,"%s-", __func__);
}
static struct early_suspend lsensor_early_suspend = {
  .level    = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2,
  .suspend  = lsensor_early_suspend_func,
  .resume   = lsensor_late_resume_func,
};

static void lsensor_create_kernel_debuglevel(void)
{
	ALS_PRINTK(1, "create kernel debuglevel!!!\n");
	
	if (kernel_debuglevel_dir!=NULL)
	{
	  debugfs_create_u32("als_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&ALS_DLL));
	}
	else
	{
		printk(KERN_ERR "kernel_debuglevel_dir als_dl Fail\n");
	}
}

static void lsensor_destroy_kernel_debuglevel(void)
{
	ALS_PRINTK(1, "destroy kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}

static int lsensor_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
  ALS_PRINTK(2,"%s+", __func__);
  flush_workqueue(ls_drv.wqueue);
  ls_drv.in_suspend = 1;
  ALS_PRINTK(2,"%s-", __func__);
  return 0;
}
static int lsensor_i2c_resume(struct i2c_client *client)
{
  ALS_PRINTK(2,"%s+", __func__);
  ls_drv.in_suspend = 0;
  lsensor_enable_onOff(0,0);
  ALS_PRINTK(2,"%s-", __func__);
  return 0;
}
static int lsensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int ret=0, i;
  char reg_00_0A[16];
  ALS_PRINTK(2,"%s+", __func__);
  ls_drv.client = client;

  reg_00_0A[1] = 0x00;
  lsensor_write_i2c(0x44,0x01,&reg_00_0A[1],sizeof(reg_00_0A[1]));
  reg_00_0A[15] = 0x29;
  lsensor_write_i2c(0x44,0x0F,&reg_00_0A[15],sizeof(reg_00_0A[15]));
  reg_00_0A[14] = 0x00;
  lsensor_write_i2c(0x44,0x0E,&reg_00_0A[14],sizeof(reg_00_0A[14]));
  reg_00_0A[15] = 0x00;
  lsensor_write_i2c(0x44,0x0F,&reg_00_0A[15],sizeof(reg_00_0A[15]));
  msleep(3);

  lsensor_read_i2c(0x44,0x00,&reg_00_0A[0],sizeof(reg_00_0A));
  for(i = 0; i<16; i++) {
    ALS_PRINTK(2,"reg[%d]: %02X", i, reg_00_0A[i]);
  }

  
  {
    unsigned char reg_init[] = {
      0x00, 

      0x52, 
      0x24, 
      ls_drv.prox_threshold_far,  
      ls_drv.prox_threshold,      
      0x00, 
      0x08, 
      0x80, 
      0x00, 
      0x00, 
      0x00, 
    };
    lsensor_write_i2c(0x44,0x00,&reg_init[0],sizeof(reg_init));
  }
  
  {
    unsigned char r00 = 0;
    lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00));
    ALS_PRINTK(1,"%s, Reg00 = %02X", __func__, r00);
  }

  ALS_PRINTK(2,"%s-", __func__);
  return ret;
}

void lsensor_i2c_shutdown(struct i2c_client *client)
{
  unsigned char r01 = 0x00;
  ALS_PRINTK(1,"%s", __func__);
  
  lsensor_write_i2c(0x44,0x01,&r01,sizeof(r01));
  
  if (ls_drv.pwr_gpio) {
    gpio_direction_output(ls_drv.pwr_gpio, 0);
    
  }
  
}

static const struct i2c_device_id lsensor_i2c_id[] = {
  { "isl29028", 0 },
  { }
};
static struct i2c_driver lsensor_i2c_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name = "isl29028"
  },
  .id_table = lsensor_i2c_id,
  .probe    = lsensor_i2c_probe,
  .suspend  = lsensor_i2c_suspend,
  .resume   = lsensor_i2c_resume,
  .shutdown = lsensor_i2c_shutdown,
};

static int lsensor_misc_open(struct inode *inode_p, struct file *fp)
{
  ls_drv.opened ++;
  ALS_PRINTK(1,"%s    <== [%d] (%04d),Task name:%s",__func__,ls_drv.opened,current->pid,current->comm);
  return 0;
}
static int lsensor_misc_release(struct inode *inode_p, struct file *fp)
{
  ls_drv.opened --;
  ALS_PRINTK(1,"%s <== [%d] (%04d),Task name:%s\n",__func__,ls_drv.opened,current->pid,current->comm);
  return 0;
}
static long lsensor_misc_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
  int ret = 0;
  long timeout;
  unsigned char r00, r00_0A[11], bkl_from_hal;
  unsigned int adata, pdata;
  struct lsensor_cal_data cal_data;

  if(_IOC_TYPE(cmd) != LSENSOR_IOC_MAGIC)
  {
    ALS_PRINTK(0,"%s: Not LSENSOR_IOC_MAGIC", __func__);
    return -ENOTTY;
  }
  if(_IOC_DIR(cmd) & _IOC_READ)
  {
    ret = !access_ok(VERIFY_WRITE, (void __user*)arg, _IOC_SIZE(cmd));
    if(ret)
    {
      ALS_PRINTK(0,"%s: access_ok check write err", __func__);
      return -EFAULT;
    }
  }
  if(_IOC_DIR(cmd) & _IOC_WRITE)
  {
    ret = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
    if(ret)
    {
      ALS_PRINTK(0,"%s: access_ok check read err", __func__);
      return -EFAULT;
    }
  }

  switch (cmd)
  {

    case LIGHTSENSOR_IOCTL_GET_CHIPID:
      ALS_PRINTK(2,"%s: LIGHTSENSOR_IOCTL_GET_CHIPID", __func__);
      if(lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00)) != 2)
        r00 = 0;
      ret = copy_to_user((void __user*) arg, &r00, _IOC_SIZE(cmd));
      break;

    case LIGHTSENSOR_IOCTL_ENABLE:
      if(ls_drv.eng_mode)
      {
        ALS_PRINTK(1,"%s: LIGHTSENSOR_IOCTL_ENABLE Skip (Eng Mode)", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(1,"%s: (%04d) LIGHTSENSOR_IOCTL_ENABLE", __func__,current->pid);
        lsensor_enable_onOff(LSENSOR_EN_AP,1);
      }
      break;

    case LIGHTSENSOR_IOCTL_DISABLE:
      if(ls_drv.eng_mode)
      {
        ALS_PRINTK(1,"%s: LIGHTSENSOR_IOCTL_DISABLE Skip (Eng Mode)", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(1,"%s: (%04d) LIGHTSENSOR_IOCTL_DISABLE", __func__,current->pid);
        lsensor_enable_onOff(LSENSOR_EN_AP,0);
      }
      break;

    case LIGHTSENSOR_IOCTL_GET_ADC:
      ALS_PRINTK(2,"%s: LIGHTSENSOR_IOCTL_GET_ADC", __func__);
      lsensor_read_i2c(0x44,0x00,&r00_0A[0],sizeof(r00_0A));
      adata = r00_0A[9] + (r00_0A[10] << 8);
      return put_user(adata, (unsigned long __user *)arg);
      break;

    case LIGHTSENSOR_IOCTL_ENABLE_AUTO_BKL:
      ALS_PRINTK(2,"%s: LIGHTSENSOR_IOCTL_ENABLE_AUTO_BKL", __func__);
      if(copy_from_user(&bkl_from_hal, (void __user*) arg, _IOC_SIZE(cmd)))
      {
        ALS_PRINTK(0,"%s: LIGHTSENSOR_IOCTL_ENABLE_AUTO_BKL Fail", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(1,"Backlight Mode = [SENSOR] (%d)",bkl_from_hal);
        ls_drv.bkl_now = lsenosr_update_idx(bkl_from_hal);
        ls_drv.lux_history[0] = ls_drv.bkl_now;
        ls_drv.lux_history[1] = ls_drv.bkl_now;
        ls_drv.lux_history[2] = ls_drv.bkl_now;
        ls_drv.jiff_update_bkl_wait_time = jiffies;
        ls_drv.jiff_resume_fast_update_time = jiffies + 7*HZ;
        lsensor_enable_onOff(LSENSOR_EN_LCD,1);
      }
      break;
    case LIGHTSENSOR_IOCTL_DISABLE_AUTO_BKL:
      ALS_PRINTK(2,"%s: LIGHTSENSOR_IOCTL_DISABLE_AUTO_BKL", __func__);
      if(copy_from_user(&bkl_from_hal, (void __user*) arg, _IOC_SIZE(cmd)))
      {
        ALS_PRINTK(0,"%s: LIGHTSENSOR_IOCTL_DISABLE_AUTO_BKL Fail", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(1,"Backlight Mode = [USER] (%d)",bkl_from_hal);
        lsensor_enable_onOff(LSENSOR_EN_LCD,0);
        lsensor_bkl_fading(ls_drv.bkl_now, bkl_from_hal);
      }
      break;

    case PROXIMITY_SENSOR_IOCTL_GET_CHIPID:
      ALS_PRINTK(2,"%s: PROXIMITY_SENSOR_IOCTL_GET_CHIPID", __func__);
      if(lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00)) != 2)
        r00 = 0;
      ret = copy_to_user((void __user*) arg, &r00, _IOC_SIZE(cmd));
      break;

    case PROXIMITY_SENSOR_IOCTL_ENABLE:
      if(ls_drv.eng_mode)
      {
        ALS_PRINTK(1,"%s: PROXIMITY_SENSOR_IOCTL_ENABLE Skip (Eng Mode)", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(1,"%s: (%04d) PROXIMITY_SENSOR_IOCTL_ENABLE, prox %d", __func__,current->pid,ls_drv.dist);
        lsensor_enable_onOff(PSENSOR_EN_AP,1);
		lsensor_update_threshold();
        
        input_report_abs(ls_drv.input_prox, ABS_DISTANCE, ls_drv.dist);
        input_sync(ls_drv.input_prox);
        ls_drv.dist_old = ls_drv.dist;
        
      }
      break;

    case PROXIMITY_SENSOR_IOCTL_DISABLE:
      if(ls_drv.eng_mode)
      {
        ALS_PRINTK(1,"%s: PROXIMITY_SENSOR_IOCTL_DISABLE Skip (Eng Mode)", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(1,"%s: (%04d) PROXIMITY_SENSOR_IOCTL_DISABLE, prox %d->1", __func__,current->pid,ls_drv.dist_old);
        lsensor_enable_onOff(PSENSOR_EN_AP,0);
        
        ls_drv.dist = 1;
        ls_drv.dist_old  = 1;
        input_report_abs(ls_drv.input_prox, ABS_DISTANCE, ls_drv.dist);
        input_sync(ls_drv.input_prox);
        
      }
      break;

    case PROXIMITY_SENSOR_IOCTL_GET_ADC:
      ALS_PRINTK(2,"%s: PROXIMITY_SENSOR_IOCTL_GET_ADC", __func__);
      lsensor_read_i2c(0x44,0x00,&r00_0A[0],sizeof(r00_0A));
        pdata = r00_0A[8];
      return put_user(pdata, (unsigned long __user *)arg);
      break;

    case LIGHTSENSOR_IOCTL_CALIBRATE:
      if(lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00)) != 2) {
        cal_data.als = 0xFFFFFFFF;
        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
        ALS_PRINTK(1,"%s: LIGHTSENSOR_IOCTL_CALIBRATE Fail!", __func__);
      } else {
        lsensor_calibration(&cal_data);
        ALS_PRINTK(1,"%s: (%04d) LIGHTSENSOR_IOCTL_CALIBRATE", __func__,current->pid);

        cal_data.prox.prox_near = cal_data.prox.prox_far;
        cal_data.prox.prox_far  = cal_data.prox.prox_far;

        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
      }
      break;

    case LIGHTSENSOR_IOCTL_CAL_ALS:
      if(lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00)) != 2) {
        cal_data.als = 0xFFFFFFFF;
        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
        ALS_PRINTK(1,"%s: LIGHTSENSOR_IOCTL_CAL_ALS Fail!", __func__);
      } else {
        lsensor_calibration(&cal_data);
        ALS_PRINTK(1,"%s: (%04d) LIGHTSENSOR_IOCTL_CAL_ALS", __func__,current->pid);

        ls_drv.als_nv = cal_data.als;

        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
      }
      break;

    case LIGHTSENSOR_IOCTL_CAL_FAR:
      if(lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00)) != 2) {
        cal_data.prox.prox_far = 0xFFFF;
        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
        ALS_PRINTK(1,"%s: LIGHTSENSOR_IOCTL_CAL_FAR Fail!", __func__);
      } else {
        lsensor_calibration(&cal_data);
        ALS_PRINTK(1,"%s: (%04d) LIGHTSENSOR_IOCTL_CAL)_FAR", __func__,current->pid);

        cal_data.prox.prox_near = (ls_drv.prox_nv >> 8) & 0xFF;
        cal_data.prox.prox_far = cal_data.prox.prox_far;

        ls_drv.prox_nv  = cal_data.prox.prox_far + (cal_data.prox.prox_near << 8);

        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
      }
      break;

    case LIGHTSENSOR_IOCTL_CAL_NEAR:
      if(lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00)) != 2) {
        cal_data.prox.prox_far = 0xFFFF;
        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
        ALS_PRINTK(1,"%s: LIGHTSENSOR_IOCTL_CAL_NEAR Fail!", __func__);
      } else {
        lsensor_calibration(&cal_data);
        ALS_PRINTK(1,"%s: (%04d) LIGHTSENSOR_IOCTL_CAL_NEAR", __func__,current->pid);

        cal_data.prox.prox_near = cal_data.prox.prox_far;
        cal_data.prox.prox_far = ls_drv.prox_nv & 0xFF;

        ls_drv.prox_nv  = cal_data.prox.prox_far + (cal_data.prox.prox_near << 8);

        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
      }
      break;

    case LSENSOR_IOC_UPDATE_CAL_DATA:
      ALS_PRINTK(2,"%s: LSENSOR_IOC_UPDATE_CAL_DATA", __func__);
      if(lsensor_read_i2c(0x44,0x00,&r00,sizeof(r00)) != 2) {
        cal_data.als = 0xFFFFFFFF;
        ret = copy_to_user((void __user*) arg, &cal_data, _IOC_SIZE(cmd));
        ALS_PRINTK(1,"%s: LSENSOR_IOC_UPDATE_CAL_DATA Fail!", __func__);
      } else {
        ret = copy_from_user(&cal_data, (void __user*) arg, _IOC_SIZE(cmd));
        ALS_PRINTK(1,"%s: LSENSOR_IOC_UPDATE_CAL_DATA, als=%d, prox_far=0x%04X, prox_near=0x%04X",
          __func__, cal_data.als, cal_data.prox.prox_far, cal_data.prox.prox_near);

        ls_drv.als_nv   = cal_data.als;
        ls_drv.prox_nv  = cal_data.prox.prox_far + (cal_data.prox.prox_near << 8);
        lsensor_update_equation_parameter(&cal_data);
      }
      break;
 
    case LPSENSOR_IOCTL_ENG_ENABLE:
      ALS_PRINTK(2,"%s: LPSENSOR_IOCTL_ENG_ENABLE", __func__);
      ls_drv.eng_mode = 1;
      lsensor_enable_onOff(LSENSOR_EN_ENG,1);
      
      break;

    case LPSENSOR_IOCTL_ENG_DISABLE:
      ALS_PRINTK(2,"%s: LPSENSOR_IOCTL_ENG_DISABLE", __func__);
      ls_drv.eng_mode = 0;
      lsensor_enable_onOff(LSENSOR_EN_ENG,0);
      if(ls_drv.info_waiting)
      {
        ls_drv.info_waiting = 0;
        complete(&(ls_drv.info_comp));
      }
      break;

    case LPSENSOR_IOCTL_ENG_CTL_R:
      if(!ls_drv.eng_mode)
      {
        ALS_PRINTK(1,"%s: LPSENSOR_IOCTL_ENG_CTL_R Skip (Not Eng Mode)", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(2,"%s: LPSENSOR_IOCTL_ENG_CTL_R", __func__);
        
        lsensor_eng_get_reg();
        lsensor_eng_data_log();
        if(copy_to_user((void __user*) arg, &ls_eng, _IOC_SIZE(cmd)))
        {
          ALS_PRINTK(2,"%s: LSENSOR_IOC_ENG_CTL_R Fail", __func__);
          ret = -EFAULT;
        }
      }
      break;

    case LPSENSOR_IOCTL_ENG_CTL_W:
      if(!ls_drv.eng_mode)
      {
        ALS_PRINTK(1,"%s: LPSENSOR_IOCTL_ENG_CTL_W Skip (Not Eng Mode)", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(2,"%s: LPSENSOR_IOCTL_ENG_CTL_W", __func__);
        
        if(copy_from_user(&ls_eng, (void __user*) arg, _IOC_SIZE(cmd)))
        {
          ALS_PRINTK(1,"%s: LPSENSOR_IOCTL_ENG_CTL_W Fail", __func__);
          ret = -EFAULT;
        }
        else
        {
          lsensor_eng_data_log();
          lsensor_eng_set_reg();
        }
		
		lsensor_read_i2c(0x44,0x01,&r00_0A[1],1);
		if( TST_BIT(r00_0A[1],7))
		{
          lsensor_update_threshold();
		}
      }
      break;

    case LPSENSOR_IOCTL_ENG_INFO :
      if(ls_drv.i2c_err)
      {
        ls_info.adata = 0xFFFF;
        ret = copy_to_user((void __user*) arg, &ls_info, _IOC_SIZE(cmd));
        ALS_PRINTK(0,"%s: LPSENSOR_IOCTL_ENG_INFO Fail!", __func__);
      }
      else if(!ls_drv.eng_mode)
      {
        ALS_PRINTK(2,"%s: LPSENSOR_IOCTL_ENG_INFO  Skip (Not Eng Mode)", __func__);
        ret = -EFAULT;
      }
      else
      {
        ALS_PRINTK(3,"%s: LPSENSOR_IOCTL_ENG_INFO ", __func__);
        
        if(ls_drv.info_waiting)
        {
          ALS_PRINTK(1,"ERROR!!! info_waiting was TRUE before use");
        }
        ls_drv.info_waiting = 1;
        INIT_COMPLETION(ls_drv.info_comp);
        timeout = wait_for_completion_timeout(&(ls_drv.info_comp), 10*HZ);
        if(!timeout)
        {
          ALS_PRINTK(1,"ERROR!!! info_comp timeout");
          ls_drv.info_waiting = 0;
        }
        lsensor_info_data_log();

        if(copy_to_user((void __user*) arg, &ls_info, _IOC_SIZE(cmd)))
        {
          ALS_PRINTK(0,"%s: LSENSOR_IOC_ENG_INFO Fail", __func__);
          ret = -EFAULT;
        }
      }
      break;

    default:
      ALS_PRINTK(2,"%s: unknown ioctl = 0x%X", __func__,cmd);
      break;
  }

  
  return ret;
}

static struct file_operations lsensor_misc_fops = {
  .owner    = THIS_MODULE,
  .open     = lsensor_misc_open,
  .release  = lsensor_misc_release,
  .unlocked_ioctl = lsensor_misc_ioctl,
};
static struct miscdevice lsensor_misc_device = {
  .minor  = MISC_DYNAMIC_MINOR,
  .name   = "lsensor_isl29028",
  .fops   = &lsensor_misc_fops,
};

#if defined(CONFIG_DEBUG_FS)
static int lsensor_debugfs_open(struct inode *inode, struct file *file)
{
  
  file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
  return 0;
}
static int lsensor_debugfs_release(struct inode *inode, struct file *file)
{
  return 0;
}
static ssize_t lsensor_debugfs_read(
  struct file *file,
  char __user *buff,
  size_t count,
  loff_t *ppos)
{
  int ret = 0, size;
  
  if(*ppos) 
  {
    size = 0;
    goto exit;
  }
  size = lsensor_ftd_show(NULL, NULL, ls_drv.debug_read_buf);
  if(size > count)
  {
    ret = -EFAULT;
    goto exit;
  }
  if(copy_to_user(buff, ls_drv.debug_read_buf, size))
  {
    ret = -EFAULT;
    goto exit;
  }
exit:
  if(!ret)
  {
    *ppos += size;
    ret = size;
  }
  
  return ret;
}
static ssize_t lsensor_debugfs_write(
  struct file *file,
  const char __user *buff,
  size_t count,
  loff_t *ppos)
{
  int ret = 0;
  
  if(count > sizeof(ls_drv.debug_write_buf))
  {
    ret = -EFAULT;
    goto exit;
  }
  if(copy_from_user(ls_drv.debug_write_buf, buff, count))
  {
    ALS_PRINTK(1,"%s, failed to copy from user",__func__);
    ret = -EFAULT;
    goto exit;
  }
  ls_drv.debug_write_buf[count] = '\0';
  lsensor_ftd_store(NULL, NULL, ls_drv.debug_write_buf, count);
exit:
  if(!ret)
    ret = count;
  
  return ret;
}
static const struct file_operations lsensor_debugfs_fops = {
  .open     = lsensor_debugfs_open,
  .release  = lsensor_debugfs_release,
  .read     = lsensor_debugfs_read,
  .write    = lsensor_debugfs_write,
};
static void lsensor_debug_init(void)
{
  ls_drv.dent = debugfs_create_dir("lsensor_isl29028", 0);
  if (IS_ERR(ls_drv.dent))
    return;
  debugfs_create_file("ftd", 0666, ls_drv.dent, NULL, &lsensor_debugfs_fops);
}
#endif

static int __init lsensor_init(void)
{
  int ret=0, err=0;
  int rc=0;
  int Pinvalue;
  unsigned long flags;

  printk("BootLog, +%s\n", __func__);

  ls_drv.intr_gpio = LSENSOR_GPIO_INT;
  
  if ((system_rev == CHICAGO_EVT3_2) ||(system_rev == CHICAGO_EVT3)) {
    ls_drv.pwr_gpio = LSENSOR_GPIO_PWR_SW;
	ALS_PRINTK(1,"%s: use gpio :%d",__func__, ls_drv.pwr_gpio);
  } else if (system_rev >= CHICAGO_DVT1) {
    ls_drv.pwr_gpio = LSENSOR_GPIO_PWR_SW1;
	ALS_PRINTK(1,"%s: use gpio :%d",__func__, ls_drv.pwr_gpio);
  }

  
  if(ls_drv.pwr_gpio) {
    rc = gpio_request(ls_drv.pwr_gpio, "gpio_isl29028_pwr");
    if (rc < 0) {
	  ALS_PRINTK(0,"%s: gpio %d request failed (%d)\n",
		  __func__, ls_drv.pwr_gpio, rc);
	  return rc;
    }
  
    rc = gpio_tlmm_config(GPIO_CFG(ls_drv.pwr_gpio, 0, GPIO_CFG_OUTPUT,  \
  			  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
    if (rc < 0) {
	  ALS_PRINTK(0,"%s: Fail to config gpio %d ,rc = (%d)\n",
		  __func__, ls_drv.pwr_gpio, rc);
	  return rc;
    }

    rc = gpio_direction_output(ls_drv.pwr_gpio, 1);
    if (rc < 0) {
	  ALS_PRINTK(0,"%s: failed to set gpio %d as output (%d)\n",
		  __func__, ls_drv.pwr_gpio, rc);
    }
 
    Pinvalue = gpio_get_value(ls_drv.pwr_gpio);
    ALS_PRINTK(0," %s:gpio %d value= %d",__func__, ls_drv.pwr_gpio,Pinvalue);
  }
  
  
  
  ret = gpio_request(ls_drv.intr_gpio, "gpio_isl29028_intr");
  if (ret < 0) {
	ALS_PRINTK(0,"%s: gpio %d request failed (%d)\n",
		__func__, ls_drv.intr_gpio, ret);
	return ret;
  }
  ret = gpio_direction_input(ls_drv.intr_gpio);
  if (ret < 0) {
	ALS_PRINTK(0,"%s: failed to set gpio %d as input (%d)",
		__func__, ls_drv.intr_gpio, ret);
  }

  ls_drv.i2c_addr = 0x44;   
  ret = i2c_add_driver(&lsensor_i2c_driver);
  if(ret < 0)
  {
    ALS_PRINTK(0,"%s: i2c_add_driver Fail, ret=%d", __func__, ret);
    err = 3;  goto exit_error;
  }

  ls_drv.info_waiting = 0;
  init_completion(&(ls_drv.info_comp));

  INIT_WORK(&ls_drv.work, lsensor_work_func);
  INIT_WORK(&ls_drv.work_bkl_fading, lsensor_bkl_fading_work_func);
  ls_drv.wqueue = create_singlethread_workqueue("lsensor_workqueue");
  if(ls_drv.wqueue) 
  {
    ALS_PRINTK(0,"%s, lsensor_workqueue created PASS!",__func__);
  }
  else  
  {
    ALS_PRINTK(0,"%s, lsensor_workqueue created FAIL!",__func__);
  }

  ls_drv.jiff_update_bkl_wait_time = jiffies;
  ls_drv.jiff_resume_fast_update_time = jiffies;

  set_irq_flags(MSM_GPIO_TO_INT(ls_drv.intr_gpio), IRQF_VALID | IRQF_NOAUTOEN);
  ret = request_irq(MSM_GPIO_TO_INT(ls_drv.intr_gpio), &lsensor_irqhandler, IRQF_TRIGGER_LOW, "lsensor_isl29028", NULL);
  ls_drv.irq_active_high = 0;

  ls_drv.inited = 1;

  ret = misc_register(&lsensor_misc_device);
  if(ret)
  {
    ALS_PRINTK(0,"%s: lsensor misc_register Fail, ret=%d", __func__, ret);
    err = 4;  goto exit_error;
  }

  
  lsensor_create_kernel_debuglevel();

  #if defined(CONFIG_DEBUG_FS)
    lsensor_debug_init();
  #endif

  #if 1
  
  ret = device_create_file(lsensor_misc_device.this_device, &lsensor_ctrl_attrs[0]);
  if(ret) ALS_PRINTK(1,"%s: create FAIL, ret=%d",lsensor_ctrl_attrs[0].attr.name,ret);

  ret = device_create_file(lsensor_misc_device.this_device, &lsensor_ctrl_attrs[2]);
  if(ret) ALS_PRINTK(1,"%s: create FAIL, ret=%d",lsensor_ctrl_attrs[2].attr.name,ret);
  #endif

  
  wake_lock_init(&ls_drv.wlock, WAKE_LOCK_SUSPEND, "lsensor_active");

  
  
  
  ls_drv.input_als = input_allocate_device();
  if(ls_drv.input_als)
  {
    ALS_PRINTK(0,"als: input_allocate_device: PASS");
  }
  else
  {
    ALS_PRINTK(0,"als: input_allocate_device: FAIL");
    ret = -1;
    goto exit_error;
  }
  input_set_capability(ls_drv.input_als, EV_MSC, MSC_RAW);
  ls_drv.input_als->name = "lsensor_isl29028";
  ret = input_register_device(ls_drv.input_als);
  if(ret)
  {
    ALS_PRINTK(0,"als: input_register_device: FAIL=%d",ret);
    goto exit_error;
  }
  else
  {
    ALS_PRINTK(0,"als: input_register_device: PASS");
  }

  ls_drv.input_prox = input_allocate_device();
  if(ls_drv.input_prox)
  {
    ALS_PRINTK(0,"prox: input_allocate_device: PASS");
  }
  else
  {
    ALS_PRINTK(0,"prox: input_allocate_device: FAIL");
    ret = -1;
    goto exit_error;
  }
  set_bit(EV_ABS, ls_drv.input_prox->evbit);
  input_set_abs_params(ls_drv.input_prox, ABS_DISTANCE, 0, 1, 0, 0);
  ls_drv.input_prox->name = "psensor_isl29028";
  ret = input_register_device(ls_drv.input_prox);
  if(ret)
  {
    ALS_PRINTK(0,"prox: input_register_device: FAIL=%d",ret);
    goto exit_error;
  }
  else
  {
    ALS_PRINTK(0,"prox: input_register_device: PASS");
  }

  
  init_timer(&ls_timer);
  ls_timer.function = lsensor_timer_func;
  ls_timer.expires = jiffies + ls_drv.jiff_em_polling_interval;

  register_early_suspend(&lsensor_early_suspend);

  
  
  spin_lock_irqsave(&lsensor_pstate_lock, flags);
  if (smem_vendor1_data == NULL) {
    smem_vendor1_data = (smem_vendor_id1_apps_data *) smem_alloc (SMEM_ID_VENDOR1, sizeof (smem_vendor_id1_apps_data));
  }

  if (!smem_vendor1_data) {
    ALS_PRINTK(0, "Can't allocate smem_vendor_id1_apps_data\n");
    goto exit_error;
  }
  spin_unlock_irqrestore(&lsensor_pstate_lock, flags);
  

  printk("BootLog, -%s, ret=%d\n", __func__,ret);
  return ret;

exit_error:

  printk("BootLog, -%s, ret=%d\n", __func__,ret);
  return ret;
}
static void __exit lsensor_exit(void)
{
  ALS_PRINTK(1,"%s+", __func__);
  misc_deregister(&lsensor_misc_device);
  lsensor_destroy_kernel_debuglevel();
  
  if (ls_drv.pwr_gpio) {
    
    gpio_direction_output(ls_drv.pwr_gpio, 0);
    gpio_free(ls_drv.pwr_gpio);
  }

  ls_drv.inited = 0;   
  ALS_PRINTK(1,"%s-", __func__);
}

module_init(lsensor_init);
module_exit(lsensor_exit);

