#include <linux/kernel.h>
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
#include <linux/jiffies.h>
#include <linux/wakelock.h>

#include <linux/slab.h>
#ifdef CONFIG_BOARD_ID_ON_ADC
#include <oem_smem_struct.h>
#endif
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif
#include "leds-as3665.h"
#include "leds-as3665_code.h"

#include <mach/chicago_hwid.h>

#if 0
static int as3665_log_on = 0;
static int as3665_log3_on = 0;

#define MSG(format, arg...) {if(as3665_log_on)  printk(KERN_INFO "[LED]" format "\n", ## arg);}
#define MSG2(format, arg...) printk("[LED]" format "\n", ## arg) 
#define MSG3(format, arg...) {if(as3665_log3_on)  printk(KERN_INFO "[LED]" format "\n", ## arg);}
#endif

extern struct dentry *kernel_debuglevel_dir;
static unsigned int LED_DLL=0;
#define ERR_LEVEL		0
#define DEBUG_LEVEL	1
#define INFO_LEVEL		2
#define LED_PRINTK(level, fmt, args...) if (level <= LED_DLL) printk("[LED]"fmt"\n",  ##args);

static DEFINE_MUTEX(led_lock);
static struct as3665_drv_data *led_drv;




static int as3665_read_i2c(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len)
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
  if(!led_drv->client)
    return -ENODEV;
  ret = i2c_transfer(led_drv->client->adapter, msgs, 2);
  if(ret == 2)
  {
    if(led_drv->i2c_err)
      LED_PRINTK(0,"%s, ret = 2, i2c_err = 0",__func__);
    led_drv->i2c_err = 0;
  }
  else
  {
    led_drv->i2c_err ++;
    if(led_drv->i2c_err < 20)
      LED_PRINTK(0,"%s, ret = %d, i2c_err = %d",__func__,ret,led_drv->i2c_err);
  }
  return ret;
}
static int as3665_write_i2c(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len)
{
  int i, ret;
  unsigned char buf_w[128];
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
  if(!led_drv->client)
    return -ENODEV;
  buf_w[0] = reg;
  for(i=0; i<len; i++)
    buf_w[i+1] = buf[i];
  ret = i2c_transfer(led_drv->client->adapter, msgs, 1);
  if(ret == 1)
  {
    if(led_drv->i2c_err)
      LED_PRINTK(0,"%s, ret = 1, i2c_err = 0",__func__);
    led_drv->i2c_err = 0;
  }
  else
  {
    led_drv->i2c_err ++;
    if(led_drv->i2c_err < 20)
      LED_PRINTK(0,"%s, ret = %d, i2c_err = %d",__func__,ret,led_drv->i2c_err);
  }
  return ret;
}
static int as3665_write_i2c_buf(uint8_t addr, uint8_t* buf, uint8_t len)
{
  int ret;
  struct i2c_msg msgs[] = {
    [0] = {
      .addr   = addr,
      .flags  = 0,
      .buf    = (void *)buf,
      .len    = len
    }
  };
  if(!led_drv->client)
    return -ENODEV;
  ret = i2c_transfer(led_drv->client->adapter, msgs, 1);
  if(ret == 1)
  {
    if(led_drv->i2c_err)
      LED_PRINTK(0,"%s, ret = 1, i2c_err = 0",__func__);
    led_drv->i2c_err = 0;
  }
  else
  {
    led_drv->i2c_err ++;
    if(led_drv->i2c_err < 20)
      LED_PRINTK(0,"%s, ret = %d, i2c_err = %d",__func__,ret,led_drv->i2c_err);
  }
  return ret;
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
static void a2i(char *in, int *out) 
{
  int i, num = 0;
  char a;
  for(i=0; i<10; i++) 
  {
    a = *in++;
    if(a >= '0' && a <= '9')
      num = num * 10 + (a - '0');
    else
      break;
  }
  *out = num;
}
static const char my_ascii[] = "0123456789ABCDEF";
static void h2a(char *in, char *out) 
{
  char c = *in;
  *out++ =  my_ascii[c >> 4];
  *out =    my_ascii[c & 0xF];
}
#define LED_I2C_BUF_LENGTH  512
static void as3665_i2c_test(unsigned char *bufLocal, size_t count)
{
  struct i2c_msg msgs[2];
  int i2c_ret, i, j;
  char id, reg[2], len, dat[LED_I2C_BUF_LENGTH/4];

  printk(KERN_INFO "\n");
  
  
  
  if(bufLocal[1]=='r' && count>=9)
  {
    a2h(&bufLocal[2], &id);     
    a2h(&bufLocal[4], &reg[0]); 
    a2h(&bufLocal[6], &len);    
    if(len >= sizeof(dat))
    {
      LED_PRINTK(0,"R %02X:%02X(%02d) Fail: max length=%d", id,reg[0],len,sizeof(dat));
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

    i2c_ret = i2c_transfer(led_drv->client->adapter, msgs,2);
    if(i2c_ret != 2)
    {
      LED_PRINTK(0,"R %02X:%02X(%02d) Fail: ret=%d", id,reg[0],len,i2c_ret);
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
    LED_PRINTK(1,"R %02X:%02X(%02d) = %s", id,reg[0],len,bufLocal);
  }
  
  
  
  else if(bufLocal[1]=='R' && count>=11)
  {
    a2h(&bufLocal[2], &id);     
    a2h(&bufLocal[4], &reg[0]); 
    a2h(&bufLocal[6], &reg[1]); 
    a2h(&bufLocal[8], &len);    
    if(len >= sizeof(dat))
    {
      LED_PRINTK(1,"R %02X:%02X%02X(%02d) Fail (max length=%d)", id,reg[0],reg[1],len,sizeof(dat));
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

    i2c_ret = i2c_transfer(led_drv->client->adapter, msgs,2);
    if(i2c_ret != 2)
    {
      LED_PRINTK(0,"R %02X:%02X%02X(%02d) Fail (ret=%d)", id,reg[0],reg[1],len,i2c_ret);
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
    LED_PRINTK(1,"R %02X:%02X%02X(%02d) = %s", id,reg[0],reg[1],len,bufLocal);
  }
  
  
  
  else if(bufLocal[1]=='w' && count>=9)
  {
    a2h(&bufLocal[2], &id);     
    len = count - 5;
    if(len & 1)
    {
      LED_PRINTK(0,"W %02X Fail (invalid data) len=%d", id,len);
      return;
    }
    len = len/2;
    if(len >= sizeof(dat))
    {
      LED_PRINTK(0,"W %02X Fail (too many data)", id);
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

    i2c_ret = i2c_transfer(led_drv->client->adapter, msgs,1);
    
    LED_PRINTK(0,"W %02X = %s", id, i2c_ret==1 ? "Pass":"Fail");
  }
  else
  {
    LED_PRINTK(1,"rd: r40000B   (addr=40(7bit), reg=00, read count=11");
    LED_PRINTK(1,"Rd: R2C010902 (addr=2C(7bit), reg=0109, read count=2");
    LED_PRINTK(1,"wr: w40009265CA (addr=40(7bit), reg & data=00,92,65,CA...");
  }
}
static void as3665_i2c_fast_read(unsigned char id, unsigned char *bufLocal, size_t count)
{
  struct i2c_msg msgs[2];
  int i2c_ret, i, j;
  char reg[2], len, dat[LED_I2C_BUF_LENGTH/4];

  printk(KERN_INFO "\n");
  
  
  
  if(count>=6)
  {
    a2h(&bufLocal[1], &reg[0]); 
    a2h(&bufLocal[3], &len);    
    if(len >= sizeof(dat))
    {
      LED_PRINTK(0,"R %02X:%02X(%02d) Fail: max length=%d", id,reg[0],len,sizeof(dat));
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

    i2c_ret = i2c_transfer(led_drv->client->adapter, msgs,2);
    if(i2c_ret != 2)
    {
      LED_PRINTK(0,"R %02X:%02X(%02d) Fail: ret=%d", id,reg[0],len,i2c_ret);
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
    LED_PRINTK(1,"R %02X:%02X(%02d) = %s", id,reg[0],len,bufLocal);
  }
}
static void as3665_i2c_fast_write(unsigned char id, unsigned char *bufLocal, size_t count)
{
  struct i2c_msg msgs[2];
  int i2c_ret, i, j;
  char len, dat[LED_I2C_BUF_LENGTH/4];

  printk(KERN_INFO "\n");
  
  
  
  if(count>=6)
  {
    len = count - 2;
    if(len & 1)
    {
      LED_PRINTK(0,"W %02X Fail (invalid data) len=%d", id,len);
      return;
    }
    len = len/2;
    if(len >= sizeof(dat))
    {
      LED_PRINTK(0,"W %02X Fail (too many data)", id);
      return;
    }

    j = 1;
    for(i=0; i<len; i++)
    {
      a2h(&bufLocal[j], &dat[i]);
      j = j + 2;
    }

    msgs[0].addr = id;
    msgs[0].flags = 0;
    msgs[0].buf = &dat[0];
    msgs[0].len = len;

    i2c_ret = i2c_transfer(led_drv->client->adapter, msgs,1);
    
    LED_PRINTK(0,"W %02X = %s", id, i2c_ret==1 ? "Pass":"Fail");
  }
}



static unsigned int led_pm_on = 0;
static void led_onOff(unsigned int onOff)
{
  mutex_lock(&led_lock);
  if(onOff) 
  {
    if(!led_pm_on)
    {
      led_pm_on = 1;
#ifdef CONFIG_PM_LOG
      PM_LOG_EVENT(PM_LOG_ON, PM_LOG_LED);
#endif
    }
  }
  else  
  {
    if(led_pm_on)
    {
      led_pm_on = 0;
#ifdef CONFIG_PM_LOG
      PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_LED);
#endif
    }
  }
  mutex_unlock(&led_lock);
}




static void as3665_led_color(struct as3665_led *dd, unsigned char color, unsigned char mode)
{
  unsigned char rgb[10][3] = {
    {255,  37,   0},    
    {255,  142,  24},   
    {255,  255,  0},    
    {90,   255,  145},  
    {255,  204,  234},  
    {195,  255,  175},  
    {135,  200,  255},  
    {255,  100,  255},  
    {111,  70,   255},  
    {206,  225,  254},  
  };
  if(color > as3665_color_max || mode > as3665_mode_max)
    return;
  dd->mode  = mode;
  dd->red   = rgb[color][0];
  dd->green = rgb[color][1];
  dd->blue  = rgb[color][2];
}
static void as3665_led_mode_dual(unsigned char id, struct as3665_led_data *data)
{
  struct as3665_led *led;
  unsigned char r00_stop[] = {  
    0x00,
    0x00,0x2A};
  unsigned char r02_03[] = {    
    0x02,
    0x00,0x70};
  unsigned char r10_18[] = {    
    0x10,
    0,0,0,0,0,0,0,0,0};
    
    
  unsigned char r80_88[] =  {   
    0x80,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned char rB0_B2[] = {    
    0xB0,
    as3665_prog_01_addr[prog_01_led1_dual],
    as3665_prog_01_addr[prog_01_led2_dual]};
  unsigned char r00_start[] = { 
    0x00,
    0x45,0x2A};

  LED_PRINTK(1,"%s %02X",__func__,id);

  if(id == AS3665_ID1)  led = &data->led[0];  
  else                  led = &data->led[2];  

  
  if(data == NULL)
  {
    LED_PRINTK(0,"%s, as3665_led_data = NULL",__func__);
  }
  else
  {
    r10_18[3] = led[0].blue;
    r10_18[4] = led[0].green;
    r10_18[5] = led[1].blue;
    r10_18[6] = led[1].green;
    r10_18[7] = led[0].red;
    r10_18[8] = led[1].red;
  }

  as3665_write_i2c_buf(id,&r00_stop[0],sizeof(r00_stop)); 
  as3665_write_i2c_buf(id,&r02_03[0],sizeof(r02_03));     
  as3665_write_i2c_buf(id,&r10_18[0],sizeof(r10_18));     
  as3665_write_i2c_buf(id,&r80_88[0],sizeof(r80_88));     
  as3665_write_i2c_buf(id,&rB0_B2[0],sizeof(rB0_B2));     
  if((id == AS3665_ID1 && led_drv->ic1_prog_in_ram != prog_01) ||
     (id == AS3665_ID2 && led_drv->ic2_prog_in_ram != prog_01))
  {
    as3665_write_i2c_buf(id,&as3665_prog_01[0],sizeof(as3665_prog_01));
    as3665_write_i2c_buf(id,&as3665_prog_01_mux[0],sizeof(as3665_prog_01_mux));
    if(id == AS3665_ID1)  led_drv->ic1_prog_in_ram = prog_01;
    else                  led_drv->ic2_prog_in_ram = prog_01;
  }
  as3665_write_i2c_buf(id,&r00_start[0],sizeof(r00_start));  
  if(id == AS3665_ID1)  led_drv->ic1_enable = 1;
  else                              led_drv->ic2_enable = 1;
  led_onOff(1);
}
static void as3665_led_mode_misc(unsigned char id, struct as3665_led_data *data)
{
  struct as3665_led *led;
  unsigned char r00_stop[] = {  
    0x00,
    0x00,0x2A};
  unsigned char r02_03[] = {    
    0x02,
    0x00,0x70};
  unsigned char r10_18[] = {    
    0x10,
    0,0,0,0,0,0,0,0,0};
    
    
  unsigned char r80_88[] =  {   
    0x80,
    0, 0, 0, 0, 0, 0, 0, 0, 0};
  unsigned char rB0_B2[] = {    
    0xB0,
    0, 0};
  unsigned char r00_start[] = { 
    0x00,
    0x40,0x02};

  LED_PRINTK(1,"%s %02X",__func__,id);

  if(id == AS3665_ID1)  led = &data->led[0];  
  else                  led = &data->led[2];  

  switch(led[0].mode)
  {
    case as3665_mode_fast:
      r00_start[1] |= 0x01; 
      rB0_B2[1] = as3665_prog_01_addr[prog_01_led1_fast];
      break;
    case as3665_mode_slow:
      r00_start[1] |= 0x01; 
      rB0_B2[1] = as3665_prog_01_addr[prog_01_led1_slow];
      break;
    case as3665_mode_charge:
      r00_start[1] |= 0x01; 
      rB0_B2[1] = as3665_prog_01_addr[prog_01_led1_charge];
      break;
    case as3665_mode_constant:
      r02_03[1] |= 0xFC;    
      r80_88[3] = 255;      
      r80_88[4] = 255;      
      r80_88[5] = 255;      
      r80_88[6] = 255;      
      r80_88[7] = 255;      
      r80_88[8] = 255;      
      break;
  }
  

  
  if(data == NULL)
  {
    LED_PRINTK(0,"%s, as3665_led_data = NULL",__func__);
  }
  else
  {
    r10_18[3] = led[0].blue;
    r10_18[4] = led[0].green;
    r10_18[5] = led[1].blue;
    r10_18[6] = led[1].green;
    r10_18[7] = led[0].red;
    r10_18[8] = led[1].red;
  }

  as3665_write_i2c_buf(id,&r00_stop[0],sizeof(r00_stop)); 
  as3665_write_i2c_buf(id,&r02_03[0],sizeof(r02_03));     
  as3665_write_i2c_buf(id,&r10_18[0],sizeof(r10_18));     
  as3665_write_i2c_buf(id,&r80_88[0],sizeof(r80_88));     
  as3665_write_i2c_buf(id,&rB0_B2[0],sizeof(rB0_B2));     
  if((id == AS3665_ID1 && led_drv->ic1_prog_in_ram != prog_01) ||
     (id == AS3665_ID2 && led_drv->ic2_prog_in_ram != prog_01))
  {
    as3665_write_i2c_buf(id,&as3665_prog_01[0],sizeof(as3665_prog_01));
    as3665_write_i2c_buf(id,&as3665_prog_01_mux[0],sizeof(as3665_prog_01_mux));
    if(id == AS3665_ID1)  led_drv->ic1_prog_in_ram = prog_01;
    else                  led_drv->ic2_prog_in_ram = prog_01;
  }
  as3665_write_i2c_buf(id,&r00_start[0],sizeof(r00_start));  
  if(id == AS3665_ID1)  led_drv->ic1_enable = 1;
  else                              led_drv->ic2_enable = 1;
  led_onOff(1);
}
static void as3665_led_mode_off(unsigned char id, struct as3665_led_data *data)
{
  unsigned char r00_stop[] = {  
    0x00,
    0x00,0x2A};
  unsigned char r02_03[] = {    
    0x02,
    0x00,0x70};
  unsigned char r80_88[] = {    
    0x80,
    0, 0, 0, 0, 0, 0, 0, 0, 0};

  LED_PRINTK(1,"%s %02X",__func__,id);

  as3665_write_i2c_buf(id,&r00_stop[0],sizeof(r00_stop)); 
  as3665_write_i2c_buf(id,&r02_03[0],sizeof(r02_03));     
  as3665_write_i2c_buf(id,&r80_88[0],sizeof(r80_88));     
  if(id == AS3665_ID1)  led_drv->ic1_enable = 0;
  else    led_drv->ic2_enable = 0;
  LED_PRINTK(1,"ic1_enable = %d,ic2_enable = %d",led_drv->ic1_enable,led_drv->ic2_enable);
  if(led_drv->ic1_enable == 0 && led_drv->ic2_enable == 0)
    led_onOff(0);
}
static void as3665_led_mode_demo(unsigned char id)
{
  unsigned char r00_stop[] = {  
    0x00,
    0x40,0x2A};
  unsigned char r02_03[] = {    
    0x02,
    0x00,0x70};
  unsigned char r10_18[] = {    
    0x10,
    0,0,255,255,255,255,255,255,0};
  unsigned char r80_88[] =  {   
    0x80,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned char rB0_B2[] = {    
    0xB0,
    0x00,0x00};
    
  LED_PRINTK(1,"%s %02X",__func__,id);

  as3665_write_i2c_buf(id,&r00_stop[0],sizeof(r00_stop)); 
  as3665_write_i2c_buf(id,&r02_03[0],sizeof(r02_03));     
  as3665_write_i2c_buf(id,&r10_18[0],sizeof(r10_18));     
  as3665_write_i2c_buf(id,&r80_88[0],sizeof(r80_88));     
  as3665_write_i2c_buf(id,&rB0_B2[0],sizeof(rB0_B2));     
  as3665_write_i2c_buf(id,&as3665_prog_02[0],sizeof(as3665_prog_02));
  as3665_write_i2c_buf(id,&as3665_prog_02_mux[0],sizeof(as3665_prog_02_mux));
  if(id == AS3665_ID1)  led_drv->ic1_prog_in_ram = prog_02;
  else                  led_drv->ic2_prog_in_ram = prog_02;
  
}

static void as3665_led_mode_party(unsigned char id, struct as3665_led_data *data)
{
  struct as3665_led *led;
  unsigned char r00_stop[] = {  
    0x00,
    0x40,0x82};
  unsigned char r02_03[] = {    
    0x02,
    0x00,0x70};
  unsigned char r10_18[] = {    
    0x10,
    0,0,255,255,255,255,255,255,0};
  unsigned char r80_88[] =  {   
    0x80,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned char rB0_B2[] = {    
    0xB0,
    0x00,0x00};
  unsigned char r00_start[] = { 
    0x00,
    0x41,0x02};

  LED_PRINTK(1,"%s %02X",__func__,id);
  if(data == NULL)
  {
    LED_PRINTK(0,"%s, as3665_led_data = NULL",__func__);
  }

  if(id == AS3665_ID1)  led = &data->led[0];  
  else                  led = &data->led[2];  

  switch(led[0].mode)
  {
    case as3665_mode_party1:
      r00_start[1] = 0x41;
      r00_start[2] = 0x02;
      rB0_B2[1] = as3665_prog_04_addr[effect_1];
      break;
    case as3665_mode_party2:
      r00_start[1] = 0x45;
      r00_start[2] = 0x0A;
      rB0_B2[1] = as3665_prog_04_addr[effect_2_1];
      rB0_B2[2] = as3665_prog_04_addr[effect_2_2];
      break;
    case as3665_mode_party3:
      r00_start[1] = 0x41;
      r00_start[2] = 0x02;
      rB0_B2[1] = as3665_prog_04_addr[effect_3];
      break;
  }

  as3665_write_i2c_buf(id,&r00_stop[0],sizeof(r00_stop)); 
  as3665_write_i2c_buf(id,&r02_03[0],sizeof(r02_03));     
  as3665_write_i2c_buf(id,&r10_18[0],sizeof(r10_18));     
  as3665_write_i2c_buf(id,&r80_88[0],sizeof(r80_88));     
  as3665_write_i2c_buf(id,&rB0_B2[0],sizeof(rB0_B2));     
  if((id == AS3665_ID1 && led_drv->ic1_prog_in_ram != prog_04) ||
     (id == AS3665_ID2 && led_drv->ic2_prog_in_ram != prog_04))
  {
    as3665_write_i2c_buf(id,&as3665_prog_04[0],sizeof(as3665_prog_04));
    as3665_write_i2c_buf(id,&as3665_prog_04_mux[0],sizeof(as3665_prog_04_mux));
    if(id == AS3665_ID1)  led_drv->ic1_prog_in_ram = prog_04;
    else                  led_drv->ic2_prog_in_ram = prog_04;
  }
  as3665_write_i2c_buf(id,&r00_start[0],sizeof(r00_start));  
  if(id == AS3665_ID1)  led_drv->ic1_enable = 1;
  else                              led_drv->ic2_enable = 1;
  led_onOff(1);
}

static void as3665_led_current_ctrl(unsigned char id, struct as3665_led_data *data)
{
  struct as3665_led *led;
  
  unsigned char r02_03[] = {
    0x02,
    0xFc,0x70};
  unsigned char r10_18[] =  {   
    0x10,
    0, 0, 0, 0, 0, 0, 0, 0, 0};
  unsigned char r19_1B[] = {  
    0x19,
    0x00,0x00,0x00};
  unsigned char r80_88[] = {   
    0x80,
    0, 0, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0};
  unsigned char rB0_B2[] = {   
    0xB0,
    as3665_prog_01_addr[prog_01_led_curr]};
  unsigned char r00_start[] = { 
    0x00,
    0x41,0x02};

  LED_PRINTK(1,"%s %02X",__func__,id);

  if(id == AS3665_ID1)  led = &data->led[0];    
  else    led = &data->led[2];    

  LED_PRINTK(1,"%02X-%02X-%02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X-%02X-%02X",
    led[0].maxcurr_r,  led[0].maxcurr_g,  led[0].maxcurr_b, led[0].red, led[0].green, led[0].blue,
    led[1].maxcurr_r,  led[1].maxcurr_g,  led[1].maxcurr_b, led[1].red, led[1].green, led[1].blue);

  if(data == NULL)
   {
     LED_PRINTK(0,"%s, as3665_led_data = NULL",__func__);
   }
   else
   {
     r10_18[3] = led[0].blue;
     r10_18[4] = led[0].green;
     r10_18[5] = led[1].blue;
     r10_18[6] = led[1].green;
     r10_18[7] = led[0].red;
     r10_18[8] = led[1].red;

     r19_1B[1] = (led[0].maxcurr_g << 6) | (led[0].maxcurr_b << 4);
     r19_1B[2] = (led[1].maxcurr_r << 6) | (led[0].maxcurr_r << 4) | (led[1].maxcurr_g<< 2) | (led[1].maxcurr_b);
   }

  
  as3665_write_i2c_buf(id,&r02_03[0],sizeof(r02_03));      
  as3665_write_i2c_buf(id,&r10_18[0],sizeof(r10_18));      
  as3665_write_i2c_buf(id,&r19_1B[0],sizeof(r19_1B));     
  as3665_write_i2c_buf(id,&r80_88[0],sizeof(r80_88));      
  as3665_write_i2c_buf(id,&rB0_B2[0],sizeof(rB0_B2));    
  if((id == AS3665_ID1 && led_drv->ic1_prog_in_ram != prog_01) ||
     (id == AS3665_ID2 && led_drv->ic2_prog_in_ram != prog_01))
  {
    as3665_write_i2c_buf(id,&as3665_prog_01[0],sizeof(as3665_prog_01));
    as3665_write_i2c_buf(id,&as3665_prog_01_mux[0],sizeof(as3665_prog_01_mux));
    if(id == AS3665_ID1)  led_drv->ic1_prog_in_ram = prog_01;
    else                              led_drv->ic2_prog_in_ram = prog_01;
  }
  as3665_write_i2c_buf(id,&r00_start[0],sizeof(r00_start));  
  if(id == AS3665_ID1)  led_drv->ic1_enable = 1;
  else                              led_drv->ic2_enable = 1;
  led_onOff(1);
}

static int as3665_led_init(unsigned char id)
{
  int ret = 0, i, ret_init = 0;
  unsigned char r00 = 0;
  unsigned char r3D_3E[] = {0, 0};  
  unsigned char r00_stop[] =  { 
    0x00,
    0x00};
  unsigned char r00_09[] =  {   
    0x00,
    
    
    0x00,0x2A,0x00,0x70,0x00,0x10,0x34,0x00,0x81,0x00}; 
    
    
  unsigned char r10_18[] =  {   
    0x10,
    0, 0, 0, 0, 0, 0, 0, 0, 0};
  unsigned char r19_1F[] =  {   
    0x19,
    
    
    0x55,0x55,0x01,0x00,0x00,0x00,0x00};    
  unsigned char r80_88[] =  {   
    0x80,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned char r8F[] = {       
    0x8F,
    0x00};
  unsigned char r9B_9D[] =  {   
    0x9B,
    0xFF,0xFF,0xFF};
    
  unsigned char rA0_A8[] =  {   
    0xA0,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20};
    
  unsigned char rB0_B2[] =  {   
    0xB0,
    0, 0, 0};

  
    if(system_rev > CHICAGO_EVT1)
    {
      r00_09[5] |= 0x40;
    }
  

  msleep(10);  

  
  ret = as3665_write_i2c_buf(id,&r00_stop[0],sizeof(r00_stop));
  if(ret != 1)
    LED_PRINTK(0,"%s, 0x%02X stop fail",__func__,id);

  
  
  

  
  for(i=0; i<10; i++)
  {
    ret = as3665_read_i2c(id,0x00,&r00,sizeof(r00));
    if(!TST_BIT(r00,7))
      break;
    msleep(1);
  }
  if(TST_BIT(r00,7))
    LED_PRINTK(0,"%s, 0x%02X ram_init timeout = %02X",__func__,id,r00);

  
  as3665_write_i2c_buf(id,&r00_09[0],sizeof(r00_09));
  as3665_write_i2c_buf(id,&r10_18[0],sizeof(r10_18));
  as3665_write_i2c_buf(id,&r19_1F[0],sizeof(r19_1F));
  as3665_write_i2c_buf(id,&r80_88[0],sizeof(r80_88));
  as3665_write_i2c_buf(id,&r8F[0],sizeof(r8F));
  as3665_write_i2c_buf(id,&r9B_9D[0],sizeof(r9B_9D));
  as3665_write_i2c_buf(id,&rA0_A8[0],sizeof(rA0_A8));
  
  as3665_write_i2c_buf(id,&rB0_B2[0],sizeof(rB0_B2));
   ret_init = as3665_write_i2c_buf(id,&as3665_prog_01[0],sizeof(as3665_prog_01));
  as3665_write_i2c_buf(id,&as3665_prog_01_mux[0],sizeof(as3665_prog_01_mux));
  if(ret_init != 1)
  {
    if(id == AS3665_ID1)	led_drv->ic1_prog_in_ram = prog_none ;
    else    led_drv->ic2_prog_in_ram = prog_none ;
    LED_PRINTK(0,"%s, 0x%02X ram_init failed!",__func__,id);
  }
  else
  {
    if(id == AS3665_ID1)  led_drv->ic1_prog_in_ram = prog_01;
    else    led_drv->ic2_prog_in_ram = prog_01;
  }
  
  ret = as3665_read_i2c(id,0x3D,&r3D_3E[0],sizeof(r3D_3E));
  LED_PRINTK(1,"%s, %02X ChipId = %02X%02X",__func__,id,r3D_3E[0],r3D_3E[1]);
  if(ret != 2)
    return -1;
  else
    return 0;
}

static void as3665_ic_enable(void)
{
  if(led_drv->inited != 1)
  {
    gpio_set_value(AS3665_GPIO_EN, 1);
    as3665_led_init(AS3665_ID1);
    as3665_led_init(AS3665_ID2);
    led_drv->inited = 1;
  }
  LED_PRINTK(1,"%s Enable LED IC",__func__);
}
static void as3665_ic_disable(void)
{
  gpio_set_value(AS3665_GPIO_EN, 0);
  led_drv->inited = 0;
  led_drv->ic1_enable = 0;
  led_drv->ic2_enable = 0;
  LED_PRINTK(1,"%s Disable LED IC",__func__);
  led_onOff(0);
}

static int as3665_led_ctrl(struct as3665_led_data *data)
{
  int ret = 0;
  char led_01_updated = 0;
  char led_23_updated = 0;
  struct as3665_led *led = data->led;
  struct as3665_led *led_backup = led_drv->data_backup.led; 
  LED_PRINTK(1,"%02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X",
    led[0].mode, led[0].red, led[0].green, led[0].blue,
    led[1].mode, led[1].red, led[1].green, led[1].blue,
    led[2].mode, led[2].red, led[2].green, led[2].blue,
    led[3].mode, led[3].red, led[3].green, led[3].blue);

  
  if(led[0].mode >= as3665_mode_max ||
    led[1].mode >= as3665_mode_max ||
    led[2].mode >= as3665_mode_max ||
    led[3].mode >= as3665_mode_max)
  {
    ret = -1;
    goto exit;
  }

  
  as3665_ic_disable();
  msleep(1);
  as3665_ic_enable();

  
  

  


  
  if(led[0].mode == as3665_mode_define3 ||
    led[1].mode == as3665_mode_define3)
  {
    led[0].mode = as3665_mode_define3;  
    led[1].mode = as3665_mode_define3;
    as3665_led_mode_dual(AS3665_ID1, data);
    led_01_updated = 1;
  }
  if(led[2].mode == as3665_mode_define3 ||
    led[3].mode == as3665_mode_define3)
  {
    led[2].mode = as3665_mode_define3;  
    led[3].mode = as3665_mode_define3;
    as3665_led_mode_dual(AS3665_ID2, data);
    led_23_updated = 1;
  }
  
  if(led[0].mode == as3665_mode_party1 &&
    led[1].mode == as3665_mode_party1)  
  {
    as3665_led_mode_party(AS3665_ID1, data);
    led_01_updated = 1;
  }
  if(led[2].mode == as3665_mode_party1 &&
    led[3].mode == as3665_mode_party1)  
  {
    as3665_led_mode_party(AS3665_ID2, data);
    led_23_updated = 1;
  }
  
  if(led[0].mode == as3665_mode_party2 &&
    led[1].mode == as3665_mode_party2)  
  {
    as3665_led_mode_party(AS3665_ID1, data);
    led_01_updated = 1;
  }
  if(led[2].mode == as3665_mode_party2 &&
    led[3].mode == as3665_mode_party2)  
  {
    as3665_led_mode_party(AS3665_ID2, data);
    led_23_updated = 1;
  }
  
  if(led[0].mode == as3665_mode_party3 &&
    led[1].mode == as3665_mode_party3)  
  {
    as3665_led_mode_party(AS3665_ID1, data);
    led_01_updated = 1;
  }
  if(led[2].mode == as3665_mode_party3 &&
    led[3].mode == as3665_mode_party3)  
  {
    as3665_led_mode_party(AS3665_ID2, data);
    led_23_updated = 1;
  }
  
  if(led[0].mode == as3665_mode_off &&
    led[1].mode == as3665_mode_off &&
    led[2].mode == as3665_mode_off &&
    led[3].mode == as3665_mode_off)
  {
    as3665_ic_disable();
    led_01_updated = 1;
    led_23_updated = 1;
  }
  else
  {
    if(led[0].mode == as3665_mode_off &&
      led[1].mode == as3665_mode_off)
    {
      as3665_led_mode_off(AS3665_ID1,data);
      led_01_updated = 1;
    }
    if(led[2].mode == as3665_mode_off &&
      led[3].mode == as3665_mode_off)
    {
      as3665_led_mode_off(AS3665_ID2,data);
      led_23_updated = 1;
    }
  }
  
  if(led[0].mode == as3665_mode_nochange &&
    led[1].mode == as3665_mode_nochange)
  {
    led_01_updated = 1;
    led[0] = led_backup[0];   
    led[1] = led_backup[1];
  }
  else
  {
    if(led[0].mode == as3665_mode_nochange)
    {
      led[0] = led_backup[0]; 
    }
    if(led[1].mode == as3665_mode_nochange)
    {
      led[1] = led_backup[1]; 
    }
  }
  if(led[2].mode == as3665_mode_nochange &&
    led[3].mode == as3665_mode_nochange)
  {
    led_23_updated = 1;
    led[2] = led_backup[2];   
    led[3] = led_backup[3];
  }
  else
  {
    if(led[2].mode == as3665_mode_nochange)
    {
      led[2] = led_backup[2]; 
    }
    if(led[3].mode == as3665_mode_nochange)
    {
      led[3] = led_backup[3]; 
    }
  }

  

  if(!led_01_updated)
  {
    as3665_led_mode_misc(AS3665_ID1,data);
  }

  if(!led_23_updated)
  {
    as3665_led_mode_misc(AS3665_ID2,data);
  }

exit:
  if(ret == 0)  
  {
    led_backup[0] = led[0];
    led_backup[1] = led[1];
    led_backup[2] = led[2];
    led_backup[3] = led[3];
  }

  return ret;
}

static void RMP(unsigned char *reg,  
  unsigned char prescale,   
  unsigned char step_time,  
  unsigned char sign,       
  unsigned char inc)        
{
  unsigned short rr, err = 0;
  if(prescale > 1)      {prescale = 1;    err ++;}
  if(step_time > 31)    {step_time = 31;  err ++;}
  if(sign > 1)          {sign = 1;        err ++;}
  if(inc > 255)         {inc = 255;       err ++;}
  if(err)
  {
    LED_PRINTK(0,"%s, err = %d", __func__,err);
  }
  rr = 0x0000 |
    ((prescale & 1)   << 14) |
    ((step_time & 31) << 9) |
    ((sign & 1)       << 8) |
    ((inc & 255)      << 0);
  reg[0] = (rr >> 8) & 0xFF;
  reg[1] = rr & 0xFF;
}
static void SPW(unsigned char *reg,  
  unsigned char pwm_value)  
{
  unsigned short rr, err = 0;
  if(pwm_value > 255) {pwm_value = 255; err ++;}
  if(err)
  {
    LED_PRINTK(0,"%s, err = %d", __func__,err);
  }
  rr = 0x4000 | (pwm_value & 255);
  reg[0] = (rr >> 8) & 0xFF;
  reg[1] = rr & 0xFF;
}
static void BRN(unsigned char *reg,  
  unsigned char loop_count, 
  unsigned char step_num)   
{
  unsigned short rr, err = 0;
  if(loop_count > 63) {loop_count = 63; err ++;}
  if(step_num > 127)  {step_num = 126;  err ++;}
  if(err)
  {
    LED_PRINTK(0,"%s, err = %d", __func__,err);
  }
  rr = 0xA000 |
    ((loop_count & 63)  << 7) |
    ((step_num & 127)   << 0);
  reg[0] = (rr >> 8) & 0xFF;
  reg[1] = rr & 0xFF;
}
static int as3665_led_ctrl2(struct as3665_led_data2 *dd)
{
  unsigned char r00_stop[] = {  
    0x00,
    0x00,0x2A};
  unsigned char r02_03[] = {    
    0x02,
    0x00,0x70};
  unsigned char r10_18[] = {    
    0x10,
    0,0,0,0,0,0,0,0,0};
  unsigned char r80_88[] =  {   
    0x80,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned char rB0_B2[] = {    
    0xB0,
    0, 40};
  unsigned char r00_start[] = { 
    0x00,
    0x45,0x2A};
  unsigned char *rr = &as3665_prog_03[0];
  unsigned char id, ram_start;  
  int ret = 0, t, i;
  for(i=0; i<4; i++)
  LED_PRINTK(1,"%03d-%03d-%03d, %05d-%05d-%05d-%05d-%05d-%05d",
    dd->led[i].red, dd->led[i].green, dd->led[i].blue,
    dd->led[i].rise, dd->led[i].high, dd->led[i].fall, dd->led[i].low,
    dd->led[i].cycle, dd->led[i].pause);

  if(led_drv->inited != 1)
    as3665_ic_enable();

  as3665_write_i2c_buf(AS3665_ID1,&r00_stop[0],sizeof(r00_stop)); 
  as3665_write_i2c_buf(AS3665_ID2,&r00_stop[0],sizeof(r00_stop)); 
  as3665_write_i2c_buf(AS3665_ID1,&r02_03[0],sizeof(r02_03));     
  as3665_write_i2c_buf(AS3665_ID2,&r02_03[0],sizeof(r02_03));     
  as3665_write_i2c_buf(AS3665_ID1,&r80_88[0],sizeof(r80_88));     
  as3665_write_i2c_buf(AS3665_ID2,&r80_88[0],sizeof(r80_88));     
  as3665_write_i2c_buf(AS3665_ID1,&rB0_B2[0],sizeof(rB0_B2));     
  as3665_write_i2c_buf(AS3665_ID2,&rB0_B2[0],sizeof(rB0_B2));     

  
  r10_18[3] = dd->led[0].blue;
  r10_18[4] = dd->led[0].green;
  r10_18[5] = dd->led[1].blue;
  r10_18[6] = dd->led[1].green;
  r10_18[7] = dd->led[0].red;
  r10_18[8] = dd->led[1].red;
  as3665_write_i2c_buf(AS3665_ID1,&r10_18[0],sizeof(r10_18));     
  r10_18[3] = dd->led[2].blue;
  r10_18[4] = dd->led[2].green;
  r10_18[5] = dd->led[3].blue;
  r10_18[6] = dd->led[3].green;
  r10_18[7] = dd->led[2].red;
  r10_18[8] = dd->led[3].red;
  as3665_write_i2c_buf(AS3665_ID2,&r10_18[0],sizeof(r10_18));     

  
  
  
  
  
  
  
  for(i=0; i<4; i++)
  {
    if(i==0)      {id = AS3665_ID1;  ram_start = 0;  }
    else if(i==1) {id = AS3665_ID1;  ram_start = 40; }
    else if(i==2) {id = AS3665_ID2;  ram_start = 0;  }
    else          {id = AS3665_ID2;  ram_start = 40; }

    
    if(ram_start == 0)
    {
      rr[1] = 0;
      rr[3] = 0x5E;
      rr[5] = 0xDE;
    }
    else
    {
      rr[1] = 40;
      rr[3] = 0x5F;
      rr[5] = 0xDF;
    }

    
    if(dd->led[i].rise == 0)
    {
      SPW(&rr[8],255);  
    }
    else if(dd->led[i].rise < 4000)
    {
      t = dd->led[i].rise / 125;  
      if(t <= 0)  t = 1;
      if(t > 31)  t = 31;
      RMP(&rr[8],0,t,0,255);
    }
    else
    {
      t = dd->led[i].rise / 4000; 
      if(t <= 0)  t = 1;
      if(t > 31)  t = 31;
      RMP(&rr[8],1,t,0,255);
    }

    
    if(dd->led[i].high == 0)
    {
      SPW(&rr[10],255);  
      SPW(&rr[12],255);  
    }
    else if(dd->led[i].high <= 250)
    {
      RMP(&rr[10],1,16,0,0);  
      SPW(&rr[12],255);  
    }
    else
    {
      RMP(&rr[10],1,16,0,0);  
      t = (dd->led[i].high - 250) / 250;  
      if(t <= 0)  t = 1;
      if(t > 63)  t = 63;
      BRN(&rr[12],t,4);
    }

    
    if(dd->led[i].fall == 0)
    {
      SPW(&rr[14],0);   
    }
    else if(dd->led[i].fall < 4000)
    {
      t = dd->led[i].fall / 125;  
      if(t <= 0)  t = 1;
      if(t > 31)  t = 31;
      RMP(&rr[14],0,t,1,255);
    }
    else
    {
      t = dd->led[i].fall / 4000; 
      if(t <= 0)  t = 1;
      if(t > 31)  t = 31;
      RMP(&rr[14],1,t,1,255);
    }

    
    if(dd->led[i].low == 0)
    {
      SPW(&rr[16],0);   
      SPW(&rr[18],0);   
    }
    else if(dd->led[i].low <= 250)
    {
      RMP(&rr[16],1,16,0,0);  
      SPW(&rr[18],0);   
    }
    else
    {
      RMP(&rr[16],1,16,0,0);  
      t = (dd->led[i].low - 250) / 250;  
      if(t <= 0)  t = 1;
      if(t > 63)  t = 63;
      BRN(&rr[18],t,7);
    }

    
    if(dd->led[i].cycle == 0 || dd->led[i].cycle == 1)
    {
      SPW(&rr[20],0);   
    }
    else
    {
      t = dd->led[i].cycle; 
      t = t - 1;
      if(t > 63)  t = 63;
      BRN(&rr[20],t,3);
    }

    
    if(dd->led[i].pause == 0)
    {
      SPW(&rr[22],0);   
      SPW(&rr[24],0);   
      SPW(&rr[26],0);   
    }
    else if(dd->led[i].pause <= 250)
    {
      RMP(&rr[22],1,16,0,0);  
      SPW(&rr[24],0);   
      SPW(&rr[26],0);   
    }
    else if(dd->led[i].pause <= 16000)
    {
      RMP(&rr[22],1,16,0,0);  
      t = (dd->led[i].pause - 250) / 250;  
      if(t <= 0)  t = 1;
      if(t > 63)  t = 63;
      BRN(&rr[24],t,10);
      SPW(&rr[26],0);   
    }
    else
    {
      RMP(&rr[22],1,16,0,0);  
      BRN(&rr[24],19,10);     
      t = (dd->led[i].pause - 5000) / 5000; 
      if(t <= 2)  t = 3;      
      if(t > 63)  t = 63;
      BRN(&rr[26],t,10);
    }
    as3665_write_i2c_buf(id,&as3665_prog_03[0],sizeof(as3665_prog_03));
  }

  as3665_write_i2c_buf(AS3665_ID1,&as3665_prog_03_mux[0],sizeof(as3665_prog_03_mux));
  as3665_write_i2c_buf(AS3665_ID2,&as3665_prog_03_mux[0],sizeof(as3665_prog_03_mux));
  led_drv->ic1_prog_in_ram = prog_03;
  led_drv->ic2_prog_in_ram = prog_03;

  as3665_write_i2c_buf(AS3665_ID1,&r00_start[0],sizeof(r00_start));  
  as3665_write_i2c_buf(AS3665_ID2,&r00_start[0],sizeof(r00_start));  
  led_drv->ic1_enable = 1;
  led_drv->ic2_enable = 1;
  led_onOff(1);
  return ret;
}




static void as3665_work_func(struct work_struct *work)
{
  int i, j, t, count = 1;
  unsigned char rB0_B2[] = {    
    0xB0,
    0x00,0x00};
  unsigned char r00_stop[] = {  
    0x00,
    0x40,0x2A};
  unsigned char r00_start[] = { 
    0x00,
    0x41,0x2A};
  unsigned char r80_88[] =  {   
    0x80,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned char fast_addr[] = {
    0,5,10};
  unsigned char slow_addr[] = {
    15,20,25};

  LED_PRINTK(1,"%s+", __func__);

  if(!led_drv->inited)
    goto exit;
  if(!led_drv->work_enable)
    goto exit;

  wake_lock(&led_drv->wlock);
  as3665_led_mode_demo(AS3665_ID1);
  as3665_led_mode_demo(AS3665_ID2);
  led_onOff(1);
  while(1)
  {

    for(i=0; i<3; i++)  
    {
      for(j=0; j<5; j++)  
      {
        
        
        as3665_write_i2c_buf(AS3665_ID1,&r00_stop[0],sizeof(r00_stop));
        as3665_write_i2c_buf(AS3665_ID2,&r00_stop[0],sizeof(r00_stop));
        
        as3665_write_i2c_buf(AS3665_ID1,&r80_88[0],sizeof(r80_88));
        as3665_write_i2c_buf(AS3665_ID2,&r80_88[0],sizeof(r80_88));
        
        rB0_B2[1] = fast_addr[i];
        as3665_write_i2c_buf(AS3665_ID1,&rB0_B2[0],sizeof(rB0_B2));
        as3665_write_i2c_buf(AS3665_ID2,&rB0_B2[0],sizeof(rB0_B2));
        
        as3665_write_i2c_buf(AS3665_ID1,&r00_start[0],sizeof(r00_start));
        as3665_write_i2c_buf(AS3665_ID2,&r00_start[0],sizeof(r00_start));
        
        for(t=0; t<2; t++)
        {
          msleep(980);
          if(!led_drv->work_enable ||
            led_drv->ic1_prog_in_ram != prog_02 ||
            led_drv->ic2_prog_in_ram != prog_02)
            goto exit_stop_ic;
        }
      }
      
      for(t=0; t<2; t++)
      {
        msleep(980);
        if(!led_drv->work_enable ||
          led_drv->ic1_prog_in_ram != prog_02 ||
          led_drv->ic2_prog_in_ram != prog_02)
          goto exit_stop_ic;
      }
    }


    for(i=0; i<3; i++)  
    {
      for(j=0; j<5; j++)  
      {
        
        
        as3665_write_i2c_buf(AS3665_ID1,&r00_stop[0],sizeof(r00_stop));
        as3665_write_i2c_buf(AS3665_ID2,&r00_stop[0],sizeof(r00_stop));
        
        as3665_write_i2c_buf(AS3665_ID1,&r80_88[0],sizeof(r80_88));
        as3665_write_i2c_buf(AS3665_ID2,&r80_88[0],sizeof(r80_88));
        
        rB0_B2[1] = slow_addr[i];
        as3665_write_i2c_buf(AS3665_ID1,&rB0_B2[0],sizeof(rB0_B2));
        as3665_write_i2c_buf(AS3665_ID2,&rB0_B2[0],sizeof(rB0_B2));
        
        as3665_write_i2c_buf(AS3665_ID1,&r00_start[0],sizeof(r00_start));
        as3665_write_i2c_buf(AS3665_ID2,&r00_start[0],sizeof(r00_start));
        
        for(t=0; t<5; t++)
        {
          msleep(980);
          if(!led_drv->work_enable ||
            led_drv->ic1_prog_in_ram != prog_02 ||
            led_drv->ic2_prog_in_ram != prog_02)
            goto exit_stop_ic;
        }
      }
      
      for(t=0; t<5; t++)
      {
        msleep(980);
        if(!led_drv->work_enable ||
          led_drv->ic1_prog_in_ram != prog_02 ||
          led_drv->ic2_prog_in_ram != prog_02)
          goto exit_stop_ic;
      }
    }

    LED_PRINTK(1,"%s, count=%d",__func__,count++);
  }

exit_stop_ic:
  as3665_write_i2c_buf(AS3665_ID1,&r00_stop[0],sizeof(r00_stop));
  as3665_write_i2c_buf(AS3665_ID2,&r00_stop[0],sizeof(r00_stop));
  led_onOff(0);
exit:
  wake_lock_timeout(&led_drv->wlock, HZ*2);
  LED_PRINTK(1,"%s-", __func__);
}




static ssize_t as3665_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  char bufLocal[LED_I2C_BUF_LENGTH];
  unsigned char id;
  

  
    

  
  if(!strcmp(attr->attr.name,"ctrl"))
    id = AS3665_ID1;
  else
    id = AS3665_ID2;

  printk(KERN_INFO "(%02X)\n", id);
  if(count >= sizeof(bufLocal))
  {
    LED_PRINTK(0,"%s input invalid, count = %d", __func__, count);
    return count;
  }
  memcpy(bufLocal,buf,count);

  switch(bufLocal[0])
  {
    
    
    case 'a': 
      {
        struct as3665_led_data aa;
        switch(bufLocal[1])
        {
          case '1':
            as3665_led_init(id);
            break;
          case '2':
            as3665_led_color(&aa.led[0],as3665_color_red,   as3665_mode_fast);
            as3665_led_color(&aa.led[1],as3665_color_orange,as3665_mode_fast);
            as3665_led_color(&aa.led[2],as3665_color_yellow,as3665_mode_slow);
            as3665_led_color(&aa.led[3],as3665_color_green, as3665_mode_slow);
            as3665_led_ctrl(&aa);
            break;
          case '3':
            as3665_led_color(&aa.led[0],as3665_color_cherry,    as3665_mode_define3);
            as3665_led_color(&aa.led[1],as3665_color_turquoise, as3665_mode_define3);
            as3665_led_color(&aa.led[2],as3665_color_blue,      as3665_mode_charge);
            as3665_led_color(&aa.led[3],as3665_color_purple1,   as3665_mode_charge);
            as3665_led_ctrl(&aa);
            break;
          case '4':
            break;
          case '5':
            break;
        }
      }
      break;

    case 't': 
      {
        static struct as3665_led_data2 dd = {
          .led[0] = {255,255,255,  125,0,125,0,0,0},
          .led[1] = {255,255,255,  125,0,125,0,0,0},
          .led[2] = {255,255,255,  125,0,125,0,0,0},
          .led[3] = {255,255,255,  125,0,125,0,0,0},
        };
        int temp = 0, num = 0;
        num = bufLocal[1]&0x03;
        a2i(&bufLocal[3], &temp);
        switch(bufLocal[2])
        {
          case 'r':
            LED_PRINTK(1,"rise = %d",temp);
            dd.led[num].rise = temp;
            break;
          case 'h':
            LED_PRINTK(1,"high = %d",temp);
            dd.led[num].high = temp;
            break;
          case 'f':
            LED_PRINTK(1,"fall = %d",temp);
            dd.led[num].fall = temp;
            break;
          case 'l':
            LED_PRINTK(1,"low = %d",temp);
            dd.led[num].low = temp;
            break;
          case 'c':
            LED_PRINTK(1,"cycle = %d",temp);
            dd.led[num].cycle = temp;
            break;
          case 'p':
            LED_PRINTK(1,"pause = %d",temp);
            dd.led[num].pause = temp;
            break;
          case 'x':
            LED_PRINTK(1,"red = %d",temp);
            dd.led[num].red = temp;
            break;
          case 'y':
            LED_PRINTK(1,"green = %d",temp);
            dd.led[num].green = temp;
            break;
          case 'z':
            LED_PRINTK(1,"blue = %d",temp);
            dd.led[num].blue = temp;
            break;
        }
        led_drv->data2 = dd;    
        as3665_led_ctrl2(&dd);
      }
      break;

    
    
    case 'y':
      if(bufLocal[1]=='1')      
      {
        int i;
        unsigned char r5F[1], r60_7F[32];
        for(i=0; i<=5; i++)
        {
          r5F[0] = i;
          as3665_write_i2c(id,0x5F,&r5F[0],sizeof(r5F));
          as3665_read_i2c(id,0x60,&r60_7F[0],sizeof(r60_7F));
          LED_PRINTK(1,"%02d~%02d=%02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X",
            i*16,i*16+7,
            r60_7F[0],r60_7F[1],r60_7F[2],r60_7F[3],r60_7F[4],r60_7F[5],r60_7F[6],r60_7F[7],
            r60_7F[8],r60_7F[9],r60_7F[10],r60_7F[11],r60_7F[12],r60_7F[13],r60_7F[14],r60_7F[15]);
          LED_PRINTK(1,"%02d~%02d=%02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X",
            i*16+8,i*16+15,
            r60_7F[16],r60_7F[17],r60_7F[18],r60_7F[19],r60_7F[20],r60_7F[21],r60_7F[22],r60_7F[23],
            r60_7F[24],r60_7F[25],r60_7F[26],r60_7F[27],r60_7F[28],r60_7F[29],r60_7F[30],r60_7F[31]);
        }
      }
      else if(bufLocal[1]=='2') 
      {
        unsigned char rB0_B6[7];
        as3665_read_i2c(id,0xB0,&rB0_B6[0],sizeof(rB0_B6));
        LED_PRINTK(1,"B0-B2= %03d %03d %03d (Start_Addr)",
          rB0_B6[0],rB0_B6[1],rB0_B6[2]);
        LED_PRINTK(1,"B4-B6= %03d %03d %03d (Seq_PC)",
          rB0_B6[4],rB0_B6[5],rB0_B6[6]);
      }
      else if(bufLocal[1]=='3') 
      {
        unsigned char rB8_BE[7];
        unsigned char rD0_DF[16];
        as3665_read_i2c(id,0xB8,&rB8_BE[0],sizeof(rB8_BE));
        as3665_read_i2c(id,0xD0,&rD0_DF[0],sizeof(rD0_DF));
        LED_PRINTK(1,"B8-BB= %02X %02X %02X (var_a1~3) %02X (var_c)",
          rB8_BE[0],rB8_BE[1],rB8_BE[2],rB8_BE[3]);
        LED_PRINTK(1,"BC-BE= %02X %02X %02X (var_b1~3)",
          rB8_BE[4],rB8_BE[5],rB8_BE[6]);
        LED_PRINTK(1,"%02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X (sram)",
          rD0_DF[0],rD0_DF[1],rD0_DF[2],rD0_DF[3],
          rD0_DF[4],rD0_DF[5],rD0_DF[6],rD0_DF[7],
          rD0_DF[8],rD0_DF[9],rD0_DF[10],rD0_DF[11],
          rD0_DF[12],rD0_DF[13],rD0_DF[14],rD0_DF[15]);
      }
      else if(bufLocal[1]=='4') 
      {
        struct as3665_led *led = led_drv->data.led;
        LED_PRINTK(1,"%02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X (now)",
          led[0].mode, led[0].red, led[0].green, led[0].blue,
          led[1].mode, led[1].red, led[1].green, led[1].blue,
          led[2].mode, led[2].red, led[2].green, led[2].blue,
          led[3].mode, led[3].red, led[3].green, led[3].blue);
        led = led_drv->data_backup.led;
        LED_PRINTK(1,"%02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X, %02X-%02X-%02X-%02X (backup)",
          led[0].mode, led[0].red, led[0].green, led[0].blue,
          led[1].mode, led[1].red, led[1].green, led[1].blue,
          led[2].mode, led[2].red, led[2].green, led[2].blue,
          led[3].mode, led[3].red, led[3].green, led[3].blue);
      }
      else if(bufLocal[1]=='5') 
      {
        struct as3665_led2 *led = led_drv->data2.led;
        int i;
        for(i=0; i<4; i++)
        {
          LED_PRINTK(1,"%03d-%03d-%03d, %05d-%05d-%05d-%05d-%05d-%05d",
            led[i].red, led[i].green, led[i].blue,
            led[i].rise, led[i].high, led[i].fall, led[i].low,
            led[i].cycle, led[i].pause);
        }
      }
      else
      {
        unsigned char r00_0F[16], r10_18[9], r19_1F[7];
        unsigned char r80_88[9], r8F[1], r9B_9D[3], rA0_A8[9];

        as3665_read_i2c(id,0x00,&r00_0F[0],sizeof(r00_0F));
        as3665_read_i2c(id,0x10,&r10_18[0],sizeof(r10_18));
        as3665_read_i2c(id,0x19,&r19_1F[0],sizeof(r19_1F));
        as3665_read_i2c(id,0x80,&r80_88[0],sizeof(r80_88));
        as3665_read_i2c(id,0x8F,&r8F[0],sizeof(r8F));
        as3665_read_i2c(id,0x9B,&r9B_9D[0],sizeof(r9B_9D));
        as3665_read_i2c(id,0xA0,&rA0_A8[0],sizeof(rA0_A8));

        LED_PRINTK(1,"00-01= %02X %02X      (Exec)",
          r00_0F[0],r00_0F[1]);
        LED_PRINTK(1,"02-03= %02X %02X      (LED_Control)",
          r00_0F[2],r00_0F[3]);
        LED_PRINTK(1,"04= %02X      (External Clock)",
          r00_0F[4]);


        LED_PRINTK(1,"10-18= %02X %02X %02X %02X  %02X %02X %02X %02X  %02X (current)",
          r10_18[0],r10_18[1],r10_18[2],r10_18[3],r10_18[4],r10_18[5],r10_18[6],r10_18[7],r10_18[8]);
        LED_PRINTK(1,"19-1B= %02X %02X %02X (LED_MaxCurr)",
          r19_1F[0],r19_1F[1],r19_1F[2]);


        
        
        LED_PRINTK(1,"80-9F= %02X %02X %02X %02X  %02X %02X %02X %02X  %02X (PWM)",
          r80_88[0],r80_88[1],r80_88[2],r80_88[3],r80_88[4],r80_88[5],r80_88[6],r80_88[7],r80_88[8]);
        LED_PRINTK(1,"9B-9D= %02X %02X %02X (Fader)",
          r9B_9D[0],r9B_9D[1],r9B_9D[2]);
        LED_PRINTK(1,"A0-A8= %02X %02X %02X %02X  %02X %02X %02X %02X  %02X (Driver_Setup)",
          rA0_A8[0],rA0_A8[1],rA0_A8[2],rA0_A8[3],rA0_A8[4],rA0_A8[5],rA0_A8[6],rA0_A8[7],rA0_A8[8]);
      }
      break;

    
    
    case 'p':   
      if(bufLocal[1]=='e')
      {
        if(bufLocal[2]=='1')
        {
          LED_PRINTK(1,"Pin EN set 1");
          gpio_set_value(AS3665_GPIO_EN, 1); 
        }
        else if(bufLocal[2]=='0')
        {
          LED_PRINTK(1,"Pin EN set 0");
          gpio_set_value(AS3665_GPIO_EN, 0); 
        }
      }
      else if(bufLocal[1]=='t')
      {
        int ret;
        if(bufLocal[2]=='1')
        {
          LED_PRINTK(1,"Pin TRIG set 1");
          gpio_set_value(AS3665_GPIO_TRIG, 1);
        }
        else if(bufLocal[2]=='0')
        {
          LED_PRINTK(1,"Pin TRIG set 0");
          gpio_set_value(AS3665_GPIO_TRIG, 0);
        }
        else if(bufLocal[2]=='o')
        {
          LED_PRINTK(1,"Pin TRIG set o/p 1");
          ret = gpio_tlmm_config(
            GPIO_CFG(AS3665_GPIO_TRIG, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
            GPIO_CFG_ENABLE);
          LED_PRINTK(1,"%s, gpio_tlmm_config %d, ret=%d", __func__, AS3665_GPIO_TRIG, ret);
          ret = gpio_direction_output(AS3665_GPIO_TRIG,1);
           LED_PRINTK(1,"%s, gpio_direction_output %d, ret=%d", __func__, AS3665_GPIO_TRIG, ret);
          
        }
        else if(bufLocal[2]=='i')
        {
          ret = gpio_tlmm_config(
            GPIO_CFG(AS3665_GPIO_TRIG, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
            GPIO_CFG_ENABLE);
          LED_PRINTK(1,"%s, gpio_tlmm_config %d, ret=%d", __func__, AS3665_GPIO_TRIG, ret);
          ret = gpio_direction_input(AS3665_GPIO_TRIG);
          LED_PRINTK(1,"%s, gpio_direction_input %d, ret=%d", __func__,AS3665_GPIO_TRIG, ret);
        }
        else
        {
          LED_PRINTK(1,"Pin TRIG get %d", gpio_get_value(AS3665_GPIO_TRIG));
        }
      }
      break;

    case 'q':   
      LED_PRINTK(1,"Pin TRIG pulse+");
      gpio_set_value(AS3665_GPIO_TRIG, 1);
      udelay(200);
      gpio_set_value(AS3665_GPIO_TRIG, 0);
      udelay(200);
      gpio_set_value(AS3665_GPIO_TRIG, 1);
      LED_PRINTK(1,"Pin TRIG pulse-");
      break;

    
    
    case 'i':
      as3665_i2c_test(bufLocal, count);
      break;
    case 'r':
      as3665_i2c_fast_read(id,bufLocal, count);
      break;
    case 'w':
      as3665_i2c_fast_write(id,bufLocal, count);
      break;

    default:
      break;
  }

  return count;
}




static ssize_t as3665_ftd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  unsigned char r3D_3E_ic1[2];
  unsigned char r3D_3E_ic2[2];
  int en_val;
  en_val = gpio_get_value(AS3665_GPIO_EN);
  if(!en_val)
  {
    LED_PRINTK(0,"EN 0->1");
    gpio_set_value(AS3665_GPIO_EN, 1); 
    msleep(1);
  }
  if(as3665_read_i2c(AS3665_ID1,0x3D,&r3D_3E_ic1[0],sizeof(r3D_3E_ic1)) != 2) 
  {
    r3D_3E_ic1[0] = 0;
    r3D_3E_ic1[1] = 0;
  }
  if(as3665_read_i2c(AS3665_ID2,0x3D,&r3D_3E_ic2[0],sizeof(r3D_3E_ic2)) != 2)
  {
    r3D_3E_ic2[0] = 0;
    r3D_3E_ic2[1] = 0;
  }
  if(!en_val)
  {
    gpio_set_value(AS3665_GPIO_EN, en_val); 
    LED_PRINTK(0,"EN 1->0");
  }
  
  return sprintf(buf, "%02X%02X%02X%02X\n", r3D_3E_ic1[0],r3D_3E_ic1[1],r3D_3E_ic2[0],r3D_3E_ic2[1]);
}


























static ssize_t as3665_ftd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  int i;
  char bufLocal[128];
  static struct as3665_led_data data;

  
    

  printk(KERN_INFO "\n");
  if(count >= sizeof(bufLocal))
  {
    LED_PRINTK(0,"%s input invalid, count = %d", __func__, count);
    return count;
  }
  memcpy(bufLocal,buf,count);

  switch(bufLocal[0])
  {
    
    case 'm':
      if(bufLocal[1]=='1')
      {
        LED_PRINTK(1,"Test Mode = ON");
        memset(&data, 0, sizeof(data));
        
        as3665_ic_disable();
      }
      else
      {
        LED_PRINTK(1,"Test Mode = OFF");
        memset(&data, 0, sizeof(data));
        
        as3665_ic_disable();
      }
      break;

    case 'd':
      if(bufLocal[1] == '0')
      {
        LED_PRINTK(1,"Stop demo");
        led_drv->work_enable = 0;
        flush_workqueue(led_drv->wqueue);
        memset(&data, 0, sizeof(data));
        
        as3665_ic_disable();
      }
      else if(bufLocal[1] == '1')
      {
        LED_PRINTK(1,"Start demo");
        if(led_drv->inited != 1)
          as3665_ic_enable();
        led_drv->work_enable = 1;
        queue_work(led_drv->wqueue, &led_drv->work);
      }
      else if(bufLocal[1] == '2')
      {
        LED_PRINTK(1,"red, orange, yellow, green");
        as3665_led_color(&data.led[0],as3665_color_red,   as3665_mode_constant);
        as3665_led_color(&data.led[1],as3665_color_orange,as3665_mode_constant);
        as3665_led_color(&data.led[2],as3665_color_yellow,as3665_mode_constant);
        as3665_led_color(&data.led[3],as3665_color_green, as3665_mode_constant);
        as3665_led_ctrl(&data);
      }
      else if(bufLocal[1] == '3')
      {
        LED_PRINTK(1,"cherry, turquoise, blue, purple1");
        as3665_led_color(&data.led[0],as3665_color_cherry,    as3665_mode_constant);
        as3665_led_color(&data.led[1],as3665_color_turquoise, as3665_mode_constant);
        as3665_led_color(&data.led[2],as3665_color_blue,      as3665_mode_constant);
        as3665_led_color(&data.led[3],as3665_color_purple1,   as3665_mode_constant);
        as3665_led_ctrl(&data);
      }
      else if(bufLocal[1] == '4')
      {
        LED_PRINTK(1,"purple2, white, red, orange");
        as3665_led_color(&data.led[0],as3665_color_purple2, as3665_mode_constant);
        as3665_led_color(&data.led[1],as3665_color_white,   as3665_mode_constant);
        as3665_led_color(&data.led[2],as3665_color_red,     as3665_mode_constant);
        as3665_led_color(&data.led[3],as3665_color_orange,  as3665_mode_constant);
        as3665_led_ctrl(&data);
      }
      break;

    case 'r':
      if(bufLocal[1]=='1')
      {
        LED_PRINTK(1,"Red ON");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_constant;
          data.led[i].red = 255;
        }
        as3665_led_ctrl(&data);
      }
      else
      {
        LED_PRINTK(1,"Red OFF");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_constant;
          data.led[i].red = 0;
        }
        as3665_led_ctrl(&data);
      }
      break;
    case 'g':
      if(bufLocal[1]=='1')
      {
        LED_PRINTK(1,"Green ON");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_constant;
          data.led[i].green = 255;
        }
        as3665_led_ctrl(&data);
      }
      else
      {
        LED_PRINTK(1,"Green OFF");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_constant;
          data.led[i].green = 0;
        }
        as3665_led_ctrl(&data);
      }
      break;
    case 'b':
      if(bufLocal[1]=='1')
      {
        LED_PRINTK(1,"Blue ON");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_constant;
          data.led[i].blue = 255;
        }
        as3665_led_ctrl(&data);
      }
      else
      {
        LED_PRINTK(1,"Blue OFF");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_constant;
          data.led[i].blue = 0;
        }
        as3665_led_ctrl(&data);
      }
      break;
    case 'w':
      if(bufLocal[1]=='1')
      {
        LED_PRINTK(1,"White ON");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_constant;
          data.led[i].red = 255;
          data.led[i].green = 255;
          data.led[i].blue = 255;
        }
        as3665_led_ctrl(&data);
      }
      else
      {
        LED_PRINTK(1,"White OFF");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_constant;
          data.led[i].red = 0;
          data.led[i].green = 0;
          data.led[i].blue = 0;
        }
        as3665_led_ctrl(&data);
      }
      break;
    case 'W':
      if(bufLocal[1]=='1')
      {
        LED_PRINTK(1,"White Blink");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_fast;
          data.led[i].red = 255;
          data.led[i].green = 255;
          data.led[i].blue = 255;
        }
        as3665_led_ctrl(&data);
      }
      else
      {
        LED_PRINTK(1,"White Blink OFF");
        for(i=0; i<4; i++)
        {
          data.led[i].mode = as3665_mode_off;
          data.led[i].red = 0;
          data.led[i].green = 0;
          data.led[i].blue = 0;
        }
        as3665_led_ctrl(&data);
      }
      break;

    case 's': 
      if(count<12)
      {
        LED_PRINTK(0,"Invalid parameter");
        LED_PRINTK(0,"example: s0102334455 (Led=00~03,Mode=00~0B,Red=00~FF,Green=00~FF,Blue=00~FF)");
        break;
      }
      {
        unsigned char led, mode, red, green, blue;
        a2h(&bufLocal[1], &led);
        a2h(&bufLocal[3], &mode);
        a2h(&bufLocal[5], &red);
        a2h(&bufLocal[7], &green);
        a2h(&bufLocal[9], &blue);
        if(led > 4)
        {
          LED_PRINTK(0,"Invalid led = 0x%02X (0~3)", led);
          break;
        }
        data.led[led].mode = mode;
        data.led[led].red = red;
        data.led[led].green = green;
        data.led[led].blue = blue;
        as3665_led_ctrl(&data);
      }
      break;

    case 'f': 
      if(count<32)
      {
        LED_PRINTK(0,"Invalid parameter");
        LED_PRINTK(0,"example: f00445566017788990244556603778899 (Mode=00~0B,Red=00~FF,Green=00~FF,Blue=00~FF)");
        break;
      }
      {
        unsigned char mode[4], red[4], green[4], blue[4];
        a2h(&bufLocal[1], &mode[0]);
        a2h(&bufLocal[3], &red[0]);
        a2h(&bufLocal[5], &green[0]);
        a2h(&bufLocal[7], &blue[0]);
        a2h(&bufLocal[9], &mode[1]);
        a2h(&bufLocal[11], &red[1]);
        a2h(&bufLocal[13], &green[1]);
        a2h(&bufLocal[15], &blue[1]);
        a2h(&bufLocal[17], &mode[2]);
        a2h(&bufLocal[19], &red[2]);
        a2h(&bufLocal[21], &green[2]);
        a2h(&bufLocal[23], &blue[2]);
        a2h(&bufLocal[25], &mode[3]);
        a2h(&bufLocal[27], &red[3]);
        a2h(&bufLocal[29], &green[3]);
        a2h(&bufLocal[31], &blue[3]);
        for(i=0; i<4; i++)
        {
          data.led[i].mode = mode[i];
          data.led[i].red = red[i];
          data.led[i].green = green[i];
          data.led[i].blue = blue[i];
        }
        as3665_led_ctrl(&data);
      }
      break;

    case 'c': 
      if(led_drv->inited != 1)
        as3665_ic_enable();
      if(count<48)
      {
        LED_PRINTK(0,"Invalid parameter");
        LED_PRINTK(0,"example: c001122334455001122334455001122334455001122334455 (maxcurr_r=00~03, maxcurr_g=00~03, maxcurr_b=00~03, Red=00~FF,Green=00~FF,Blue=00~FF)");
        break;
      }
      {
        unsigned char maxcurr_r[4], maxcurr_g[4], maxcurr_b[4], red[4], green[4], blue[4];
        a2h(&bufLocal[1], &maxcurr_r[0]);
        a2h(&bufLocal[3], &maxcurr_g[0]);
        a2h(&bufLocal[5], &maxcurr_b[0]);
        a2h(&bufLocal[7], &red[0]);
        a2h(&bufLocal[9], &green[0]);
        a2h(&bufLocal[11], &blue[0]);
        a2h(&bufLocal[13], &maxcurr_r[1]);
        a2h(&bufLocal[15], &maxcurr_g[1]);
        a2h(&bufLocal[17], &maxcurr_b[1]);
        a2h(&bufLocal[19], &red[1]);
        a2h(&bufLocal[21], &green[1]);
        a2h(&bufLocal[23], &blue[1]);
        a2h(&bufLocal[25], &maxcurr_r[2]);
        a2h(&bufLocal[27], &maxcurr_g[2]);
        a2h(&bufLocal[29], &maxcurr_b[2]);
        a2h(&bufLocal[31], &red[2]);
        a2h(&bufLocal[33], &green[2]);
        a2h(&bufLocal[35], &blue[2]);
        a2h(&bufLocal[37], &maxcurr_r[3]);
        a2h(&bufLocal[39], &maxcurr_g[3]);
        a2h(&bufLocal[41], &maxcurr_b[3]);
        a2h(&bufLocal[43], &red[3]);
        a2h(&bufLocal[45], &green[3]);
        a2h(&bufLocal[47], &blue[3]);
        for(i=0; i<4; i++)
        {
          data.led[i].maxcurr_r = maxcurr_r[i]&0x03;
          data.led[i].maxcurr_g = maxcurr_g[i]&0x03;
          data.led[i].maxcurr_b = maxcurr_b[i]&0x03;
          data.led[i].red = red[i];
          data.led[i].green = green[i];
          data.led[i].blue = blue[i];
        }
        as3665_led_current_ctrl(AS3665_ID1,&data);
        as3665_led_current_ctrl(AS3665_ID2,&data);
      }
      break;

    case 'E':
      as3665_ic_enable( );
      break;
    case 'D':
      as3665_ic_disable( );
      break;

    default:
      break;
  }

  return count;
}

static struct device_attribute as3665_ctrl_attrs[] = {
  __ATTR(ctrl, 0220, NULL,  as3665_ctrl_store),
  __ATTR(ctrl2, 0220, NULL, as3665_ctrl_store),
  __ATTR(ftd, 0664, as3665_ftd_show, as3665_ftd_store),   
};




static void as3665_early_suspend_func(struct early_suspend *h)
{
  LED_PRINTK(2,"%s+", __func__);
  led_drv->in_early_suspend = 1;
  
  
  LED_PRINTK(2,"%s-", __func__);
}
static void as3665_late_resume_func(struct early_suspend *h)
{
  LED_PRINTK(2,"%s+", __func__);
  led_drv->in_early_suspend = 0;
  LED_PRINTK(2,"%s-", __func__);
}
static struct early_suspend as3665_early_suspend = {
  .level    = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
  .suspend  = as3665_early_suspend_func,
  .resume   = as3665_late_resume_func,
};




static void led_create_kernel_debuglevel(void)
{
  if (kernel_debuglevel_dir!=NULL)
  {
    debugfs_create_u32("led_dll", S_IRUGO | S_IWUGO,
      kernel_debuglevel_dir, (u32 *)(&LED_DLL));
  }
  else
  {
    printk(KERN_ERR "kernel_debuglevel_dir led_dll Fail\n");
  }
}




static int as3665_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
  LED_PRINTK(2,"%s+", __func__);
  
  led_drv->in_suspend = 1;
  LED_PRINTK(2,"%s-", __func__);
  return 0;
}
static int as3665_i2c_resume(struct i2c_client *client)
{
  LED_PRINTK(2,"%s+", __func__);
  led_drv->in_suspend = 0;
  LED_PRINTK(2,"%s-", __func__);
  return 0;
}
static int as3665_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int ret;

  LED_PRINTK(0,"%s+", __func__);
  led_drv->client = client;

  ret = as3665_led_init(AS3665_ID1);
  if(ret)
  {
    led_drv->ic1_fail = 1;
  }
  ret = as3665_led_init(AS3665_ID2);
  if(ret)
  {
    led_drv->ic2_fail = 1;
  }

  LED_PRINTK(0,"%s-, ret=%d", __func__, ret);
  
  return 0;
}
static void as3665_i2c_shutdown(struct i2c_client *client)
{
  LED_PRINTK(2,"%s", __func__);
  gpio_set_value(AS3665_GPIO_EN, 0); 
}


static const struct i2c_device_id as3665_i2c_id[] = {
  { "as3665_i2c", 0 },
  { }
};
static struct i2c_driver as3665_i2c_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name = "as3665_i2c"
  },
  .id_table = as3665_i2c_id,
  .probe    = as3665_i2c_probe,
  .suspend  = as3665_i2c_suspend,
  .resume   = as3665_i2c_resume,
  .shutdown = as3665_i2c_shutdown,
};

static int as3665_misc_open(struct inode *inode_p, struct file *fp)
{
  led_drv->opened ++;
  LED_PRINTK(2,"%s    <== [%d] (%04d)",__func__,led_drv->opened,current->pid);
  return 0;
}
static int as3665_misc_release(struct inode *inode_p, struct file *fp)
{
  led_drv->opened --;
  LED_PRINTK(2,"%s <== [%d] (%04d)\n",__func__,led_drv->opened,current->pid);
  return 0;
}
static long as3665_misc_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
  long ret = 0;

  

  
  
  if(_IOC_TYPE(cmd) != AS3665_IOC_MAGIC)
  {
    LED_PRINTK(0,"%s: Not AS3665_IOC_MAGIC", __func__);
    return -ENOTTY;
  }
  if(_IOC_DIR(cmd) & _IOC_READ)
  {
    ret = !access_ok(VERIFY_WRITE, (void __user*)arg, _IOC_SIZE(cmd));
    if(ret)
    {
      LED_PRINTK(0,"%s: access_ok check write err", __func__);
      return -EFAULT;
    }
  }
  if(_IOC_DIR(cmd) & _IOC_WRITE)
  {
    ret = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
    if(ret)
    {
      LED_PRINTK(0,"%s: access_ok check read err", __func__);
      return -EFAULT;
    }
  }

  
  
  switch (cmd)
  {
    case AS3665_IOC_LED_WR:
      LED_PRINTK(1,"%s: (%04d) AS3665_IOC_LED_WR", __func__,current->pid);
      if(copy_from_user(&led_drv->data, (void __user*) arg, _IOC_SIZE(cmd)))
      {
        LED_PRINTK(0,"%s: AS3665_IOC_LED_WR Fail",__func__);
        ret = -EFAULT;
      }
      as3665_led_ctrl(&led_drv->data);
      break;
    case AS3665_IOC_LED_WR2:
      LED_PRINTK(1,"%s: (%04d) AS3665_IOC_LED_WR2", __func__,current->pid);
      if(copy_from_user(&led_drv->data2, (void __user*) arg, _IOC_SIZE(cmd)))
      {
        LED_PRINTK(0,"%s: AS3665_IOC_LED_WR2 Fail",__func__);
        ret = -EFAULT;
      }
      as3665_led_ctrl2(&led_drv->data2);
      break;
    default:
      LED_PRINTK(1,"%s: unknown ioctl = 0x%X", __func__,cmd);
      break;
  }

  
  return ret;
}

static struct file_operations as3665_misc_fops = {
  .owner    = THIS_MODULE,
  .open     = as3665_misc_open,
  .release  = as3665_misc_release,
  .unlocked_ioctl = as3665_misc_ioctl,
};
static struct miscdevice as3665_misc_device = {
  .minor  = MISC_DYNAMIC_MINOR,
  .name   = "as3665",
  .fops   = &as3665_misc_fops,
};





#if defined(CONFIG_DEBUG_FS)
static int as3665_debugfs_open(struct inode *inode, struct file *file)
{
  
  file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
  return 0;
}
static int as3665_debugfs_release(struct inode *inode, struct file *file)
{
  return 0;
}
static ssize_t as3665_debugfs_read(
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
  size = as3665_ftd_show(NULL, NULL, led_drv->debug_read_buf);
  if(size > count)
  {
    ret = -EFAULT;
    goto exit;
  }
  if(copy_to_user(buff, led_drv->debug_read_buf, size))
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
static ssize_t as3665_debugfs_write(
    struct file *file,
    const char __user *buff,
    size_t count,
    loff_t *ppos)
{
  int ret = 0;
  
  if(count > sizeof(led_drv->debug_write_buf))
  {
    ret = -EFAULT;
    goto exit;
  }
  if(copy_from_user(led_drv->debug_write_buf, buff, count))
  {
    LED_PRINTK(0,"%s, failed to copy from user",__func__);
    ret = -EFAULT;
    goto exit;
  }
  led_drv->debug_write_buf[count] = '\0';
  as3665_ftd_store(NULL, NULL, led_drv->debug_write_buf, count);
exit:
  if(!ret)
    ret = count;
  
  return ret;
}
static const struct file_operations as3665_debugfs_fops = {
  .open     = as3665_debugfs_open,
  .release  = as3665_debugfs_release,
  .read     = as3665_debugfs_read,
  .write    = as3665_debugfs_write,
};
static void as3665_debug_init(void)
{
  led_drv->dent = debugfs_create_dir("as3665", 0);
  if (IS_ERR(led_drv->dent))
    return;
  debugfs_create_file("ftd", 0666, led_drv->dent, NULL, &as3665_debugfs_fops);
}
#endif




static int __init as3665_init(void)
{
  int ret;  
  int status, i;

  printk("BootLog, +%s\n", __func__);

  
    if(system_rev <= CHICAGO_EVB2 ||system_rev > CHICAGO_EVT2)
    {
      LED_PRINTK(0,"%s, system_rev = %d (skip driver loading)",__func__,system_rev);
      ret = -ENODEV;
      goto exit_error;
    }
    else
    {
      LED_PRINTK(1,"%s, system_rev = %d",__func__,system_rev);
    }
  

  led_drv = kmalloc(sizeof(struct as3665_drv_data), GFP_KERNEL);
  if(led_drv == NULL)
  {
    LED_PRINTK(0,"%s, kmalloc fail!",__func__);
    ret = -ENOMEM;
    goto exit_error;
  }
  else
  {
    memset(led_drv,0,sizeof(struct as3665_drv_data));
  }

  
  
  
  
  

  
  status = gpio_tlmm_config(
    GPIO_CFG(AS3665_GPIO_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
    GPIO_CFG_ENABLE);  
  if(status)  LED_PRINTK(1,"%s, gpio_tlmm_config %d, ret=%d", __func__, AS3665_GPIO_EN, status);
  status = gpio_request(AS3665_GPIO_EN, "AS3665_GPIO_EN");
  if(status)  LED_PRINTK(0,"%s, Failed to request GPIO %d", __func__, AS3665_GPIO_EN);
  status = gpio_direction_output(AS3665_GPIO_EN,1); 
  gpio_set_value(AS3665_GPIO_EN, 1); 
  if(status)  LED_PRINTK(1,"%s, gpio_direction_output %d, ret=%d", __func__, AS3665_GPIO_EN, status);

  
  status = gpio_tlmm_config(
    GPIO_CFG(AS3665_GPIO_TRIG, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
    GPIO_CFG_ENABLE);  
  if(status)  LED_PRINTK(1,"%s, gpio_tlmm_config %d, ret=%d", __func__, AS3665_GPIO_TRIG, status);
  status = gpio_request(AS3665_GPIO_TRIG, "AS3665_GPIO_TRIG");
  if(status)  LED_PRINTK(0,"%s, Failed to request GPIO %d", __func__, AS3665_GPIO_TRIG);
  status = gpio_direction_input(AS3665_GPIO_TRIG);
  
  
  

  
  
  
  ret = i2c_add_driver(&as3665_i2c_driver);
  if(ret < 0)
  {
    LED_PRINTK(0,"%s: i2c_add_driver Fail, ret=%d", __func__, ret);
    
  }

  
  
  
  ret = misc_register(&as3665_misc_device);
  if(ret)
  {
    LED_PRINTK(0,"%s: led misc_register Fail, ret=%d", __func__, ret);
    
  }

  
  
  
  
  #if defined(CONFIG_DEBUG_FS)
    as3665_debug_init();
  #endif

  #if 1
  
  for(i=0; i<ARRAY_SIZE(as3665_ctrl_attrs); i++)
  {
    ret = device_create_file(as3665_misc_device.this_device, &as3665_ctrl_attrs[i]);
    if(ret) LED_PRINTK(0,"%s: create FAIL, ret=%d",as3665_ctrl_attrs[i].attr.name,ret);
  }
  #endif

  
  wake_lock_init(&led_drv->wlock, WAKE_LOCK_SUSPEND, "as3665_active");

  register_early_suspend(&as3665_early_suspend);

  
  INIT_WORK(&led_drv->work, as3665_work_func);
  led_drv->wqueue = create_singlethread_workqueue("as3665_workqueue");
  if(led_drv->wqueue) 
  {
    LED_PRINTK(1,"%s, as3665_workqueue created PASS!",__func__);
  }
  else  
  {
    LED_PRINTK(0,"%s, as3665_workqueue created FAIL!",__func__);
  }

  
  led_create_kernel_debuglevel();

  led_drv->inited = 1;

  printk("BootLog, -%s, ret=%d\n", __func__,ret);
  return ret;

exit_error:



  printk("BootLog, -%s, ret=%d\n", __func__,ret);
  return ret;
}


module_init(as3665_init);
