#ifndef _LEDS_AS3665_H_
#define _LEDS_AS3665_H_
#include <asm/ioctl.h>
#include <linux/kernel.h>

#define MYBIT(b)        (1<<b)
#define TST_BIT(x,b)    ((x & (1<<b)) ? 1 : 0)
#define CLR_BIT(x,b)    (x &= (~(1<<b)))
#define SET_BIT(x,b)    (x |= (1<<b))

#define AS3665_ID1      (0x8C >> 1)
#define AS3665_ID2      (0x8E >> 1)

#define AS3665_GPIO_EN    9     
#define AS3665_GPIO_TRIG  10    

enum as3665_color {
  as3665_color_red,
  as3665_color_orange,
  as3665_color_yellow,
  as3665_color_green,
  as3665_color_cherry,
  as3665_color_turquoise,
  as3665_color_blue,
  as3665_color_purple1,
  as3665_color_purple2,
  as3665_color_white,
  as3665_color_max
};

enum as3665_mode  {
  as3665_mode_off = 0,    
  as3665_mode_nochange,   
  as3665_mode_fast,
  as3665_mode_slow,
  as3665_mode_charge,
  as3665_mode_constant,
  as3665_mode_define1,
  as3665_mode_define2,
  as3665_mode_define3,
  as3665_mode_party1,
  as3665_mode_party2,
  as3665_mode_party3,
  as3665_mode_max
};
struct as3665_led {
  
  unsigned char mode;
  
  unsigned char maxcurr_r;
  unsigned char maxcurr_g;
  unsigned char maxcurr_b;
  
  unsigned char red;
  unsigned char green;
  unsigned char blue;
}; 
struct as3665_led_data {
  struct as3665_led led[4]; 
};


struct as3665_led2 {
    
    unsigned char red;
    unsigned char green;
    unsigned char blue;
    
    unsigned int rise;    
    unsigned int high;    
    unsigned int fall;    
    unsigned int low;     
    unsigned int cycle;   
    unsigned int pause;   
};
struct as3665_led_data2 {
  struct as3665_led2 led[4]; 
};

struct as3665_drv_data {
  int inited;
  int ic1_enable;   
  int ic2_enable;   
  int opened;   
  int in_early_suspend;
  int in_suspend;
  int i2c_err;
  char  ic1_fail;
  char  ic2_fail;
  char  ic1_prog_in_ram;  
  char  ic2_prog_in_ram;  
  char  work_enable;      

  struct i2c_client *client;
  struct wake_lock wlock;
  struct work_struct work;  
  struct workqueue_struct *wqueue;

  
  struct as3665_led_data  data;         
  struct as3665_led_data  data_backup;  

  
  struct as3665_led_data2 data2;

  
  struct dentry		*dent;
  char debug_write_buf[128];
  char debug_read_buf[128];
};

#define AS3665_IOC_MAGIC       'l' 
#define AS3665_IOC_LED_WR     _IOW(AS3665_IOC_MAGIC, 4, struct as3665_led_data)   
#define AS3665_IOC_LED_WR2    _IOW(AS3665_IOC_MAGIC, 5, struct as3665_led_data2)  

#endif  

