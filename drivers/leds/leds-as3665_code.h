#ifndef _LEDS_AS3665_CODE_H_
#define _LEDS_AS3665_CODE_H_

enum as3665_prog_in_ram  {
  prog_none = 0,    
  prog_01,          
  prog_02,          
  prog_03,          
  prog_04,          
  prog_max
};

enum prog_01_mode {
  prog_01_led1_fast = 0,
  prog_01_led1_slow,
  prog_01_led1_charge,
  prog_01_led2_fast,
  prog_01_led2_slow,
  prog_01_led2_charge,
  prog_01_led1_dual,
  prog_01_led2_dual,
  prog_01_led_curr,
  prog_01_max
};

enum as3665_prog_in_ram_4{
  effect_1,
  effect_2_1,
  effect_2_2,
  effect_3,
};
static unsigned char as3665_prog_04_addr[] = {
0,
13,
19,
28,
};


static unsigned char as3665_prog_01_addr[] = {
  0, 10, 20,    
  27, 37, 47,   
  54, 64,       
  70,            
};
static unsigned char as3665_prog_01[] = {
0xFE,0x00,  

0x9C,0x5D,  
0x9C,0xDD,
0x06,0xFF,
0x07,0xFF,
0x60,0x00,
0xA1,0x84,
0xA2,0x02,
0x60,0x00,
0xA3,0x87,
0x00,0x00,
0x9C,0x5D,  
0x9C,0xDD,
0x1E,0xFF,
0x1F,0xFF,
0x60,0x00,
0xA1,0x84,
0xA2,0x02,
0x60,0x00,
0xB3,0x87,
0x00,0x00,
0x9C,0x5D,  
0x9C,0xDD,
0x1E,0xFF,
0x1F,0xFF,
0x60,0x00,
0xA1,0x84,
0x00,0x00,
0x9C,0x5F,  
0x9C,0xDF,
0x06,0xFF,
0x07,0xFF,
0x60,0x00,
0xA1,0x84,
0xA2,0x02,
0x60,0x00,
0xA3,0x87,
0x00,0x00,
0x9C,0x5F,  
0x9C,0xDF,
0x1E,0xFF,
0x1F,0xFF,
0x60,0x00,
0xA1,0x84,
0xA2,0x02,
0x60,0x00,
0xB3,0x87,
0x00,0x00,
0x9C,0x5F,  
0x9C,0xDF,
0x1E,0xFF,
0x1F,0xFF,
0x60,0x00,
0xA1,0x84,
0x00,0x00,
0x9C,0x5E,  
0x9C,0xDE,
0x1E,0xFF,
0xE0,0x04,
0x1F,0xFF,
0x42,0x00,
0xA2,0x02,
0x60,0x00,
0xB3,0x87,
0x00,0x00,
0x9C,0x5F,  
0x9C,0xDF,
0xE0,0x80,
0x1E,0xFF,
0x1F,0xFF,
0x00,0x00,
0x9C,0x5E,	
0x9C,0xDE,
0x40,0xFF,
0xC0,0x00,
};
static unsigned char as3665_prog_01_mux[] = {
0xFE, 93,  


0x00,0xFC,
0x00,0x4C,
0x00,0xB0
};

static unsigned char as3665_prog_02[] = {
0xFE,0x00,  

0x9C,0x50,  
0x9C,0xD0,
0x08,0xFF,
0x09,0xFF,
0xC0,0x00,
0x9C,0x51,  
0x9C,0xD1,
0x08,0xFF,
0x09,0xFF,
0xC0,0x00,
0x9C,0x52,  
0x9C,0xD2,
0x08,0xFF,
0x09,0xFF,
0xC0,0x00,
0x9C,0x50,  
0x9C,0xD0,
0x20,0xFF,
0x21,0xFF,
0xC0,0x00,
0x9C,0x51,  
0x9C,0xD1,
0x20,0xFF,
0x21,0xFF,
0xC0,0x00,
0x9C,0x52,  
0x9C,0xD2,
0x20,0xFF,
0x21,0xFF,
0xC0,0x00,
0x00,0x00,
};
static unsigned char as3665_prog_02_mux[] = {
0xFE, 80,  

0x00,0xC0,
0x00,0x28,
0x00,0x14,
};

unsigned char as3665_prog_03[] = {
0xFE, 0,    
            
0x9C,0x5E,  
0x9C,0xDE,  
0x40,0x00,  
0x02,0xFF,  
0x60,0x00,  
0xA0,0x84,  
0x03,0xFF,  
0x60,0x00,  
0xA0,0x87,  
0xA0,0x83,  
0x60,0x00,  
0xA0,0x8A,  
0xA0,0x8A,  
0x00,0x00,  
};
static unsigned char as3665_prog_03_mux[] = {
0xFE, 94,  


0x00,0x4C,
0x00,0xB0
};

unsigned char as3665_prog_04[] = {  
0xFE,0x00,
0x9C,0x46,
0x9C,0xCC,
0x04,0xFF,
0x40,0x00,
0xA1,0x02,
0x9D,0x80,
0x04,0xFF,
0x40,0x00,
0x9D,0x80,
0xA2,0x06,
0x04,0xFF,
0x15,0xFF,
0x00,0x00,
0x9C,0x4D,
0x9C,0xD2,
0x10,0x55,
0x9D,0x80,
0xA0,0x02,
0x00,0x00,
0x9C,0x4D,
0x9C,0xD2,
0x9D,0x80,
0x9D,0x80,
0x9D,0x80,
0x11,0x55,
0x9D,0x80,
0xA0,0x05,
0x00,0x00,
0x9C,0x53,
0x9C,0xD8,
0x10,0xFF,
0x9D,0x80,
0x11,0xFF,
0x9D,0x80,
0x10,0xFF,
0x9D,0x80,
0x11,0xFF,
0x9D,0x80,
0x10,0xFF,
0x9D,0x80,
0x11,0xFF,
0x00,0x00,
};
static unsigned char as3665_prog_04_mux[] = {
0xFE, 70, 
0x00,0xFC,
0x00,0xC0,
0x00,0x28,
0x00,0x14,
0x00,0xE8,
0x00,0xD4,
0x00,0x3C,
0x00,0x1C,
0x00,0x38,
0x00,0x70,
0x00,0xE0,
0x00,0xC4,
0x00,0x8C,
0x00,0xD4,
0x00,0x14,
0x00,0x28,
0x00,0xC0,
0x00,0x14,
0x00,0x28,
};

#endif  

