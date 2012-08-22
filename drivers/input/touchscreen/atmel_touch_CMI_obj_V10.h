

#ifndef _ATMEL_TOUCH_CMI_OBJ_V10_H_
#define _ATMEL_TOUCH_CMI_OBJ_V10_H_

static int T6_msg_handler ( uint8_t *value );
static int T9_msg_handler ( uint8_t *value );
static int T15_msg_handler ( uint8_t *value );
static int T25_msg_handler ( uint8_t *value );
                                   

uint8_t maxTouchCfg_T38_CMI_V10[] =
{
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
};


uint8_t maxTouchCfg_T7_CMI_V10[] =
{
	0x32,
	0xC,
	0x32,
};


uint8_t maxTouchCfg_T8_CMI_V10[] =
{
	0x19,
	0x0,
	0x5,
	0x5,
	0x0,
	0x0,
	0x1,
	0x0,
	0x4,
	0x81,
};


uint8_t maxTouchCfg_T9_CMI_V10[] =
{
	0x83,
	0x0,
	0x0,
	0x11,
	0xC,
	0x0,
	0x10,
	0x37,
	0x3,
	0x5,
	0x32,
	0x2,
	0x2,
	0x0,
	0x4,
	0xF,
	0xF,
	0xA,
	0x0,
	0x0,
	0x0,
	0x0,
	0x5,
	0x5,
	0x20,
	0x20,
	0x8F,
	0x3C,
	0x8F,
	0x50,
	0x14,
	0xF,
	0x0,
	0x0,
	0x0,
};


uint8_t maxTouchCfg_T15_CMI_V10[] =
{
	0x83,
	0x11,
	0x4,
	0x1,
	0x4,
	0x0,
	0x0,
	0x3C,
	0x5,
	0x0,
	0x0,
};


uint8_t maxTouchCfg_T18_CMI_V10[] =
{
	0x0,
	0x0,
};


uint8_t maxTouchCfg_T19_CMI_V10[] =
{
	0x0,
	0x0,
	0x0,
	0x3C,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
};


uint8_t maxTouchCfg_T23_CMI_V10[] =
{
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
};


uint8_t maxTouchCfg_T25_CMI_V10[] =            
{                                           
	0x3,
	0x0,
	0xD8,
	0x67,
	0xA8,
	0x56,
	0xD8,
	0x67,
	0xA8,
	0x56,
	0x0,
	0x0,
	0x0,
	0x0,
};                                          


uint8_t maxTouchCfg_T40_CMI_V10[] =
{
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
};


uint8_t maxTouchCfg_T42_CMI_V10[] =
{
	0x0,
	0x0,
	0x28,
	0x23,
	0x80,
	0x2,
	0x0,
	0xA,
};


uint8_t maxTouchCfg_T46_CMI_V10[] =
{
	0x0,
	0x2,
	0x8,
	0x10,
	0x0,
	0x0,
	0x1,
	0x0,
	0x0,
};


uint8_t maxTouchCfg_T47_CMI_V10[] =
{
	0x0,
	0x14,
	0x32,
	0x5,
	0x2,
	0x32,
	0x28,
	0x0,
	0x0,
	0x3F,
};


uint8_t maxTouchCfg_T48_CMI_V10[] =
{
	0x1,
	0x80,
	0x62,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0xA,
	0x14,
	0x0,
	0x0,
	0x0,
	0xA,
	0x6,
	0x0,
	0x0,
	0x64,
	0x4,
	0x40,
	0xA,
	0x0,
	0x14,
	0x0,
	0x0,
	0x0,
	0x0,
	0x14,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
};

struct obj_t maxTouchCfg_T5_obj =
{
	.size = 0,
	.obj_addr = 0,
	.value_array = NULL,
	.msg_handler = NULL,
	.config_crc = 0,
};

struct obj_t maxTouchCfg_T6_obj =
{                                 
	.value_array = NULL,
	.msg_handler = T6_msg_handler,
	.config_crc = 0,
};

struct obj_t maxTouchCfg_T38_obj =
{                                 	
	.value_array = maxTouchCfg_T38_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 0,
};

struct obj_t maxTouchCfg_T7_obj =
{                                 
	.value_array = maxTouchCfg_T7_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T8_obj =
{                                 
	.value_array = maxTouchCfg_T8_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T9_obj =
{                                 
	.value_array = maxTouchCfg_T9_CMI_V10,
	.msg_handler = T9_msg_handler,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T15_obj =
{                                 
	.value_array = maxTouchCfg_T15_CMI_V10,
	.msg_handler = T15_msg_handler,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T18_obj =
{                                 
	.value_array = maxTouchCfg_T18_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T19_obj =
{                                 
	.value_array = maxTouchCfg_T19_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T23_obj =
{                                 
	.value_array = maxTouchCfg_T23_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T25_obj =
{                                 
	.value_array = maxTouchCfg_T25_CMI_V10,
	.msg_handler = T25_msg_handler,
	.config_crc = 1,
	
};

struct obj_t maxTouchCfg_T37_obj =
{                                 
	.value_array = NULL,
	.msg_handler = NULL,
	.config_crc = 0,
};

struct obj_t maxTouchCfg_T44_obj =
{                                 	
	.value_array = NULL,
	.msg_handler = NULL,
	.config_crc = 0,
};

struct obj_t maxTouchCfg_T40_obj =
{                                 
	.value_array = maxTouchCfg_T40_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T42_obj =
{                                 
	.value_array = maxTouchCfg_T42_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T46_obj =
{                                 
	.value_array = maxTouchCfg_T46_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T47_obj =
{                                 
	.value_array = maxTouchCfg_T47_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

struct obj_t maxTouchCfg_T48_obj =
{                                 
	.value_array = maxTouchCfg_T48_CMI_V10,
	.msg_handler = NULL,
	.config_crc = 1,
};

#endif
