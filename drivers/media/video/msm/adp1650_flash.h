#ifndef LEDFLASH_ADP1650_H
#define LEDFLASH_ADP1650_H


#define INFO_REG			(uint8_t) 0x00
#define TIMER_REG			(uint8_t) 0x02
#define CURRENT_SET_REG 		(uint8_t) 0x03
#define OUTPUT_MODE_REG 		(uint8_t) 0x04


#define TIMER_200MS			(uint8_t) 0x01
#define TIMER_1600MS			(uint8_t) 0x0f


#define TIMER_200MS			(uint8_t) 0x01
#define TIMER_300MS			(uint8_t) 0x02
#define TIMER_1600MS			(uint8_t) 0x0f


#define CUR_FL_1000MA			(uint8_t) 0xe
#define CUR_FL_700MA			(uint8_t) 0x8
#define CUR_FL_500MA			(uint8_t) 0x4
#define CUR_FL_300MA			(uint8_t) 0x0

#define CUR_TOR_25MA			(uint8_t) 0x00
#define CUR_TOR_50MA			(uint8_t) 0x01
#define CUR_TOR_75MA			(uint8_t) 0x02
#define CUR_TOR_100MA			(uint8_t) 0x03
#define CUR_TOR_125MA			(uint8_t) 0x04
#define CUR_TOR_150MA			(uint8_t) 0x05
#define CUR_TOR_175MA			(uint8_t) 0x06
#define CUR_TOR_200MA			(uint8_t) 0x07


#define TIMER_MODE_DEF    (uint8_t) 0xa0



#define OUTPUT_MODE_DEF 		(uint8_t) 0xa0
#define OUTPUT_MODE_DEF_EDGE 		(uint8_t) 0x80
#define OUTPUT_MODE_ASSIST		(uint8_t) 0x0a
#define OUTPUT_MODE_FLASH		(uint8_t) 0x0f 
#define OUTPUT_MODE_FLASH_SW		(uint8_t) 0x0b 

#endif
