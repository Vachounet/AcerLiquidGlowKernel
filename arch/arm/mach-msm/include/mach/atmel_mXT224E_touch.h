




#ifndef _ATMEL_MXT224E_TOUCH_H_
#define _ATMEL_MXT224E_TOUCH_H_

#define ATMEL_TS_NAME "touch_ATMEL_mXT224E"
#define ATMEL_KEYARRAY_NAME "capkey_ATMEL_mXT224E"
#define NUM_OF_ID_INFO_BLOCK                     7 
#define OBJECT_TABLE_ELEMENT_SIZE                6
#define ATMEL_REPORT_POINTS     		         4
#define ATMEL_MULTITOUCH_BUF_LEN                 20
#define INFO_BLOCK_CHECKSUM_SIZE                 3
#define CONFIG_CHECKSUM_SIZE                     4
#define MAX_OBJ_ELEMENT_SIZE                     30
#define WRITE_T6_SIZE                            1
#define ADD_ID_INFO_BLOCK                        0x0000 
#define OBJECT_TABLE_ELEMENT_1                   0x0007 
#define GEN_MESSAGEPROCESOR_T5                   0x5
#define GEN_COMMANDPROCESSOR_T6                  0x6
#define SPT_USERDATA_T38                         0x26
#define DEBUG_DIAGNOSTIC_T37                     0x25
#define GEN_POWERCONFIG_T7                       0x7
#define GEN_ACQUISITIONCONFIG_T8                 0x8	
#define TOUCH_MULTITOUCHSCREEN_T9                0x9
#define TOUCH_KEYARRAY_T15                       0xf                        
#define SPT_COMMSCONFIG_T18                      0x12                       
#define SPT_GPIOPWM_T19                          0x13
#define TOUCH_PROXIMITY_T23                      0x17        
#define SPT_SELFTEST_T25                         0x19
#define PROCI_GRIPSUPPRESSION_T40                0x28
#define PROCI_TOUCHSUPPRESSION_T42               0x2A
#define SPT_MESSAGECOUNT_T44                     0x2C
#define SPT_CTECONFIG_T46                        0x2E
#define PROCI_STYLUS_T47                         0x2F
#define PROCI_NOISESUPPRESSION_T48               0x30
#define CFGERR_MASK                              0x8
#define T6_RESET_VALUE                           0x8
#define T6_BACKUPNV_VALUE                        0x55
#define TOUCH_PRESS_MASK                         0x40
#define TOUCH_RELEASE_MASK                       0x20
#define T6_CALIBRATE_VALUE                       0x01
#define TOUCH_PRESS_RELEASE_MASK                 0x60
#define TOUCH_MOVE_MASK                          0x10
#define TOUCH_DETECT_MASK                        0x80
#define x_channel                                18
#define y_channel                                12
#define ATMEL_X_ORIGIN                           0
#define ATMEL_Y_ORIGIN                           0
#define ATMEL_X_MAX                              1024
#define ATMEL_Y_MAX                              1024
#define ATMEL_TOUCAN_PANEL_X                     1024
#define ATMEL_TOUCAN_PANEL_Y                     1024                
#define NUM_OF_REF_MODE_PAGE                     4
#define SIZE_OF_REF_MODE_PAGE                    130    
#define TOTAL_TP_COUNT                           500  
#define T7_MAX_SIZE                              3   
     
struct atmel_platform_data {
    unsigned int irq_gpio;
    unsigned int rst_gpio;
};

enum atmel_touch_state_t {
    NOT_TOUCH_STATE = 0,
    ONE_TOUCH_STATE,
    TWO_TOUCH_STATE,
    MAX_TOUCH_STATE
};

struct obj_t                      
{                                 
	uint16_t      obj_addr; 
	uint8_t       size;       
	uint8_t       instance;
	uint8_t       num_reportid;   
	uint8_t       reportid_ub; 
	uint8_t       reportid_lb; 
	uint8_t       *value_array;
	int (*msg_handler)(uint8_t* message_data);
	uint8_t       config_crc;
};

struct id_info_t                      
{                                 
	uint8_t  family_id; 
	uint8_t  variant_id;       
	uint8_t  version;
	uint8_t  build;
	uint8_t  matrix_x_size;
	uint8_t  matrix_y_size;
	uint8_t  num_obj_element;	
};

enum touch_state_t {
    RELEASE = 0,
    PRESS,
    MOVE
};

struct atmel_point_t {
    uint   x;
    uint   y;
};

#if 0
struct atmel_multipoints_t {
    struct atmel_point_t   points[ATMEL_REPORT_POINTS];
    enum atmel_touch_state_t state;
};
#endif


struct touch_point_status_t 
{
    struct atmel_point_t  coord;
    enum touch_state_t state;
    int16_t z;
    uint16_t w;
};


struct atmel_power_T7_t {
	uint8_t 	idleacqint;
	uint8_t 	actvacqint;
    uint8_t 	actv2idleto;
};

struct atmel_acquisition_T8_t {
    uint8_t		chrgtime;
	uint8_t 	tchdrift;
	uint8_t 	driftst;
	uint8_t	    tchautocal;
	uint8_t		sync;
	uint8_t		atchcalst;
	uint8_t     atchcalsthr;
	uint8_t		atchfrccalthr;
	uint8_t     atchfrccalratio;
};

struct atmel_multitouchscreen_T9_t {
	uint8_t   	ctrl;
	uint8_t     xorigin;
	uint8_t     yorigin;
	uint8_t     xsize;
	uint8_t     ysize;
	uint8_t     akscfg;
	uint8_t     blen; 
	uint8_t     tchthr; 
	uint8_t     tchdi;
	uint8_t     orient;
	uint8_t     mrgtimeout;
	uint8_t     movhysti; 
	uint8_t     movhystn; 
	uint8_t     movfilter;
	uint8_t     numtouch;
	uint8_t     mrghyst; 
	uint8_t     mrgthr; 
	uint8_t     amphyst; 
	uint8_t     xrange; 
	uint8_t     yrange; 
	uint8_t     xloclip; 
	uint8_t     xhiclip; 
	uint8_t     yloclip; 
	uint8_t     yhiclip; 
	uint8_t     xedgectrl;
	uint8_t     xedgedist;
	uint8_t     yedgectrl;
	uint8_t     yedgedist;
	uint8_t     jumplimit;
	uint8_t     tchhyst;
	uint8_t     xpitch;
	uint8_t     ypitch;
	uint8_t     nexttchdi;
};

struct atmel_keyarray_T15_t {
    uint8_t		ctrl;
	uint8_t		xorigin;
	uint8_t		yorigin;
	uint8_t 	xsize;
	uint8_t 	ysize;
	uint8_t 	akscfg;
	uint8_t 	blen; 
	uint8_t 	tchthr; 
	uint8_t 	tchdi;  
};

struct atmel_grip_suppression_T40_t {
    uint8_t		ctrl;
	uint8_t		xlogrip;
	uint8_t		xhigrip;
	uint8_t		ylogrip;
	uint8_t		yhigrip;
};

struct atmel_touch_suppression_T42_t {
    uint8_t		ctrl;
	uint8_t		apprthr;
	uint8_t		maxapprarea;
	uint8_t 	maxtcharea;
	uint8_t 	supstrength;
	uint8_t 	supextto;
	uint8_t 	maxnumtchs;
	uint8_t 	shapestrength;
};

struct atmel_cte_T46_t {
    uint8_t		ctrl;
	uint8_t		mode;
	uint8_t		idlesyncsperx;
	uint8_t 	actvsyncsperx;
	uint8_t 	adcspersync;
	uint8_t 	pulsesperadc;
	uint8_t 	xslew;
	uint8_t 	syncdelay;
};

struct atmel_stylus_T47_t {
    uint8_t		ctrl;
	uint8_t		contmin;
	uint8_t		contmax;
	uint8_t 	stability;
	uint8_t 	maxtcharea;
	uint8_t 	amplthr;
	uint8_t 	styshape;
	uint8_t 	hoversup;
	uint8_t 	confthr;
	uint8_t 	syncsperx;
};

struct atmel_noise_suppression_T48_t {
    uint8_t		ctrl;
	uint8_t		cfg;
	uint8_t		calcfg;
	uint8_t 	basefreq;
	uint8_t 	mffreq_0;
	uint8_t 	mffreq_1;
	uint8_t 	gcactvinvldadcs;
	uint8_t 	gcidleinvldadcs;
	uint8_t 	gcmaxadcsperx;
	uint8_t 	gclimitmin;
	uint8_t 	gclimitmax;
	uint8_t 	gccountmintgt;
	uint8_t 	mfinvlddiffthr; 
	uint8_t 	mfincadcspxthr; 
	uint8_t 	mferrorthr; 
	uint8_t 	selfreqmax;
	uint8_t 	blen; 
	uint8_t     tchthr; 
	uint8_t     tchdi;
	uint8_t     movhysti; 
	uint8_t     movhystn; 
	uint8_t     movfilter;
	uint8_t     numtouch;
	uint8_t     mrghyst; 
	uint8_t     mrgthr; 
	uint8_t     xloclip; 
	uint8_t     xhiclip; 
	uint8_t     yloclip; 
	uint8_t     yhiclip; 
	uint8_t     xedgectrl;
	uint8_t     xedgedist;
	uint8_t     yedgectrl;
	uint8_t     yedgedist;
	uint8_t     jumplimit;
	uint8_t     tchhyst;
	uint8_t     nexttchdi;
};

struct atmel_power_switch_t {
    uint   on;
};
struct atmel_references_mode_t {
	uint16_t  data[x_channel][y_channel];
};

struct atmel_deltas_mode_t {
	int16_t  deltas[x_channel][y_channel];
};

struct atmel_fvs_mode_t {
    uint   enter;
};

struct atmel_id_info_t {
    uint8_t   version;
	uint8_t   chip_id;
};

struct atmel_selftest_t {
	uint      on;
	uint8_t   value;
};

struct atmel_EM_key_delta_t {
    int   onoff;
};


#define __ATMELTOUCHDRVIO 0xAA
#define ATMEL_TOUCH_SET_POWER_MODE _IOW(__ATMELTOUCHDRVIO, 1, struct atmel_power_T7_t)
#define ATMEL_TOUCH_GET_POWER_MODE _IOR(__ATMELTOUCHDRVIO, 2, struct atmel_power_T7_t)
#define ATMEL_TOUCH_SET_ACQUISITION _IOW(__ATMELTOUCHDRVIO, 3, struct atmel_acquisition_T8_t)
#define ATMEL_TOUCH_GET_ACQUISITION _IOR(__ATMELTOUCHDRVIO, 4, struct atmel_acquisition_T8_t)
#define ATMEL_TOUCH_SET_MULTITOUCHSCREEN _IOW(__ATMELTOUCHDRVIO, 5, struct atmel_multitouchscreen_T9_t)
#define ATMEL_TOUCH_GET_MULTITOUCHSCREEN _IOR(__ATMELTOUCHDRVIO, 6, struct atmel_multitouchscreen_T9_t)
#define ATMEL_TOUCH_SET_KEYARRAY _IOW(__ATMELTOUCHDRVIO, 7, struct atmel_keyarray_T15_t)
#define ATMEL_TOUCH_GET_KEYARRAY _IOR(__ATMELTOUCHDRVIO, 8, struct atmel_keyarray_T15_t)
#define ATMEL_TOUCH_SET_GRIPSUPPRESSION _IOW(__ATMELTOUCHDRVIO, 9, struct atmel_grip_suppression_T40_t)
#define ATMEL_TOUCH_GET_GRIPSUPPRESSION _IOR(__ATMELTOUCHDRVIO, 10, struct atmel_grip_suppression_T40_t)
#define ATMEL_TOUCH_SET_TOUCHSUPPRESSION _IOW(__ATMELTOUCHDRVIO, 11, struct atmel_touch_suppression_T42_t)
#define ATMEL_TOUCH_GET_TOUCHSUPPRESSION _IOR(__ATMELTOUCHDRVIO, 12, struct atmel_touch_suppression_T42_t)
#define ATMEL_TOUCH_SET_CTE _IOW(__ATMELTOUCHDRVIO, 13, struct atmel_cte_T46_t)
#define ATMEL_TOUCH_GET_CTE _IOR(__ATMELTOUCHDRVIO, 14, struct atmel_cte_T46_t)
#define ATMEL_TOUCH_SET_STYLUS _IOW(__ATMELTOUCHDRVIO, 15, struct atmel_stylus_T47_t)
#define ATMEL_TOUCH_GET_STYLUS _IOR(__ATMELTOUCHDRVIO, 16, struct atmel_stylus_T47_t)
#define ATMEL_TOUCH_SET_NOISESUPPRESSION _IOW(__ATMELTOUCHDRVIO, 17, struct atmel_noise_suppression_T48_t)
#define ATMEL_TOUCH_GET_NOISESUPPRESSION _IOR(__ATMELTOUCHDRVIO, 18, struct atmel_noise_suppression_T48_t)
#define ATMEL_TOUCH_SET_POWER_SWITCH _IOW(__ATMELTOUCHDRVIO, 19, struct atmel_power_switch_t)
#define ATMEL_TOUCH_GET_REFERENCES_MODE _IOR(__ATMELTOUCHDRVIO, 20, struct atmel_references_mode_t)
#define ATMEL_TOUCH_GET_DELTAS_MODE _IOR(__ATMELTOUCHDRVIO, 21, struct atmel_deltas_mode_t)
#define ATMEL_TOUCH_GET_VERSION _IOR(__ATMELTOUCHDRVIO, 22, struct atmel_id_info_t)
#define ATMEL_TOUCH_SET_SELFTEST_FVS_MODE _IOW(__ATMELTOUCHDRVIO, 23, struct atmel_selftest_t)
#define ATMEL_TOUCH_SET_EM_KEY_DELTA _IOW(__ATMELTOUCHDRVIO, 24, struct atmel_EM_key_delta_t)
#endif
