
#ifndef _OEM_SMEM_STRUCT_H_
#define _OEM_SMEM_STRUCT_H_

typedef  unsigned long long uint64;
typedef  unsigned long int  uint32;  
typedef  unsigned short  uint16;  
typedef  unsigned char  uint8;  
  typedef enum
  {
	  CHICAGO_BAND_1_8						 = 18,
	  CHICAGO_BAND_2_5						 = 25,
	  CHICAGO_BAND_1_2_5 					 = 125,
	  CHICAGO_BAND_NOT_SUPPORTED			 = 0x7fffffff
  } CHICAGO_BAND_TYPE;

  typedef enum
  {
	CHICAGO_EVB1					= 	0x1,
	CHICAGO_EVB2					= 	0x2,
	CHICAGO_EVT1					=	0x10,
	CHICAGO_EVT1_2_A				=	0x11,
	CHICAGO_EVT1_2_AA			= 	0x12,
	CHICAGO_EVT2					=	0x20,
	CHICAGO_EVT3					=	0x30,
	CHICAGO_EVT3_2					=	0x31,
	CHICAGO_DVT1					=	0x100,
	CHICAGO_DVT2					=	0x200,
	CHICAGO_PVT1					=	0x1000,
	CHICAGO_PVT2					=	0x2000,
	  CHICAGO_UNKNOWN_BORAD_ID					= 0x7fffffff
  } CHICAGO_BOARD_ID_TYPE;

  typedef struct
  {
	  CHICAGO_BAND_TYPE 			chicago_band_type;
	  CHICAGO_BOARD_ID_TYPE		chicago_board_id;
	  uint32							chicago_hw_id_adc_value;
	  uint32							chicago_ddr_size_in_megabyte;
  } CHICAGO_HW_INFO;
  
typedef enum {
  DEFAULT_NV_RESTORE_STATUS_NOT_READY = 0,
  DEFAULT_NV_RESTORE_STATUS_READY =1,
  DEFAULT_NV_RESTORE_STATUS_MAX = 0x7fffffff,
} default_nv_restore_status_type;
 
  typedef struct 
  {
   uint8    imei[9];
   uint8    serial_no[16];
   uint8    drm_key[400];
   uint8    bt_mac_addr[12];
   uint8    wlan_mac_addr[12];
   uint8    simlockkey[128];
  } device_otp_smem_data_s;

  typedef struct
  {
    long  time;
    long  cap;
    long  volt;
    long  ai;
    unsigned char ischarging;
    char __padding[3];
  } smem_bat_info_data;

  typedef enum
  {
      WCDMA_TX = 0,
      WCDMA_RX,
      GSM_TX,
      GSM_RX,
      GPS,
      AP_WAKE,
      TOUCH,
      SDCARD,
      CAMERA,
      BT,
      WIFI,
      AUDIO_SPEAKER,
      AUDIO_HEADSET,
      AUDIO_MIC,
      BK_LIGHT_KEYPAD,
      BK_LIGHT_LCD,
      FLASH_LIGHT,
      LCD,
      VIBRATOR,
      SENSOR_PROXI,
      SENSOR_ALS,
      SENSOR_ECOMPASS,
      GSENSOR,
      NFC,
      LED,
      FM_RADIO, 
      LAST_COMP    
  } comp_id_type;

#ifdef QISDA_PM_LOG_COMPNAME  
  const char *CompName[LAST_COMP] = {
      "WCDMA_TX", 
      "WCDMA_RX", 
      "GSM_TX", 
      "GSM_RX", 
      "GPS",
      "AP_WAKE",
      "TOUCH",
      "SDCARD",
      "CAMERA",
      "BT",
      "WIFI",
      "AUDIO_SPEAKER",
      "AUDIO_HEADSET",
      "AUDIO_MIC",
      "BK_LIGHT_KEYPAD",
      "BK_LIGHT_LCD",
      "FLASH_LIGHT",
      "LCD",
      "VIBRATOR",
      "SENSOR_PROXI",
      "SENSOR_ALS",
      "SENSOR_ECOMPASS",
      "GSENSOR",
      "NFC",
      "LED",
      "FM_RADIO"
      };

#endif

#define BOOT_FLASH_NUM_PART_ENTRIES 16
#define BOOT_FLASH_PART_NAME_LENGTH 16

struct boot_flash_usr_partition_entry {
  char name[BOOT_FLASH_PART_NAME_LENGTH];
  unsigned img_size;
  unsigned short padding;
  unsigned short which_flash;

  unsigned char reserved_flag1;
  unsigned char reserved_flag2;
  unsigned char reserved_flag3;
  unsigned char reserved_flag4;
};

struct boot_flash_usr_partition_table {
  unsigned magic1;
  unsigned magic2;
  unsigned version;
  unsigned numparts;
  struct boot_flash_usr_partition_entry part_entry[BOOT_FLASH_NUM_PART_ENTRIES];
};

  typedef struct
  {
    uint32  IsRPCTimeout;
    device_otp_smem_data_s  device_otp_data;
    device_otp_smem_data_s  device_otp_data_nv_data_currently;
    char  cust_mob_sw_rev[128];
    uint8 acer_22_code_sn[22];
  } smem_vendor_id0_amss_data;
  
  typedef struct
  {
    smem_bat_info_data diag_pm_batt;
    uint32 diag_pm_comp[LAST_COMP- TOUCH];
    uint32 diag_pm_comp_map;
    unsigned char tcpports[8192];
    unsigned char udpports[8192];
    unsigned char ipforwarding;
    uint8 p_state;
  }smem_vendor_id1_apps_data;
  
  typedef struct
  {    
    CHICAGO_HW_INFO  c7_hw_info;
    default_nv_restore_status_type default_nv_restore_status;
    struct boot_flash_usr_partition_table boot_smem_usr_ptable;
    uint8 oemsbl_sd_detect_status;
   }smem_vendor_id2_bl_data;
  

#endif
