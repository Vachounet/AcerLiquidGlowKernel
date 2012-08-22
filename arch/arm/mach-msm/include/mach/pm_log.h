#ifndef __PM_LOG_H_
#define __PM_LOG_H_

enum pm_log_debug_level {
	PMLOG_DBG_ERR 	= 0,
	PMLOG_DBG_TRACE 	= 1,
};

enum {
	PM_LOG_ON = 0,
	PM_LOG_OFF
};

enum {
	PM_LOG_TOUCH = 0,
	PM_LOG_SDCARD,
	PM_LOG_CAMERA,
	PM_LOG_BLUETOOTH,
	PM_LOG_WIFI,
	PM_LOG_AUDIO_SPK,
	PM_LOG_AUDIO_HS,
	PM_LOG_AUTIO_MIC,
	PM_LOG_BL_KEYPAD,
	PM_LOG_BL_LCD,
	PM_LOG_FLASHLIGHT,
	PM_LOG_LCD,
	PM_LOG_VIBRATOR,
	PM_LOG_SENSOR_PROXIMITY,
	PM_LOG_SENSOR_ALS,
	PM_LOG_SENSOR_ECOMPASS,
	PM_LOG_G_SENSOR,
	PM_LOG_NFC,
	PM_LOG_LED,
	PM_LOG_FM_RADIO,
	PM_LOG_NUM
};


#define	PM_LOG_NAME	\
{	\
	"Touch",	\
	"SD Card",	\
	"Camera",		\
	"Bluetooth",		\
	"WiFi",			\
	"Audio Speaker",	\
	"Audio Headset",	\
	"Audio Microphone",	\
	"Backlight (Keypad)",	\
	"Backlight (LCD)",	\
	"Flashlight",		\
	"LCD",			\
	"Vibrator",		\
	"Sensor (Proximity)",	\
	"Sensor (ALS)",		\
	"Sensor (ECOMPASS)",		\
	"Sensor (G sensor)",	\
	"NFC",	\
	"LED",	\
	"FM Radio",	\
}

#ifdef CONFIG_PM_LOG
#define	PM_LOG_EVENT(x,y)	pm_log_event(x,y)
#else
#define	PM_LOG_EVENT(x,y)	do{} while(0)
#endif

void pm_log_event(int mode, unsigned which_src);

#endif
