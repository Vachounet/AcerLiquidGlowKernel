#ifndef __BOOT_MODE_H
#define __BOOT_MODE_H


#define RECOVERY_MODE   	0x77665502
#define FASTBOOT_MODE   	0x77665500
#define USER_REBOOT     	0x77665510
#define REBOOT_PWRKEY_MODE	0x7766aa04
#define REBOOT_CHARGE_MODE	0x7766aa05
#define NONE_MODE       	0x77665501
#define EUU_HEX_2_FASTBOOT	0x7766aa0e
#define RD_HEX_2_FASTBOOT	0x7766aa0f
#define SD_RECOVERY_MODE	0x7766aa09
#define MENU_RECOVERY_MODE	0x7766aa0A
#define ABNORMAL_KERNEL_REBOOT	0x7766aa12
#define ABNORMAL_HOTKEY_REBOOT	0x7766aa13
#define ABNORMAL_AMSS_REBOOT	0x7766aa14
#define ABNORMAL_SMPL_REBOOT    0x7766aa15
#define KERNEL_REBOOT_FOR_CPO   0x7766aa16
#define ABNORMAL_PHONE_REBOOT         0x7766aa17
#define HEX_DL_MODE         0x7766aa28
#define MULTI_DL_FASTBOOT_MODE_HS1   	0x7766aa20
#define MULTI_DL_FASTBOOT_MODE_HS2   	0x7766aa21
#define MULTI_DL_FASTBOOT_MODE_HS3   	0x7766aa22
#define MULTI_DL_FASTBOOT_MODE_HS4   	0x7766aa23
#define MULTI_DL_FASTBOOT_MODE_HS5   	0x7766aa24
#define MULTI_DL_FASTBOOT_MODE_HS6   	0x7766aa25
#define MULTI_DL_FASTBOOT_MODE_HS7   	0x7766aa26
#define MULTI_DL_FASTBOOT_MODE_HS8   	0x7766aa27
#define RAMDUMP_MODE_AMSS_CRASH         0x7766aa30
#define RAMDUMP_MODE_ABNORMAL_PHONE     0x7766aa31
#define LOW_BAT_MODE                    0x7766aa40
#define INVALID_BAT_MODE                0x7766aa41
#define FACTORY_RESET_MODE	0x776655EF
#define RESTART_REASON_MAGIC	0xA4BC385F
#define RECOVERY_MODE_STR   	"recovery"
#define FASTBOOT_MODE_STR   	"fastboot"
#define USER_REBOOT_STR     	"user-reboot"
#define REBOOT_PWRKEY_MODE_STR	"pwrkey-on"
#define REBOOT_CHARGE_MODE_STR	"charge"
#define NONE_MODE_STR       	"normal"
#define EUU_HEX_2_FASTBOOT_STR	"euuhex2fastboot"
#define RD_HEX_2_FASTBOOT_STR		"rdhex2fastboot"
#define SD_RECOVERY_MODE_STR		"sd-recovery"
#define MENU_RECOVERY_MODE_STR		"menu-recovery"
#define ABNORMAL_KERNEL_REBOOT_STR	"kernel-crash"
#define ABNORMAL_HOTKEY_REBOOT_STR	"hotkey-crash"
#define ABNORMAL_AMSS_REBOOT_STR	"amss-crash"
#define ABNORMAL_SMPL_REBOOT_STR		"smpl-reboot"
#define KERNEL_REBOOT_FOR_CPO_STR   "CPO-test"
#define ABNORMAL_PHONE_REBOOT_STR       "abnormal-phone-reboot"
#define RAMDUMP_MODE_AMSS_CRASH_STR           "amss-crash-ramdump"
#define RAMDUMP_MODE_ABNORMAL_PHONE_STR       "abnormal-phone-ramdump"
#define HEX_DL_MODE_STR			"hexdownload"
#endif
