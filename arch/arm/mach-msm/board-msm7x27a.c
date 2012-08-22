/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/usbdiag.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/usb/android.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <mach/rpc_pmapp.h>
#include <linux/smsc911x.h>
#include <linux/atmel_maxtouch.h>
#include <linux/ion.h>
#include "devices.h"
#include "timer.h"
#include "board-msm7x27a-regulator.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>
#include "pm-boot.h"
#include "board-msm7627a.h"

#include <mach/chicago_hwid.h>

#define kernel_debuglevel_dir_path "debuglevel"
struct dentry *kernel_debuglevel_dir;
EXPORT_SYMBOL(kernel_debuglevel_dir);
#include <linux/debugfs.h>


#ifdef CONFIG_BATTERY_MSM
#include <mach/msm_battery.h>
#endif

#ifdef CONFIG_CHARGER_MSM7x27A
#include <linux/msm-charger.h>
#endif

#ifdef CONFIG_SMB136_CHARGER
#include <linux/i2c/smb136.h>
#endif

#ifdef CONFIG_BATTERY_BQ27520
#include <linux/i2c/bq27520.h>
#endif



#ifdef CONFIG_PN544_NFC
#include <linux/nfc/pn544.h>
#endif


#define C7_HEADSET

#ifdef CONFIG_SURF_FFA_GPIO_KEYPAD
#include "msm-keypad-devices.h"
#endif


#include "msm-keypad-devices.h"
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TMA340
#include <mach/touch_TMA340_cypress.h>
#endif


#ifdef CONFIG_TOUCHSCREEN_ATMEL_mXT224E
#include <mach/atmel_mXT224E_touch.h>
#endif



#include <mach/pm_log.h>


#define PMEM_KERNEL_EBI1_SIZE	0x3A000
#define MSM_PMEM_AUDIO_SIZE	0x5B000


#define wifi_mac_str_length 12
unsigned char wifi_mac[wifi_mac_str_length] = {'N', 'O', 'N', 'E'};
EXPORT_SYMBOL(wifi_mac);



unsigned int ATH6KL_DLL = 0;
EXPORT_SYMBOL(ATH6KL_DLL);


#if defined(CONFIG_GPIO_SX150X)
enum {
	SX150X_CORE,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_CORE]	= {
		.gpio_base		= GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0x02,
		.io_open_drain_ena	= 0xfef8,
		.irq_summary		= -1,
	},
};
#endif


static struct platform_device msm_wlan_ar6000_pm_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
};


static int __init wifi_mac_setup(char *options)
{
    if (!options || !*options) {
        return 0;
    } else {
        memcpy(wifi_mac, options, wifi_mac_str_length);
    }
    return 0;
}
__setup("wlanmac=", wifi_mac_setup);

#if defined(CONFIG_BOSCH_BMA250)

#define G_SENSOR_INT	28
#define G_SENSOR_INT_IRQ_NUM	(MSM_GPIO_TO_INT(G_SENSOR_INT))
#endif
#if defined(CONFIG_SENSORS_AK8975)
#define E_COMPASS_INT	18
#define E_COMPASS_INT_IRQ_NUM	(MSM_GPIO_TO_INT(E_COMPASS_INT))
#endif
#if defined(CONFIG_SENSORS_ISL29028)
#define ALS_INT_N		20
#define	ALS_INT_N_IRQ_NUM		(MSM_GPIO_TO_INT(ALS_INT_N))
#endif

#if (defined(CONFIG_BOSCH_BMA250) || defined(CONFIG_BOSCH_BMA250)|| defined(CONFIG_SENSORS_ISL29028))
static unsigned sensors_gpio_config[] = {
	#if defined(CONFIG_BOSCH_BMA250)
	
	GPIO_CFG(G_SENSOR_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	#endif
	#if defined(CONFIG_SENSORS_AK8975)
	
	GPIO_CFG(E_COMPASS_INT, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	#endif
	#if defined(CONFIG_SENSORS_ISL29028)
	
	GPIO_CFG(ALS_INT_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	#endif
	
};

static int sensors_config_gpio_table(void)
{
	int rc = 0, i = 0;

	for (i = 0; i < ARRAY_SIZE(sensors_gpio_config); i++) {
		rc = gpio_tlmm_config(sensors_gpio_config[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s not able to get gpio\n", __func__);
		}
	}
	return rc;
}
#endif


#ifdef CONFIG_BOSCH_BMA250
static struct i2c_board_info bma250_board_info[] __initdata= {

	{
		I2C_BOARD_INFO("bma250", 0x18),
		.irq = G_SENSOR_INT_IRQ_NUM,
		.platform_data = NULL,
	},
};
#endif

#ifdef CONFIG_SENSORS_AK8975
#include <linux/akm8975.h>
static struct akm8975_platform_data akm8975_data = {
	.intr = E_COMPASS_INT,
	.init = NULL,
 	.exit = NULL,
	.power_on = NULL,
	.power_off = NULL,
};
static struct i2c_board_info akm8975_board_info[] __initdata= {
	{
		I2C_BOARD_INFO("akm8975", 0x0d),
		.irq = E_COMPASS_INT_IRQ_NUM,
		.platform_data = &akm8975_data,
	},
};
#endif


#if (defined(CONFIG_SENSORS_ISL29028))
static struct i2c_board_info isl29028_board_info[]  __initdata= {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
		.irq = ALS_INT_N_IRQ_NUM,
		.platform_data = NULL,
	},
};
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static struct i2c_board_info core_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
	},
};

static void __init register_i2c_devices(void)
{
	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
		sx150x_data[SX150X_CORE].io_open_drain_ena = 0xe0f0;

	core_exp_i2c_info[0].platform_data =
			&sx150x_data[SX150X_CORE];

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				core_exp_i2c_info,
				ARRAY_SIZE(core_exp_i2c_info));
}
#endif

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
		"qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
		"qup_sda" },
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc;

	if (adap_id < 0 || adap_id > 1)
		return;

	/* Each adapter gets 2 lines from the table */
	if (config_type)
		rc = msm_gpios_request_enable(&qup_i2c_gpios_hw[adap_id*2], 2);
	else
		rc = msm_gpios_request_enable(&qup_i2c_gpios_io[adap_id*2], 2);
	if (rc < 0)
		pr_err("QUP GPIO request/enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_PMEM_MDP_SIZE       0x2300000
#define MSM7x25A_MSM_PMEM_MDP_SIZE       0x1500000

#define MSM_PMEM_ADSP_SIZE      0x1100000
#define MSM7x25A_MSM_PMEM_ADSP_SIZE      0xB91000


#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE		0x4BF000
#define MSM7x25A_MSM_FB_SIZE	0x1C2000
#define MSM8x25_MSM_FB_SIZE	0x5FA000
#else
#define MSM_FB_SIZE		0x32A000
#define MSM7x25A_MSM_FB_SIZE	0x12C000
#define MSM8x25_MSM_FB_SIZE	0x3FC000
#endif

#endif

#ifdef CONFIG_ION_MSM
#define MSM_ION_HEAP_NUM        4
static struct platform_device ion_dev;
static int msm_ion_camera_size;
static int msm_ion_audio_size;
static int msm_ion_sf_size;
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_POWEROFF_CHARGING
enum ANDROIDBOOTMODE androidboot_mode=ANDROIDBOOTMODE_MAX;
EXPORT_SYMBOL(androidboot_mode);
int __init board_androidboot_mode_setup(char *s)
{
    if (!strcmp(s, "normal"))
        androidboot_mode = ANDROIDBOOTMODE_NORMAL;
    else if (!strcmp(s, "factory"))
        androidboot_mode = ANDROIDBOOTMODE_FACTORY;
    else if (!strcmp(s, "recovery"))
        androidboot_mode = ANDROIDBOOTMODE_RECOVERY;
    else if (!strcmp(s, "BootRecovery"))
        androidboot_mode = ANDROIDBOOTMODE_RECOVERY;
    else if (!strcmp(s, "SDBootRecovery"))
        androidboot_mode = ANDROIDBOOTMODE_SDRECOVERY;
    else if (!strcmp(s, "BootOffCharge"))
        androidboot_mode = ANDROIDBOOTMODE_BOOTOFFCHARGE;
    else
        androidboot_mode = ANDROIDBOOTMODE_NORMAL;

    printk(KERN_ERR "[SEAN]androidboot_mode=%d by string %s\n",androidboot_mode,s);
    return 1;
}__setup("androidboot.mode=", board_androidboot_mode_setup);

int QrootVariable=0;
EXPORT_SYMBOL(QrootVariable);
int __init board_Qroot_setup(char *s)
{
    if (!strcmp(s, "1"))
        QrootVariable = 1;

    printk(KERN_ERR "[SEAN]QrootVariable is true\n");
    return 1;
}__setup("Qroot=", board_Qroot_setup);

int QadbVariable=0;
EXPORT_SYMBOL(QadbVariable);
int __init board_Qadb_setup(char *s)
{
    if (!strcmp(s, "1"))
        QadbVariable = 1;

    printk(KERN_ERR "[SEAN]QadbVariable is true\n");
    return 1;
}__setup("Qadb=", board_Qadb_setup);

#ifdef CONFIG_BUILD_SHIP
int QdiagVariable=0;
#else
int QdiagVariable=1;
#endif
EXPORT_SYMBOL(QdiagVariable);
int __init board_Qdiag_setup(char *s)
{
    if (!strcmp(s, "1"))
        QdiagVariable = 1;

    printk(KERN_ERR "[SEAN]QdiagVariable is true\n");
    return 1;
}__setup("Qdiag=", board_Qdiag_setup);

int QftdVariable=0;
EXPORT_SYMBOL(QftdVariable);
int __init board_Qftd_setup(char *s)
{
    if (!strcmp(s, "1"))
        QftdVariable = 1;

    printk(KERN_ERR "[SEAN]QftdVariable is true\n");
    return 1;
}__setup("Qftd=", board_Qftd_setup);

int QfixusbidVariable=0;
EXPORT_SYMBOL(QfixusbidVariable);
int __init board_Qfixusbid_setup(char *s)
{
    if (!strcmp(s, "1"))
        QfixusbidVariable = 1;

    printk(KERN_ERR "[SEAN]QfixusbidVariable is true\n");
    return 1;
}__setup("Qfixusbid=", board_Qfixusbid_setup);

int QlognotifyVariable=0;
EXPORT_SYMBOL(QlognotifyVariable);
int __init board_Qlognotify_setup(char *s)
{
    if (!strcmp(s, "1"))
        QlognotifyVariable = 1;

    printk(KERN_ERR "[SEAN]QlognotifyVariable is true\n");
    return 1;
}__setup("Qlognotify=", board_Qlognotify_setup);

int QnooffchargeVariable=0;
EXPORT_SYMBOL(QnooffchargeVariable);
int __init board_Qnooffcharge_setup(char *s)
{    
    if (!strcmp(s, "1"))
        QnooffchargeVariable = 1;
    printk(KERN_ERR "[SEAN]QnooffchargeVariable is true\n");
    return 1;
}__setup("Qnooffcharge=", board_Qnooffcharge_setup);
#endif

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc = 0;
	unsigned gpio;

	gpio = GPIO_HOST_VBUS_EN;

	rc = gpio_request(gpio, "i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", gpio);
		return;
	}
	gpio_direction_output(gpio, !!on);
	gpio_set_value_cansleep(gpio, !!on);
	gpio_free(gpio);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};

static void __init msm7x2x_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct regulator *reg_hsusb;
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		reg_hsusb = regulator_get(NULL, "usb");
		if (IS_ERR(reg_hsusb)) {
			rc = PTR_ERR(reg_hsusb);
			pr_err("%s: could not get regulator: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_set_voltage(reg_hsusb, 3300000, 3300000);
		if (rc) {
			pr_err("%s: could not set voltage: %d\n",
					__func__, rc);
			goto reg_free;
		}

		return 0;
	}
	/* else fall through */
reg_free:
	regulator_put(reg_hsusb);
out:
	reg_hsusb = NULL;
	return rc;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (IS_ERR_OR_NULL(reg_hsusb))
		return reg_hsusb ? PTR_ERR(reg_hsusb) : -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	return enable ?
		regulator_enable(reg_hsusb) :
		regulator_disable(reg_hsusb);
}

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret = 0;

	if (init)
		ret = msm_pm_app_rpc_init(callback);
	else
		msm_pm_app_rpc_deinit(callback);

	return ret;
}
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init	 = msm_hsusb_pmic_notif_init,
#else
	.vbus_power		 = msm_hsusb_vbus_power,
#endif
	.rpc_connect		 = hsusb_rpc_connect,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.chg_init		 = hsusb_chg_init,
	.chg_connected		 = hsusb_chg_connected,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
};
#endif

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x90000300,
		.end   = 0x900003ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(4),
		.end   = MSM_GPIO_TO_INT(4),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif
static struct msm_pm_platform_data msm7x27a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 1,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

u32 msm7627a_power_collapse_latency(enum msm_pm_sleep_mode mode)
{
	switch (mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		return msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN:
		return msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency;
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		return msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		return msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	default:
		return 0;
	}
}

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS,
	.p_addr = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}

early_param("fb_size", fb_size_setup);

#ifdef C7_HEADSET 
static struct resource headset_resources[] = {
	{
		.name	= "jack_int",
		.start	= 94,
		.end		= 94,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "hook_int",
		.start	= 92,
		.end		= 92,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device qsd_headset_device = {
	.name = "headset",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(headset_resources),
	.resource	= headset_resources,
};
#endif

static struct regulator_bulk_data regs_lcdc[] = {
	{ .supply = "gp2",   .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "msme1", .min_uV = 1800000, .max_uV = 1800000 },
};

static uint32_t lcdc_gpio_initialized;

static void lcdc_toshiba_gpio_init(void)
{
	int rc = 0;
	if (!lcdc_gpio_initialized) {
		if (gpio_request(GPIO_SPI_CLK, "spi_clk")) {
			pr_err("failed to request gpio spi_clk\n");
			return;
		}
		if (gpio_request(GPIO_SPI_CS0_N, "spi_cs")) {
			pr_err("failed to request gpio spi_cs0_N\n");
			goto fail_gpio6;
		}
		if (gpio_request(GPIO_SPI_MOSI, "spi_mosi")) {
			pr_err("failed to request gpio spi_mosi\n");
			goto fail_gpio5;
		}
		if (gpio_request(GPIO_SPI_MISO, "spi_miso")) {
			pr_err("failed to request gpio spi_miso\n");
			goto fail_gpio4;
		}
		if (gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr")) {
			pr_err("failed to request gpio_disp_pwr\n");
			goto fail_gpio3;
		}
		if (gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en")) {
			pr_err("failed to request gpio_bkl_en\n");
			goto fail_gpio2;
		}
		pmapp_disp_backlight_init();

		rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_lcdc), regs_lcdc);
		if (rc) {
			pr_err("%s: could not get regulators: %d\n",
					__func__, rc);
			goto fail_gpio1;
		}

		rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_lcdc),
				regs_lcdc);
		if (rc) {
			pr_err("%s: could not set voltages: %d\n",
					__func__, rc);
			goto fail_vreg;
		}
		lcdc_gpio_initialized = 1;
	}
	return;
fail_vreg:
	regulator_bulk_free(ARRAY_SIZE(regs_lcdc), regs_lcdc);
fail_gpio1:
	gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio2:
	gpio_free(GPIO_DISPLAY_PWR_EN);
fail_gpio3:
	gpio_free(GPIO_SPI_MISO);
fail_gpio4:
	gpio_free(GPIO_SPI_MOSI);
fail_gpio5:
	gpio_free(GPIO_SPI_CS0_N);
fail_gpio6:
	gpio_free(GPIO_SPI_CLK);
	lcdc_gpio_initialized = 0;
}

static uint32_t lcdc_gpio_table[] = {
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_SPI_MOSI,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_SPI_MISO,
};

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n;

	if (lcdc_gpio_initialized) {
		/* All are IO Expander GPIOs */
		for (n = 0; n < (len - 1); n++)
			gpio_direction_output(table[n], 1);
	}
}

static void lcdc_toshiba_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}

static int msm_fb_lcdc_power_save(int on)
{
	int rc = 0;
	/* Doing the init of the LCDC GPIOs very late as they are from
		an I2C-controlled IO Expander */
	lcdc_toshiba_gpio_init();

	if (lcdc_gpio_initialized) {
		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);

		rc = on ? regulator_bulk_enable(
				ARRAY_SIZE(regs_lcdc), regs_lcdc) :
			  regulator_bulk_disable(
				ARRAY_SIZE(regs_lcdc), regs_lcdc);

		if (rc)
			pr_err("%s: could not %sable regulators: %d\n",
					__func__, on ? "en" : "dis", rc);
	}

	return rc;
}


static int lcdc_toshiba_set_bl(int level)
{
	int ret;

	ret = pmapp_disp_backlight_set_brightness(level);
	if (ret)
		pr_err("%s: can't set lcd backlight!\n", __func__);

	return ret;
}


static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = NULL,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};

static int lcd_panel_spi_gpio_num[] = {
		GPIO_SPI_MOSI,  /* spi_sdi */
		GPIO_SPI_MISO,  /* spi_sdoi */
		GPIO_SPI_CLK,   /* spi_clk */
		GPIO_SPI_CS0_N, /* spi_cs  */
};

static struct msm_panel_common_pdata lcdc_toshiba_panel_data = {
	.panel_config_gpio = lcdc_toshiba_config_gpios,
	.pmic_backlight = lcdc_toshiba_set_bl,
	.gpio_num	  = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_toshiba_panel_device = {
	.name   = "lcdc_toshiba_fwvga_pt",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_toshiba_panel_data,
	}
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

#ifdef CONFIG_MSM_V4L2_VIDEO_OVERLAY_DEVICE
static struct resource msm_v4l2_video_overlay_resources[] = {
	{
		.flags = IORESOURCE_DMA,
	}
};
#endif

#define LCDC_TOSHIBA_FWVGA_PANEL_NAME	"lcdc_toshiba_fwvga_pt"
#define MIPI_CMD_RENESAS_FWVGA_PANEL_NAME	"mipi_cmd_renesas_fwvga"

static int msm_fb_detect_panel(const char *name)
{
	int ret = -ENODEV;

	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf()) {
		if (!strncmp(name, "lcdc_toshiba_fwvga_pt", 21) ||
				!strncmp(name, "mipi_cmd_renesas_fwvga", 22))
			ret = 0;
	} else if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa()) {
		if (!strncmp(name, "mipi_cmd_renesas_fwvga", 22))
			ret = 0;
	}

#if !defined(CONFIG_FB_MSM_LCDC_AUTO_DETECT) && \
	!defined(CONFIG_FB_MSM_MIPI_PANEL_AUTO_DETECT) && \
	!defined(CONFIG_FB_MSM_LCDC_MIPI_PANEL_AUTO_DETECT)
		if (machine_is_msm7x27a_surf() ||
			machine_is_msm7625a_surf()) {
			if (!strncmp(name, LCDC_TOSHIBA_FWVGA_PANEL_NAME,
				strnlen(LCDC_TOSHIBA_FWVGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
				return 0;
		}
#endif
	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#ifdef CONFIG_MSM_V4L2_VIDEO_OVERLAY_DEVICE
static struct platform_device msm_v4l2_video_overlay_device = {
	.name   = "msm_v4l2_overlay_pd",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_v4l2_video_overlay_resources),
	.resource       = msm_v4l2_video_overlay_resources,
};
#endif

#ifdef CONFIG_FB_MSM_MIPI_DSI
#ifdef CONFIG_FB_MSM_MIPI_DSI_CHIMEI


#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
static int mipi_chimei_set_bl(int level)
{
	int ret=0;
		
	return ret;
}
#else
static int mipi_chimei_set_bl(int level)
{
	int ret;

	ret = pmapp_disp_backlight_set_brightness(level);

	if (ret)
		pr_err("%s: can't set lcd backlight!\n", __func__);

	return ret;
}
#endif

static struct msm_panel_common_pdata mipi_chimei_pdata = {
	.pmic_backlight = mipi_chimei_set_bl,
};


static struct platform_device mipi_dsi_chimei_panel_device = {
	.name = "mipi_chimei",
	.id = 0,
	.dev    = {
		.platform_data = &mipi_chimei_pdata,
	}
};
#else
static int mipi_renesas_set_bl(int level)
{
	int ret;

	ret = pmapp_disp_backlight_set_brightness(level);

	if (ret)
		pr_err("%s: can't set lcd backlight!\n", __func__);

	return ret;
}

static struct msm_panel_common_pdata mipi_renesas_pdata = {
	.pmic_backlight = mipi_renesas_set_bl,
};


static struct platform_device mipi_dsi_renesas_panel_device = {
	.name = "mipi_renesas",
	.id = 0,
	.dev    = {
		.platform_data = &mipi_renesas_pdata,
	}
};
#endif
#endif

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(NO_MIC_HEADSET, 38),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(FM_DIGITAL_STEREO_HEADSET, 26),
	SND(FM_DIGITAL_SPEAKER_PHONE, 27),
	SND(FM_DIGITAL_BT_A2DP_HEADSET, 28),
	SND(STEREO_HEADSET_AND_SPEAKER, 31),
	SND(CURRENT, 0x7FFFFFFE),
	SND(FM_ANALOG_STEREO_HEADSET, 35),
	SND(FM_ANALOG_STEREO_HEADSET_CODEC, 36),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};


#ifdef CONFIG_BATTERY_MSM
static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design     = 2800,
	.voltage_max_design     = 4300,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage	 = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};
#endif


#ifdef CONFIG_SMSC911X
static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= 0x90000000,
		.end	= 0x90007fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(48),
		.end	= MSM_GPIO_TO_INT(48),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "smsc911x_irq"  },
	{ GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "eth_fifo_sel" },
};

#define ETH_FIFO_SEL_GPIO	49
static void msm7x27a_cfg_smsc911x(void)
{
	int res;

	res = msm_gpios_request_enable(smsc911x_gpios,
				 ARRAY_SIZE(smsc911x_gpios));
	if (res) {
		pr_err("%s: unable to enable gpios for SMSC911x\n", __func__);
		return;
	}

	/* ETH_FIFO_SEL */
	res = gpio_direction_output(ETH_FIFO_SEL_GPIO, 0);
	if (res) {
		pr_err("%s: unable to get direction for gpio %d\n", __func__,
							 ETH_FIFO_SEL_GPIO);
		msm_gpios_disable_free(smsc911x_gpios,
						 ARRAY_SIZE(smsc911x_gpios));
		return;
	}
	gpio_set_value(ETH_FIFO_SEL_GPIO, 0);
}
#endif




#if 0
static struct regulator_bulk_data regs_camera[] = {
	{ .supply = "msme1", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "gp2",   .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "usb2",  .min_uV = 1800000, .max_uV = 1800000 },
};

static void __init msm_camera_vreg_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_camera), regs_camera);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		return;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_camera), regs_camera);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		return;
	}
}

static struct regulator *  vreg_cam_dvdd;
static struct regulator *  vreg_cam_mipi;
#define CAM_LDO_EN 8
static void msm_camera_vreg_config(int vreg_en)
{
	int rc = 0;

       printk("%s++, vreg_en is %d\n", __func__, vreg_en);
	if( vreg_cam_dvdd == NULL)
	{
           vreg_cam_dvdd = regulator_get(NULL, "rfrx1");
           if(rc)
           {
                pr_err("%s: vreg_get(%s) failed (%ld)\n",__func__, "rfrx1", PTR_ERR(vreg_cam_dvdd));
                return;
            }
	 }

	if( vreg_cam_mipi == NULL)
	{
           vreg_cam_mipi = regulator_get(NULL, "usb2");
           if(rc)
           {
                pr_err("%s: vreg_get(%s) failed (%ld)\n",__func__, "usb2", PTR_ERR(vreg_cam_mipi));
                return;
            }
	 }

        if(vreg_en)
        {
		PM_LOG_EVENT(PM_LOG_ON, PM_LOG_CAMERA);
	  	rc = regulator_set_voltage(vreg_cam_dvdd, 1800000, 1800000);
	  	if (rc) {
				pr_err("%s: vreg_cam_dvdd  set_level failed (%d)\n",
					__func__, rc);
				return;
	   	}

	  	rc = regulator_set_voltage(vreg_cam_mipi, 1800000, 1800000);
	  	if (rc) {
				pr_err("%s: vreg_cam_mipi set_level failed (%d)\n",
					__func__, rc);
				return;
	   	}

	  	rc = regulator_enable(vreg_cam_dvdd);
		if (rc) {
			pr_err("%s: vreg_cam_dvdd enable failed (%d)\n",
				__func__, rc);
			return;
		}

		rc = gpio_request(CAM_LDO_EN, "CAM_LDO_EN");
              if(rc <0)
              {
                   printk("%s, get GPIO CAM_LDO_EN failed\n",__func__);
               }
               gpio_direction_output(CAM_LDO_EN, 1);


	  	rc = regulator_enable(vreg_cam_mipi);
		if (rc) {
			pr_err("%s: vreg_cam_mipi enable failed (%d)\n",
				__func__, rc);
			return;
		}

          } else
          {

              
	  	rc = regulator_disable(vreg_cam_dvdd);
		if (rc) {
			pr_err("%s: vreg_cam_dvdd disable failed (%d)\n",
				__func__, rc);
			return;
		}
              vreg_cam_dvdd = NULL;

              
		gpio_direction_output(CAM_LDO_EN, 0);
              gpio_free(CAM_LDO_EN);


              
	  	rc = regulator_disable(vreg_cam_mipi);
		if (rc) {
			pr_err("%s: vreg_cam_mipi disable failed (%d)\n",
				__func__, rc);
			return;
		}
		vreg_cam_mipi = NULL;

		PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_CAMERA);
          }

	    printk("%s--, vreg_en is %d\n", __func__, vreg_en);

	}




static int config_gpio_table(uint32_t *table, int len)
{
	int rc = 0, i = 0;

	for (i = 0; i < len; i++) {
		rc = gpio_tlmm_config(table[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s not able to get gpio\n", __func__);
			for (i--; i >= 0; i--)
				gpio_tlmm_config(camera_off_gpio_table[i],
							GPIO_CFG_ENABLE);
			break;
		}
	}
	return rc;
}

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data;
static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data;
static int config_camera_on_gpios_rear(void)
{
	int rc = 0;

	msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_rear(void)
{

	msm_camera_vreg_config(0);

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_on_gpios_front(void)
{
	int rc = 0;

	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
		msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_front(void)
{
	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
		msm_camera_vreg_config(0);

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

struct msm_camera_device_platform_data msm_camera_device_data_rear = {
	.camera_gpio_on  = config_camera_on_gpios_rear,
	.camera_gpio_off = config_camera_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

struct msm_camera_device_platform_data msm_camera_device_data_front = {
	.camera_gpio_on  = config_camera_on_gpios_front,
	.camera_gpio_off = config_camera_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_S5K4E1
static struct msm_camera_sensor_platform_info s5k4e1_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_s5k4e1 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1_data = {
	.sensor_name    = "s5k4e1",
	.sensor_reset_enable = 1,
	.sensor_reset   = GPIO_CAM_GP_CAMIF_RESET_N,
	.sensor_pwd             = 85,
	.vcm_pwd                = GPIO_CAM_GP_CAM_PWDN,
	.vcm_enable             = 1,
	.pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_s5k4e1,
	.sensor_platform_info   = &s5k4e1_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_s5k4e1 = {
	.name   = "msm_camera_s5k4e1",
	.dev    = {
		.platform_data = &msm_camera_sensor_s5k4e1_data,
	},
};
#endif

#ifdef CONFIG_IMX072
static struct msm_camera_sensor_platform_info imx072_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_imx072 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_imx072_data = {
	.sensor_name    = "imx072",
	.sensor_reset_enable = 1,
	.sensor_reset   = GPIO_CAM_GP_CAMIF_RESET_N, 
	.sensor_pwd             = 85,
	.vcm_pwd                = GPIO_CAM_GP_CAM_PWDN,
	.vcm_enable             = 1,
	.pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_imx072,
	.sensor_platform_info = &imx072_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_imx072 = {
	.name   = "msm_camera_imx072",
	.dev    = {
		.platform_data = &msm_camera_sensor_imx072_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_OV9726
static struct msm_camera_sensor_platform_info ov9726_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov9726 = {
	.flash_type             = MSM_CAMERA_FLASH_NONE,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data = {
	.sensor_name    = "ov9726",
	.sensor_reset_enable = 0,
	.sensor_reset   = GPIO_CAM_GP_CAM1MP_XCLR,
	.sensor_pwd             = 85,
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_device_data_front,
	.flash_data             = &flash_ov9726,
	.sensor_platform_info   = &ov9726_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov9726 = {
	.name   = "msm_camera_ov9726",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov9726_data,
	},
};
#else
static inline void msm_camera_vreg_init(void) { }
#endif

#ifdef CONFIG_MT9E013
static struct msm_camera_sensor_platform_info mt9e013_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 0,
	.sensor_reset_enable = 1,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_mt9e013,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif





#if (defined(CONFIG_MT9P017)||defined(CONFIG_Q_MT9P017))
static struct msm_camera_sensor_platform_info mt9p017_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_src msm_flash_mt9p017_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
};
static struct msm_camera_sensor_flash_data flash_mt9p017 = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_mt9p017_src

};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p017_data = {
	.sensor_name    = "mt9p017",
	.sensor_reset_enable = 1,
	.sensor_reset   = 37, 
	.sensor_pwd             = 85,
	.vcm_pwd                = 1, 
	.vcm_enable             = 0,
       .pdata                  = &msm_camera_device_data_rear,
	.flash_data             = &flash_mt9p017,
	.sensor_platform_info = &mt9p017_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_mt9p017 = {
	.name   = "msm_camera_mt9p017",
	.dev    = {
		.platform_data = &msm_camera_sensor_mt9p017_data,
	},
};

static struct i2c_board_info flashlight_adp1650_board_info[] = {
	{
		
		I2C_BOARD_INFO("CAM_FLASH_adp1650", 0x30),
		

	},
};

#endif

static struct i2c_board_info i2c_camera_devices[] = {
	#ifdef CONFIG_S5K4E1
	{

	},
	{
		I2C_BOARD_INFO("s5k4e1_af", 0x8c >> 1),
	},

	#endif
	#ifdef CONFIG_WEBCAM_OV9726
	{
		I2C_BOARD_INFO("ov9726", 0x10),
	},
	#endif
	#ifdef CONFIG_IMX072
	{
		I2C_BOARD_INFO("imx072", 0x34),
	},
	#endif
	#ifdef CONFIG_MT9E013
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
	#endif
	#ifdef CONFIG_MT9P017

	{
		
		I2C_BOARD_INFO("mt9p017", 0x6C >>1),
		
	},
	#endif
	{
		I2C_BOARD_INFO("sc628a", 0x6E),
	},
};
#endif



static struct regulator *  vreg_isl_dvdd;
static int __msm_isl29028_vreg_config(int vreg_en)
{
	int rc = 0;
	if (vreg_isl_dvdd == NULL) {
		vreg_isl_dvdd = regulator_get(NULL, "gp2");
		if (IS_ERR(vreg_isl_dvdd)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp2", PTR_ERR(vreg_isl_dvdd));
			return rc;
		}
		rc = regulator_set_voltage(vreg_isl_dvdd, 2850000,2850000);
		if (rc) {
			pr_err("%s: GP2 set_level failed (%d)\n",
				__func__, rc);
			return rc;
		}
	}
	if (vreg_en) {
		rc = regulator_enable(vreg_isl_dvdd);
		if (rc) {
			pr_err("%s: GP2 enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
	} else {
		rc = regulator_disable(vreg_isl_dvdd);
		if (rc) {
			pr_err("%s: GP2 disable failed (%d)\n",
				__func__, rc);
		}
	}
	return rc;
}



#if 0
static struct msm_gpio uart2dm_gpios[] = {
#if 0 
	{GPIO_CFG(19, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rfr_n" },
	{GPIO_CFG(20, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_cts_n" },
#endif
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

static void msm7x27a_cfg_uart2dm_serial(void)
{
	int ret;
	ret = msm_gpios_request_enable(uart2dm_gpios,
					ARRAY_SIZE(uart2dm_gpios));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);
}
#else
static void msm7x27a_cfg_uart2dm_serial(void) { }
#endif

static struct platform_device *rumi_sim_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&smc91x_device,
	&msm_device_uart1,
	&msm_device_nand,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
};

static struct platform_device *surf_ffa_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1,
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	&msm_device_nand,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_fb_device,
#ifdef C7_HEADSET 
	&qsd_headset_device,
#endif
	&lcdc_toshiba_panel_device,

#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif

#ifdef CONFIG_SMSC911X
	&smsc911x_device,
#endif
#ifdef CONFIG_FB_MSM_MIPI_DSI
#ifdef CONFIG_FB_MSM_MIPI_DSI_CHIMEI
	&mipi_dsi_chimei_panel_device,
#else
	&mipi_dsi_renesas_panel_device,
#endif
#endif
	&msm_kgsl_3d0,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#ifdef CONFIG_ION_MSM
	&ion_dev,
#endif
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa())
		fb_size = MSM7x25A_MSM_FB_SIZE;
	else
		fb_size = MSM_FB_SIZE;

	size = fb_size;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

#ifdef CONFIG_MSM_V4L2_VIDEO_OVERLAY_DEVICE
	fb_size = MSM_V4L2_VIDEO_OVERLAY_BUF_SIZE;
	addr = alloc_bootmem_align(fb_size, 0x1000);
	msm_v4l2_video_overlay_resources[0].start = __pa(addr);
	msm_v4l2_video_overlay_resources[0].end =
	msm_v4l2_video_overlay_resources[0].start + fb_size - 1;
	pr_debug("allocating %lu bytes at %p (%lx physical) for v4l2\n",
	fb_size, addr, __pa(addr));
#endif
}

#ifdef CONFIG_ION_MSM
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct ion_co_heap_pdata co_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
};
#endif

/**
 * These heaps are listed in the order they will be allocated.
 * Don't swap the order unless you know what you are doing!
 */
static struct ion_platform_data ion_pdata = {
	.nr = MSM_ION_HEAP_NUM,
	.has_outer_cache = 1,
	.heaps = {
		{
			.id	= ION_SYSTEM_HEAP_ID,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= ION_VMALLOC_HEAP_NAME,
		},
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		/* PMEM_ADSP = CAMERA */
		{
			.id	= ION_CAMERA_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_CAMERA_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
		},
		/* PMEM_AUDIO */
		{
			.id	= ION_AUDIO_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
		},
		/* PMEM_MDP = SF */
		{
			.id	= ION_SF_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_SF_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
		},
#endif
	}
};

static struct platform_device ion_dev = {
	.name = "ion-msm",
	.id = 1,
	.dev = { .platform_data = &ion_pdata },
};
#endif

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};


static void __init size_pmem_devices(void)
{

	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		pmem_mdp_size = MSM7x25A_MSM_PMEM_MDP_SIZE;
		pmem_adsp_size = MSM7x25A_MSM_PMEM_ADSP_SIZE;
	} else {
		pmem_mdp_size = MSM_PMEM_MDP_SIZE;
		pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
	}

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
#endif

#ifdef CONFIG_ION_MSM
	msm_ion_camera_size = pmem_adsp_size;
	msm_ion_audio_size = (MSM_PMEM_AUDIO_SIZE + PMEM_KERNEL_EBI1_SIZE);
	msm_ion_sf_size = pmem_mdp_size;
#endif
}

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27a_reserve_table[p->memory_type].size += p->size;
}
#endif
#endif

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
#endif
}

static void __init size_ion_devices(void)
{
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	ion_pdata.heaps[1].size = msm_ion_camera_size;
	ion_pdata.heaps[2].size = msm_ion_audio_size;
	ion_pdata.heaps[3].size = msm_ion_sf_size;
#endif
}

static void __init reserve_ion_memory(void)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += msm_ion_camera_size;
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += msm_ion_audio_size;
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += msm_ion_sf_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
	size_ion_devices();
	reserve_ion_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

static void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	msm_reserve();
}

static void __init msm_device_i2c_init(void)
{
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;
}

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
	.mdp_rev = MDP_REV_303,
};


#ifdef CONFIG_FB_MSM
#define GPIO_LCDC_BRDG_PD	128
#define GPIO_LCDC_BRDG_RESET_N	129

#define LCDC_RESET_PHYS		0x90008014

#ifndef CONFIG_FB_MSM_MIPI_DSI_CHIMEI
static	void __iomem *lcdc_reset_ptr;

static unsigned mipi_dsi_gpio[] = {
	GPIO_CFG(GPIO_LCDC_BRDG_RESET_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* LCDC_BRDG_RESET_N */
	GPIO_CFG(GPIO_LCDC_BRDG_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* LCDC_BRDG_RESET_N */
};
#endif

enum {
	DSI_SINGLE_LANE = 1,
	DSI_TWO_LANES,
};

static int msm_fb_get_lane_config(void)
{
	int rc = DSI_TWO_LANES;

	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		rc = DSI_SINGLE_LANE;
		pr_info("DSI Single Lane\n");
	} else {
		pr_info("DSI Two Lanes\n");
	}
	return rc;
}

#ifndef CONFIG_FB_MSM_MIPI_DSI_CHIMEI
static int msm_fb_dsi_client_reset(void)
{
	int rc = 0;

	rc = gpio_request(GPIO_LCDC_BRDG_RESET_N, "lcdc_brdg_reset_n");
	if (rc < 0) {
		pr_err("failed to request lcd brdg reset_n\n");
		return rc;
	}

	rc = gpio_request(GPIO_LCDC_BRDG_PD, "lcdc_brdg_pd");
	if (rc < 0) {
		pr_err("failed to request lcd brdg pd\n");
		return rc;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		goto gpio_error;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[1], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge pd enable\n");
		goto gpio_error2;
	}

	rc = gpio_direction_output(GPIO_LCDC_BRDG_RESET_N, 1);
	rc |= gpio_direction_output(GPIO_LCDC_BRDG_PD, 1);
	gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);

	if (!rc) {
		if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf()) {
			lcdc_reset_ptr = ioremap_nocache(LCDC_RESET_PHYS,
				sizeof(uint32_t));

			if (!lcdc_reset_ptr)
				return 0;
		}
		return rc;
	} else {
		goto gpio_error;
	}

gpio_error2:
	pr_err("Failed GPIO bridge pd\n");
	gpio_free(GPIO_LCDC_BRDG_PD);

gpio_error:
	pr_err("Failed GPIO bridge reset\n");
	gpio_free(GPIO_LCDC_BRDG_RESET_N);
	return rc;
}
#endif 

static struct regulator_bulk_data regs_dsi[] = {
	{ .supply = "gp2",   .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "msme1", .min_uV = 1800000, .max_uV = 1800000 },
};

static int dsi_gpio_initialized;

#ifdef CONFIG_FB_MSM_MIPI_DSI_CHIMEI
#define GPIO_LCD_RESET	49
static int mipi_dsi_panel_power(int on)
{
	int rc = 0;

	if (unlikely(!dsi_gpio_initialized)) {
#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A
		
		
		
#else
		pmapp_disp_backlight_init();
#endif

		rc = gpio_tlmm_config(GPIO_CFG(GPIO_LCD_RESET, 0,
					GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("Failed to enable LCD reset enable\n");
			dsi_gpio_initialized=0;
			return rc;
		}

		rc = gpio_request(GPIO_LCD_RESET, "lcd_reset");
		if (rc < 0) {
			pr_err("failed to request lcd reset_n\n");
			dsi_gpio_initialized=0;
			return rc;
		}

		rc = gpio_direction_output(GPIO_LCD_RESET, 1);
		if (rc < 0) {
			pr_err("failed to set direction for lcd_reset\n");
			goto fail_gpio1;
		}

		rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_dsi), regs_dsi);
		if (rc) {
			pr_err("%s: could not get regulators: %d\n",
					__func__, rc);
			goto fail_gpio1;
		}

		rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_dsi), regs_dsi);
		if (rc) {
			pr_err("%s: could not set voltages: %d\n",
					__func__, rc);
			goto fail_vreg;
		}

		dsi_gpio_initialized = 1;
	}

	if (on)
	{
		rc = regulator_bulk_enable(ARRAY_SIZE(regs_dsi), regs_dsi);
		if (rc)
			pr_err("%s: could not enable regulators: %d\n",
					__func__, rc);

		PM_LOG_EVENT(PM_LOG_ON, PM_LOG_LCD);

		gpio_set_value_cansleep(GPIO_LCD_RESET, 0);
		mdelay(1);
		gpio_set_value_cansleep(GPIO_LCD_RESET, 1);
		mdelay(10);
	}
	else
	{
		gpio_set_value_cansleep(GPIO_LCD_RESET, 0);
		msleep(120);

		rc = regulator_bulk_disable(ARRAY_SIZE(regs_dsi), regs_dsi);
		if (rc)
			pr_err("%s: could not disable regulators: %d\n",
					__func__, rc);

		PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_LCD);
	}

	return rc;


fail_vreg:
	regulator_bulk_free(ARRAY_SIZE(regs_dsi), regs_dsi);

fail_gpio1:
	gpio_free(GPIO_LCD_RESET);
	dsi_gpio_initialized = 0;
	return rc;
}
#else
static int mipi_dsi_panel_power(int on)
{
	int rc = 0;
	uint32_t lcdc_reset_cfg;

	/* I2C-controlled GPIO Expander -init of the GPIOs very late */
	if (unlikely(!dsi_gpio_initialized)) {
		pmapp_disp_backlight_init();

		rc = gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr");
		if (rc < 0) {
			pr_err("failed to request gpio_disp_pwr\n");
			return rc;
		}

		if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf()) {
			rc = gpio_direction_output(GPIO_DISPLAY_PWR_EN, 1);
			if (rc < 0) {
				pr_err("failed to enable display pwr\n");
				goto fail_gpio1;
			}

			rc = gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en");
			if (rc < 0) {
				pr_err("failed to request gpio_bkl_en\n");
				goto fail_gpio1;
			}

			rc = gpio_direction_output(GPIO_BACKLIGHT_EN, 1);
			if (rc < 0) {
				pr_err("failed to enable backlight\n");
				goto fail_gpio2;
			}
		}

		rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_dsi), regs_dsi);
		if (rc) {
			pr_err("%s: could not get regulators: %d\n",
					__func__, rc);
			goto fail_gpio2;
		}

		rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_dsi), regs_dsi);
		if (rc) {
			pr_err("%s: could not set voltages: %d\n",
					__func__, rc);
			goto fail_vreg;
		}
		if (pmapp_disp_backlight_set_brightness(100))
			pr_err("backlight set brightness failed\n");

		dsi_gpio_initialized = 1;
	}

	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf()) {
		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
	} else if (machine_is_msm7x27a_ffa() ||
			 machine_is_msm7625a_ffa()) {
		if (on) {
			/* This line drives an active low pin on FFA */
			rc = gpio_direction_output(GPIO_DISPLAY_PWR_EN, !on);
			if (rc < 0)
				pr_err("failed to set direction for "
					"display pwr\n");
		} else {
			gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, !on);
			rc = gpio_direction_input(GPIO_DISPLAY_PWR_EN);
			if (rc < 0)
				pr_err("failed to set direction for "
					"display pwr\n");
		}
	}

	if (on) {
		gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);

		if (machine_is_msm7x27a_surf() ||
				 machine_is_msm7625a_surf()) {
			lcdc_reset_cfg = readl_relaxed(lcdc_reset_ptr);
			rmb();
			lcdc_reset_cfg &= ~1;

			writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
			msleep(20);
			wmb();
			lcdc_reset_cfg |= 1;
			writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
		} else {
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N, 0);
			msleep(20);
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N, 1);
		}
	} else {
		gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 1);
	}

	rc = on ? regulator_bulk_enable(ARRAY_SIZE(regs_dsi), regs_dsi) :
		  regulator_bulk_disable(ARRAY_SIZE(regs_dsi), regs_dsi);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, on ? "en" : "dis", rc);

	return rc;

fail_vreg:
	regulator_bulk_free(ARRAY_SIZE(regs_dsi), regs_dsi);
fail_gpio2:
	gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio1:
	gpio_free(GPIO_DISPLAY_PWR_EN);
	dsi_gpio_initialized = 0;
	return rc;
}
#endif 
#endif

#define MDP_303_VSYNC_GPIO 97

#ifdef CONFIG_FB_MSM_MDP303
static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio = MDP_303_VSYNC_GPIO,
	.dsi_power_save   = mipi_dsi_panel_power,
#ifndef CONFIG_FB_MSM_MIPI_DSI_CHIMEI
	.dsi_client_reset = msm_fb_dsi_client_reset,
#endif
	.get_lane_config = msm_fb_get_lane_config,
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);
#ifdef CONFIG_FB_MSM_MDP303
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
#endif
}

#define MSM_EBI2_PHYS			0xa0d00000
#define MSM_EBI2_XMEM_CS2_CFG1		0xa0d10030

static void __init msm7x27a_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void __iomem *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_PHYS, sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_rumi3() || machine_is_msm7x27a_surf() ||
			machine_is_msm7625a_surf())
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
		ebi2_cfg |= (1 << 31);

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);
}

#if 0
#define ATMEL_TS_I2C_NAME "maXTouch"

static struct regulator_bulk_data regs_atmel[] = {
	{ .supply = "ldo2",  .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "smps3", .min_uV = 1800000, .max_uV = 1800000 },
};

#define ATMEL_TS_GPIO_IRQ 82

static int atmel_ts_power_on(bool on)
{
	int rc = on ?
		regulator_bulk_enable(ARRAY_SIZE(regs_atmel), regs_atmel) :
		regulator_bulk_disable(ARRAY_SIZE(regs_atmel), regs_atmel);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, on ? "en" : "dis", rc);
	else
		msleep(50);

	return rc;
}

static int atmel_ts_platform_init(struct i2c_client *client)
{
	int rc;
	struct device *dev = &client->dev;

	rc = regulator_bulk_get(dev, ARRAY_SIZE(regs_atmel), regs_atmel);
	if (rc) {
		dev_err(dev, "%s: could not get regulators: %d\n",
				__func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_atmel), regs_atmel);
	if (rc) {
		dev_err(dev, "%s: could not set voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	rc = gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (rc) {
		dev_err(dev, "%s: gpio_tlmm_config for %d failed\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto reg_free;
	}

	/* configure touchscreen interrupt gpio */
	rc = gpio_request(ATMEL_TS_GPIO_IRQ, "atmel_maxtouch_gpio");
	if (rc) {
		dev_err(dev, "%s: unable to request gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto ts_gpio_tlmm_unconfig;
	}

	rc = gpio_direction_input(ATMEL_TS_GPIO_IRQ);
	if (rc < 0) {
		dev_err(dev, "%s: unable to set the direction of gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto free_ts_gpio;
	}
	return 0;

free_ts_gpio:
	gpio_free(ATMEL_TS_GPIO_IRQ);
ts_gpio_tlmm_unconfig:
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs_atmel), regs_atmel);
out:
	return rc;
}

static int atmel_ts_platform_exit(struct i2c_client *client)
{
	gpio_free(ATMEL_TS_GPIO_IRQ);
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	regulator_bulk_free(ARRAY_SIZE(regs_atmel), regs_atmel);
	return 0;
}

static u8 atmel_ts_read_chg(void)
{
	return gpio_get_value(ATMEL_TS_GPIO_IRQ);
}

static u8 atmel_ts_valid_interrupt(void)
{
	return !atmel_ts_read_chg();
}

#define ATMEL_X_OFFSET 13
#define ATMEL_Y_OFFSET 0

static struct maxtouch_platform_data atmel_ts_pdata = {
	.numtouch = 4,
	.init_platform_hw = atmel_ts_platform_init,
	.exit_platform_hw = atmel_ts_platform_exit,
	.power_on = atmel_ts_power_on,
	.display_res_x = 480,
	.display_res_y = 864,
	.min_x = ATMEL_X_OFFSET,
	.max_x = (505 - ATMEL_X_OFFSET),
	.min_y = ATMEL_Y_OFFSET,
	.max_y = (863 - ATMEL_Y_OFFSET),
	.valid_interrupt = atmel_ts_valid_interrupt,
	.read_chg = atmel_ts_read_chg,
};

static struct i2c_board_info atmel_ts_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO(ATMEL_TS_I2C_NAME, 0x4a),
		.platform_data = &atmel_ts_pdata,
		.irq = MSM_GPIO_TO_INT(ATMEL_TS_GPIO_IRQ),
	},
};
#endif


#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TMA340
#define CYTTSP_TS_GPIO_IRQ		82
#define CYTTSP_TS_GPIO_RST		88
#define CYTTSP_TS_I2C_ADDR		0x4D

static struct cyttsp_platform_data cyttsp_pdata = {
	.rst_gpio = CYTTSP_TS_GPIO_RST,
	.irq_gpio = CYTTSP_TS_GPIO_IRQ,
};

static struct i2c_board_info cyttsp_info[] __initdata = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, CYTTSP_TS_I2C_ADDR),
		.platform_data = &cyttsp_pdata,
		.irq = MSM_GPIO_TO_INT(CYTTSP_TS_GPIO_IRQ),
	},
};
#endif


#ifdef CONFIG_TOUCHSCREEN_ATMEL_mXT224E
#define ATMEL_TS_GPIO_IRQ		82
#define ATMEL_TS_GPIO_RST		88
#define ATMEL_TS_I2C_ADDR		0x4A

static struct atmel_platform_data atmel_pdata = {
	.rst_gpio = ATMEL_TS_GPIO_RST,
	.irq_gpio = ATMEL_TS_GPIO_IRQ,
};

static struct i2c_board_info atmel_info[] __initdata = {
	{
		I2C_BOARD_INFO(ATMEL_TS_NAME, ATMEL_TS_I2C_ADDR),
		.platform_data = &atmel_pdata,
		.irq = MSM_GPIO_TO_INT(ATMEL_TS_GPIO_IRQ),
	},
};
#endif


#if 0
#define KP_INDEX(row, col) ((row)*ARRAY_SIZE(kp_col_gpios) + (col))

static unsigned int kp_row_gpios[] = {31, 32, 33, 34, 35};
static unsigned int kp_col_gpios[] = {36, 35, 38, 39, 40};

static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
	[KP_INDEX(0, 0)] = KEY_7,
	[KP_INDEX(0, 1)] = KEY_DOWN,
	[KP_INDEX(0, 2)] = KEY_UP,
	[KP_INDEX(0, 3)] = KEY_RIGHT,
	[KP_INDEX(0, 4)] = KEY_ENTER,

	[KP_INDEX(1, 0)] = KEY_LEFT,
	[KP_INDEX(1, 1)] = KEY_SEND,
	[KP_INDEX(1, 2)] = KEY_1,
	[KP_INDEX(1, 3)] = KEY_4,
	[KP_INDEX(1, 4)] = KEY_CLEAR,

	[KP_INDEX(2, 0)] = KEY_6,
	[KP_INDEX(2, 1)] = KEY_5,
	[KP_INDEX(2, 2)] = KEY_8,
	[KP_INDEX(2, 3)] = KEY_3,
	[KP_INDEX(2, 4)] = KEY_NUMERIC_STAR,

	[KP_INDEX(3, 0)] = KEY_9,
	[KP_INDEX(3, 1)] = KEY_NUMERIC_POUND,
	[KP_INDEX(3, 2)] = KEY_0,
	[KP_INDEX(3, 3)] = KEY_2,
	[KP_INDEX(3, 4)] = KEY_SLEEP,

	[KP_INDEX(4, 0)] = KEY_BACK,
	[KP_INDEX(4, 1)] = KEY_HOME,
	[KP_INDEX(4, 2)] = KEY_MENU,
	[KP_INDEX(4, 3)] = KEY_VOLUMEUP,
	[KP_INDEX(4, 4)] = KEY_VOLUMEDOWN,
};

/* SURF keypad platform device information */
static struct gpio_event_matrix_info kp_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= kp_row_gpios,
	.input_gpios	= kp_col_gpios,
	.noutputs	= ARRAY_SIZE(kp_row_gpios),
	.ninputs	= ARRAY_SIZE(kp_col_gpios),
	.settle_time.tv_nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv_nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS,
};

static struct gpio_event_info *kp_info[] = {
	&kp_matrix_info.info
};

static struct gpio_event_platform_data kp_pdata = {
	.name		= "7x27a_kp",
	.info		= kp_info,
	.info_count	= ARRAY_SIZE(kp_info)
};

static struct platform_device kp_pdev = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &kp_pdata,
	},
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data = &msm7x27a_proccomm_regulator_data
	}
};

static void __init msm7627a_rumi3_init(void)
{
	msm7x27a_init_ebi2();
	platform_add_devices(rumi_sim_devices,
			ARRAY_SIZE(rumi_sim_devices));
}

#define LED_GPIO_PDM		96
#define UART1DM_RX_GPIO		45

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static int __init msm7x27a_init_ar6000pm(void)
{
	msm_wlan_ar6000_pm_device.dev.platform_data = &ar600x_wlan_power;
	return platform_device_register(&msm_wlan_ar6000_pm_device);
}
#else
static int __init msm7x27a_init_ar6000pm(void) { return 0; }
#endif

#ifdef CONFIG_WLAN_ALLOC_STATIC_MEM
#define ADDR_AR6003_FW_LEN			SZ_128K
#define ADDR_AR6003_SDIO_DMA_BUFFER	SZ_32K
#define ADDR_AR6003_BDATA_LEN		SZ_4K
#define ADDR_AR6003_OTP_LEN			SZ_4K
#define ADDR_AR6003_PATCH_LEN		SZ_1K


void *addr_ar6003_bdata = NULL;
void *addr_ar6003_otp = NULL;
void *addr_ar6003_fw = NULL;
void *addr_ar6003_patch = NULL;
void *addr_ar6003_sdio_dma = NULL;
EXPORT_SYMBOL(addr_ar6003_bdata);
EXPORT_SYMBOL(addr_ar6003_otp);
EXPORT_SYMBOL(addr_ar6003_fw);
EXPORT_SYMBOL(addr_ar6003_patch);
EXPORT_SYMBOL(addr_ar6003_sdio_dma);

static void __init msm7x27a_init_ar6000_static_memory(void)
{
	printk("%s\n", __func__);
    addr_ar6003_bdata  = kzalloc(ADDR_AR6003_BDATA_LEN, GFP_NOWAIT);
	if (!addr_ar6003_bdata) {
		pr_err("%s: alloc ar6003_bdata fail!\n", __func__);
	}
    addr_ar6003_otp = kzalloc(ADDR_AR6003_OTP_LEN, GFP_NOWAIT);
	if (!addr_ar6003_otp) {
		pr_err("%s: alloc ar6003_otp fail!\n", __func__);
	}
    addr_ar6003_fw = kzalloc(ADDR_AR6003_FW_LEN, GFP_NOWAIT);
	if (!addr_ar6003_fw) {
		pr_err("%s: alloc ar6003_fw fail!\n", __func__);
	}
    addr_ar6003_patch = kzalloc(ADDR_AR6003_PATCH_LEN, GFP_NOWAIT);
	if (!addr_ar6003_patch) {
		pr_err("%s: alloc ar6003_patch fail!\n", __func__);
	}
	addr_ar6003_sdio_dma = kzalloc(ADDR_AR6003_SDIO_DMA_BUFFER, GFP_NOWAIT);
	if (!addr_ar6003_sdio_dma) {
		pr_err("%s: addr_ar6003_sdio_dma fail!\n", __func__);
	}
    printk(KERN_CRIT "bdata=0x%p\notp=0x%p\nfw=0x%p\npatch=0x%p\nsdio_dma=0x%p\n",
            addr_ar6003_bdata,
			addr_ar6003_otp,
			addr_ar6003_fw,
			addr_ar6003_patch,
			addr_ar6003_sdio_dma);
}
#endif

static void __init msm7x27a_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
		pr_err("%s: could not register regulator device: %d\n",
				__func__, rc);
}


#ifdef CONFIG_SMB136_CHARGER
static struct regulator *chg_regulator;

static unsigned smb136_chg_gpio_data[] = {                  		            
	GPIO_CFG(SMB136_CHG_STAT_INT,	0,	GPIO_CFG_INPUT,		GPIO_CFG_PULL_DOWN,		GPIO_CFG_2MA),  
	GPIO_CFG(SMB136_CHG_EN,			0,	GPIO_CFG_OUTPUT,	GPIO_CFG_PULL_UP,		GPIO_CFG_2MA),  
	GPIO_CFG(OVP_EXT_EN,			0,	GPIO_CFG_OUTPUT,	GPIO_CFG_PULL_DOWN,		GPIO_CFG_2MA),  
	GPIO_CFG(OVP_FLAG_INT,			0,	GPIO_CFG_INPUT,		GPIO_CFG_PULL_UP,		GPIO_CFG_2MA),  
};

static int smb136_chg_platform_init(void)
{
	int rc, pin;

	chg_regulator = regulator_get(NULL, "smps3");
	if (IS_ERR(chg_regulator)) {
		pr_err("%s: regulator get for smps3 failed\n", __func__);
		return PTR_ERR(chg_regulator);
	}

	rc = regulator_set_voltage(chg_regulator, 1800000, 1800000);
	if (rc) {
		pr_err("%s: regulator set level failed (%d) for smps3\n", __func__, rc);
		goto regulator_put_s3;
	}

	rc = regulator_enable(chg_regulator);
	if (rc < 0) {
		pr_err("%s: regulator enable failed (%d)\n", __func__, rc);
		goto regulator_put_s3;
	}

	for (pin = 0; pin < ARRAY_SIZE(smb136_chg_gpio_data); pin++) {
		rc = gpio_tlmm_config(smb136_chg_gpio_data[pin], GPIO_CFG_ENABLE);
		if (rc < 0) {
			pr_err("gpio_tlmm_config(0x%08x, GPIO_CFG_ENABLE) failed: rc(%d)\n",
				smb136_chg_gpio_data[pin], rc);
			pr_err("pin %d func %d dir %d pull %d drvstr %d\n",
				GPIO_PIN(smb136_chg_gpio_data[pin]), GPIO_FUNC(smb136_chg_gpio_data[pin]),
				GPIO_DIR(smb136_chg_gpio_data[pin]), GPIO_PULL(smb136_chg_gpio_data[pin]),
				GPIO_DRVSTR(smb136_chg_gpio_data[pin]));
			goto gpio_tlmm_unconfig;
		}
	}

	return 0;

gpio_tlmm_unconfig:
	for (pin = 0; pin < ARRAY_SIZE(smb136_chg_gpio_data); pin++) {
		gpio_tlmm_config(smb136_chg_gpio_data[pin], GPIO_CFG_DISABLE);
	}

regulator_put_s3:
	regulator_put(chg_regulator);

	return rc;
}

static struct smb136_platform_data smb136_chg_data __initdata = {
	.chg_stat_gpio = SMB136_CHG_STAT_INT,
	.chg_en_gpio = SMB136_CHG_EN,
	.ext_ovp_en_gpio = OVP_EXT_EN,
	.ovp_flag_n_gpio = OVP_FLAG_INT,
	.batt_mah_rating = 1300,
};

static struct i2c_board_info smb136_chg_board_info[] __initdata = {
	{
		I2C_BOARD_INFO(SMB136_DRIVER_NAME, SMB136_CHARGER_ADDR),
	
		.platform_data = &smb136_chg_data,
	},
};

static void __init msm7x27a_init_charger(void)
{
	int rc;

	rc = smb136_chg_platform_init();
	if (rc) {
		pr_err("%s: initialize regulator or gpio failed (%d)\n", __func__, rc);
		return;
	}

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		smb136_chg_board_info,
		ARRAY_SIZE(smb136_chg_board_info));
}
#endif

#ifdef CONFIG_BATTERY_BQ27520
static unsigned bq27520_batt_gpio_data[] = {                  		            

	GPIO_CFG(BQ27520_BATT_LOW,	0,	GPIO_CFG_INPUT,		GPIO_CFG_PULL_UP,		GPIO_CFG_2MA),  

	GPIO_CFG(BQ27520_GAUGE_INT,	0,	GPIO_CFG_INPUT,		GPIO_CFG_PULL_UP,		GPIO_CFG_2MA),  
};

static int bq27520_batt_platform_init(void)
{
	int rc, pin;

	for (pin = 0; pin < ARRAY_SIZE(bq27520_batt_gpio_data); pin++) {
		rc = gpio_tlmm_config(bq27520_batt_gpio_data[pin], GPIO_CFG_ENABLE);
		if (rc < 0) {
			pr_err("gpio_tlmm_config(0x%08x, GPIO_CFG_ENABLE) failed: rc(%d)\n",
				bq27520_batt_gpio_data[pin], rc);
			pr_err("pin %d func %d dir %d pull %d drvstr %d\n",
				GPIO_PIN(bq27520_batt_gpio_data[pin]), GPIO_FUNC(bq27520_batt_gpio_data[pin]),
				GPIO_DIR(bq27520_batt_gpio_data[pin]), GPIO_PULL(bq27520_batt_gpio_data[pin]),
				GPIO_DRVSTR(bq27520_batt_gpio_data[pin]));
			goto gpio_tlmm_unconfig;
		}
	}

	return 0;

gpio_tlmm_unconfig:
	for (pin = 0; pin < ARRAY_SIZE(bq27520_batt_gpio_data); pin++) {
		gpio_tlmm_config(bq27520_batt_gpio_data[pin], GPIO_CFG_DISABLE);
	}

	return rc;
}

static struct bq27520_platform_data bq27520_pdata = {
	.name		= "bq27520-fuel-gauge",
	.vreg_name	= "smps3",
	.vreg_value	= 1800000,
	.soc_int	= BQ27520_GAUGE_INT,

	.bat_low	= BQ27520_BATT_LOW,

	.bi_tout	= 0, 
	.chip_en	= 0, 
	.enable_dlog	= 0, 

#ifdef CONFIG_BUILD_SHIP
	.enable_it	= 1, 
#else
	.enable_it	= 0, 
#endif

};

static struct i2c_board_info bq27520_board_info[] = {
	{
		I2C_BOARD_INFO(BQ27520_DRIVER_NAME, 0xaa>>1),
		.platform_data = &bq27520_pdata,
	},
};

static void __init msm7x27a_init_fuel_gauge(void)
{
	int rc;

	rc = bq27520_batt_platform_init();
	if (rc) {
		pr_err("%s: initialize regulator or gpio failed (%d)\n", __func__, rc);
		return;
	}

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		bq27520_board_info,
		ARRAY_SIZE(bq27520_board_info));
}
#endif

#ifdef CONFIG_CHARGER_MSM7x27A
static struct msm_charger_platform_data msm_charger_data = {
	.safety_time = 180,
	.update_time = 1,
	.max_voltage = 4220,
	.min_voltage = 3200,
	.get_batt_capacity_percent = NULL,
};

static struct platform_device msm_charger_device = {
	.name = MSM_CHG_DRIVER_NAME,
	.id = -1,
	.dev = {
		.platform_data = &msm_charger_data,
	}
};
#endif



#ifdef CONFIG_PN544_NFC
#define NFC_HOST_INT_GPIO               "NFC-intr"
#define NFC_ENABLE_GPIO                 "NFC-enable"
#define NFC_FW_RESET_GPIO               "NFC-reset"



static unsigned pn544_nfc_gpio_data[] = {															
	GPIO_CFG(PN544_FW_RESET_GPIO,	0,	GPIO_CFG_OUTPUT,	GPIO_CFG_PULL_DOWN,		GPIO_CFG_2MA),  
	GPIO_CFG(PN544_HOST_INT_GPIO,	0,	GPIO_CFG_INPUT,		GPIO_CFG_PULL_DOWN,		GPIO_CFG_2MA),  
	GPIO_CFG(PN544_ENABLE_GPIO,		0,	GPIO_CFG_OUTPUT, 	GPIO_CFG_PULL_DOWN, 	GPIO_CFG_2MA),	
};

static int pn544_nfc_request_resources(struct i2c_client *client)
{
    int ret;

    ret = gpio_request(PN544_HOST_INT_GPIO, NFC_HOST_INT_GPIO);
    if (ret)
    {
        dev_err(&client->dev, "Request NFC INT GPIO fails %d\n", ret);
        return -1;
    }

    ret = gpio_direction_input(PN544_HOST_INT_GPIO);
    if (ret)
    {
        dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
        goto err_int;
    }

    ret = gpio_request(PN544_ENABLE_GPIO, NFC_ENABLE_GPIO);
    if (ret)
    {
        dev_err(&client->dev,
                "Request for NFC Enable GPIO fails %d\n", ret);
        goto err_int;
    }


	if (system_rev <= CHICAGO_EVT1)
	    ret = gpio_direction_output(PN544_ENABLE_GPIO, 1);
	else
		ret = gpio_direction_output(PN544_ENABLE_GPIO, 0);

    if (ret)
    {
        dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
        goto err_enable;
    }

    ret = gpio_request(PN544_FW_RESET_GPIO, NFC_FW_RESET_GPIO);
    if (ret)
    {
        dev_err(&client->dev,
                "Request for NFC FW Reset GPIO fails %d\n", ret);
        goto err_enable;
    }
    ret = gpio_direction_output(PN544_FW_RESET_GPIO, 0);
    if (ret)
    {
        dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
        goto err_fw;
    }

    return 0;

err_fw:
    gpio_free(PN544_FW_RESET_GPIO);
err_enable:
    gpio_free(PN544_ENABLE_GPIO);
err_int:
    gpio_free(PN544_HOST_INT_GPIO);
    return -1;
}

static void pn544_nfc_free_resources(void)
{
    gpio_free(PN544_HOST_INT_GPIO);
    gpio_free(PN544_ENABLE_GPIO);
    gpio_free(PN544_FW_RESET_GPIO);
}

static void pn544_nfc_enable(int fw)
{
    gpio_set_value(PN544_FW_RESET_GPIO, fw ? 1 : 0);
    msleep(PN544_GPIO4VEN_TIME);

	if (system_rev <= CHICAGO_EVT1)
    	gpio_set_value(PN544_ENABLE_GPIO, 1);
	else
		gpio_set_value(PN544_ENABLE_GPIO, 0);

}

static void pn544_nfc_disable(void)
{

	if (system_rev <= CHICAGO_EVT1)
    	gpio_set_value(PN544_ENABLE_GPIO, 0);
	else
    	gpio_set_value(PN544_ENABLE_GPIO, 1);

}

static int pn544_nfc_test(void)
{
    
     return 1;
}

static struct pn544_nfc_platform_data pn544_nfc_data =
{
    .request_resources = pn544_nfc_request_resources,
    .free_resources = pn544_nfc_free_resources,
    .enable = pn544_nfc_enable,
    .test = pn544_nfc_test,
    .disable = pn544_nfc_disable,
    .irq_gpio = PN544_HOST_INT_GPIO,
};

static struct i2c_board_info pn544_nfc_board_info[] __initdata =
{
    {
        I2C_BOARD_INFO(PN544_DRIVER_NAME, 0x28),
        .platform_data = &pn544_nfc_data,
        .irq = MSM_GPIO_TO_INT(PN544_HOST_INT_GPIO),
    },
};

static int pn544_nfc_platform_init(void)
{
	int rc, pin;

	for (pin = 0; pin < ARRAY_SIZE(pn544_nfc_gpio_data); pin++) {
		rc = gpio_tlmm_config(pn544_nfc_gpio_data[pin], GPIO_CFG_ENABLE);
		if (rc < 0) {
			pr_err("gpio_tlmm_config(0x%08x, GPIO_CFG_ENABLE) failed: rc(%d)\n",
				pn544_nfc_gpio_data[pin], rc);
			pr_err("pin %d func %d dir %d pull %d drvstr %d\n",
				GPIO_PIN(pn544_nfc_gpio_data[pin]), GPIO_FUNC(pn544_nfc_gpio_data[pin]),
				GPIO_DIR(pn544_nfc_gpio_data[pin]), GPIO_PULL(pn544_nfc_gpio_data[pin]),
				GPIO_DRVSTR(pn544_nfc_gpio_data[pin]));
			goto gpio_tlmm_unconfig;
		}
	}

	return 0;

gpio_tlmm_unconfig:
	for (pin = 0; pin < ARRAY_SIZE(pn544_nfc_gpio_data); pin++) {
		gpio_tlmm_config(pn544_nfc_gpio_data[pin], GPIO_CFG_DISABLE);
	}

	return rc;
}

static void __init msm7x27a_init_nfc(void)
{
	int rc;

	rc = pn544_nfc_platform_init();
	if (rc) {
		pr_err("%s: initialize regulator or gpio failed (%d)\n", __func__, rc);
		return;
	}

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		pn544_nfc_board_info,
		ARRAY_SIZE(pn544_nfc_board_info));
}
#endif



static unsigned gpio_initialize_configs[] = {
	
	GPIO_CFG(86, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 
};

static void chicago_gpio_tlmm_init(void)
{
	int rc = 0, i = 0;

	for (i = 0; i < ARRAY_SIZE(gpio_initialize_configs); i++) {
		rc = gpio_tlmm_config(gpio_initialize_configs[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s can setup gpio: %d", __func__, GPIO_PIN(gpio_initialize_configs[i]));
		}
	}
}


static void __init msm7x2x_init(void)
{
	int rc;
	msm7x2x_misc_init();

	/* Initialize regulators first so that other devices can use them */
	msm7x27a_init_regulators();

	/* Common functions for SURF/FFA/RUMI3 */
	msm_device_i2c_init();
	msm7x27a_init_ebi2();
	msm7x27a_cfg_uart2dm_serial();
#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_otg_pdata.swfi_latency =
		msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif
	msm_device_gadget_peripheral.dev.platform_data =
		&msm_gadget_pdata;
#ifdef CONFIG_SMSC911X
		msm7x27a_cfg_smsc911x();
#endif
	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);
	platform_add_devices(surf_ffa_devices,
			ARRAY_SIZE(surf_ffa_devices));
	/* Ensure ar6000pm device is registered before MMC/SDC */

	
	msm7x27a_init_ar6000pm();
	msm7x27a_init_ar6000_static_memory();
#ifdef CONFIG_MMC_MSM
	msm7627a_init_mmc();
#endif
	msm_fb_add_devices();
#ifdef CONFIG_USB_EHCI_MSM_72K
	msm7x2x_init_host();
#endif

	msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
	register_i2c_devices();
#endif
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	msm7627a_bt_power_init();
#endif
#if 0
	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		atmel_ts_pdata.min_x = 0;
		atmel_ts_pdata.max_x = 480;
		atmel_ts_pdata.min_y = 0;
		atmel_ts_pdata.max_y = 320;
	}
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		atmel_ts_i2c_info,
		ARRAY_SIZE(atmel_ts_i2c_info));
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TMA340
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		cyttsp_info,
		ARRAY_SIZE(cyttsp_info));
#endif


#ifdef CONFIG_TOUCHSCREEN_ATMEL_mXT224E
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		atmel_info,
		ARRAY_SIZE(atmel_info));
#endif


#ifdef CONFIG_SURF_FFA_GPIO_KEYPAD
	if (machine_is_msm7x27a_ffa())
		platform_device_register(&keypad_device_7k_ffa);
	else
		platform_device_register(&keypad_device_surf);
#endif

#if defined(CONFIG_MSM_CAMERA)
	msm7627a_camera_init();
#endif


	
	
	platform_device_register(&hs_pdev);

	
	#if 0
	/* configure it as a pdm function*/
	if (gpio_tlmm_config(GPIO_CFG(LED_GPIO_PDM, 3,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, LED_GPIO_PDM);
	else
		platform_device_register(&led_pdev);
	#endif
	

#ifdef CONFIG_MSM_RPC_VIBRATOR
	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
		msm_init_pmic_vibrator();
#endif
	/*7x25a kgsl initializations*/
	msm7x25a_kgsl_3d0_init();


#ifdef CONFIG_CHARGER_MSM7x27A
	platform_device_register(&msm_charger_device);
#endif



#if defined(CONFIG_I2C) && defined(CONFIG_BATTERY_BQ27520)
	msm7x27a_init_fuel_gauge();
#endif



#if defined(CONFIG_I2C) && defined(CONFIG_SMB136_CHARGER)
	msm7x27a_init_charger();
#endif



#if defined(CONFIG_I2C) && defined(CONFIG_PN544_NFC)
	msm7x27a_init_nfc();
#endif



#ifdef CONFIG_BOSCH_BMA250
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			bma250_board_info,
			ARRAY_SIZE(bma250_board_info));
#endif
#ifdef CONFIG_SENSORS_AK8975
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			akm8975_board_info,
			ARRAY_SIZE(akm8975_board_info));
#endif


 
#ifdef CONFIG_SENSORS_ISL29028
	rc = __msm_isl29028_vreg_config(1);
		if (rc < 0)
			printk(KERN_INFO "Power on ISL29028 fail!\n");

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			isl29028_board_info,
			ARRAY_SIZE(isl29028_board_info));
#endif 

#if (defined(CONFIG_SENSORS_AK8975) || defined(CONFIG_BOSCH_BMA250) || defined(CONFIG_SENSORS_ISL29028))
	sensors_config_gpio_table();
#endif
	
	chicago_gpio_tlmm_init();
	

}

static void __init msm7x2x_init_early(void)
{
	msm_msm7x2x_allocate_memory_regions();
}

MACHINE_START(MSM7X27A_RUMI3, "QCT MSM7x27a RUMI3")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7627a_rumi3_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_SURF, "QCT MSM7x27a SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_FFA, "QCT MSM7x27a FFA")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7625A_SURF, "QCT MSM7625a SURF")
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = msm_common_io_init,
	.reserve        = msm7x27a_reserve,
	.init_irq       = msm_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7625A_FFA, "QCT MSM7625a FFA")
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = msm_common_io_init,
	.reserve        = msm7x27a_reserve,
	.init_irq       = msm_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
