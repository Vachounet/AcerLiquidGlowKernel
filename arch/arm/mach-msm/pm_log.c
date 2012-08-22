#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/remote_spinlock.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/android_alarm.h>
#include <linux/ktime.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/syscalls.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <mach/msm_iomap.h>
#include <mach/pm_log.h>
#include "smd_private.h"
#include <oem/oem_smem_struct.h>


#define LOG_BUF_LEN   (1 << CONFIG_LOG_BUF_SHIFT)

#define MAX_POWER_OFF_SAVE_TIMES	20
#define POWEROFF_SAVED_PATH "/var/poweroff"
#define POWEROFF_NAME "poweroff.log"
#define POWEROFF_FILE POWEROFF_SAVED_PATH "/" POWEROFF_NAME


extern struct dentry *kernel_debuglevel_dir;

static unsigned int PMLOG_DLL = 0;

static smem_vendor_id1_apps_data *smem_vendor1_data = NULL;


static char pm_log_name[PM_LOG_NUM][20] = PM_LOG_NAME;

static struct timespec	pm_log_start[PM_LOG_NUM];

static unsigned int log_size = LOG_BUF_LEN;
static char log_buf[LOG_BUF_LEN];

static uint32_t poweroff_reason = 0x998877BB;
static char shutdown_cmd[80];


struct wake_lock wake_lock_pmlog;
static DEFINE_SPINLOCK(pm_lock);

typedef enum {
	ALIGN_LEFT = 0,
	ALIGN_CENTER,
	ALIGN_RIGHT,
} ALIGNMENT;

#define PM_LOG_DPRINTK(level, fmt, args...) \
	do { \
		if (level <= PMLOG_DLL) \
			printk(KERN_INFO "%s: thread %s " fmt, __FUNCTION__, current->comm, ##args); \
	} while (0)

static int pmlog_file_exist(char *path)
{
	struct file *filp = NULL;
	filp = filp_open(path, O_RDWR, S_IRWXU);
	if (!IS_ERR(filp)) {
		filp_close(filp, NULL);
		return 1;
	}
	return 0;
}
		
static void pmlog_move_oldfile(void)
{
	char old_path[512];
	char new_path[512];
	mm_segment_t oldfs;
	int i;

	oldfs = get_fs();
	set_fs(get_ds());

	for (i=MAX_POWER_OFF_SAVE_TIMES; i>=0; i--) {	
		snprintf(old_path, 512, "%s/%s.%d", POWEROFF_SAVED_PATH, POWEROFF_NAME, i);
		snprintf(new_path, 512, "%s/%s.%d", POWEROFF_SAVED_PATH, POWEROFF_NAME, i+1);
		if (pmlog_file_exist(old_path)) {
			if (i==MAX_POWER_OFF_SAVE_TIMES) 
				sys_unlink(old_path);
			else 
				sys_rename(old_path, new_path);
		}
	}
	snprintf(old_path, 512, "%s/%s", POWEROFF_SAVED_PATH, POWEROFF_NAME);
	snprintf(new_path, 512, "%s/%s.0", POWEROFF_SAVED_PATH, POWEROFF_NAME);
	if (pmlog_file_exist(old_path)) sys_rename(old_path, new_path);

	set_fs(oldfs);
}

static int format_string(char *buffer, size_t length,
		int column_size, ALIGNMENT align,
		bool newline, const char *fmt, ...)
{
	char string_buffer[80] ;
	va_list args;
	int size = 0;
	int shift = 0;
	int min_length = (newline) ? (column_size+2) : (column_size+1);

	if (column_size >= 80) return 0;
	if (min_length >= length) return 0;

	memset(string_buffer, ' ', 80);

	va_start(args, fmt);
	size = vsnprintf(string_buffer, 80, fmt, args);
	va_end(args);

	memset(buffer, ' ', column_size);
	if (newline) buffer[column_size] = '\n';
	buffer[min_length-1] = 0;

	switch(align) {
		case ALIGN_CENTER:
			shift = (column_size - size)/2;
			break;
		case ALIGN_RIGHT:
			shift = column_size - size;
			break;
		case ALIGN_LEFT:
		default:
			shift = 0;
			break;
	}

	memcpy((buffer + shift), string_buffer, size);
	return min_length-1;
}

#define	FORMAT_COLUMN_R(buffer, length, format, arg...) \
	format_string(buffer, length, 20, ALIGN_RIGHT, false, format, ## arg);

#define FORMAT_COLUMN_RN(buffer, length, format, arg...) \
	format_string(buffer, length, 20, ALIGN_RIGHT, true, format, ## arg);

#define FORMAT_COLUMN_C(buffer, length, format, arg...) \
	format_string(buffer, length, 20, ALIGN_CENTER, false, format, ## arg);

#define	FORMAT_COLUMN_L(buffer, length, format, arg...) \
	format_string(buffer, length, 20, ALIGN_LEFT, false, format, ## arg);

#define FORMAT_COLUMN_N(buffer, length, format, arg...) \
	format_string(buffer, length, 20, ALIGN_RIGHT, true, format, ## arg);

#define	FORMAT_COLUMN_CN(buffer, length, format, arg...) \
	format_string(buffer, length, 20, ALIGN_CENTER, true, format, ## arg);

#define	FORMAT_COLUMN_SR(buffer, length, size, format, arg...) \
	format_string(buffer, length, size, ALIGN_RIGHT, false, format, ## arg);

#define FORMAT_COLUMN_SRN(buffer, length, size, format, arg...) \
	format_string(buffer, length, size, ALIGN_RIGHT, true, format, ## arg);

#define FORMAT_COLUMN_SC(buffer, length, size, format, arg...) \
	format_string(buffer, length, size, ALIGN_CENTER, false, format, ## arg);

#define	FORMAT_COLUMN_SCN(buffer, length, size, format, arg...) \
	format_string(buffer, length, size, ALIGN_CENTER, true, format, ## arg);

void pm_log_on(unsigned which_src)
{
	unsigned long	flags;
	
	if (!smem_vendor1_data) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "Can't find pm_log.\n");
		return ;
	}
	spin_lock_irqsave(&pm_lock, flags);
	pm_log_start[which_src] = ktime_to_timespec(alarm_get_elapsed_realtime());
	set_bit(which_src , &(smem_vendor1_data->diag_pm_comp_map));
	spin_unlock_irqrestore(&pm_lock, flags);
	PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "%s start to use, map = %ld\n", pm_log_name[which_src], smem_vendor1_data->diag_pm_comp_map);
}

void pm_log_off(unsigned which_src)
{
	unsigned long	flags;
	struct timespec	diff;

	
	if (!smem_vendor1_data) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "Can't find pm_log.\n");
		return ;
	}

	PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "%s = stop using\n", pm_log_name[which_src]);
	spin_lock_irqsave(&pm_lock, flags);
		if ((pm_log_start[which_src].tv_sec != 0) || (pm_log_start[which_src].tv_nsec != 0)) {
		diff = timespec_sub(ktime_to_timespec(alarm_get_elapsed_realtime()), pm_log_start[which_src]);
		if (smem_vendor1_data) {
			PM_LOG_DPRINTK(PMLOG_DBG_TRACE, " %s runtime = %ld map = %ld+\n", pm_log_name[which_src], smem_vendor1_data->diag_pm_comp[which_src], smem_vendor1_data->diag_pm_comp_map);
			smem_vendor1_data->diag_pm_comp[which_src] += (unsigned long) ((diff.tv_sec*MSEC_PER_SEC)+(diff.tv_nsec/NSEC_PER_MSEC));
			clear_bit(which_src , &(smem_vendor1_data->diag_pm_comp_map));
			PM_LOG_DPRINTK(PMLOG_DBG_TRACE, " %s runtime = %ld map = %ld-\n", pm_log_name[which_src], smem_vendor1_data->diag_pm_comp[which_src], smem_vendor1_data->diag_pm_comp_map);
		}
		
		memset(&pm_log_start[which_src], 0, sizeof(struct timespec));
	}
	spin_unlock_irqrestore(&pm_lock, flags);
}

void pm_log_event(int mode, unsigned which_src)
{
	if (which_src < PM_LOG_NUM) {
		switch( mode) {
		case PM_LOG_ON:
			pm_log_on(which_src); break;
		case PM_LOG_OFF:
			pm_log_off(which_src); break;
		}
	}
}
EXPORT_SYMBOL(pm_log_event);


static int pmlog_notify_sys(struct notifier_block *this, unsigned long code,
	void *unused)
{
	char *cmd = unused;
	int len;
	wake_lock(&wake_lock_pmlog);
	if (cmd)
	{
		strncpy(shutdown_cmd, cmd, 80);
		shutdown_cmd[79] = 0;
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "Command:%s\n", cmd);
		
		
		if ((SYS_POWER_OFF == code) || (SYS_RESTART == code))
		{
			struct timespec ts;
			struct rtc_time tm;		

			
			pmlog_move_oldfile();
			
			getnstimeofday(&ts);
			rtc_time_to_tm(ts.tv_sec, &tm);
			
			len = snprintf(log_buf, log_size, "Power Off Time: %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
					tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
	                tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

			
			if (SYS_POWER_OFF == code){
				if (!strncmp(cmd, "User", 5)) {
					poweroff_reason = 0x99887700;
				} else if (!strncmp(cmd, "Sys-Rq o", 9)) {
					poweroff_reason = 0x99887701;
				} else if (!strncmp(cmd, "Hibernation Shutdown", 21)) {
					poweroff_reason = 0x99887702;
				} else if (!strncmp(cmd, "Orderly", 8)) {
					poweroff_reason = 0x99887703;
				} else if (!strncmp(cmd, "Watchdog - Over Heat", 21)) {
					poweroff_reason = 0x99887704;
				} else if (!strncmp(cmd, "Power Key", 10)) {
					poweroff_reason = 0x99887705;
				}
				
				if (poweroff_reason != 0x998877BB)
					len += snprintf( log_buf+len, log_size-len, "PowerOff reason: %s(%x)\n", cmd, poweroff_reason);
				else
					len += snprintf( log_buf+len, log_size-len, "PowerOff reason: Unknown(%x)\n", poweroff_reason);				
			}else {
				len += snprintf( log_buf + len, log_size-len, "Reboot reason: %s\n", cmd);
			}				
			
		}
	}
	wake_unlock(&wake_lock_pmlog);
	return NOTIFY_DONE;
}

static struct notifier_block pmlog_notifier = {
	.notifier_call = pmlog_notify_sys,
	.priority = INT_MAX,	
};

#if defined(CONFIG_DEBUG_FS)

#define MAX_DEBUG_BUFFER	1024
static char debug_buffer[MAX_DEBUG_BUFFER];

static int debug_read_pmlog(char *buf, int max)
{
	int	size = 0,j;
	if (!smem_vendor1_data) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "Can't find pm_log.\n");
		return 0;
	}
	size += FORMAT_COLUMN_C(buf+size, PAGE_SIZE-size, "Device Name");
	size += FORMAT_COLUMN_C(buf+size, PAGE_SIZE-size, "Run Time (ms)");
	size += FORMAT_COLUMN_CN(buf+size, PAGE_SIZE-size, "Last Start");
	size += snprintf(buf+size, PAGE_SIZE-size, "===============================================================\n");

	for(j = 0; j < PM_LOG_NUM ; j++) {
		size += FORMAT_COLUMN_C(buf+size, PAGE_SIZE-size, "%s", pm_log_name[j]);
		size += FORMAT_COLUMN_C(buf+size, PAGE_SIZE-size, "%ld",
			smem_vendor1_data->diag_pm_comp[j]);
		size += FORMAT_COLUMN_CN(buf+size, PAGE_SIZE-size, "%ld.%ld",
			pm_log_start[j].tv_sec, pm_log_start[j].tv_nsec/NSEC_PER_MSEC);
	}
	return size;
}

static ssize_t debug_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, MAX_DEBUG_BUFFER);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

#endif	

static int __init pm_log_init(void)
{
	int ret = 0;

	if (smem_vendor1_data == NULL) {
		smem_vendor1_data = (smem_vendor_id1_apps_data *) smem_alloc (SMEM_ID_VENDOR1, sizeof (smem_vendor_id1_apps_data));
	}

	if (!smem_vendor1_data) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "Can't allocate smem_vendor_id1_apps_data\n");
		return -EIO;
	}

	memset(pm_log_start, 0, sizeof(struct timespec) * PM_LOG_NUM);
	
	memset(smem_vendor1_data->diag_pm_comp, 0, sizeof(uint32) * PM_LOG_NUM);

	
	ret = register_reboot_notifier(&pmlog_notifier);
	if (ret) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "cannot register reboot notifier. (err=0x%x)\n", ret);
		return ret;
	}
	
	wake_lock_init(&wake_lock_pmlog, WAKE_LOCK_SUSPEND, "pmlog_lock");

#if defined(CONFIG_DEBUG_FS)
	debugfs_create_file("pm_log", 0444, NULL, debug_read_pmlog, &debug_ops);
	if (kernel_debuglevel_dir!=NULL)
	{
		debugfs_create_u32("pmlog_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&PMLOG_DLL));
	}
	else
	{
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "kernel_debuglevel_dir pmlog_dll Fail\n");
	}
#endif	

	return 0;
}
arch_initcall(pm_log_init);
