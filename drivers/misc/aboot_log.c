#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/io-mapping.h>
#include <asm/setup.h>
#include <linux/workqueue.h>

#include <linux/reboot.h> 
#include <boot_mode.h>    
#include <linux/export.h> 


static char ABOOT_LOG_PROC_ENTRY[] = "aboot_log";
static char __iomem *aboot_log_buffer = NULL;
static uint32_t aboot_log_start = 0, aboot_log_size = 0;
static struct proc_dir_entry *proc_entry;
static struct work_struct proc_removal_work;


static int proc_aboot_log_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;

	if (aboot_log_buffer == NULL)
		return -1;

	if (off > aboot_log_size) {
		*eof = 1;
		return 0;
	}

	if (off + count >= aboot_log_size) {
		*eof = 1;
		len = aboot_log_size - off;
	}
	else {
		len = count;
	}
	*(int *)start = count;
	memcpy(page, &aboot_log_buffer[off], len);

	return len;
}


static void aboot_log_remove_proc_work(struct work_struct *work)
{
	remove_proc_entry (ABOOT_LOG_PROC_ENTRY, NULL);
	iounmap(aboot_log_buffer);
}



extern uint32_t get_restart_reason(void);
static void backup_restart_reason (unsigned long reason)
{
	int *restart_reason_ptr = (int *) aboot_log_buffer;

	if (restart_reason_ptr != NULL) {
		restart_reason_ptr[0] = RESTART_REASON_MAGIC;
		restart_reason_ptr[1] = reason;
	}
}
EXPORT_SYMBOL(backup_restart_reason);

static int aboot_reboot_call
	(struct notifier_block *this, unsigned long code, void *_cmd)
{
	backup_restart_reason(get_restart_reason());
	return NOTIFY_DONE;
}

static struct notifier_block aboot_reboot_notifier = {
	.notifier_call = aboot_reboot_call,
	.priority = -1, 
};




static int panic_backup_restart_reason(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	backup_restart_reason(get_restart_reason());
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_backup_restart_reason,
	.priority 	= -1,
};



static int __init proc_aboot_log_init(void)
{
	if (aboot_log_size == 0)
		return 1;

	aboot_log_buffer = ioremap (aboot_log_start, aboot_log_size);

	if (aboot_log_buffer == NULL) {
		printk (KERN_ERR "failed to map aboot_log buffer\n");
		return -ENOMEM;
	}

	
	register_reboot_notifier(&aboot_reboot_notifier);
	
	
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	

	
	if (aboot_log_buffer[aboot_log_size - 1] == '\0') {  
		aboot_log_size = strlen (aboot_log_buffer);
	}

	proc_entry = create_proc_entry(ABOOT_LOG_PROC_ENTRY, S_IRUGO, NULL);
	if (!proc_entry) {
		printk (KERN_ERR "failed to create aboot_log entry\n");
		return -EPERM;
	}

	proc_entry->read_proc = proc_aboot_log_read;
	proc_entry->data = aboot_log_buffer;

	INIT_WORK(&proc_removal_work, aboot_log_remove_proc_work);

	return 1;
}


static void __exit proc_aboot_log_exit(void)
{
	schedule_work(&proc_removal_work);
}


static int __init parse_tag_mem_aboot_log(const struct tag *tag)
{
	printk ("%s %x %x\n", __func__, tag->u.mem.start, tag->u.mem.size);
	aboot_log_start = tag->u.mem.start;
	aboot_log_size = tag->u.mem.size;
	return 0;
}

__tagtable(ATAG_MEM_ABOOT_LOG, parse_tag_mem_aboot_log);


module_init(proc_aboot_log_init);
module_exit(proc_aboot_log_exit);

