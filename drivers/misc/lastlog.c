#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io-mapping.h>
#include <ftags.h>
#include <asm/setup.h>
#include <linux/workqueue.h>
#include "../staging/android/logger.h"
#include <linux/vmalloc.h>
#include <linux/rtc.h>
#include <linux/ktime.h>
#include <linux/memblock.h>

typedef struct
{
	long offset;
	long size;
	long end_pos;
} log_info;


#ifdef OEM_BUILD_VERSION
static char oem_build_version[] = OEM_BUILD_VERSION;
#else
static char oem_build_version[] = "N/A";
#endif


#define LASTLOG_PROC_FOLDER "lastlog"
static char __iomem *lastlog_buffer = NULL;
static uint32_t lastlog_start = 0, lastlog_size = 0;
static struct proc_dir_entry *proc_dentry;
static struct work_struct proc_removal_work;

static int proc_lastlog_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	log_info *pdata = data;
	char *buffer_start;
	int len;

	buffer_start = &lastlog_buffer[pdata->offset];

	if (off > pdata->size) {
		*eof = 1;
		return 0;
	}

	if (off + count >= pdata->size) {
		*eof = 1;
		len = pdata->size - off;
	} else {
		len = count;
	}
	*(int *)start = count;
	memcpy (page, &buffer_start[off], len);

	return len;
}



static int proc_lastlog_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	
	
	
	return count;
}



static void lastlog_remove_proc_work(struct work_struct *work)
{
	struct proc_dir_entry *tmp, *befree;

	tmp = proc_dentry->subdir;

	do {
		befree = tmp;
		tmp = tmp->next;
		kfree(befree->data);
		remove_proc_entry(befree->name, proc_dentry);
	} while (tmp);

	remove_proc_entry (LASTLOG_PROC_FOLDER, NULL);
	iounmap(lastlog_buffer);

	
	
	
}



static int lastlog_m_show(struct seq_file *s, void *v)
{
	const char priority_chars[] = {'?', '?', 'V', 'D', 'I','W','E', 'F', 'S'};
	struct logger_entry *entry = v;
	struct logger_entry tmp;
	char* tagaddr;
	char *msgaddr;
	struct rtc_time rtctime;
	int priority;

	memcpy (&tmp, entry, sizeof(tmp));

	rtctime = rtc_ktime_to_tm(ktime_set (tmp.sec, tmp.nsec));
	priority = entry->msg[0];
	tagaddr = &(entry->msg[1]);
	msgaddr = tagaddr + strlen(tagaddr) + 1;

	seq_printf (s, "%02d-%02d %02d:%02d:%02d.%03d %c/%-8s(%5d:%d): ",
						rtctime.tm_mon + 1, rtctime.tm_mday, rtctime.tm_hour,
						rtctime.tm_min, rtctime.tm_sec + 1,
						tmp.nsec / 1000000, priority_chars[priority],
						tagaddr, tmp.pid, tmp.tid);
	if (entry->msg[tmp.len-1] == '\0') {
		seq_printf (s, "%s", msgaddr);
	} else {
		seq_write (s, msgaddr, tmp.len - (msgaddr - entry->msg));
	}

	if (s->buf[s->count-1] != '\n')
		seq_putc(s, '\n');

	return 0;
}



static void *lastlog_m_start(struct seq_file *s, loff_t *pos)
{
	log_info *pdata = (log_info *)s->private;
	char *buffer_start;
	loff_t l = *pos;

	if (l + sizeof(struct logger_entry) >= pdata->size) {
		return NULL;
	}

	if (pdata->end_pos != 0 && l >= pdata->end_pos) {
		return NULL;
	}

	buffer_start = &lastlog_buffer[pdata->offset];
	while (l + sizeof(struct logger_entry) < pdata->size) {
		if (buffer_start[l+2] == 0xAD && buffer_start[l+3] == 0xDE)
			break;
		l ++;
	}

	if (l + sizeof(struct logger_entry) >= pdata->size)
	{
		pdata->end_pos = l;
		return NULL;
	}

	*pos = l;

	return &buffer_start[*pos];
}



static void *lastlog_m_next(struct seq_file *s, void *v, loff_t *pos)
{
	log_info *pdata = (log_info *)s->private;
	char *buffer_start = v;
	int i;
	loff_t l = *pos;
	unsigned short msg_len;

	if (l + sizeof(struct logger_entry) >= pdata->size) {
		return NULL;
	}

	if (pdata->end_pos != 0 && l >= pdata->end_pos) {
		return NULL;
	}

	msg_len = buffer_start[0] + (buffer_start[1] << 8);
	i = sizeof(struct logger_entry) + msg_len;

	while (i + l + sizeof(struct logger_entry) < pdata->size) {
		if (buffer_start[i+2] == 0xAD && buffer_start[i+3] == 0xDE)
			break;
		i ++;
	}

	if (i + l + sizeof(struct logger_entry) >= pdata->size) {
		pdata->end_pos = l;
		return NULL;
	}

	*pos = i + l;

	return &buffer_start[i];
}



static void lastlog_m_stop(struct seq_file *swap, void *v)
{
}



static const struct seq_operations lastlog_ops = {
	.start =	lastlog_m_start,
	.next =		lastlog_m_next,
	.stop =		lastlog_m_stop,
	.show =		lastlog_m_show,
};


static int lastlog_open(struct inode *inode, struct file *file)
{
	int ret;
	struct proc_dir_entry *pde = PDE(inode);

	ret = seq_open (file, &lastlog_ops);

	if (!ret)
		((struct seq_file *)file->private_data)->private = pde->data;


	return ret;
}



static const struct file_operations proc_lastlog_operations = {
	.open		= lastlog_open,
	.read		= seq_read,
	.release	= seq_release,
};


static int proc_show_sw_version(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = strlen(data);
	memcpy (page, data, len);
	*eof = 1;
	return len;
}


static int __init proc_lastlog_init(void)
{
	extern panic_log_struct __panic_log_begin, __panic_log_end;
	struct proc_dir_entry *pe;
	panic_log_struct *panic_logs_ptr;
	int offset;

	if (lastlog_start == 0)
		return 1;

	lastlog_buffer = ioremap (lastlog_start, lastlog_size);

	if (lastlog_buffer == NULL) {
		printk (KERN_ERR "failed to map lastlog buffer\n");
		return -ENOMEM;
	}

	proc_dentry = proc_mkdir (LASTLOG_PROC_FOLDER, NULL);

	if (!proc_dentry) {
		printk (KERN_ERR "failed to create lastlog folder\n");
		return -EPERM;
	}

	
	pe = create_proc_entry("sw_version", S_IRUGO | S_IWUGO, proc_dentry);
	if (pe) {
		pe->read_proc = proc_show_sw_version;
		pe->data = oem_build_version;
	}
	

	offset = 0;
	for (panic_logs_ptr = &__panic_log_begin; panic_logs_ptr < &__panic_log_end && panic_logs_ptr->magic == PANIC_LOG_MAGIC; panic_logs_ptr ++) {
		log_info *pdata = kmalloc (sizeof(log_info), GFP_KERNEL);
		pdata->offset = offset;
		pdata->size = panic_logs_ptr->size;
		pdata->end_pos = 0;
		offset += pdata->size;

		if (panic_logs_ptr->type == LOGTYPE_RAW) {
			
			char *buffer_start = &lastlog_buffer[pdata->offset];

			while (pdata->size > 0 && buffer_start[pdata->size - 1] == 0)
			{
				pdata->size--;
			}
			
			pe = create_proc_entry(panic_logs_ptr->name, S_IRUGO | S_IWUGO, proc_dentry); 
			if (pe) {
				pe->read_proc = proc_lastlog_read;
				pe->write_proc = proc_lastlog_write;
				pe->data = pdata;
			}

			
			if (!strcmp("kmsg", panic_logs_ptr->name)) {
				pe = create_proc_entry("last_kmsg", S_IRUGO | S_IWUGO, NULL);
				if (pe) {
					pe->read_proc = proc_lastlog_read;
					pe->write_proc = proc_lastlog_write;
					pe->data = pdata;
				}
			}
			
		} else if (panic_logs_ptr->type == LOGTYPE_ALOG) {
			proc_create_data(panic_logs_ptr->name, S_IRUGO | S_IWUGO, proc_dentry, &proc_lastlog_operations, pdata);
		} else {
			printk (KERN_ERR "%s not support logtype: %d\n", __func__, panic_logs_ptr->type);
		}

	}

	INIT_WORK(&proc_removal_work, lastlog_remove_proc_work);

	return 1;
}


static void __exit proc_lastlog_exit(void)
{
	schedule_work(&proc_removal_work);
}


static int __init parse_tag_mem_lastlog(const struct tag *tag)
{
	printk ("%s %x %x\n", __func__, tag->u.mem.start, tag->u.mem.size);
	lastlog_start = tag->u.mem.start;
	lastlog_size = tag->u.mem.size;
	return 0;
}

__tagtable(ATAG_MEM_LASTLOG, parse_tag_mem_lastlog);


module_init(proc_lastlog_init);
module_exit(proc_lastlog_exit);
