#ifndef __FTAGS_H__
#define __FTAGS_H__

#define FTAGS_MAGIC 0x46544147
#define PANIC_LOG_MAGIC 0x504E4C47


#ifndef __ASSEMBLY__

enum {
	LOGTYPE_RAW = 0,
	LOGTYPE_ALOG,
};

typedef struct {
	int magic;
	char *name;
	char *addr;
	int size;
	int *head;
	int type;
} panic_log_struct;

#define PANIC_LOG_INIT(_logs, _name, _buf, _phead, _type) \
const panic_log_struct __painc_logs_##_logs __attribute ((__section__(".panic_logs"))) = { \
	.magic = PANIC_LOG_MAGIC, \
	.name = _name, \
	.addr = _buf, \
	.size = sizeof(_buf), \
	.head = _phead, \
	.type = _type, \
};

#endif


#endif
