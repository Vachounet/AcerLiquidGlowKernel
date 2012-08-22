#ifndef _Q_MTD_RAW_H_
#define _Q_MTD_RAW_H_

#define Q_MTD_READ		_IOWR('M', 33, struct mtd_oob_buf)
#define Q_MTD_WRITE		_IOWR('M', 34, struct mtd_oob_buf)

#define Q_MTD_DLOG	_IOWR('M', 35, struct mtd_oob_buf)

#define Q_MTD_CLEAR_ALL_VAR		_IO('M', 37)

#endif
