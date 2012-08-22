/* drivers/video/msm/logo.c
 *
 * Show Logo in RLE 565 format
 *
 * Copyright (C) 2008 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>

#include <linux/irq.h>
#include <asm/system.h>
#include <mach/board.h>

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)
#define fb_size(fb)	((fb)->var.xres * (fb)->var.yres * ((fb)->var.bits_per_pixel/8))
#define fb_bpp(fb)	((fb)->var.bits_per_pixel)

#ifdef CONFIG_POWEROFF_CHARGING
extern enum ANDROIDBOOTMODE androidboot_mode;
#endif

static void memset16(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	count >>= 1;
	while (count--)
		*ptr++ = val;
}

static void memset32(void *_ptr, unsigned val, unsigned count)
{
	unsigned *ptr = _ptr;
	count >>= 2;
	while (count--)
		*ptr++ = val;
}

/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
int load_565rle_image(char *filename, bool bf_supported)
{
	struct fb_info *info;
	int fd, count, err = 0;
	unsigned max;
	unsigned short *data, *bits, *ptr;
	unsigned rgb32, red, green, blue, alpha;


#if 0 
	if (androidboot_mode==ANDROIDBOOTMODE_BOOTOFFCHARGE)
	{
	        printk(KERN_ERR "skip load_565rle_image at offcharge for kernel\n");
		return err;
	}
#endif


	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = sys_lseek(fd, (off_t)0, 2);
	if (count <= 0) {
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if (sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
	}

	max = fb_width(info) * fb_height(info);
	ptr = data;
	if (bf_supported && (info->node == 1 || info->node == 2)) {
		err = -EPERM;
		pr_err("%s:%d no info->creen_base on fb%d!\n",
		       __func__, __LINE__, info->node);
		goto err_logo_free_data;
	}
	bits = (unsigned short *)(info->screen_base);
	while (count > 3) {
		unsigned n = ptr[0];
		if (n > max)
			break;

                if (fb_bpp(info) == 16)
                {
			memset16(bits, ptr[1], n << 1);
			bits += n;
                }
                else
                {
                        
                        rgb32 = ((ptr[1] >> 11) & 0x1F);
                        red = (rgb32 << 3) | (rgb32 >> 2);
                        rgb32 = ((ptr[1] >> 5) & 0x3F);
                        green = (rgb32 << 2) | (rgb32 >> 4);
                        rgb32 = ((ptr[1]) & 0x1F);
                        blue = (rgb32 << 3) | (rgb32 >> 2);
                        alpha = 0xff;
                        rgb32 = (alpha << 24) | (blue << 16)
                        | (green << 8) | (red);
                        memset32((uint32_t *)bits, rgb32, n << 2);
                        bits += (n * 2);
                }
		max -= n;
		ptr += 2;
		count -= 4;
	}

err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);
	return err;
}
EXPORT_SYMBOL(load_565rle_image);
