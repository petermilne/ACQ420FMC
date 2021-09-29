/* ------------------------------------------------------------------------- *
 * acq465_drv.c
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2021 Peter Milne, D-TACQ Solutions Ltd
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 28 September 2021
 *    Author: pgm                                                         
 *                                                                           *
 *  This program is free software; you can redistribute it and/or modify     *
 *  it under the terms of Version 2 of the GNU General Public License        *
 *  as published by the Free Software Foundation;                            *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program; if not, write to the Free Software              *
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/*
 * instantiates ACQ465 spi buffer
 * also creates a device driver hook for the spi buffer
 * AD7134 has 0x48 addresses. Map the buffer as follows:
 * acq465.1/chip0 : 0x0000 .. 0x007f
 * acq465.1/chip1 : 0x0080 .. 0x0100
 * ...
 * acq465.1/chip7 : 0x0780 .. 0x07ff
 *
 * acq465.2/chip0 : 0x0800 .. 0x087f
 *
 * acq465.5/chip0 : 0x1000 .. 0x107f         # site 5 takes second 4K page
 *
 * acq465_knobs app mmaps this buffer and updates it.
 * a task on a poll loop in this driver monitors changes and sends them to SPI
 * We have one linux device per site, but we have to be aware of the 8 physical devices..
 */
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/spi/spi.h>

#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include "hbm.h"

#include "acq465_ioctl.h"

#define REVID 		"0.0.0"
#define MODULE_NAME	"acq465"

int acq465_sites[6] = { 0,  };
int acq465_sites_count = 0;
module_param_array(acq465_sites, int, &acq465_sites_count, 0644);

static int n_acq465;
module_param(n_acq465, int, 0444);

static int spi_bus_num = 1;
module_param(spi_bus_num, int, 0444);

static int set_ident = 1;
module_param(set_ident, int, 0444);

#define REGS_LEN	0x80
#define NCHIPS		8
#define MODULE_SPI_BUFFER_LEN 	(NCHIPS*REGS_LEN)					// 1K
#define TOTAL_SPI_BUFFER_LEN	((acq465_sites_count&~0x3)+4)*MODULE_SPI_BUFFER_LEN
// round up to integer pages
// 6 sites -> 2 pages, but if there is only 2 sites eg ACQ1002, we can save a PAGE..



static struct HBM* driver_spi_buffer;     // single buffer for all devices.
static struct HBM* client_spi_buffer;     // single buffer for all devices.

#define DRV_PA(devnum) 	(driver_spi_buffer->pa+MODULE_SPI_BUFFER_LEN*((devnum)-1))
#define DRV_VA(devnum) 	((unsigned char*)driver_spi_buffer->va+MODULE_SPI_BUFFER_LEN*((devnum)-1))

#define CLI_PA(devnum) 	(client_spi_buffer->pa+MODULE_SPI_BUFFER_LEN*((devnum)-1))
#define CLI_VA(devnum) 	((unsigned char*)client_spi_buffer->va+MODULE_SPI_BUFFER_LEN*((devnum)-1))

int site2cs(int site)
{
	return site - 1;
}
int cs2site(int cs)
{
	return cs + 1;
}

struct acq465_dev {
	dev_t devno;
	struct cdev cdev;
	struct platform_device *pdev;
	char devname[16];

	/* client stores to cli_buffer. dev_buffer caches device values
	 * update flushes client changes to to device
	 */
	struct BUF {
		unsigned char* va;
		unsigned pa;
	} cli_buf, dev_buf;

	struct spi_device *spi;
};


#define DEVP(adev) (&adev->pdev->dev)

static struct proc_dir_entry *acq465_proc_root;
static struct acq465_dev* acq465_devs[7];	/* 6 sites index from 1 */

/* we're ASSUMING that ad7134 doesn't mind access to non-existant regs ..
 * otherwise we have to have a sparse reg table
 */
#define AD7134_MAXREG	0x48				/* address in bytes */
#define AD7134_REGSLEN	((AD7134_MAXREG+1))            /* register set length in bytes */

static void print_hbm(struct HBM* hbm){
	dev_info(0, "hbm: pa: 0x%08x va: %p len:%d\n", hbm->pa, hbm->va, hbm->len);
}
static void _ident(unsigned char* drv_buf, int site, int id)
{
	unsigned char* pb = drv_buf + (site-1)*MODULE_SPI_BUFFER_LEN;
	int chip;
	int ii;
	for (chip = 0; chip < NCHIPS; ++chip, pb += REGS_LEN){
		for (ii = 0; ii < REGS_LEN; ++ii){
			pb[ii] = ii < AD7134_REGSLEN? ii+id: chip|(site<<4);
		}
	}
}
static void ident(unsigned char* drv_buf, int id)
{
	int nn;

	for (nn = 0; nn < acq465_sites_count; ++nn){
		_ident(drv_buf, acq465_sites[nn], id);
	}
}
int get_site(struct acq465_dev *adev)
{
	return adev->pdev->id;
}

static void ad7134_setReadout(struct acq465_dev* adev, int readout)
{
	char cmd[3] = { 0x1, 0, 0x1 };
	if (!readout) cmd[2] = 0;

	dev_dbg(DEVP(adev), "ad7134_setReadout () spi_write %02x %02x %02x",
						cmd[0], cmd[1], cmd[2]);
	spi_write(adev->spi, &cmd, 3);
}
static void ad7134_cache_invalidate(struct acq465_dev* adev)
/* read back cache from device */
{
/* change to bytes, chipselect for individual chips
	short *cache = adev->dev_buf->va;
	int reg;
	ad7134_setReadout(adev, 1);
	for (reg = 2; reg <= ADS5294_MAXREG; ++reg){
		unsigned short tmp = spi_w8r16(adev->spi, reg);
		cache[reg] = tmp>>8 | ((tmp&0x00ff)<<8);
		dev_dbg(DEVP(adev),
			"ad7134_cache_invalidate() spi_w8r16 %02x %02x %02x",
			reg, cache[reg]>>8, cache[reg]&0x0ff);
	}
	ad7134_setReadout(adev, 0);

	memcpy(adev->cli_buf->va, adev->dev_buf->va, ADS5294_REGSLEN);
*/
}
static void ad7134_cache_flush(struct acq465_dev* adev)
/* flush changes in cache to device */
{
/*
	short *cache = getByteBuffer(adev->dev_buf);
	short *clibuf = getByteBuffer(adev->cli_buf);

	int reg;
	for (reg = 2; reg <= ADS5294_MAXREG; ++reg){
		if (cache[reg] != clibuf[reg]){
			char cmd[3];
			cache[reg] = clibuf[reg];
			cmd[0] = reg;
			cmd[1] = cache[reg] >> 8;
			cmd[2] = cache[reg] &0x0ff;
			dev_dbg(DEVP(adev),
				"ad7134_cache_flush() spi_write %02x %02x %02x",
				cmd[0], cmd[1], cmd[2]);
			spi_write(adev->spi, &cmd, 3);
		}
	}
*/
}

static void ad7134_reset(struct acq465_dev* adev)
{
	char cmd[3] = { 0x0, 0, 0x1 };

	dev_dbg(DEVP(adev), "ad7134_reset () spi_write %02x %02x %02x",
						cmd[0], cmd[1], cmd[2]);
	spi_write(adev->spi, &cmd, 3);
}





#define acq465_of_match 0

struct acq465_dev* acq465_allocate_dev(struct platform_device *pdev)
{
	int site = pdev->id;
	struct acq465_dev* adev =
		kzalloc(sizeof(struct acq465_dev), GFP_KERNEL);

	if (adev == NULL){
		return NULL;
	}
	acq465_devs[site] = adev;

	adev->pdev = pdev;
	snprintf(adev->devname, 16, "%s.%d", pdev->name, pdev->id);

	return adev;
}

int acq465_open(struct inode *inode, struct file *file)
{
        file->private_data = container_of(inode->i_cdev, struct acq465_dev, cdev);
        return 0;
}

int acq465_cli_buffer_mmap(struct file* file, struct vm_area_struct* vma)
/**
 * mmap the host buffer.
 */
{
	struct acq465_dev* adev = (struct acq465_dev*)file->private_data;

	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = PAGE_SIZE; // maybe 2 pages?
	unsigned pfn = adev->cli_buf.pa >> PAGE_SHIFT;

	dev_dbg(&adev->pdev->dev, "%c vsize %lu psize %lu %s",
		'D', vsize, psize, vsize>psize? "EINVAL": "OK");

	if (vsize > psize){
		return -EINVAL;                   /* request too big */
	}
	if (remap_pfn_range(
		vma, vma->vm_start, pfn, vsize, vma->vm_page_prot)){
		return -EAGAIN;
	}else{
		return 0;
	}
}

static long
acq465_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct acq465_dev* adev = (struct acq465_dev*)file->private_data;
	switch(cmd){
	case ACQ465_CACHE_INVALIDATE:
		ad7134_cache_invalidate(adev);
		return 0;
	case ACQ465_CACHE_FLUSH:
		ad7134_cache_flush(adev);
		return 0;
	case ACQ465_RESET:
		ad7134_reset(adev);
		return 0;
	default:
		return -ENODEV;
	}
}
struct file_operations acq465_fops = {
        .owner = THIS_MODULE,
        .open = acq465_open,
        .mmap = acq465_cli_buffer_mmap,
        .unlocked_ioctl = acq465_unlocked_ioctl
};

/* read 0x100 bytes, 0x10 bytes at a time */
static void *acq465_proc_seq_start_buffers(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
        	struct acq465_dev *adev = s->private;
        	dev_info(DEVP(adev), "acq465_proc_seq_start_buffers() %s adev:%p clibuf:%p",
        			adev->devname, adev, adev->cli_buf.va);
        	seq_printf(s, "chip:%c\n", 'A'+0);
        	return adev->cli_buf.va;
        }

        return NULL;
}
#define BUFREAD	0x10	/* words to read 	*/
#define BUFMAX  0x100	/* total words to read 	*/
static int acq465_proc_seq_show_spibuf_row(struct seq_file *s, void *v)
{
	struct acq465_dev *adev = s->private;
	unsigned char* regs = (unsigned char*)v;
	unsigned char* base = adev->cli_buf.va;
	unsigned offregs;
	int ir;

	while ((offregs = (regs - base)) > AD7134_MAXREG){
		base += REGS_LEN;
	}

	seq_printf(s, "%02x:", offregs%AD7134_MAXREG);
	for (ir = 0; ir < BUFREAD-1; ++ir){
		seq_printf(s, "%02x ", regs[ir]);
	}
	seq_printf(s, "%02x\n", regs[ir]);
	return 0;
}

static void* acq465_proc_seq_next_buffers(
		struct seq_file *s, void* v, loff_t *pos)
{
	*pos += BUFREAD;
	if (*pos < MODULE_SPI_BUFFER_LEN){
		loff_t offset = *pos%REGS_LEN;
		if (offset  < AD7134_REGSLEN){
			return v + BUFREAD;
		}else{
			struct acq465_dev *adev = s->private;

			int chip = *pos/REGS_LEN + 1;
			if (chip > 7){
				return NULL;
			}else{
				seq_printf(s, "chip:%c\n", 'A'+chip);
				*pos = chip * REGS_LEN;
				return adev->cli_buf.va + *pos;
			}
		}
	}else{
		return NULL;
	}
}
static void acq465_proc_seq_stop(struct seq_file *s, void* v)
{

}

static int acq465_proc_open_spibuf(struct inode *inode, struct file *file)
{
	static struct seq_operations proc_seq_ops_spi_buf = {
	        .start = acq465_proc_seq_start_buffers,
	        .next = acq465_proc_seq_next_buffers,
	        .stop = acq465_proc_seq_stop,
	        .show = acq465_proc_seq_show_spibuf_row
	};
	int rc = seq_open(file, &proc_seq_ops_spi_buf);
	if (rc == 0){
		struct seq_file *m = file->private_data;
		struct acq465_dev* adev = PDE_DATA(inode);
		m->private = PDE_DATA(inode);

		dev_info(DEVP(adev), "acq465_proc_open_spibuf() %s adev:%p", adev->devname, adev);
	}
	return rc;
}

struct file_operations acq465_proc_fops = {
	        .owner = THIS_MODULE,
	        .open = acq465_proc_open_spibuf,
	        .read = seq_read,
	        .llseek = seq_lseek,
	        .release = seq_release
};


static int acq465_probe(struct platform_device *pdev)
{
	struct acq465_dev* adev = acq465_allocate_dev(pdev);
	struct device* dev = &pdev->dev;
	int rc;

	dev_info(dev, "acq465_probe: %d %s", pdev->id, adev->devname);
	rc = alloc_chrdev_region(&adev->devno, 0, 0, adev->devname);
	if (rc < 0) {
		dev_err(&pdev->dev, "unable to register chrdev\n");
	        goto fail;
	}

        cdev_init(&adev->cdev, &acq465_fops);
        adev->cdev.owner = THIS_MODULE;
        rc = cdev_add(&adev->cdev, adev->devno, 1);
        if (rc < 0){
        	goto fail;
        }

        if (driver_spi_buffer == 0){
        	driver_spi_buffer = hbm_allocate1(dev, TOTAL_SPI_BUFFER_LEN, 0, DMA_BIDIRECTIONAL);
        	client_spi_buffer = hbm_allocate1(dev, TOTAL_SPI_BUFFER_LEN, 0, DMA_BIDIRECTIONAL);

        	if (set_ident){
        		ident(DRV_VA(1), 0);
        		ident(CLI_VA(1), 1);
        	}
        }
        /* there's no actual DMA, we're after the pa, but DMA_NONE BUGs */
        adev->cli_buf.pa = CLI_PA(pdev->id);
        adev->cli_buf.va = CLI_VA(pdev->id);

        if (!set_ident){
        	memset(adev->cli_buf.va, 0, MODULE_SPI_BUFFER_LEN);
        }

        adev->dev_buf.pa = DRV_PA(pdev->id);
        adev->dev_buf.va = DRV_VA(pdev->id);

        if (acq465_proc_root == 0){
        	acq465_proc_root = proc_mkdir("driver/acq465", 0);
        }

        proc_create_data("spibuf", 0,
        		proc_mkdir(adev->devname, acq465_proc_root),
        		&acq465_proc_fops, adev);
	return 0;

fail:
	kfree(adev);
	return -1;
}

static int acq465_remove(struct platform_device *pdev)
{
	int site = pdev->id;

	kfree(acq465_devs[site]);
	acq465_devs[site] = 0;
	return -1;
}

static int ad7134spi_probe(struct spi_device *spi)
{
	struct acq465_dev* adev = acq465_devs[cs2site(spi->chip_select)];
	adev->spi = spi;
	dev_info(&spi->dev, "ad7134spi_probe() bus:%d cs:%d",
			spi->master->bus_num, spi->chip_select);

	ad7134_cache_invalidate(adev);
	return 0;
}
static int ad7134spi_remove(struct spi_device *spi)
{
	return 0;
}

static void __init acq465_init_site(int site)
{
	static struct spi_board_info ad7134spi_spi_slave_info = {
			.modalias	= "ad7134spi",
			.platform_data	= 0,
			.irq		= -1,
			.max_speed_hz	= 20000000,
			.bus_num	= 0,
			.chip_select	= 0,
	};
	struct platform_device* pdev =
			kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	pdev->name = MODULE_NAME;
	pdev->id = site;

	ad7134spi_spi_slave_info.bus_num = spi_bus_num;
	platform_device_register(pdev);

	ad7134spi_spi_slave_info.chip_select = site - 1;
	spi_register_board_info(&ad7134spi_spi_slave_info, 1);
}

static void __init acq465_remove_site(int site)
{
	printk("acq465_remove_site %d ERROR removal not supported\n", site);
}
static struct platform_driver acq465_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = acq465_of_match,
        },
        .probe = acq465_probe,
        .remove = acq465_remove,
};

static struct spi_driver ad7134spi_driver = {
	.driver = {
		.name	= "ad7134spi",
		.owner	= THIS_MODULE,
	},
	//.id_table = m25p_ids,
	.probe	= ad7134spi_probe,
	.remove	= ad7134spi_remove,
};

static void __exit acq465_exit(void)
{
	for (; n_acq465--;){
		acq465_remove_site(acq465_sites[n_acq465]);
	}
	platform_driver_unregister(&acq465_driver);
	spi_unregister_driver(&ad7134spi_driver);
}

extern void acq480_hook_spi(void);

static int __init acq465_init(void)
{
        int status = 0;


	printk("D-TACQ ACQ465 Driver %s\n", REVID);

	platform_driver_register(&acq465_driver);
	acq465_proc_root = proc_mkdir("driver/acq465", 0);

	spi_register_driver(&ad7134spi_driver);

	acq480_hook_spi();

	for (n_acq465 = 0; n_acq465 < acq465_sites_count; ++n_acq465){
		acq465_init_site(acq465_sites[n_acq465]);
	}
        return status;
}

module_init(acq465_init);
module_exit(acq465_exit);



MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ ACQ465ELF i2c Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
