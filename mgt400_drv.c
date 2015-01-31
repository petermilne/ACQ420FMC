/* ------------------------------------------------------------------------- */
/* mgt400_drv.c  D-TACQ mgt400_comms driver
 *
 *  Created on: 12 Jan 2015
 *      Author: pgm
 *
 * ------------------------------------------------------------------------- */
/*   Copyright (C) 2015 Peter Milne, D-TACQ Solutions Ltd                    *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
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

#include "acq400.h"
#include "mgt400.h"
#include "dmaengine.h"

#define REVID "0.105"

#ifdef MODULE_NAME
#undef MODULE_NAME
#endif
#define MODULE_NAME 	"mgt400"

int ndevices;
module_param(ndevices, int, 0444);
MODULE_PARM_DESC(ndevices, "number of devices found in probe");
#undef MAXDEVICES
#define MAXDEVICES 2

/* index from 0. There's only one physical MGT400, but may have 2 channels */
struct mgt400_dev* mgt400_devices[MAXDEVICES+2];

/* Define debugging for use during our driver bringup */
#undef PDEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO fmt, ## args)

#define OF_IRQ_USEME		1	/* index of IRQ# in dtb */
#define OF_IRQ_COUNT		3	/* number of items */
#define OF_IRQ_MAGIC		32	/* add to the OF number to get actual */



#undef DEVP
#define DEVP(mdev)		(&(mdev)->pdev->dev)


void mgt400wr32(struct mgt400_dev *adev, int offset, u32 value)
{
	if (adev->RW32_debug){
		dev_info(DEVP(adev), "mgt400wr32 %p [0x%02x] = %08x\n",
				adev->dev_virtaddr + offset, offset, value);
	}else{
		dev_dbg(DEVP(adev), "mgt400wr32 %p [0x%02x] = %08x\n",
				adev->dev_virtaddr + offset, offset, value);
	}

	iowrite32(value, adev->dev_virtaddr + offset);
}

u32 mgt400rd32(struct mgt400_dev *adev, int offset)
{
	u32 rc = ioread32(adev->dev_virtaddr + offset);
	if (adev->RW32_debug){
		dev_info(DEVP(adev), "mgt400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	}else{
		dev_dbg(DEVP(adev), "mgt400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	}
	return rc;
}


static struct mgt400_dev* mgt400_allocate_dev(struct platform_device *pdev)
/* Allocate and init a private structure to manage this device */
{
	struct mgt400_dev* mdev = kzalloc(sizeof(struct mgt400_dev), GFP_KERNEL);
        if (mdev == NULL) {
                return NULL;
        }
        mdev->pdev = pdev;
        return mdev;
}



/* assume we can count packets by scanning the descriptor id at 100Hz.
 * 16 ID's, handles packet rate to 1600 pps - easy doable.
 */

static void _channel_buffer_counter(u32 current_descr, unsigned long* packet_count)
{
	unsigned long pc0 = *packet_count;
	unsigned long pc = pc0;

	for (; (pc&DESCR_ID) != (current_descr&DESCR_ID); ++pc){
		;
	}
	if (pc != pc0){
		*packet_count = pc;
	}
}

static void _update_histogram(unsigned long *histo, unsigned count, unsigned mask)
{
	if (count > mask) count = mask;
	histo[count]++;
}

static void _update_histograms(struct mgt400_dev* mdev)
{
	u32 data_sr = mgt400rd32(mdev, DMA_FIFO_SR);
	u32 desc_sr = mgt400rd32(mdev, DESC_FIFO_SR);

	_update_histogram(mdev->pull.data_histo,
		GET_DMA_DATA_FIFO_COUNT(data_sr>>DMA_DATA_PULL_SHL), DATA_HMASK);
	_update_histogram(mdev->push.data_histo,
		GET_DMA_DATA_FIFO_COUNT(data_sr>>DMA_DATA_PUSH_SHL), DATA_HMASK);

	_update_histogram(mdev->pull.desc_histo,
		GET_DMA_DATA_FIFO_COUNT(desc_sr>>DMA_DATA_PULL_SHL), DESC_HMASK);
	_update_histogram(mdev->push.desc_histo,
		GET_DMA_DATA_FIFO_COUNT(desc_sr>>DMA_DATA_PUSH_SHL), DESC_HMASK);

}
static void _mgt400_buffer_counter(struct mgt400_dev* mdev)
{
	_channel_buffer_counter(
		mgt400rd32(mdev, DMA_PUSH_DESC_SR), &mdev->push.buffer_count);
	_channel_buffer_counter(
		mgt400rd32(mdev, DMA_PULL_DESC_SR), &mdev->pull.buffer_count);
	_update_histograms(mdev);
}
static ktime_t kt_period;

enum hrtimer_restart mgt400_buffer_counter(struct hrtimer* hrt)
{
	struct mgt400_dev *mdev =
		container_of(hrt, struct mgt400_dev, buffer_counter_timer);

	_mgt400_buffer_counter(mdev);
	hrtimer_forward_now(hrt, kt_period);
	return HRTIMER_RESTART;
}
void mgt400_start_buffer_counter(struct mgt400_dev* mdev)
{
	hrtimer_init(&mdev->buffer_counter_timer, CLOCK_REALTIME, HRTIMER_MODE_REL);
	mdev->buffer_counter_timer.function = mgt400_buffer_counter;
	hrtimer_start(&mdev->buffer_counter_timer, kt_period, HRTIMER_MODE_REL);
}

void mgt400_stop_buffer_counter(struct mgt400_dev* mdev)
{
	hrtimer_cancel(&mdev->buffer_counter_timer);
}
static int mgt400_device_tree_init(struct mgt400_dev* mdev)
{
	struct device_node *of_node = mdev->pdev->dev.of_node;

	dev_info(DEVP(mdev), "mgt400_device_tree_init() 01 %p", of_node);
        if (of_node) {
        	u32 irqs[OF_IRQ_COUNT];

        	if (of_property_read_u32(of_node, "site",
        			&mdev->of_prams.site) < 0){
        		dev_warn(DEVP(mdev), "error: site NOT specified in DT\n");
        		return -1;
        	}
        	if (of_property_read_u32(of_node, "sn",
        			&mdev->of_prams.sn) < 0){
        		dev_warn(DEVP(mdev), "error: sn NOT specified in DT\n");
        		        		return -1;
        	}else{
        		snprintf(mdev->devname, 16, "mgt400.%c",
        				mdev->of_prams.sn+'A');
        	}
                if (of_property_read_u32_array(
                		of_node, "interrupts", irqs, OF_IRQ_COUNT)){
                	dev_warn(DEVP(mdev), "failed to find IRQ values");
                }else{
                	mdev->of_prams.irq = irqs[OF_IRQ_USEME] + OF_IRQ_MAGIC;
                }
                dev_info(DEVP(mdev), "mgt400 \"%s\" site:%d sn:%d",
                		mdev->devname, mdev->of_prams.site, mdev->of_prams.sn);
                return 0;
        }else{
        	return -1;
        }
}

int mgt400_open(struct inode *inode, struct file *file)
{
	file->private_data = kzalloc(PDSZ, GFP_KERNEL);
	PD(file)->dev = container_of(inode->i_cdev, struct mgt400_dev, cdev);
	PD(file)->minor = MINOR(inode->i_rdev);
	return 0;
}

int get_histo_from_minor(struct mgt400_dev *mdev, int minor,
		unsigned long **the_histo, int* maxentries)
{
	switch(minor){
	case  MINOR_PUSH_DATA_HISTO:
		*the_histo = mdev->push.data_histo;
		*maxentries = DATA_HISTOLEN;
		return 0;
	case  MINOR_PUSH_DESC_HISTO:
		*the_histo = mdev->push.desc_histo;
		*maxentries = DESC_HISTOLEN;
		return 0;
	case  MINOR_PULL_DATA_HISTO:
		*the_histo = mdev->pull.data_histo;
		*maxentries = DATA_HISTOLEN;
		return 0;
	case  MINOR_PULL_DESC_HISTO:
		*the_histo = mdev->pull.desc_histo;
		*maxentries = DESC_HISTOLEN;
		return 0;
	default:
		return -ENODEV;
	}
}

int mgt400_clear_histo(struct mgt400_dev *mdev, int minor)
{
	unsigned long *the_histo;
	int maxentries;
	int rc = get_histo_from_minor(mdev, minor, &the_histo, &maxentries);
	if (rc){
		return rc;
	}else{
		memset(the_histo, 0, maxentries*sizeof(unsigned));
		return 0;
	}
}
ssize_t mgt400_read_histo(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	unsigned long *the_histo;
	int maxentries;
	unsigned cursor = *f_pos;	/* f_pos counts in entries */
	int rc;

	rc = get_histo_from_minor(PD(file)->dev,
			PD(file)->minor, &the_histo, &maxentries);
	if (rc != 0){
		return rc;
	}

	if (cursor >= maxentries){
		return 0;
	}else{
		int headroom = (maxentries - cursor) * sizeof(unsigned);
		if (count > headroom){
			count = headroom;
		}
	}
	count -= count%sizeof(unsigned);
	rc = copy_to_user(buf, the_histo+cursor, count);
	if (rc){
		return -1;
	}

	*f_pos += count/sizeof(unsigned);
	return count;
}

int mgt400_release(struct inode *inode, struct file *file)
{
	kfree(PD(file));
	return 0;
}
struct file_operations mgt400_fops = {
        .owner = THIS_MODULE,
        .open = mgt400_open,
        .read = mgt400_read_histo,
        .release = mgt400_release,
};

static int mgt400_probe(struct platform_device *pdev)
{
        int rc = 0;
        struct mgt400_dev* mdev = mgt400_allocate_dev(pdev);
        dev_info(&pdev->dev, "mgt400_probe()");

        if (ndevices >= MAXDEVICES){
        	dev_err(&pdev->dev, "ERROR: MAXDEVICES:%d", MAXDEVICES);
        	rc = -ENODEV;
        	goto remove;
        }
        if (!mdev){
        	dev_err(&pdev->dev, "unable to allocate device structure\n");
        	rc = -ENODEV;
        	goto remove;
        }

        mdev->pdev->dev.id = ndevices;
        mgt400_devices[ndevices++] = mdev;

        mdev->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (mdev->mem == NULL){
        	dev_err(DEVP(mdev), "No resources found");
        	rc = -ENODEV;
        	goto remove;
        }else if (mgt400_device_tree_init(mdev)){
        	rc = -ENODEV;
        	goto remove;
        }
        if (!request_mem_region(mdev->mem->start,
                mdev->mem->end-mdev->mem->start+1, mdev->devname)) {
                dev_err(DEVP(mdev), "can't reserve i/o memory at 0x%08X\n",
                        mdev->mem->start);
                rc = -ENODEV;
                goto fail;
        }
        mdev->dev_virtaddr =
               	ioremap(mdev->mem->start, mdev->mem->end-mdev->mem->start+1);

        rc = alloc_chrdev_region(&mdev->devno, ACQ420_MINOR_0,
                		ACQ420_MINOR_MAX, mdev->devname);
        if (rc < 0) {
        	dev_err(DEVP(mdev), "unable to register chrdev\n");
                goto fail;
        }

        cdev_init(&mdev->cdev, &mgt400_fops);
        mdev->cdev.owner = THIS_MODULE;
        rc = cdev_add(&mdev->cdev, mdev->devno, MINOR_COUNT);
        if (rc < 0){
        	goto fail;
        }

        mgt400_start_buffer_counter(mdev);

        mgt400_createSysfs(&mdev->pdev->dev);
        mgt400_createDebugfs(mdev);
        mgt400wr32(mdev, ZDMA_CR, ZDMA_CR_ENABLE);
        return rc;

fail:
remove:
	kfree(mdev);
	return rc;
}

static int _mget400_remove(struct mgt400_dev* mdev){
	mgt400_stop_buffer_counter(mdev);
	return 0;
}

static int mgt400_remove(struct platform_device *pdev)
/* undo all the probe things in reverse */
{
	if (pdev->id == -1){
		return -1;
	}else{
		return _mget400_remove(mgt400_devices[pdev->id]);
	}
}

#ifdef CONFIG_OF
static struct of_device_id mgt400_of_match[] /* __devinitdata */ = {
        { .compatible = "D-TACQ,mgt400"  },
        { /* end of table */}
};
MODULE_DEVICE_TABLE(of, mgt400_of_match);
#else
#define mgt400_of_match NULL
#endif /* CONFIG_OF */



static struct platform_driver mgt400_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = mgt400_of_match,
        },
        .probe = mgt400_probe,
        .remove = mgt400_remove,
};

static struct proc_dir_entry *mgt400_proc_root;

void mgt400_module_init_proc(void)
{
	mgt400_proc_root = proc_mkdir("driver/acq400/mgt400", 0);
}
void mgt400_module_remove_proc(void)
{
	remove_proc_entry("driver/acq400/mgt400", NULL);
}

static void __exit mgt400_exit(void)
{
	platform_driver_unregister(&mgt400_driver);
	mgt400_module_remove_proc();
}

static int __init mgt400_init(void)
{
        int status;

	printk("D-TACQ MGT400 Comms Module Driver %s\n", REVID);
	kt_period = ktime_set(0, 10000000);
	mgt400_module_init_proc();
        status = platform_driver_register(&mgt400_driver);

        return status;
}

module_init(mgt400_init);
module_exit(mgt400_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ ACQ400_FMC Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
