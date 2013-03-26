/* ------------------------------------------------------------------------- */
/* ACQ420_FMC_drv.c  ACQ420 FMC D-TACQ DRIVER		                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2012 Craig Noble, D-TACQ Solutions Ltd                    *
 *                      <craig dot noble at D hyphen TACQ dot com>           *
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



#include "acq420FMC.h"
#include "hbm.h"

#define REVID "0.89"

/* Define debugging for use during our driver bringup */
#undef PDEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO fmt, ## args)



/* we _shouldn't need any globals, but right now there's no obvious way round */

int ndevices;
module_param(ndevices, int, 0444);
MODULE_PARM_DESC(ndevices, "number of devices found in probe");

int adc_18b = 1;
module_param(adc_18b, int, 0444);
MODULE_PARM_DESC(adc_18b, "set TRUE on load if 18 bit devices fitted [1]");

int data_32b = 0;
module_param(data_32b, int, 0444);
MODULE_PARM_DESC(data_32b, "set TRUE on load if 32 bit data required [0]");

int ADC_CONV_TIME = ALG_ADC_CONV_TIME_DEF;
module_param(ADC_CONV_TIME, int, 0444);
MODULE_PARM_DESC(ADC_CONV_TIME, "hardware tweak, change at load only");

int nbuffers = 16;
module_param(nbuffers, int, 0444);
MODULE_PARM_DESC(nbuffers, "number of capture buffers");

int bufferlen = 0x10000;
module_param(bufferlen, int, 0444);
MODULE_PARM_DESC(bufferlen, "length of capture buffer");

int hitide = HITIDE;
module_param(hitide, int, 0644);
MODULE_PARM_DESC(hitide, "hitide value (words)");

int lotide = HITIDE/2;
module_param(lotide, int, 0644);
MODULE_PARM_DESC(lotide, "lotide value (words)");

/* driver supports multiple devices.
 * ideally we'd have no globals here at all, but it works, for now
 */
#define MAXDEVICES 6
struct acq420_dev* acq420_devices[MAXDEVICES];



// @@todo pgm: crude
const char* acq420_names[] = { "0", "1", "2", "3", "4", "5" };
const char* acq420_devnames[] = {
	"acq420.0", "acq420.1", "acq420.2",
	"acq420.3", "acq420.4", "acq420.5",
};

int acq420_release(struct inode *inode, struct file *file);

void acq420wr32(struct acq420_dev *adev, int offset, u32 value)
{
	iowrite32(value, adev->dev_virtaddr + offset);
}

u32 acq420rd32(struct acq420_dev *adev, int offset)
{
	return ioread32(adev->dev_virtaddr + offset);
}

static u32 acq420_get_fifo_samples(struct acq420_dev *adev)
{
	return acq420rd32(adev, ALG_SAMPLES);
}

static void acq420_reset_fifo(struct acq420_dev *adev)
/* Raise and Release reset */
{
	u32 ctrl = acq420rd32(adev, ALG_CTRL);

	acq420wr32(adev, ALG_CTRL, ctrl | ALG_CTRL_RESETALL);
	acq420wr32(adev, ALG_CTRL, ctrl);
}

static void acq420_enable_fifo(struct acq420_dev *adev)
{
	u32 ctrl = acq420rd32(adev, ALG_CTRL);
	if (adev->ramp_en){
		ctrl |= ALG_CTRL_RAMP_ENABLE;
	}
	acq420wr32(adev, ALG_CTRL, ctrl|ALG_CTRL_ENABLE_ALL);
}

static void acq420_disable_fifo(struct acq420_dev *adev)
{
	u32 ctrl = acq420rd32(adev, ALG_CTRL);
	ctrl &= ~ALG_CTRL_RAMP_ENABLE;
	acq420wr32(adev, ALG_CTRL, ctrl & ~ALG_CTRL_ENABLE_ALL);
}


static void acq420_enable_interrupt(struct acq420_dev *adev)
{
	u32 int_ctrl = acq420rd32(adev, ALG_INT_CTRL);
	acq420wr32(adev, ALG_HITIDE, 	hitide);
	acq420wr32(adev, ALG_INT_CTRL,	int_ctrl|0x1);
}

static void acq420_disable_interrupt(struct acq420_dev *adev)
{
	//u32 control = acq420rd32(adev, ALG_INT_CTRL);
	//u32 status  = acq420rd32(adev, ALG_INT_STAT);
	// printk("Interrupt status is 0x%08x\n", status);
	//control &= ~0x1;
	// printk("New interrupt enable is 0x%08x\n", control);

	acq420wr32(adev, ALG_INT_CTRL, 0x0);
}

static u32 acq420_get_interrupt(struct acq420_dev *adev)
{
	return acq420rd32(adev, ALG_INT_STAT);
}

static void acq420_clear_interrupt(struct acq420_dev *adev)
{
	acq420wr32(adev, ALG_INT_STAT, acq420_get_interrupt(adev));
}

static void acq420_init_defaults(struct acq420_dev *adev)
{
	acq420wr32(adev, ALG_ADC_CONV_TIME, ADC_CONV_TIME);
	acq420wr32(adev, ALG_ADC_OPTS,
			(adc_18b? ALG_ADC_OPTS_IS_18B: 0)|
			(data_32b? ALG_ADC_OPTS_32B_data: 0));
}

static void acq420_clear_histo(struct acq420_dev *adev)
{
	memset(adev->fifo_histo, 0, DATA_FIFO_SZ*sizeof(u32));
}
/*
static void acq420_force_interrupt(int interrupt)
{
	u32 status;
	status = ioread32(acq420_dev->dev_virtaddr + ALG_INT_FORCE);
	iowrite32((interrupt), acq420_dev->dev_virtaddr + ALG_INT_FORCE);
}
*/

void getEmpty(struct acq420_dev* adev)
{
	if (!list_empty(&adev->EMPTIES)){
		adev->cursor.hb = list_first_entry(
				&adev->EMPTIES, struct HBM, list);
		list_del(&adev->cursor.hb->list);
		adev->cursor.hb->bstate = BS_FILLING;
		adev->cursor.offset = 0;
	} else {
		dev_warn(&adev->pdev->dev, "get Empty: Q is EMPTY!\n");
	}
}

void putFull(struct acq420_dev* adev)
{
	adev->cursor.hb->bstate = BS_FULL;
	list_add_tail(&adev->cursor.hb->list, &adev->REFILLS);
}
int getHeadroom(struct acq420_dev* adev)
{
	if (!adev->oneshot){
		if (adev->cursor.hb == 0){
			getEmpty(adev);
		} else if (adev->cursor.offset >= adev->cursor.hb->len){
			putFull(adev);
			getEmpty(adev);
		}
	}
	return adev->cursor.hb->len - adev->cursor.offset;
}



/* File operations */
int acq420_open_main(struct inode *inode, struct file *file)
{
        int rc = 0;
        struct acq420_dev* dev = ACQ420_DEV(file);

        if (mutex_lock_interruptible(&dev->mutex)) {
                return -ERESTARTSYS;
        }

        /* We're only going to allow one write at a time, so manage that via
         * reference counts
         */
        switch (file->f_flags & O_ACCMODE) {
        case O_RDONLY:
                break;
        case O_WRONLY:
                if (dev->writers || dev->busy) {
                        rc = -EBUSY;
                        goto out;
                } else {
                        dev->writers++;
                }
                break;
        case O_RDWR:
        default:
                if (dev->writers || dev->busy) {
                        rc = -EBUSY;
                        goto out;
                } else {
                        dev->writers++;
                }
        }

        dev->opens++;

out:
        mutex_unlock(&dev->mutex);
        return rc;
}

static int acq420_dma_open(struct inode *inode, struct file *file)
{
	return 0;
}

int acq420_dma_mmap_host(struct file* file, struct vm_area_struct* vma)
/**
 * mmap the host buffer.
 */
{
	struct acq420_dev* adev = ACQ420_DEV(file);
	int ibuf = BUFFER(PD(file)->minor);
	struct HBM *hb = adev->hb[ibuf];
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = hb->len;
	unsigned pfn = hb->pa >> PAGE_SHIFT;

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
int acq420_open_hb(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_fops_dma = {
		.open = acq420_dma_open,
		.mmap = acq420_dma_mmap_host
	};
	file->f_op = &acq420_fops_dma;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}



ssize_t acq420_histo_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	unsigned *the_histo = ACQ420_DEV(file)->fifo_histo;
	int maxentries = DATA_FIFO_SZ+1;
	unsigned cursor = *f_pos;	/* f_pos counts in entries */
	int rc;

	if (cursor >= maxentries){
		return 0;
	}else{
		int headroom = (maxentries - cursor) * sizeof(unsigned);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, the_histo+cursor, count);
	if (rc){
		return -1;
	}

	*f_pos += count/sizeof(unsigned);
	return count;
}

int acq420_open_histo(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_fops_histo = {
			.read = acq420_histo_read,
			.release = acq420_release
	};
	file->f_op = &acq420_fops_histo;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}
int acq420_open(struct inode *inode, struct file *file)
{

        struct acq420_dev *dev;
        int minor;

        file->private_data = kzalloc(PDSZ, GFP_KERNEL);

        PD(file)->dev = dev = container_of(inode->i_cdev, struct acq420_dev, cdev);
        PD(file)->minor = minor = MINOR(inode->i_rdev);

        dev_dbg(&dev->pdev->dev, "hello: minor:%d\n", minor);

        if (minor >= ACQ420_MINOR_BUF && minor <= ACQ420_MINOR_BUF2){
        	return acq420_open_hb(inode, file);
        } else if (minor >= ACQ420_MINOR_CHAN && minor <= ACQ420_MINOR_CHAN2){
        	return -ENODEV;  	// @@todo maybe later0
        } else {
        	switch(minor){
        	case ACQ420_MINOR_CONTINUOUS:
        		dev_warn(&dev->pdev->dev, "@@todo");
        		return -ENODEV;
        	case ACQ420_MINOR_HISTO:
        		return acq420_open_histo(inode, file);
        	case ACQ420_MINOR_0:
        		return acq420_open_main(inode, file);
        	default:
        		return -ENODEV;
        	}

        }
}

int acq420_release(struct inode *inode, struct file *file)
{
        struct acq420_dev *dev = ACQ420_DEV(file);

        if (mutex_lock_interruptible(&dev->mutex)) {
                return -EINTR;
        }

        /* Manage writes via reference counts */
        switch (file->f_flags & O_ACCMODE) {
        case O_RDONLY:
                break;
        case O_WRONLY:
                dev->writers--;
                break;
        case O_RDWR:
        default:
                dev->writers--;
        }

        dev->closes++;

        mutex_unlock(&dev->mutex);

        kfree(PD(file));
        return 0;
}

static void acq420_fault_callback(unsigned int channel,
        unsigned int fault_type,
        unsigned int fault_address,
        void *data)
{
        struct acq420_dev *dev = data;

        dev_err(&dev->pdev->dev,
                "DMA fault type 0x%08x at address 0x%0x on channel %d\n",
                fault_type, fault_address, channel);

        dev->errors++;
        acq420_reset_fifo(dev);
        dev->busy = 0;
        wake_up_interruptible(&dev->waitq);
}

static void acq420_done_callback(unsigned int channel, void *data)
{
        struct acq420_dev *dev = data;

	   // printk("DMA Done!");

        dev->bytes_written += dev->count;
        dev->busy = 0;

        wake_up_interruptible(&dev->waitq);
}

ssize_t acq420_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq420_dev *adev = ACQ420_DEV(file);
	struct HBM hbm = {};

	// u64 checkval= 0;
	int rc = 0;

	if (mutex_lock_interruptible(&adev->mutex)) {
		return -EINTR;
	}

	adev->oneshot = 1;
	adev->reads++;
	adev->count = count;
	adev->this_count = 0;
	adev->busy = 1;

	/** @@todo pgm: in a streaming system, preallocate streaming dma buffers (not coherent) */
	/* Allocate a DMA buffer for the transfer */

	hbm.va = dma_zalloc_coherent(DEVP(adev), count, &hbm.pa, GFP_KERNEL);
	if (!hbm.va) {
		dev_err(&adev->pdev->dev,
			"coherent DMA buffer allocation failed\n");
		rc = -ENOMEM;
		goto fail_buffer;
	}
	hbm.len = count;
	adev->cursor.hb = &hbm;
	adev->cursor.offset = 0;

	if (request_dma(adev->dma_channel, MODULE_NAME)) {
		dev_err(DEVP(adev), "unable to alloc DMA channel %d\n",
			adev->dma_channel);
		rc = -EBUSY;
		goto fail_client_data;
	}

	acq420_clear_histo(adev);
	acq420_enable_fifo(adev);
	acq420_reset_fifo(adev);

	adev->DMA_READY = 1;

	/*Wait for FIFO to fill*/
	acq420_enable_interrupt(adev);

	/* Kick off the DMA */
	mutex_unlock(&adev->mutex);

	do {
		if (wait_event_interruptible(adev->waitq, adev->busy == 0)){
			rc = -EINTR;
			break;
		}
	} while(adev->this_count < count);

	acq420_reset_fifo(adev);
	acq420_disable_fifo(adev);

	copy_to_user(buf, hbm.va, count);

	free_dma(adev->dma_channel);

	dma_free_coherent(DEVP(adev), count, hbm.va, hbm.pa);
	adev->cursor.hb = 0;
	return count;

fail_client_data:
	dma_free_coherent(DEVP(adev), count, hbm.va, hbm.pa);

fail_buffer:
	mutex_unlock(&adev->mutex);
	return rc;
}

ssize_t acq420_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
#if 0
        struct acq420_dev *dev = ACQ420_DEV(file);
        size_t transfer_size;

        int retval = 0;

        if (mutex_lock_interruptible(&dev->mutex)) {
                return -EINTR;
        }

        dev->writes++;

        transfer_size = count;
        if (count > dev->fifo_depth) {
                transfer_size = dev->fifo_depth;
        }

        /* Allocate a DMA buffer for the transfer */
        dev->buffer_v_addr = dma_alloc_coherent(&dev->pdev->dev, transfer_size,
                &dev->buffer_d_addr, GFP_KERNEL);
        if (!dev->buffer_v_addr) {
                dev_err(&dev->pdev->dev,
                        "coherent DMA buffer allocation failed\n");
                retval = -ENOMEM;
                goto fail_buffer;
        }

        PDEBUG("dma buffer alloc - d @0x%0x v @0x%0x\n",
                (u32)dev->buffer_d_addr, (u32)dev->buffer_v_addr);

        if (request_dma(dev->dma_channel, MODULE_NAME)) {
                dev_err(&dev->pdev->dev,
                        "unable to alloc DMA channel %d\n",
                        dev->dma_channel);
                retval = -EBUSY;
                goto fail_client_data;
        }

        dev->busy = 1;
        dev->count = transfer_size;

        set_dma_mode(dev->dma_channel, DMA_MODE_WRITE);
        set_dma_addr(dev->dma_channel, dev->buffer_d_addr);
        set_dma_count(dev->dma_channel, transfer_size);
        set_pl330_client_data(dev->dma_channel, dev->client_data);
        set_pl330_done_callback(dev->dma_channel,
                acq420_done_callback, dev);
        set_pl330_fault_callback(dev->dma_channel,
                acq420_fault_callback, dev);
        set_pl330_incr_dev_addr(dev->dma_channel, 0);

        /* Load our DMA buffer with the user data */
        copy_from_user(dev->buffer_v_addr, buf, transfer_size);

        acq420_reset_fifo(dev);
        /* Kick off the DMA */
        enable_dma(dev->dma_channel);

        mutex_unlock(&dev->mutex);

        wait_event_interruptible(dev->waitq, dev->busy == 0);

        /* Deallocate the DMA buffer and free the channel */
        free_dma(dev->dma_channel);

        dma_free_coherent(&dev->pdev->dev, dev->count, dev->buffer_v_addr,
                dev->buffer_d_addr);

        PDEBUG("dma write %d bytes\n", transfer_size);

        return transfer_size;

fail_client_data:
        dma_free_coherent(&dev->pdev->dev, transfer_size, dev->buffer_v_addr,
                dev->buffer_d_addr);
fail_buffer:
        mutex_unlock(&dev->mutex);
        return retval;
#else
        return -EINVAL;
#endif
}

static void init_dmac_consts(struct acq420_dev *adev)
{
	set_dma_mode(adev->dma_channel, DMA_MODE_READ);
	set_pl330_client_data(adev->dma_channel, adev->client_data);
	set_pl330_done_callback(adev->dma_channel, acq420_done_callback, adev);
	set_pl330_fault_callback(adev->dma_channel, acq420_fault_callback, adev);
	set_pl330_incr_dev_addr(adev->dma_channel, 0);
}

static void add_fifo_histo(struct acq420_dev *adev, u32 status)
{
	adev->fifo_histo[STATUS_TO_HISTO(status)]++;
}
static irqreturn_t fire_dma(int irq, void *dev_id)
{
	struct acq420_dev *adev = (struct acq420_dev *)dev_id;
	u32 status = acq420_get_fifo_samples(adev);

	do {
		int headroom = min(getHeadroom(adev), MAXDMA);
		int bytes = status * 4;

		bytes = min(bytes, headroom);

		if (headroom == 0){
			dev_info(DEVP(adev), "headroom==0, quit count:%d\n",
					adev->this_count);
			return IRQ_HANDLED;
		}

		adev->busy = 1;
		init_dmac_consts(adev);		/* @@todo move this out the loop */
		set_dma_addr(adev->dma_channel, adev->cursor.hb->pa + adev->cursor.offset);
		set_dma_count(adev->dma_channel, (size_t)bytes);
		enable_dma(adev->dma_channel);

		add_fifo_histo(adev, status);

		/* Wait for Completion*/
		wait_event_interruptible(adev->waitq, adev->busy == 0);
		adev->cursor.offset += bytes;
		if (adev->oneshot){
			adev->this_count += bytes;
		}
		status = acq420_get_fifo_samples(adev);
	} while(status >= lotide);

	acq420_enable_interrupt(adev);

	return IRQ_HANDLED;
}

static irqreturn_t acq420_int_handler(int irq, void *dev_id)
{
	irqreturn_t irq_status;
	struct acq420_dev *dev = (struct acq420_dev *)dev_id;
	u32 status = acq420_get_interrupt(dev);
	// u32 samples = acq420_get_fifo_status();

	iowrite32((0xCAFEBABE), dev->dev_virtaddr);

	// printk("FIFO contains %d samples\n", samples);

	if (status == 0x1 && dev->DMA_READY == 1) {
		dev->DMA_READY = 0;
	}
	acq420_disable_interrupt(dev);
	acq420_clear_interrupt(dev);
	// printk("Got an IRQ!\n");

	irq_status = IRQ_WAKE_THREAD;
	return irq_status;
}

struct file_operations acq420_fops = {
        .owner = THIS_MODULE,
        .read = acq420_read,
        .write = acq420_write,
        .open = acq420_open,
        .release = acq420_release
};




#ifdef CONFIG_OF
static struct of_device_id xfifodma_of_match[] __devinitdata = {
        { .compatible = "D-TACQ,ACQ420_FMC", },
        { /* end of table */}
};
MODULE_DEVICE_TABLE(of, xfifodma_of_match);
#else
#define xfifodma_of_match NULL
#endif /* CONFIG_OF */

static int acq420_init_pl330(struct acq420_dev* acq420_dev)
{
        acq420_dev->client_data =
        	kzalloc(sizeof(struct pl330_client_data), GFP_KERNEL);

        if (!acq420_dev->client_data) {
               return -1;
        }

        acq420_dev->client_data->dev_addr =
                acq420_dev->dev_physaddr + AXI_FIFO;
        acq420_dev->client_data->dev_bus_des.burst_size = 4;
        acq420_dev->client_data->dev_bus_des.burst_len =
                acq420_dev->burst_length;
        acq420_dev->client_data->mem_bus_des.burst_size = 4;
        acq420_dev->client_data->mem_bus_des.burst_len =
                acq420_dev->burst_length;
        return 0;
}

static void acq420_device_tree_init(struct acq420_dev* acq420_dev)
{
	struct platform_device *pdev = acq420_dev->pdev;
        if (pdev->dev.of_node) {
                if (of_property_read_u32(pdev->dev.of_node, "dma-channel",
                        &acq420_dev->dma_channel) < 0) {
                        dev_warn(&pdev->dev,
                                "DMA channel unspecified - assuming 0\n");
                        acq420_dev->dma_channel = 0;
                }
                dev_info(&pdev->dev,
                        "read DMA channel is %d\n", acq420_dev->dma_channel);
                if (of_property_read_u32(pdev->dev.of_node, "fifo-depth",
                        &acq420_dev->fifo_depth) < 0) {
                        dev_warn(&pdev->dev,
                                "depth unspecified, assuming 0xffffffff\n");
                        acq420_dev->fifo_depth = 0xffffffff;
                }
                dev_info(&pdev->dev,
                        "DMA fifo depth is %d\n", acq420_dev->fifo_depth);
                if (of_property_read_u32(pdev->dev.of_node, "burst-length",
                        &acq420_dev->burst_length) < 0) {
                        dev_warn(&pdev->dev,
                                "burst length unspecified - assuming 1\n");
                        acq420_dev->burst_length = 1;
                }
                dev_info(&pdev->dev,
                        "DMA burst length is %d\n",
                        acq420_dev->burst_length);
        }
}

static struct acq420_dev* acq420_allocate_dev(struct platform_device *pdev)
/* Allocate and init a private structure to manage this device */
{
	struct acq420_dev* adev = kzalloc(sizeof(struct acq420_dev), GFP_KERNEL);
        if (adev == NULL) {
                return NULL;
        }
        init_waitqueue_head(&adev->waitq);

        adev->irq = ADC_HT_INT; /* @@todo should come from device tree? */
        adev->pdev = pdev;
        mutex_init(&adev->mutex);
        adev->fifo_histo = kzalloc(DATA_FIFO_SZ*sizeof(u32), GFP_KERNEL);
        return adev;
}

static int acq420_remove(struct platform_device *pdev);

static int acq420_probe(struct platform_device *pdev)
{
        int status;
        struct resource *acq420_resource;
        struct acq420_dev* adev = acq420_allocate_dev(pdev);

        if (!adev){
        	dev_err(&pdev->dev,
        			"unable to allocate device structure\n");
        	return -ENOMEM;
        }
        pdev->id = ndevices;
        /* Get our platform device resources */
        dev_info(&pdev->dev, "id:%d We have %d resources\n", pdev->id, pdev->num_resources);
        acq420_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (acq420_resource == NULL) {
                dev_err(&pdev->dev, "No resources found\n");
                return -ENODEV;
        }

        acq420_device_tree_init(adev);

        status = alloc_chrdev_region(&adev->devno,
        		ACQ420_MINOR_0, ACQ420_MINOR_MAX, acq420_devnames[ndevices]);
        //status = register_chrdev_region(acq420_dev->devno, 1, MODULE_NAME);
        if (status < 0) {
                dev_err(&pdev->dev, "unable to register chrdev\n");
                goto fail;
        }

        /* Register with the kernel as a character device */
        cdev_init(&adev->cdev, &acq420_fops);
        adev->cdev.owner = THIS_MODULE;
        adev->dev_physaddr = acq420_resource->start;
        adev->dev_addrsize = acq420_resource->end -
                acq420_resource->start + 1;
        if (!request_mem_region(adev->dev_physaddr,
                adev->dev_addrsize, MODULE_NAME)) {
                dev_err(&pdev->dev, "can't reserve i/o memory at 0x%08X\n",
                        adev->dev_physaddr);
                status = -ENODEV;
                goto fail;
        }
        adev->dev_virtaddr =
        	ioremap(adev->dev_physaddr, adev->dev_addrsize);
        dev_dbg(&pdev->dev, "acq420: mapped 0x%0x to 0x%0x\n",
        	adev->dev_physaddr, (unsigned int)adev->dev_virtaddr);

        if (acq420_init_pl330(adev)){
        	dev_err(&pdev->dev, "can't allocate PL330 client data\n");
        	status = -1;
        	goto fail;
        }
        status = cdev_add(&adev->cdev, adev->devno, ACQ420_MINOR_MAX);
        acq420_init_proc(adev, ndevices);

        status = devm_request_threaded_irq(
        		&adev->pdev->dev, adev->irq,
        		acq420_int_handler, fire_dma,
        		IRQF_SHARED, acq420_devnames[adev->pdev->id],
        		adev);

	if (status)	{
		printk("%s unable to secure IRQ%d\n", "ACQ420", 
			adev->irq);
		goto fail;
	}

	if (hbm_allocate(&pdev->dev,
			nbuffers, bufferlen, &adev->buffers, DMA_FROM_DEVICE)){
		dev_err(&pdev->dev, "failed to allocate buffers");
		goto fail;
	}else{
		struct HBM* cursor;
		int ix = 0;
		adev->hb = kmalloc(nbuffers*sizeof(struct HBM*), GFP_KERNEL);
		list_for_each_entry(cursor, &adev->buffers, list){
			WARN_ON(cursor->ix != ix);
			adev->hb[cursor->ix] = cursor;
			ix++;
		}
		dev_info(&pdev->dev, "setting nbuffers %d\n", ix);
		adev->nbuffers = ix;
	}
        //acq420_reset_fifo();
        dev_info(&pdev->dev, "added ACQ420 FMC successfully\n");


        acq420_devices[ndevices++] = adev;
        acq420_createSysfs(&pdev->dev);
        acq420_init_defaults(adev);

        return 0;

        fail:
        	dev_err(&pdev->dev, "Bailout!\n");
        acq420_remove(pdev);
        return status;
}

static int acq420_remove(struct platform_device *pdev)
/* undo all the probe things in reverse */
{
	if (pdev->id == -1){
		return -1;
	}else{
		struct acq420_dev* acq420_dev = acq420_devices[pdev->id];

		if (acq420_dev == 0){
			return -1;
		}
		hbm_free(&pdev->dev, &acq420_dev->buffers);
		acq420_delSysfs(&acq420_dev->pdev->dev);
		acq420_del_proc(acq420_dev);
		dev_dbg(&pdev->dev, "cdev_del %p\n", &acq420_dev->cdev);
		cdev_del(&acq420_dev->cdev);
		unregister_chrdev_region(acq420_dev->devno, 1);

		/* Unmap the I/O memory */
		if (acq420_dev->dev_virtaddr) {
			iounmap(acq420_dev->dev_virtaddr);
			release_mem_region(acq420_dev->dev_physaddr,
					acq420_dev->dev_addrsize);
		}
		/* Free the PL330 buffer client data descriptors */
		if (acq420_dev->client_data) {
			kfree(acq420_dev->client_data);
		}

		kfree(acq420_dev);

		return 0;
	}
}
static struct platform_driver acq420_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = xfifodma_of_match,
        },
        .probe = acq420_probe,
        .remove = acq420_remove,
};

static void __exit acq420_exit(void)
{
	platform_driver_unregister(&acq420_driver);
	acq420_module_remove_proc();

}

static int __init acq420_init(void)
{
        int status;

	printk("D-TACQ ACQ420 FMC Driver %s\n", REVID);
	acq420_module_init_proc();
        status = platform_driver_register(&acq420_driver);

        return status;
}

module_init(acq420_init);
module_exit(acq420_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ ACQ420_FMC Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);




