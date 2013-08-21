/* ------------------------------------------------------------------------- */
/* ACQ420_FMC_drv.c  ACQ420 FMC D-TACQ DRIVER		                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2013 Peter Milne, D-TACQ Solutions Ltd                    *
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



#include "acq420FMC.h"
#include "hbm.h"

#include <linux/debugfs.h>
#include <linux/poll.h>
#define REVID "2.009"

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

int adc_conv_time = ADC_CONV_TIME_1000;
module_param(adc_conv_time, int, 0444);
MODULE_PARM_DESC(adc_conv_time, "hardware tweak, change at load only");

int nbuffers = 16;
module_param(nbuffers, int, 0444);
MODULE_PARM_DESC(nbuffers, "number of capture buffers");

int bufferlen = 0x10000;
module_param(bufferlen, int, 0444);
MODULE_PARM_DESC(bufferlen, "length of capture buffer");

int hitide = HITIDE;
module_param(hitide, int, 0644);
MODULE_PARM_DESC(hitide, "hitide value (words)");

int lotide = HITIDE-4;
module_param(lotide, int, 0644);
MODULE_PARM_DESC(lotide, "lotide value (words)");

int run_buffers = 0;
module_param(run_buffers, int, 0644);
MODULE_PARM_DESC(run_buffers, "#buffers to process in continuous (0: infinity)");

int FIFERR = ADC_FIFO_STA_ERR;
module_param(FIFERR, int, 0644);
MODULE_PARM_DESC(FIFERR, "fifo status flags considered ERROR");

/* driver supports multiple devices.
 * ideally we'd have no globals here at all, but it works, for now
 */
#define MAXDEVICES 6
struct acq420_dev* acq420_devices[MAXDEVICES];

#define DMA_NS_MAX	10
int dma_ns_lines[DMA_NS_MAX];
int dma_ns[DMA_NS_MAX];
int dma_ns_num = DMA_NS_MAX;
module_param_array(dma_ns, int, &dma_ns_num, 0444);
module_param_array(dma_ns_lines, int, &dma_ns_num, 0444);


// @@todo pgm: crude
const char* acq420_names[] = { "0", "1", "2", "3", "4", "5" };
const char* acq420_devnames[] = {
	"acq420.0", "acq420.1", "acq420.2",
	"acq420.3", "acq420.4", "acq420.5",
};


struct dentry* acq420_debug_root;

int acq420_release(struct inode *inode, struct file *file);

void acq420wr32(struct acq420_dev *adev, int offset, u32 value)
{
	dev_dbg(DEVP(adev), "acq420wr32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, value);

	iowrite32(value, adev->dev_virtaddr + offset);
}

u32 acq420rd32(struct acq420_dev *adev, int offset)
{
	u32 rc = ioread32(adev->dev_virtaddr + offset);
	dev_dbg(DEVP(adev), "acq420rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	return rc;
}
static void acq420_init_format(struct acq420_dev *adev)
{
	acq420wr32(adev, ADC_FORMAT,
			(adev->adc_18b? ADC_OPTS_IS_18B: 0)|
			(adev->data32? ADC_OPTS_32B_data: 0));
}
static void acq420_init_defaults(struct acq420_dev *adev)
{
	acq420wr32(adev, ADC_CONV_TIME, adc_conv_time);
	adev->data32 = data_32b;
	adev->adc_18b = adc_18b;
	acq420_init_format(adev);
	acq420wr32(adev, ADC_CTRL, ADC_CTRL_MODULE_EN);
	adev->nchan_enabled = 4;
	adev->word_size = adev->data32? 4: 2;
}

static void acq435_init_defaults(struct acq420_dev *adev)
{
	dev_info(DEVP(adev), "acq435_init_defaults()");
	adev->data32 = 1;
	adev->nchan_enabled = 32;
	adev->word_size = 4;
	hitide = 512;
	lotide = hitide - 4;
	acq420wr32(adev, ADC_CLKDIV, 16);
	acq420wr32(adev, ADC_CTRL, ADC_CTRL_MODULE_EN);
}
static u32 acq420_get_fifo_samples(struct acq420_dev *adev)
{
	return acq420rd32(adev, ADC_FIFO_SAMPLES);
}

static u32 acq420_samples2bytes(struct acq420_dev *adev, u32 samples)
{
	return samples * adev->nchan_enabled * adev->word_size;
}

static void acq420_reset_fifo(struct acq420_dev *adev)
/* Raise and Release reset */
{
	u32 ctrl = acq420rd32(adev, ADC_CTRL);

	acq420wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_RST_ALL);
	acq420wr32(adev, ADC_CTRL, ctrl);
}

static void acq420_enable_fifo(struct acq420_dev *adev)
{
	u32 ctrl = acq420rd32(adev, ADC_CTRL);
	if (adev->ramp_en){
		ctrl |= ADC_CTRL_RAMP_EN;
	}else{
		ctrl &= ~ADC_CTRL_RAMP_EN;
		acq420_init_format(adev);
	}
	acq420wr32(adev, ADC_CTRL, ctrl|ADC_CTRL_ENABLE_ALL);
}

static void acq420_disable_fifo(struct acq420_dev *adev)
{
	u32 ctrl = acq420rd32(adev, ADC_CTRL);
	acq420wr32(adev, ADC_CTRL, ctrl & ~ADC_CTRL_ENABLE_ALL);
}


static void acq420_enable_interrupt(struct acq420_dev *adev)
{
	u32 int_ctrl = acq420rd32(adev, ADC_INT_CSR);
	acq420wr32(adev, ADC_HITIDE, 	hitide);
	acq420wr32(adev, ADC_INT_CSR,	int_ctrl|0x1);
}

static void acq420_disable_interrupt(struct acq420_dev *adev)
{
	//u32 control = acq420rd32(adev, ALG_INT_CTRL);
	//u32 status  = acq420rd32(adev, ALG_INT_STAT);
	// printk("Interrupt status is 0x%08x\n", status);
	//control &= ~0x1;
	// printk("New interrupt enable is 0x%08x\n", control);

	acq420wr32(adev, ADC_INT_CSR, 0x0);
}

static u32 acq420_get_interrupt(struct acq420_dev *adev)
{
	return acq420rd32(adev, ADC_INT_CSR);
}

static void acq420_clear_interrupt(struct acq420_dev *adev, u32 status)
{
	/** @@todo: how to INTACK?
	acq420wr32(adev, ALG_INT_CSR, acq420_get_interrupt(adev));
 *
 */
	/* peter assumes R/C ie write a 1 to clear the bit .. */
	acq420wr32(adev, ADC_INT_CSR, status);
}


static void acq420_clear_histo(struct acq420_dev *adev)
{
	memset(adev->fifo_histo, 0, FIFO_HISTO_SZ*sizeof(u32));
}

int acq420_isFifoError(struct acq420_dev *adev)
{
	u32 fifsta = acq420rd32(adev, ADC_FIFO_STA);
	int err = (fifsta&FIFERR) != 0;
	if (err){
		dev_warn(DEVP(adev), "FIFERR mask:%08x actual:%08x\n",
				FIFERR, fifsta);
	}
	return err;
}

void acq435_onStart(struct acq420_dev *adev)
{
	u32 ctrl = acq420rd32(adev, ADC_CTRL);

	dev_info(DEVP(adev), "acq435_onStart()");
	ctrl = 0;

	acq420wr32(adev, ADC_CTRL, ctrl |= ADC_CTRL_MODULE_EN);

	if (adev->ramp_en){
		ctrl |= ADC_CTRL_RAMP_EN;
	}else{
		ctrl &= ~ADC_CTRL_RAMP_EN;
	}
	// set mode (assume done)
	// set clkdiv (assume done)
	// set timing bus (assume done)

	acq420wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_FIFO_RST);
	acq420wr32(adev, ADC_CTRL, ctrl);
	acq420wr32(adev, ADC_CTRL, ctrl  |= ADC_CTRL_ADC_EN);
	acq420wr32(adev, ADC_CTRL, ctrl  |= ADC_CTRL_FIFO_EN);
	/** clear FIFO flags .. workaround hw bug */
	acq420wr32(adev, ADC_FIFO_STA, ADC_FIFO_FLAGS);
	if (!adev->is_slave){
		acq420wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_ADC_RST);
		acq420wr32(adev, ADC_CTRL, ctrl);
	}

	if (ctrl&ADC_CTRL_RAMP_EN){
		dev_info(DEVP(adev), "acq435_onStart() RAMP MODE");
	}
}
void acq420_onStart(struct acq420_dev *adev)
{
	dev_info(DEVP(adev), "acq420_onStart()");
	acq420_enable_fifo(adev);
	acq420_reset_fifo(adev);
	/** clear FIFO flags .. workaround hw bug */
	acq420wr32(adev, ADC_FIFO_STA, ADC_FIFO_FLAGS);
}
static void acq400_getID(struct acq420_dev *adev)
{
	u32 modid = acq420rd32(adev, MOD_ID);
	adev->mod_id = modid >> MOD_ID_TYPE_SHL;

	dev_info(DEVP(adev), "Device MODID %08x", modid);
}
/*
static void acq420_force_interrupt(int interrupt)
{
	u32 status;
	status = ioread32(acq420_dev->dev_virtaddr + ALG_INT_FORCE);
	iowrite32((interrupt), acq420_dev->dev_virtaddr + ALG_INT_FORCE);
}
*/

unsigned long long t0;
int ins;

/** @@todo ... hook to ZYNQ timers how? */

extern unsigned long long otick(void);
extern unsigned delta_nsec(unsigned long long t0, unsigned long long t1);

#define DMA_NS_INIT \
	do { 					\
		ins=0; t0 = otick(); 		\
	} while(0)

#define DMA_NS \
	do { 						\
		dma_ns_lines[ins] = __LINE__; 		\
		dma_ns[ins] = delta_nsec(t0, otick()); 	\
		ins++; 					\
		BUG_ON(ins >= DMA_NS_MAX);		\
	} while(0)


void timer_test(void)
{
	DMA_NS_INIT;

	while(ins+1 < DMA_NS_MAX){
		DMA_NS;
	}
}

dma_cookie_t
dma_async_memcpy_pa_to_buf(
		struct dma_chan *chan, struct HBM *dest,
		dma_addr_t dma_src, size_t len)
{
	struct dma_device *dev = chan->device;
	struct dma_async_tx_descriptor *tx;
	unsigned long flags = DMA_SRC_NO_INCR | DMA_CTRL_ACK |
				DMA_COMPL_SRC_UNMAP_SINGLE |
				DMA_COMPL_DEST_UNMAP_SINGLE;

	DMA_NS;
	tx = dev->device_prep_dma_memcpy(chan, dest->pa, dma_src, len, flags);

	DMA_NS;
	if (!tx) {
		return -ENOMEM;
	} else{
		dma_cookie_t cookie;
		tx->callback = NULL;
		cookie = tx->tx_submit(tx);

		DMA_NS;

		preempt_disable();
		__this_cpu_add(chan->local->bytes_transferred, len);
		__this_cpu_inc(chan->local->memcpy_count);
		preempt_enable();
		return cookie;
	}
}


int cpsc_dma_memcpy(struct acq420_dev* adev, struct HBM* dest, u32 src, size_t len)
{
	dma_cookie_t cookie;
	DMA_NS_INIT;
	DMA_NS;
	if (adev->dma_chan == 0){
		dev_err(DEVP(adev), "%p id:%d dma_find_channel set zero",
				adev, adev->pdev->dev.id);
		return -1;
	}
	cookie = dma_async_memcpy_pa_to_buf(adev->dma_chan, dest, src, len);
	dma_sync_wait(adev->dma_chan, cookie);
	DMA_NS;
	return len;
}

int getEmpty(struct acq420_dev* adev)
{
	if (!list_empty(&adev->EMPTIES)){
		mutex_lock(&adev->list_mutex);
		adev->cursor.hb = list_first_entry(
				&adev->EMPTIES, struct HBM, list);
		list_del(&adev->cursor.hb->list);
		mutex_unlock(&adev->list_mutex);
		adev->cursor.hb->bstate = BS_FILLING;
		adev->cursor.offset = 0;
		++adev->rt.nget;
		return 0;
	} else {
		dev_warn(&adev->pdev->dev, "get Empty: Q is EMPTY!\n");
		return -1;
	}
}

void putFull(struct acq420_dev* adev)
{
	mutex_lock(&adev->list_mutex);
	adev->cursor.hb->bstate = BS_FULL;
	list_add_tail(&adev->cursor.hb->list, &adev->REFILLS);
	adev->cursor.hb = 0;
	adev->cursor.offset = 0;
	mutex_unlock(&adev->list_mutex);
	++adev->rt.nput;
	if (run_buffers && adev->rt.nput >= run_buffers){
		adev->rt.please_stop = 1;
	}
	wake_up_interruptible(&adev->refill_ready);
}

int getFull(struct acq420_dev* adev)
{
	struct HBM *hbm;
	if (wait_event_interruptible(
			adev->refill_ready,
			!list_empty(&adev->REFILLS) ||
			adev->rt.refill_error ||
			adev->rt.please_stop)){
		return -EINTR;
	} else if (adev->rt.please_stop){
		return GET_FULL_DONE;
	} else if (adev->rt.refill_error){
		return GET_FULL_REFILL_ERR;
	}

	mutex_lock(&adev->list_mutex);
	hbm = list_first_entry(&adev->REFILLS, struct HBM, list);
	list_move_tail(&hbm->list, &adev->OPENS);
	hbm->bstate = BS_FULL_APP;
	mutex_unlock(&adev->list_mutex);
	return GET_FULL_OK;
}

void putEmpty(struct acq420_dev* adev)
{
	struct HBM *hbm;
	mutex_lock(&adev->list_mutex);
	hbm = list_first_entry(&adev->OPENS, struct HBM, list);
	hbm->bstate = BS_EMPTY;
	list_move_tail(&hbm->list, &adev->EMPTIES);
	mutex_unlock(&adev->list_mutex);
}
int getHeadroom(struct acq420_dev* adev)
{
	if (!adev->oneshot){
		if (adev->cursor.hb == 0){
			if (getEmpty(adev)){
				return 0;
			}
		} else if (adev->cursor.offset >= adev->cursor.hb->len){
			putFull(adev);
			if (getEmpty(adev)){
				return 0;
			}
		}
	}
	return adev->cursor.hb->len - adev->cursor.offset;
}



/* File operations */
int acq420_open_main(struct inode *inode, struct file *file)
{
        int rc = 0;
        struct acq420_dev* adev = ACQ420_DEV(file);

        if (mutex_lock_interruptible(&adev->mutex)) {
                return -ERESTARTSYS;
        }

        /* We're only going to allow one write at a time, so manage that via
         * reference counts
         */
        switch (file->f_flags & O_ACCMODE) {
        case O_RDONLY:
                break;
        case O_WRONLY:
                if (adev->writers || adev->busy) {
                        rc = -EBUSY;
                        goto out;
                } else {
                        adev->writers++;
                }
                break;
        case O_RDWR:
        default:
                if (adev->writers || adev->busy) {
                        rc = -EBUSY;
                        goto out;
                } else {
                        adev->writers++;
                }
        }

        adev->stats.opens++;

out:
        mutex_unlock(&adev->mutex);
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

	if (!IS_BUFFER(PD(file)->minor)){
		dev_warn(DEVP(adev), "ERROR: device node not a buffer");
		return -1;
	}
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
		.mmap = acq420_dma_mmap_host,
		// sendfile method is no more.. it's probably not quite this easy ..
		// sure enough, it's not !
		// most likely the HB's have to be on a block device ..
		.splice_read	= generic_file_splice_read
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
	int maxentries = FIFO_HISTO_SZ;
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



static bool filter_true(struct dma_chan *chan, void *param)
{
	return true;
}


int get_dma_chan(struct acq420_dev *adev)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	adev->dma_chan = dma_request_channel(mask, filter_true, NULL);

	return adev->dma_chan == 0 ? -1: 0;
}
int acq420_continuous_start(struct inode *inode, struct file *file)
{
	struct acq420_dev *adev = ACQ420_DEV(file);

	adev->oneshot = 0;

	if (mutex_lock_interruptible(&adev->mutex)) {
		return -EINTR;
	}

	if (get_dma_chan(adev)){
		dev_err(DEVP(adev), "no dma chan");
		return -EBUSY;
	}
	dev_dbg(DEVP(adev), "acq420_continuous_start() %p id:%d : dma_chan: %p",
		adev, adev->pdev->dev.id, adev->dma_chan);




#if 0
	// @@todo .. attempt to recycle last buffer. BLOWS!
	if (adev->cursor.hb){
		mutex_lock(&adev->list_mutex);
		dev_info(DEVP(adev), "recycle hb: %p\n", adev->cursor.hb);
		dev_info(DEVP(adev), "recycle hb: [%d] va:%p pa:%08x\n",
			adev->cursor.hb->ix, adev->cursor.hb->va, adev->cursor.hb->pa);
		adev->cursor.hb->bstate = BS_EMPTY;
		list_move_tail(&adev->cursor.hb->list,&adev->EMPTIES);
		adev->cursor.hb = 0;
		adev->cursor.offset = 0;
		dev_info(DEVP(adev), "cleared cursor\n");
		mutex_unlock(&adev->list_mutex);
	}
#endif
	adev->cursor.offset = 0;
	memset(&adev->rt, 0, sizeof(struct RUN_TIME));
	acq420_clear_histo(adev);

	if (IS_ACQ435(adev)){
		acq435_onStart(adev);
	} else {
		acq420_onStart(adev);

	}

	adev->DMA_READY = 1;
	adev->busy = 1;
	/*Wait for FIFO to fill*/
	acq420_enable_interrupt(adev);

	/* Kick off the DMA */
	mutex_unlock(&adev->mutex);

	return 0;
}

ssize_t acq420_continuous_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
/* NB: waits for a full buffer to ARRIVE, but only returns the 2 char ID */
{
	struct acq420_dev *adev = ACQ420_DEV(file);
	int rc = 0;
	char lbuf[8];
	struct HBM *hbm;
	int nread;
	unsigned long now;


	adev->stats.reads++;
	adev->count = count;
	adev->this_count = 0;

	if (adev->rt.please_stop){
		return -1;		/* EOF ? */
	}
	if (!list_empty(&adev->OPENS)){
		putEmpty(adev);
	}
	switch((rc = getFull(adev))){
	case GET_FULL_OK:
		break;
	case GET_FULL_DONE:
		dev_warn(DEVP(adev), "finished");
		return -1;		/* finished  ret 0 WBN? */
	case GET_FULL_REFILL_ERR:
		dev_warn(DEVP(adev), "refill error\n");
		return -1;
	default:
		dev_warn(DEVP(adev), "interrupted\n");
		return rc;
	}

	if (list_empty(&adev->OPENS)){
		dev_warn(DEVP(adev), "no buffer available");
		return -1;
	}
	hbm = list_first_entry(&adev->OPENS, struct HBM, list);

	/* update every hb0 or at least once per second */
	now = get_seconds();
	// this kludge because EPICS updates 4 buffers at a time
	if (hbm->ix == 3 || (now != adev->hb0_last && (hbm->ix&3) == 3)){
		dev_dbg(DEVP(adev), "last:%lu now:%lu ix:%u",
				adev->hb0_last, now, adev->cursor.hb->ix);

		adev->rt.hb0_count++;
		adev->rt.hb0_ix = hbm->ix&~3;
		adev->hb0_last = now;
		wake_up_interruptible(&adev->hb0_marker);
	}
	dma_sync_single_for_cpu(DEVP(adev), hbm->pa, hbm->len, hbm->dir);

	nread = sprintf(lbuf, "%02d\n", hbm->ix);
	rc = copy_to_user(buf, lbuf, nread);
	if (rc){
		return -rc;
	}

	return nread;
}
int acq420_continuous_stop(struct inode *inode, struct file *file)
{
	struct acq420_dev *adev = ACQ420_DEV(file);
	//acq420_reset_fifo(adev);
	acq420_disable_fifo(adev);

	if (!list_empty(&adev->OPENS)){
		putEmpty(adev);
	}
	mutex_lock(&adev->list_mutex);
	{
		struct HBM *cur;
		struct HBM *tmp;
		list_for_each_entry_safe(cur, tmp, &adev->REFILLS, list){
			cur->bstate = BS_EMPTY;
			list_move_tail(&cur->list, &adev->EMPTIES);
		}
	}
	/* dangerous - causes DMA in flight to fail
	if (adev->cursor.hb){
		adev->cursor.hb->bstate = BS_EMPTY;
		list_move_tail(&adev->cursor.hb->list,&adev->EMPTIES);
		adev->cursor.hb = 0;
	} */
	adev->busy = 0;

	mutex_unlock(&adev->list_mutex);

	dma_release_channel(adev->dma_chan);
	return acq420_release(inode, file);
}

static unsigned int acq420_continuous_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	struct acq420_dev *adev = ACQ420_DEV(file);

	if (!list_empty(&adev->REFILLS)){
		return POLLIN|POLLRDNORM;
	}else if (adev->rt.refill_error){
		return POLLERR;
	}else if (adev->rt.please_stop){
		return POLLHUP;
	}else{
		poll_wait(file, &adev->refill_ready, poll_table);
		if (!list_empty(&adev->REFILLS)){
			return POLLIN|POLLRDNORM;
		}else if (adev->rt.refill_error){
			return POLLERR;
		}else if (adev->rt.please_stop){
			return POLLHUP;
		}else{
			return 0;
		}
	}
}
ssize_t acq420_hb0_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq420_dev *adev = ACQ420_DEV(file);
	char HB0[16];

	int rc;
	/* force wait until next .. this is very conservative, we could
	 * stash the hb0 in DESCR for use between calls.
	 * But this way is self-regulating. This is for human monitor,
	 * not an attempt to handle ALL the data
	 */
	unsigned hb0_count = adev->rt.hb0_count;

	if (wait_event_interruptible(
			adev->hb0_marker,
			adev->rt.hb0_count != hb0_count ||
			adev->rt.refill_error ||
			adev->rt.please_stop)){
		return -EINTR;
	} else if (adev->rt.please_stop){
		return -GET_FULL_DONE;
	} else if (adev->rt.refill_error){
		return -GET_FULL_REFILL_ERR;
	}
	sprintf(HB0, "%02d\n", adev->rt.hb0_ix);
	count = min(count, strlen(HB0));
	rc = copy_to_user(buf, HB0, count);
	if (rc){
		return -1;
	}

	*f_pos += count;
	return count;
}

int acq420_null_release(struct inode *inode, struct file *file)
{
	return 0;
}
int acq420_open_histo(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_fops_histo = {
			.read = acq420_histo_read,
			.release = acq420_null_release
	};
	file->f_op = &acq420_fops_histo;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}



int acq420_open_continuous(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_fops_continuous = {
			.open = acq420_continuous_start,
			.read = acq420_continuous_read,
			.release = acq420_continuous_stop,
			.poll = acq420_continuous_poll
	};
	int rc = acq420_open_main(inode, file);
	if (rc){
		return rc;
	}
	file->f_op = &acq420_fops_continuous;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}

int acq420_open_hb0(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_fops_histo = {
			.read = acq420_hb0_read,
			.release = acq420_null_release
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
        		return acq420_open_continuous(inode, file);
        	case ACQ420_MINOR_HISTO:
        		return acq420_open_histo(inode, file);
        	case ACQ420_MINOR_HB0:
        		return acq420_open_hb0(inode, file);
        	case ACQ420_MINOR_0:
        		return acq420_open_main(inode, file);
        	default:
        		return -ENODEV;
        	}

        }
}

int acq420_release(struct inode *inode, struct file *file)
{
        struct acq420_dev *adev = ACQ420_DEV(file);

        if (mutex_lock_interruptible(&adev->mutex)) {
                return -EINTR;
        }

        /* Manage writes via reference counts */
        switch (file->f_flags & O_ACCMODE) {
        case O_RDONLY:
                break;
        case O_WRONLY:
                adev->writers--;
                break;
        case O_RDWR:
        default:
                adev->writers--;
        }

        adev->stats.closes++;

        mutex_unlock(&adev->mutex);

        kfree(PD(file));
        return 0;
}

static void acq420_fault_callback(unsigned int channel,
        unsigned int fault_type,
        unsigned int fault_address,
        void *data)
{
        struct acq420_dev *adev = data;

        dev_err(&adev->pdev->dev,
                "DMA fault type 0x%08x at address 0x%0x on channel %d\n",
                fault_type, fault_address, channel);

        adev->stats.errors++;
        acq420_reset_fifo(adev);
        adev->busy = 0;
        wake_up_interruptible(&adev->waitq);
}

static void acq420_done_callback(unsigned int channel, void *data)
{
        struct acq420_dev *dev = data;

	   // printk("DMA Done!");

        dev->stats.bytes_written += dev->count;
        dev->busy = 0;

        wake_up_interruptible(&dev->waitq);
}

ssize_t acq420_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
{
	return -EINVAL;
}

ssize_t acq420_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
        return -EINVAL;
}

int acq420_mmap_bar(struct file* file, struct vm_area_struct* vma)
{
	struct acq420_dev *adev = ACQ420_DEV(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = adev->dev_addrsize;
	unsigned pfn = adev->dev_physaddr >> PAGE_SHIFT;

	dev_dbg(DEVP(adev), "%c vsize %lu psize %lu %s",
		'D', vsize, psize, vsize>psize? "EINVAL": "OK");

	if (vsize > psize){
		return -EINVAL;                   /* request too big */
	}
	if (io_remap_pfn_range(
		vma, vma->vm_start, pfn, vsize, vma->vm_page_prot)){
		return -EAGAIN;
	}else{
		return 0;
	}
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
		int bytes = acq420_samples2bytes(adev, status);
		struct HBM _cursor;

		bytes = min(bytes, headroom);

		if (adev->rt.please_stop){
			goto stop_exit;
		}
		if (headroom == 0){
			dev_info(DEVP(adev), "headroom==0, quit count:%d set error\n",
					adev->this_count);
			adev->rt.refill_error = 1;
			goto stop_exit;
		}

		_cursor = *adev->cursor.hb;
		_cursor.pa += adev->cursor.offset;
		cpsc_dma_memcpy(adev, &_cursor, FIFO_PA(adev), bytes);
		add_fifo_histo(adev, status);
		adev->stats.dma_transactions++;

		adev->cursor.offset += bytes;
		if (adev->oneshot){
			adev->this_count += bytes;
		}
		status = acq420_get_fifo_samples(adev);
		if (acq420_isFifoError(adev)){
			adev->rt.refill_error = 1;
			goto stop_exit;
		}
	} while(status >= lotide);

	acq420_enable_interrupt(adev);

	return IRQ_HANDLED;

stop_exit:
	wake_up_interruptible(&adev->refill_ready);
	wake_up_interruptible(&adev->hb0_marker);
	return IRQ_HANDLED;
}

static irqreturn_t acq420_int_handler(int irq, void *dev_id)
{
	struct acq420_dev *adev = (struct acq420_dev *)dev_id;
	u32 status = acq420_get_interrupt(adev);
	irqreturn_t irq_status = IRQ_WAKE_THREAD;

//	iowrite32((0xCAFEBABE), adev->dev_virtaddr);

	if (status == 0x1 && adev->DMA_READY == 1) {
		adev->DMA_READY = 0;
	}
	acq420_disable_interrupt(adev);
//	acq420_clear_interrupt(adev, status);
	adev->stats.fifo_interrupts++;

	return irq_status;
}

struct file_operations acq420_fops = {
        .owner = THIS_MODULE,
        .read = acq420_read,
        .write = acq420_write,
        .open = acq420_open,
        .release = acq420_release,
        .mmap = acq420_mmap_bar
};




#ifdef CONFIG_OF
static struct of_device_id xfifodma_of_match[] /* __devinitdata */ = {
        { .compatible = "D-TACQ,acq420fmc", },
        { /* end of table */}
};
MODULE_DEVICE_TABLE(of, xfifodma_of_match);
#else
#define xfifodma_of_match NULL
#endif /* CONFIG_OF */


static void acq420_device_tree_init(struct acq420_dev* adev)
{
	struct device_node *of_node = adev->pdev->dev.of_node;

        if (of_node) {
        	u32 irqs[OF_IRQ_COUNT];

                if (of_property_read_u32(of_node, "dma-channel",
                        &adev->of_prams.dma_channel) < 0) {
                        dev_warn(DEVP(adev),
                        	"DMA channel unspecified - assuming 0\n");
                        adev->of_prams.dma_channel = 0;
                }
                dev_info(DEVP(adev),
                        "acq420_device_tree_init() read DMA channel is %d\n",
                        adev->of_prams.dma_channel);

                if (of_property_read_u32(of_node, "fifo-depth",
                        &adev->of_prams.fifo_depth) < 0) {
                        dev_warn(DEVP(adev),
                                "depth unspecified, assuming 0xffffffff\n");
                        adev->of_prams.fifo_depth = 0xffffffff;
                }
                dev_info(DEVP(adev),
                	"acq420_device_tree_init() DMA fifo depth is %d\n",
                	adev->of_prams.fifo_depth);

                if (of_property_read_u32(of_node, "burst-length",
                        &adev->of_prams.burst_length) < 0) {
                        dev_warn(DEVP(adev),
                                "burst length unspecified - assuming 1\n");
                        adev->of_prams.burst_length = 1;
                }

                of_property_read_u32(of_node, "fake",  &adev->of_prams.fake);

                dev_info(DEVP(adev),
                	"acq420_device_tree_init() DMA burst length is %d\n",
                        adev->of_prams.burst_length);

                if (of_property_read_u32_array(
                		of_node, "interrupts", irqs, OF_IRQ_COUNT)){
                	dev_warn(DEVP(adev), "failed to find IRQ values");
                }else{
                	adev->of_prams.irq = irqs[OF_IRQ_HITIDE] + OF_IRQ_MAGIC;
                }
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
        init_waitqueue_head(&adev->refill_ready);
        init_waitqueue_head(&adev->hb0_marker);


        adev->pdev = pdev;
        mutex_init(&adev->mutex);
        adev->fifo_histo = kzalloc(FIFO_HISTO_SZ*sizeof(u32), GFP_KERNEL);

        INIT_LIST_HEAD(&adev->EMPTIES);
        INIT_LIST_HEAD(&adev->REFILLS);
        INIT_LIST_HEAD(&adev->OPENS);
        mutex_init(&adev->list_mutex);
        return adev;
}

static void acq420_createDebugfs(struct acq420_dev* adev)
{
	char* pcursor;
	if (!acq420_debug_root){
		acq420_debug_root = debugfs_create_dir("acq420", 0);
		if (!acq420_debug_root){
			dev_warn(&adev->pdev->dev, "failed create dir acq420");
			return;
		}
	}
	pcursor = adev->debug_names = kmalloc(4096, GFP_KERNEL);

#define DBG_REG_CREATE(reg) 					\
	sprintf(pcursor, "%s.0x%02x", #reg, reg);		\
	debugfs_create_x32(pcursor, S_IRUGO, 			\
		adev->debug_dir, adev->dev_virtaddr+(reg));     \
	pcursor += strlen(pcursor) + 1

	adev->debug_dir = debugfs_create_dir(
			acq420_devnames[adev->pdev->dev.id], acq420_debug_root);

	if (!adev->debug_dir){
		dev_warn(&adev->pdev->dev, "failed create dir acq420.x");
		return;
	}
	DBG_REG_CREATE(MOD_ID);
	DBG_REG_CREATE(ADC_CTRL);
	DBG_REG_CREATE(TIM_CTRL);
	DBG_REG_CREATE(ADC_HITIDE);
	DBG_REG_CREATE(ADC_FIFO_SAMPLES);
	DBG_REG_CREATE(ADC_FIFO_STA);
	DBG_REG_CREATE(ADC_INT_CSR);
	DBG_REG_CREATE(ADC_CLK_CTR);
	DBG_REG_CREATE(ADC_SAMPLE_CTR);
	DBG_REG_CREATE(ADC_CLKDIV);
	if (IS_ACQ420(adev)){
		DBG_REG_CREATE(ADC_GAIN);
		DBG_REG_CREATE(ADC_FORMAT);
		DBG_REG_CREATE(ADC_CONV_TIME);
	} else if (IS_ACQ435(adev)){
		DBG_REG_CREATE(ACQ435_MODE);
	}


}

static void acq420_removeDebugfs(struct acq420_dev* adev)
{
	debugfs_remove_recursive(adev->debug_dir);
	kfree(adev->debug_names);
}
static int acq420_remove(struct platform_device *pdev);

#define dev_dbg	dev_info

static int acq420_probe(struct platform_device *pdev)
{
        int status;
        struct resource *acq420_resource;
        struct acq420_dev* adev = acq420_allocate_dev(pdev);

        if (!adev){
        	dev_err(DEVP(adev), "unable to allocate device structure\n");
        	return -ENOMEM;
        }
        pdev->dev.id = ndevices;
        /* Get our platform device resources */
        dev_dbg(DEVP(adev), "id:%d We have %d resources\n", pdev->id, pdev->num_resources);
        acq420_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (acq420_resource == NULL) {
                dev_err(&pdev->dev, "No resources found\n");
                return -ENODEV;
        }

        acq420_device_tree_init(adev);

        if (adev->of_prams.irq == 0){
        	adev->of_prams.irq = ADC_HT_INT; /* @@todo should come from device tree? */
        	dev_warn(&pdev->dev, "Using default IRQ %d", adev->of_prams.irq);
        }

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
                adev->dev_addrsize, acq420_devnames[ndevices])) {
                dev_err(&pdev->dev, "can't reserve i/o memory at 0x%08X\n",
                        adev->dev_physaddr);
                status = -ENODEV;
                goto fail;
        }
        adev->dev_virtaddr =
        	ioremap(adev->dev_physaddr, adev->dev_addrsize);
        dev_dbg(DEVP(adev), "acq420: mapped 0x%0x to 0x%0x\n",
        	adev->dev_physaddr, (unsigned int)adev->dev_virtaddr);

        status = cdev_add(&adev->cdev, adev->devno, ACQ420_MINOR_MAX);
        acq420_init_proc(adev, ndevices);

        status = devm_request_threaded_irq(
        		DEVP(adev), adev->of_prams.irq,
        		acq420_int_handler, fire_dma,
        		IRQF_SHARED, acq420_devnames[DEVP(adev)->id],
        		adev);

	if (status)	{
		printk("%s unable to secure IRQ%d\n", "ACQ420", 
			adev->of_prams.irq);
		goto fail;
	}

	if (hbm_allocate(DEVP(adev),
			nbuffers, bufferlen, &adev->EMPTIES, DMA_FROM_DEVICE)){
		dev_err(&pdev->dev, "failed to allocate buffers");
		goto fail;
	}else{
		struct HBM* cursor;
		int ix = 0;
		adev->hb = kmalloc(nbuffers*sizeof(struct HBM*), GFP_KERNEL);
		list_for_each_entry(cursor, &adev->EMPTIES, list){
			WARN_ON(cursor->ix != ix);
			adev->hb[cursor->ix] = cursor;
			ix++;
		}
		dev_info(DEVP(adev), "setting nbuffers %d\n", ix);
		adev->nbuffers = ix;
	}

        acq420_devices[ndevices++] = adev;

        if (adev->of_prams.fake){
        	dev_info(&pdev->dev, "fake device, no hardware init");
        	return 0;
        }
        acq400_getID(adev);
        acq420_createSysfs(&pdev->dev);
        acq420_createDebugfs(adev);

        if (IS_ACQ435(adev)){
        	dev_info(&pdev->dev, "ACQ435 device init");
        	acq435_init_defaults(adev);
        } else {
        	dev_info(&pdev->dev, "ACQ420 device init");
        	acq420_init_defaults(adev);
        }
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
		struct acq420_dev* adev = acq420_devices[pdev->id];

		if (adev == 0){
			return -1;
		}

		acq420_removeDebugfs(adev);
		hbm_free(&pdev->dev, &adev->EMPTIES);
		hbm_free(&pdev->dev, &adev->REFILLS);
		acq420_delSysfs(&adev->pdev->dev);
		acq420_del_proc(adev);
		dev_dbg(&pdev->dev, "cdev_del %p\n", &adev->cdev);
		cdev_del(&adev->cdev);
		unregister_chrdev_region(adev->devno, 1);

		/* Unmap the I/O memory */
		if (adev->dev_virtaddr) {
			iounmap(adev->dev_virtaddr);
			release_mem_region(adev->dev_physaddr,
					adev->dev_addrsize);
		}
		/* Free the PL330 buffer client data descriptors */
		if (adev->client_data) {
			kfree(adev->client_data);
		}

		kfree(adev);

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
	timer_test();
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




