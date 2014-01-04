/* ------------------------------------------------------------------------- */
/* acq400_drv.c  D-TACQ ACQ400 FMC  DRIVER		                     */
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


#include "acq400.h"
#include "hbm.h"



#define REVID "2.342"

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

int debcount;
module_param(debcount, int, 0644);
MODULE_PARM_DESC(debcount, "NZ if counter debounce ever .. happened");

int maxdma = MAXDMA;
module_param(maxdma, int, 0644);
MODULE_PARM_DESC(maxdma, "set maximum DMA len bytes");

/* driver supports multiple devices.
 * ideally we'd have no globals here at all, but it works, for now
 */
#define MAXDEVICES 6
struct acq400_dev* acq400_devices[MAXDEVICES];

#define DMA_NS_MAX     40
int dma_ns_lines[DMA_NS_MAX];
int dma_ns[DMA_NS_MAX];
int dma_ns_num = DMA_NS_MAX;
module_param_array(dma_ns, int, &dma_ns_num, 0444);
module_param_array(dma_ns_lines, int, &dma_ns_num, 0444);


int good_sites[MAXDEVICES];
int good_sites_count = 0;
module_param_array(good_sites, int, &good_sites_count, 0444);

int ao420_dma_threshold = 999999;
module_param(ao420_dma_threshold, int, 0644);
MODULE_PARM_DESC(ao420_dma_threshold, "use DMA for transfer to AO [set 999999 to disable]");

int ao420_mapping[AO_CHAN] = { 4, 3, 2, 1 };
int ao420_mapping_count = 4;
module_param_array(ao420_mapping, int, &ao420_mapping_count, 0644);

// @@todo pgm: crude: index by site, index from 10
const char* acq400_names[] = { "0", "1", "2", "3", "4", "5", "6" };
const char* acq400_devnames[] = {
	"acq400.0", "acq400.1", "acq400.2",
	"acq400.3", "acq400.4", "acq400.5", "acq400.6"
};

struct dentry* acq400_debug_root;

#define AO420_NBUFFERS 	2
#define AO420_BUFFERLEN	0x200000

int ai_data_loop(void *data);

const char* devname(struct acq400_dev *adev)
{
	return acq400_devnames[adev->of_prams.site];
}
/* correct for FPGA mismatch with front panel connectors */
int ao420_physChan(int lchan /* 1..4 */ )
{
	int ip;
	for (ip = 0; ip < AO_CHAN; ++ip){
		if (ao420_mapping[ip] == lchan){
			return ip;
		}
	}
	BUG();
}

int isGoodSite(int site)
{
	if (site == 0){
		return 1;
	}else if (site < 0 || site > MAXDEVICES){
		return 0;
	}else{
		int ii;
		for (ii = 0; ii < good_sites_count; ++ii){
			if (good_sites[ii] == site){
				return 1;
			}
		}
		return 0;
	}
}
int acq400_release(struct inode *inode, struct file *file);

void acq400wr32(struct acq400_dev *adev, int offset, u32 value)
{
	dev_dbg(DEVP(adev), "acq400wr32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, value);

	iowrite32(value, adev->dev_virtaddr + offset);
}

u32 acq400rd32(struct acq400_dev *adev, int offset)
{
	u32 rc = ioread32(adev->dev_virtaddr + offset);
	dev_dbg(DEVP(adev), "acq400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	return rc;
}

u32 acq400rd32_upcount(struct acq400_dev *adev, int offset)
{
	u32 c1 = acq400rd32(adev, offset);
	u32 c2;

	while((c2 = acq400rd32(adev, offset)) < c1){
		c1 = c2;
		++debcount;
	}
	return c2;
}
static u32 acq420_set_fmt(struct acq400_dev *adev, u32 adc_ctrl)
/* DOES NOT ACTUALLY WRITE HARDWARE! */
{
	if (adev->adc_18b){
		adc_ctrl |= ADC_CTRL_18B;
	}else{
		adc_ctrl &= ~ADC_CTRL_18B;
	}
	if (adev->data32){
		adc_ctrl |= ADC_CTRL32B_data;
	}else{
		adc_ctrl &= ~ADC_CTRL32B_data;
	}
	return adc_ctrl;
}
static void acq420_init_defaults(struct acq400_dev *adev)
{
	dev_info(DEVP(adev), "ACQ420 device init");
	acq400wr32(adev, ADC_CONV_TIME, adc_conv_time);
	adev->data32 = data_32b;
	adev->adc_18b = adc_18b;
	acq400wr32(adev, ADC_CTRL, ADC_CTRL_MODULE_EN|acq420_set_fmt(adev, 0));
	adev->nchan_enabled = 4;
	adev->word_size = adev->data32? 4: 2;
	adev->hitide = hitide;
	adev->lotide = lotide;
	adev->sysclkhz = SYSCLK_M100;
}

static void acq43X_init_defaults(struct acq400_dev *adev)
{
	dev_info(DEVP(adev), "ACQ435 device init");
	adev->data32 = 1;
	adev->nchan_enabled = 32;
	adev->word_size = 4;
	adev->hitide = 128;
	adev->lotide = adev->hitide - 4;
	adev->sysclkhz = SYSCLK_M100;
	acq400wr32(adev, ADC_CLKDIV, 16);
	acq400wr32(adev, ADC_CTRL, ADC_CTRL_MODULE_EN);
}

static void ao420_init_defaults(struct acq400_dev *adev)
{
	dev_info(DEVP(adev), "AO420 device init");
	adev->cursor.hb = adev->hb[0];
	adev->hitide = 2048;
	adev->lotide = 1024;
	adev->sysclkhz = SYSCLK_M66;
}
static u32 acq420_get_fifo_samples(struct acq400_dev *adev)
{
	return acq400rd32(adev, ADC_FIFO_SAMPLES);
}

static u32 acq420_samples2bytes(struct acq400_dev *adev, u32 samples)
{
	return samples * adev->nchan_enabled * adev->word_size;
}

static void acq420_reset_fifo(struct acq400_dev *adev)
/* Raise and Release reset */
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);

	acq400wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_RST_ALL);
	acq400wr32(adev, ADC_CTRL, ctrl);
}

static void acq420_enable_fifo(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);
	if (adev->ramp_en){
		ctrl |= ADC_CTRL_RAMP_EN;
	}else{
		ctrl &= ~ADC_CTRL_RAMP_EN;
		ctrl = acq420_set_fmt(adev, ctrl);
	}
	acq400wr32(adev, ADC_CTRL, ctrl|ADC_CTRL_ENABLE_ALL);
}

static void acq420_disable_fifo(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);
	acq400wr32(adev, ADC_CTRL, ctrl & ~ADC_CTRL_ENABLE_ALL);
}


static void acq420_enable_interrupt(struct acq400_dev *adev)
{
	u32 int_ctrl = acq400rd32(adev, ADC_INT_CSR);
/*
	static int maxrep;

	if (++maxrep < 10){
		dev_info(DEVP(adev), "acq420_enable_interrupt() %p\n",
				adev->dev_physaddr);
	}
*/
	acq400wr32(adev, ADC_INT_CSR,	int_ctrl|0x1);
}

static void ao420_enable_interrupt(struct acq400_dev *adev)
{
	u32 int_ctrl = acq400rd32(adev, ADC_INT_CSR);
/*
	static int maxrep;

	if (++maxrep < 10){
		dev_info(DEVP(adev), "ao420_enable_interrupt() %p\n",
			adev->dev_physaddr);
	}
*/
	acq400wr32(adev, ADC_INT_CSR,	int_ctrl|0x1);
}

static void acq420_disable_interrupt(struct acq400_dev *adev)
{
	acq400wr32(adev, ADC_INT_CSR, 0x0);
}
static void ao420_disable_interrupt(struct acq400_dev *adev)
{
	acq420_disable_interrupt(adev);
}

static u32 acq420_get_interrupt(struct acq400_dev *adev)
{
	return acq400rd32(adev, ADC_INT_CSR);
}

static void acq420_clear_interrupt(struct acq400_dev *adev, u32 status)
{
	/** @@todo: how to INTACK?
	acq400wr32(adev, ALG_INT_CSR, acq420_get_interrupt(adev));
 *
 */
	/* peter assumes R/C ie write a 1 to clear the bit .. */
	acq400wr32(adev, ADC_INT_CSR, status);
}


static void acq400_clear_histo(struct acq400_dev *adev)
{
	memset(adev->fifo_histo, 0, FIFO_HISTO_SZ*sizeof(u32));
}

int acq420_isFifoError(struct acq400_dev *adev)
{
	u32 fifsta = acq400rd32(adev, ADC_FIFO_STA);
	int err = (fifsta&FIFERR) != 0;
	if (err){
		dev_warn(DEVP(adev), "FIFERR mask:%08x actual:%08x\n",
				FIFERR, fifsta);
	}
	return err;
}

void acq43X_onStart(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);

	dev_info(DEVP(adev), "acq435_onStart()");
	ctrl = 0;

	acq400wr32(adev, ADC_CTRL, ctrl |= ADC_CTRL_MODULE_EN);

	if (adev->ramp_en){
		ctrl |= ADC_CTRL_RAMP_EN;
	}else{
		ctrl &= ~ADC_CTRL_RAMP_EN;
	}
	if (adev->spad_en){
		ctrl |= ACQ435_CTRL_SPAD;
	}else{
		ctrl &= ~ACQ435_CTRL_SPAD;
	}
	// set mode (assume done)
	// set clkdiv (assume done)
	// set timing bus (assume done)
	/** clear FIFO flags .. workaround hw bug */
	acq400wr32(adev, ADC_FIFO_STA, ADC_FIFO_FLAGS);

	acq400wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_FIFO_RST);
	acq400wr32(adev, ADC_CTRL, ctrl);
	acq400wr32(adev, ADC_CTRL, ctrl  |= ADC_CTRL_ADC_EN);
	acq400wr32(adev, ADC_CTRL, ctrl  |= ADC_CTRL_FIFO_EN);

}
void acq420_onStart(struct acq400_dev *adev)
{
	dev_info(DEVP(adev), "acq420_onStart()");
	acq420_enable_fifo(adev);
	acq420_reset_fifo(adev);
	/** clear FIFO flags .. workaround hw bug */
	acq400wr32(adev, ADC_FIFO_STA, ADC_FIFO_FLAGS);
}
static void acq400_getID(struct acq400_dev *adev)
{
	u32 modid;

	dev_info(DEVP(adev), "About to read MODID from %p\n", adev->dev_virtaddr+MOD_ID);

	modid = acq400rd32(adev, MOD_ID);
	adev->mod_id = modid;

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

#if 0
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
#define DMA_NS_TEST	timer_test()
#else
#define DMA_NS_INIT
#define DMA_NS
#define DMA_NS_TEST
#endif


void acq400_dma_callback(void *param)
{
	struct acq400_dev* adev = (struct acq400_dev*)param;
	adev->dma_callback_done++;
	wake_up_interruptible(&adev->DMA_READY);
}

dma_cookie_t
dma_async_memcpy_pa_to_buf(
		struct acq400_dev* adev,
		struct dma_chan *chan, struct HBM *dest,
		dma_addr_t dma_src, size_t len, unsigned long flags)
{
	struct dma_device *dev = chan->device;
	struct dma_async_tx_descriptor *tx;
	flags |= DMA_SRC_NO_INCR | DMA_CTRL_ACK |
				DMA_COMPL_SRC_UNMAP_SINGLE |
				DMA_COMPL_DEST_UNMAP_SINGLE;

	DMA_NS;
	dev_dbg(dev->dev, "dev->prep_dma_memcpy %d 0x%08x 0x%08x %d %08lx",
			chan->chan_id, dest->pa, dma_src, len, flags);
	tx = dev->device_prep_dma_memcpy(chan, dest->pa, dma_src, len, flags);

	DMA_NS;
	if (!tx) {
		return -ENOMEM;
	} else{
		dma_cookie_t cookie;
		tx->callback = acq400_dma_callback;
		tx->callback_param = adev;
		cookie = tx->tx_submit(tx);
		dev_dbg(dev->dev, "submit(%d) done %x", chan->chan_id, cookie);
		DMA_NS;

		preempt_disable();
		__this_cpu_add(chan->local->bytes_transferred, len);
		__this_cpu_inc(chan->local->memcpy_count);
		preempt_enable();
		return cookie;
	}
}

dma_cookie_t
dma_async_memcpy(
	struct dma_chan *chan, dma_addr_t src, 	dma_addr_t dest, size_t len)
{
	struct dma_device *dev = chan->device;
	struct dma_async_tx_descriptor *tx;
	unsigned long flags = DMA_DST_NO_INCR | DMA_CTRL_ACK |
				DMA_COMPL_SRC_UNMAP_SINGLE |
				DMA_COMPL_DEST_UNMAP_SINGLE;

	DMA_NS;
	tx = dev->device_prep_dma_memcpy(chan, dest, src, len, flags);

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

int dma_memcpy(
	struct acq400_dev* adev, dma_addr_t dest, dma_addr_t src, size_t len)
{
	dma_cookie_t cookie;
	DMA_NS_INIT;
	DMA_NS;
	if (adev->dma_chan == 0){
		dev_err(DEVP(adev), "%p id:%d dma_find_channel set zero",
				adev, adev->pdev->dev.id);
		return -1;
	}
	cookie = dma_async_memcpy(adev->dma_chan[0], src, dest, len);
	dma_sync_wait(adev->dma_chan[0], cookie);
	DMA_NS;
	return len;
}

int cpsc_dma_memcpy(struct acq400_dev* adev, struct HBM* dest, u32 src, size_t len)
{
	dma_cookie_t cookie;
	DMA_NS_INIT;
	DMA_NS;
	if (adev->dma_chan == 0){
		dev_err(DEVP(adev), "%p id:%d dma_find_channel set zero",
				adev, adev->pdev->dev.id);
		return -1;
	}
	dev_dbg(DEVP(adev), "chan:%d destpa:0x%08x srcpa:0x%08x len:%d",
			adev->dma_chan[0]->chan_id, dest->pa, src, len);

	cookie = dma_async_memcpy_pa_to_buf(adev, adev->dma_chan[0], dest, src, len, 0);
	dma_sync_wait(adev->dma_chan[0], cookie);
	DMA_NS;
	return len;
}

struct HBM * getEmpty(struct acq400_dev* adev)
{
	if (!list_empty(&adev->EMPTIES)){
		struct HBM *hbm;
		mutex_lock(&adev->list_mutex);
		hbm = list_first_entry(
				&adev->EMPTIES, struct HBM, list);
		list_move_tail(&hbm->list, &adev->INFLIGHT);
		hbm->bstate = BS_FILLING;
		mutex_unlock(&adev->list_mutex);

		++adev->rt.nget;
		return hbm;
	} else {
		dev_warn(&adev->pdev->dev, "get Empty: Q is EMPTY!\n");
		return 0;
	}
}

void putFull(struct acq400_dev* adev)
{
	if (!list_empty(&adev->INFLIGHT)){
		struct HBM *hbm;
		mutex_lock(&adev->list_mutex);
		hbm = list_first_entry(
				&adev->INFLIGHT, struct HBM, list);
		hbm->bstate = BS_FULL;
		list_move_tail(&hbm->list, &adev->REFILLS);
		mutex_unlock(&adev->list_mutex);

		++adev->rt.nput;
		if (run_buffers && adev->rt.nput >= run_buffers){
			adev->rt.please_stop = 1;
		}
		wake_up_interruptible(&adev->refill_ready);
	}else{
		dev_warn(&adev->pdev->dev, "putFull: Q is EMPTY!\n");
	}
}

int getFull(struct acq400_dev* adev)
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

void putEmpty(struct acq400_dev* adev)
{
	struct HBM *hbm;
	mutex_lock(&adev->list_mutex);
	hbm = list_first_entry(&adev->OPENS, struct HBM, list);
	hbm->bstate = BS_EMPTY;
	list_move_tail(&hbm->list, &adev->EMPTIES);
	mutex_unlock(&adev->list_mutex);
}


/* File operations */
int acq400_open_main(struct inode *inode, struct file *file)
{
        int rc = 0;
        struct acq400_dev* adev = ACQ400_DEV(file);

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

int acq400_dma_mmap_host(struct file* file, struct vm_area_struct* vma)
/**
 * mmap the host buffer.
 */
{
	struct acq400_dev* adev = ACQ400_DEV(file);
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


int acq400_open_hb(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_fops_dma = {
		.open = acq420_dma_open,
		.mmap = acq400_dma_mmap_host,
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



ssize_t acq400_histo_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	unsigned *the_histo = ACQ400_DEV(file)->fifo_histo;
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


int _get_dma_chan(struct acq400_dev *adev, int ic)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	adev->dma_chan[ic] = dma_request_channel(mask, filter_true, NULL);
	if (adev->dma_chan[ic] == 0){
			dev_err(DEVP(adev), "%p id:%d dma_find_channel set zero",
					adev, adev->pdev->dev.id);
	}
	return adev->dma_chan[ic] == 0 ? -1: 0;
}

int get_dma_chan(struct acq400_dev *adev)
{
	return _get_dma_chan(adev, 0) || _get_dma_chan(adev, 1);
}

void _release_dma_chan(struct acq400_dev *adev, int ic)
{
	if (adev->dma_chan[ic]){
		dma_release_channel(adev->dma_chan[ic]);
		adev->dma_chan[ic] = 0;
	}
}
void release_dma_chan(struct acq400_dev *adev)
{
	_release_dma_chan(adev, 0);
	_release_dma_chan(adev, 1);
}
int acq420_continuous_start(struct inode *inode, struct file *file)
{
	struct acq400_dev *adev = ACQ400_DEV(file);

	adev->oneshot = 0;
	adev->stats.shot++;
	adev->stats.run = 1;

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
	acq400_clear_histo(adev);
	adev->w_task = kthread_run(ai_data_loop, adev, devname(adev));

	while(!adev->task_active){
		yield();
	}

	acq400wr32(adev, ADC_HITIDE, 	adev->hitide);
	if (IS_ACQ43X(adev)){
		acq43X_onStart(adev);
	} else {
		acq420_onStart(adev);

	}

	adev->busy = 1;

	/*Wait for FIFO to fill*/
	acq420_enable_interrupt(adev);

	/* Kick off the DMA */
	mutex_unlock(&adev->mutex);

	return 0;
}

ssize_t acq400_continuous_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
/* NB: waits for a full buffer to ARRIVE, but only returns the 2 char ID */
{
	struct acq400_dev *adev = ACQ400_DEV(file);
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
	struct acq400_dev *adev = ACQ400_DEV(file);
	//acq420_reset_fifo(adev);
	acq420_disable_fifo(adev);
	dev_info(DEVP(adev), "kthread_stop called\n");
	kthread_stop(adev->w_task);

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

	release_dma_chan(adev);
	adev->stats.run = 0;
	return acq400_release(inode, file);
}

static unsigned int acq420_continuous_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	struct acq400_dev *adev = ACQ400_DEV(file);

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
ssize_t acq400_hb0_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
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

int acq400_null_release(struct inode *inode, struct file *file)
{
	return 0;
}
int acq400_open_histo(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_histo = {
			.read = acq400_histo_read,
			.release = acq400_null_release
	};
	file->f_op = &acq400_fops_histo;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}



int acq420_open_continuous(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_continuous = {
			.open = acq420_continuous_start,
			.read = acq400_continuous_read,
			.release = acq420_continuous_stop,
			.poll = acq420_continuous_poll
	};
	int rc = acq400_open_main(inode, file);
	if (rc){
		return rc;
	}
	file->f_op = &acq400_fops_continuous;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}

int acq420_open_hb0(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_histo = {
			.read = acq400_hb0_read,
			.release = acq400_null_release
	};
	file->f_op = &acq400_fops_histo;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}

int acq400_open(struct inode *inode, struct file *file)
{

        struct acq400_dev *dev;
        int minor;

        file->private_data = kzalloc(PDSZ, GFP_KERNEL);

        PD(file)->dev = dev = container_of(inode->i_cdev, struct acq400_dev, cdev);
        PD(file)->minor = minor = MINOR(inode->i_rdev);

        //dev_dbg(&dev->pdev->dev, "hello: minor:%d\n", minor);

        if (minor >= ACQ420_MINOR_BUF && minor <= ACQ420_MINOR_BUF2){
        	return acq400_open_hb(inode, file);
        } else if (minor >= ACQ420_MINOR_CHAN && minor <= ACQ420_MINOR_CHAN2){
        	return -ENODEV;  	// @@todo maybe later0
        } else {
        	switch(minor){
        	case ACQ420_MINOR_CONTINUOUS:
        		return acq420_open_continuous(inode, file);
        	case ACQ420_MINOR_HISTO:
        		return acq400_open_histo(inode, file);
        	case ACQ420_MINOR_HB0:
        		return acq420_open_hb0(inode, file);
        	case ACQ420_MINOR_0:
        		return acq400_open_main(inode, file);
        	default:
        		return -ENODEV;
        	}

        }
}

int acq400_release(struct inode *inode, struct file *file)
{
        struct acq400_dev *adev = ACQ400_DEV(file);

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

static void acq400_fault_callback(unsigned int channel,
        unsigned int fault_type,
        unsigned int fault_address,
        void *data)
{
        struct acq400_dev *adev = data;

        dev_err(&adev->pdev->dev,
                "DMA fault type 0x%08x at address 0x%0x on channel %d\n",
                fault_type, fault_address, channel);

        adev->stats.errors++;
        acq420_reset_fifo(adev);
        adev->busy = 0;
        wake_up_interruptible(&adev->waitq);
}

ssize_t acq400_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
{
	return -EINVAL;
}

ssize_t acq400_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
        return -EINVAL;
}

int acq400_mmap_bar(struct file* file, struct vm_area_struct* vma)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
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


static void add_fifo_histo(struct acq400_dev *adev, u32 status)
{
	adev->fifo_histo[STATUS_TO_HISTO(status)]++;
}

#define AO420_MAX_FIFO_SAMPLES	0x00003fff	/* actualite not doxy */
#define AO420_MAX_FILL_BLOCK	0x1000		/* BYTES, SWAG */
#define AO420_FILL_THRESHOLD	0x400		/* fill to here */

/** @todo : assumes PACKED DATA */
#define AOSAMPLES2BYTES(xx) ((xx) * AO_CHAN * sizeof(short))

static int ao420_getFifoSamples(struct acq400_dev* adev) {
	return acq400rd32(adev, DAC_FIFO_SAMPLES)&DAC_FIFO_SAMPLES_MASK;
}

static int ao420_getFifoHeadroom(struct acq400_dev* adev) {
	return AO420_MAX_FIFO_SAMPLES - ao420_getFifoSamples(adev);
}



void write32(volatile u32* to, volatile u32* from, int nwords)
{
	int ii;

	for (ii = 0; ii < nwords; ++ii){
		iowrite32(from[ii], to+ii);
	}
}

static void ao420_write_fifo_dma(struct acq400_dev* adev, int frombyte, int bytes)
{
	int rc = dma_memcpy(adev,
		adev->dev_physaddr+AXI_FIFO,
		adev->cursor.hb->pa+frombyte, bytes);

	if (rc != bytes){
		dev_err(DEVP(adev), "dma_memcpy FAILED :%d\n", rc);
	}
}

static void ao420_write_fifo(struct acq400_dev* adev, int frombyte, int bytes)
{
	write32(adev->dev_virtaddr+AXI_FIFO,
		adev->cursor.hb->va+frombyte/sizeof(u32),
		bytes/sizeof(u32));
}


static void ao420_fill_fifo(struct acq400_dev* adev)
{
	int headroom;

	while((headroom = ao420_getFifoHeadroom(adev)) > AO420_FILL_THRESHOLD){
		int remaining = adev->AO_playloop.length - adev->AO_playloop.cursor;

		remaining = min(remaining, headroom);
		remaining = min(remaining, AO420_MAX_FILL_BLOCK);

		dev_dbg(DEVP(adev), "headroom:%d remaining:%d using:%d\n",
			headroom, adev->AO_playloop.length - adev->AO_playloop.cursor, remaining);

		if (remaining){
			int cursor = AOSAMPLES2BYTES(adev->AO_playloop.cursor);
			int lenbytes = AOSAMPLES2BYTES(remaining);
			if (adev->dma_chan != 0 && remaining > ao420_dma_threshold){
				ao420_write_fifo_dma(adev, cursor, lenbytes);
			}else{
				ao420_write_fifo(adev, cursor, lenbytes);
			}
			adev->AO_playloop.cursor += remaining;
		}

		if (adev->AO_playloop.cursor >= adev->AO_playloop.length){
			adev->AO_playloop.cursor = 0;
		}
	}
	dev_dbg(DEVP(adev), "ao420_fill_fifo() done filling, samples:%08x\n",
			acq400rd32(adev, DAC_FIFO_SAMPLES));
}
static irqreturn_t ao420_dma(int irq, void *dev_id)
/* keep the AO420 FIFO full. Recycle buffer only */
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;

	if (adev->AO_playloop.length){
		u32 start_samples = ao420_getFifoSamples(adev);
		ao420_fill_fifo(adev);
		ao420_enable_interrupt(adev);
		add_fifo_histo(adev, start_samples);
	}

	return IRQ_HANDLED;
}

void ao420_reset_playloop(struct acq400_dev* adev)
{
	unsigned cr = acq400rd32(adev, DAC_CTRL);

	if (adev->AO_playloop.length == 0){
		release_dma_chan(adev);
		ao420_disable_interrupt(adev);
		cr |= DAC_CTRL_LL|ADC_CTRL_ENABLE_ALL;
		acq400wr32(adev, DAC_CTRL, cr);
	}else{
		if (adev->dma_chan == 0 &&
				ao420_dma_threshold < AO420_MAX_FIFO_SAMPLES){
			if (get_dma_chan(adev)){
				dev_err(DEVP(adev), "no dma chan");
				ao420_dma_threshold = AO420_MAX_FIFO_SAMPLES;
			}
		}
		acq400_clear_histo(adev);
		cr &= ~DAC_CTRL_LL;
		adev->AO_playloop.cursor = 0;
		acq400wr32(adev, DAC_CTRL, cr);
		ao420_fill_fifo(adev);
		acq400wr32(adev, DAC_LOTIDE, 	adev->lotide);
		ao420_enable_interrupt(adev);
		acq400wr32(adev, DAC_CTRL, cr|ADC_CTRL_ENABLE_ALL);
	}
}


void go_rt(void)
{
	struct task_struct *task = current;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 2 };

	sched_setscheduler(task, SCHED_FIFO, &param);
}

#define TIMEOUT 10000

int ai_data_loop(void *data)
{
	struct acq400_dev *adev = (struct acq400_dev *)data;
	/* wait for event from OTHER channel */
	unsigned flags[2] = { DMA_WAIT_EV1, DMA_WAIT_EV0 };
	int nloop = 0;

#define DMA_ASYNC_MEMCPY(adev, chan, hbm) \
	dma_async_memcpy_pa_to_buf(adev, adev->dma_chan[chan], hbm, \
			FIFO_PA(adev), hbm->len, flags[chan])
#define DMA_ASYNC_MEMCPY_NWFE(adev, chan, hbm) \
	dma_async_memcpy_pa_to_buf(adev, adev->dma_chan[chan], hbm, \
			FIFO_PA(adev), hbm->len, 0)


	struct HBM* hbm0 = getEmpty(adev);
	struct HBM* hbm1 = getEmpty(adev);

	dev_dbg(DEVP(adev), "hbm0:0x%08x chan:%d\n", hbm0->pa, adev->dma_chan[0]->chan_id);
	dev_dbg(DEVP(adev), "hbm1:0x%08x chan:%d\n", hbm1->pa, adev->dma_chan[1]->chan_id);

	/* prime the DMAC with buffers 0 and 1 ready to go. */
	adev->dma_cookies[1] = DMA_ASYNC_MEMCPY(adev, 1, hbm1);
	dma_async_issue_pending(adev->dma_chan[1]);
	adev->dma_cookies[0] = DMA_ASYNC_MEMCPY_NWFE(adev, 0, hbm0);
	dma_async_issue_pending(adev->dma_chan[0]);

	yield();
	go_rt();
	adev->task_active = 1;

	/* wait initial hitide interrupt?, avoid dma timeout on TRIG */


	for(; !kthread_should_stop(); ++nloop){
		int ic;
		for (ic = 0; ic < 2 && !kthread_should_stop(); ++ic){
			struct HBM* hbm;
			dev_dbg(DEVP(adev), "wait for chan %d %p %d\n", ic,
				adev->dma_chan[ic], adev->dma_cookies[ic]);

			if (wait_event_interruptible_timeout(
					adev->DMA_READY,
					adev->dma_callback_done || kthread_should_stop(),
					TIMEOUT) <= 0){
				goto quit;
			}
			--adev->dma_callback_done;
			if (kthread_should_stop()){
				goto quit;
			}
			while(dma_sync_wait(adev->dma_chan[ic], adev->dma_cookies[ic]) != DMA_SUCCESS){
				dev_err(DEVP(adev), "dma_sync_wait nloop:%d chan:%d timeout but keeping going", nloop, ic);
					//break;
				if (kthread_should_stop()){
					goto quit;
				}
			}
			dev_dbg(DEVP(adev), "SUCCESS: SYNC %d done\n", ic);

			putFull(adev);
			hbm = getEmpty(adev);
			adev->dma_cookies[ic] = DMA_ASYNC_MEMCPY(adev, ic, hbm);
			dma_async_issue_pending(adev->dma_chan[ic]);
			acq420_enable_interrupt(adev);
		}
	}
quit:
	adev->task_active = 0;
	return 0;
#undef DMA_ASYNC_MEMCPY
#undef DMA_ASYNC_MEMCPY_NWFE
}

static irqreturn_t acq400_isr(int irq, void *dev_id)
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;
	volatile u32 status = acq420_get_interrupt(adev);

	add_fifo_histo(adev, acq420_get_fifo_samples(adev));
	acq420_disable_interrupt(adev);
	//acq420_clear_interrupt(adev, status);
	adev->stats.fifo_interrupts++;
	wake_up_interruptible(&adev->w_waitq);
	return IRQ_HANDLED;
}

static irqreturn_t ao400_isr(int irq, void *dev_id)
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;
	volatile u32 status = acq420_get_interrupt(adev);

	acq420_disable_interrupt(adev);
	//acq420_clear_interrupt(adev, status);
	adev->stats.fifo_interrupts++;
	wake_up_interruptible(&adev->w_waitq);
	return IRQ_WAKE_THREAD;
}
struct file_operations acq400_fops = {
        .owner = THIS_MODULE,
        .read = acq400_read,
        .write = acq400_write,
        .open = acq400_open,
        .release = acq400_release,
        .mmap = acq400_mmap_bar
};




#ifdef CONFIG_OF
static struct of_device_id xfifodma_of_match[] /* __devinitdata */ = {
        { .compatible = "D-TACQ,acq400fmc", },
        { .compatible = "D-TACQ,acq420fmc", },
        { .compatible = "D-TACQ,acq430fmc", },
        { .compatible = "D-TACQ,acq435elf", },
        { .compatible = "D-TACQ,ao420fmc",  },
        { .compatible = "D-TACQ,acq2006sc"  },
        { .compatible = "D-TACQ,acq1001sc"  },
        { .compatible = "D-TACQ,acq1002sc"  },
        { /* end of table */}
};
MODULE_DEVICE_TABLE(of, xfifodma_of_match);
#else
#define xfifodma_of_match NULL
#endif /* CONFIG_OF */


static int acq400_device_tree_init(struct acq400_dev* adev)
{
	struct device_node *of_node = adev->pdev->dev.of_node;

        if (of_node) {
        	u32 irqs[OF_IRQ_COUNT];

        	if (of_property_read_u32(of_node, "site",
        			&adev->of_prams.site) < 0){
        		dev_warn(DEVP(adev), "error: site NOT specified in DT\n");
        		return -1;
        	}else{
        		if (!isGoodSite(adev->of_prams.site)){
        			dev_warn(DEVP(adev),
        					"warning: site %d NOT GOOD\n",
        					adev->of_prams.site);
        			return -1;
        		}else{
        			dev_info(DEVP(adev), "site:%d GOOD\n",
        					adev->of_prams.site);
        		}
        	}
        	/*
                if (of_property_read_u32(of_node, "dma-channel",
                        &adev->of_prams.dma_channel) < 0) {
                        dev_warn(DEVP(adev),
                        	"DMA channel unspecified - assuming 0\n");
                        adev->of_prams.dma_channel = 0;
                }
                dev_info(DEVP(adev),
                        "acq400_device_tree_init() read DMA channel is %d\n",
                        adev->of_prams.dma_channel);

                if (of_property_read_u32(of_node, "fifo-depth",
                        &adev->of_prams.fifo_depth) < 0) {
                        dev_warn(DEVP(adev),
                                "depth unspecified, assuming 0xffffffff\n");
                        adev->of_prams.fifo_depth = 0xffffffff;
                }
                dev_info(DEVP(adev),
                	"acq400_device_tree_init() DMA fifo depth is %d\n",
                	adev->of_prams.fifo_depth);

                if (of_property_read_u32(of_node, "burst-length",
                        &adev->of_prams.burst_length) < 0) {
                        dev_warn(DEVP(adev),
                                "burst length unspecified - assuming 1\n");
                        adev->of_prams.burst_length = 1;
                }

                dev_info(DEVP(adev),
                	"acq400_device_tree_init() DMA burst length is %d\n",
                        adev->of_prams.burst_length);
        	 */
                if (of_property_read_u32_array(
                		of_node, "interrupts", irqs, OF_IRQ_COUNT)){
                	dev_warn(DEVP(adev), "failed to find IRQ values");
                }else{
                	adev->of_prams.irq = irqs[OF_IRQ_HITIDE] + OF_IRQ_MAGIC;
                }

                return 0;
        }
        return -1;
}

static struct acq400_dev* acq400_allocate_dev(struct platform_device *pdev)
/* Allocate and init a private structure to manage this device */
{
	struct acq400_dev* adev = kzalloc(sizeof(struct acq400_dev), GFP_KERNEL);
        if (adev == NULL) {
                return NULL;
        }
        init_waitqueue_head(&adev->waitq);
        init_waitqueue_head(&adev->DMA_READY);
        init_waitqueue_head(&adev->refill_ready);
        init_waitqueue_head(&adev->hb0_marker);


        adev->pdev = pdev;
        mutex_init(&adev->mutex);
        adev->fifo_histo = kzalloc(FIFO_HISTO_SZ*sizeof(u32), GFP_KERNEL);

        INIT_LIST_HEAD(&adev->EMPTIES);
        INIT_LIST_HEAD(&adev->INFLIGHT);
        INIT_LIST_HEAD(&adev->REFILLS);
        INIT_LIST_HEAD(&adev->OPENS);
        mutex_init(&adev->list_mutex);
        init_waitqueue_head(&adev->w_waitq);
        return adev;
}

static void acq400_createDebugfs(struct acq400_dev* adev)
{
	char* pcursor;
	if (!acq400_debug_root){
		acq400_debug_root = debugfs_create_dir("acq400", 0);
		if (!acq400_debug_root){
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
			acq400_devnames[adev->of_prams.site], acq400_debug_root);

	if (!adev->debug_dir){
		dev_warn(&adev->pdev->dev, "failed create dir acq400.x");
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
		DBG_REG_CREATE(ADC_CONV_TIME);
	} else if (IS_ACQ43X(adev)){
		DBG_REG_CREATE(ACQ435_MODE);
		if (IS_ACQ435(adev)){
			DBG_REG_CREATE(ACQ435_SPADN(0));
			DBG_REG_CREATE(ACQ435_SPADN(1));
			DBG_REG_CREATE(ACQ435_SPADN(2));
			DBG_REG_CREATE(ACQ435_SPADN(3));
			DBG_REG_CREATE(ACQ435_SPADN(4));
			DBG_REG_CREATE(ACQ435_SPADN(5));
			DBG_REG_CREATE(ACQ435_SPADN(6));
			DBG_REG_CREATE(ACQ435_SPADN(7));
		}
	} else if (IS_AO420(adev)){
		DBG_REG_CREATE(AO420_RANGE);
		DBG_REG_CREATE(AO420_DACSPI);
	}


}

static void acq400_removeDebugfs(struct acq400_dev* adev)
{
	debugfs_remove_recursive(adev->debug_dir);
	kfree(adev->debug_names);
}

static void acq2006_createDebugfs(struct acq400_dev* adev)
{
	char* pcursor;
	int site;
	int sites = IS_ACQ2006SC(adev)? 6: IS_ACQ1001SC(adev)? 2: 0;
	if (!acq400_debug_root){
		acq400_debug_root = debugfs_create_dir("acq400", 0);
		if (!acq400_debug_root){
			dev_warn(&adev->pdev->dev, "failed create dir acq420");
			return;
		}
	}
	pcursor = adev->debug_names = kmalloc(4096, GFP_KERNEL);

#define DBG_REG_CREATE_2006(name, reg) 					\
	sprintf(pcursor, "%s.0x%02x", name, reg);		\
	debugfs_create_x32(pcursor, S_IRUGO, 			\
		adev->debug_dir, adev->dev_virtaddr+(reg));     \
	pcursor += strlen(pcursor) + 1

	adev->debug_dir = debugfs_create_dir(
			acq400_devnames[adev->of_prams.site], acq400_debug_root);

	if (!adev->debug_dir){
		dev_warn(&adev->pdev->dev, "failed create dir acq400.x");
		return;
	}
	DBG_REG_CREATE(MOD_ID);
	DBG_REG_CREATE(MOD_CON);
	DBG_REG_CREATE(DATA_ENGINE_0);
	DBG_REG_CREATE(DATA_ENGINE_1);
	DBG_REG_CREATE(DATA_ENGINE_2);
	DBG_REG_CREATE(DATA_ENGINE_3);

	DBG_REG_CREATE_2006("CLK_EXT", ACQ2006_CLK_COUNT(EXT_DX));
	DBG_REG_CREATE_2006("CLK_MB",  ACQ2006_CLK_COUNT(MB_DX));
	for (site = 1; site <= sites; ++site){
		char name[20];
		sprintf(name, "CLK_%d", site);
		DBG_REG_CREATE_2006(name, ACQ2006_CLK_COUNT(SITE2DX(site)));
	}

	DBG_REG_CREATE_2006("TRG_EXT", ACQ2006_TRG_COUNT(EXT_DX));
	DBG_REG_CREATE_2006("TRG_MB",  ACQ2006_TRG_COUNT(MB_DX));
	for (site = 1; site <= sites; ++site){
		char name[20];
		sprintf(name, "TRG_%d", site);
		DBG_REG_CREATE_2006(name, ACQ2006_TRG_COUNT(SITE2DX(site)));
	}

	DBG_REG_CREATE_2006("SYN_EXT", ACQ2006_SYN_COUNT(EXT_DX));
	DBG_REG_CREATE_2006("SYN_MB",  ACQ2006_SYN_COUNT(MB_DX));
	for (site = 1; site <= sites; ++site){
		char name[20];
		sprintf(name, "SYN_%d", site);
		DBG_REG_CREATE_2006(name, ACQ2006_SYN_COUNT(SITE2DX(site)));
	}

	DBG_REG_CREATE_2006("EVT_EXT", ACQ2006_EVT_COUNT(EXT_DX));
	DBG_REG_CREATE_2006("EVT_MB",  ACQ2006_EVT_COUNT(MB_DX));
	for (site = 1; site <= sites; ++site){
		char name[20];
		sprintf(name, "EVT_%d", site);
		DBG_REG_CREATE_2006(name, ACQ2006_EVT_COUNT(SITE2DX(site)));
	}
}

static int acq400_remove(struct platform_device *pdev);



static int allocate_hbm(struct acq400_dev* adev, int nb, int bl, int dir)
{
	if (hbm_allocate(DEVP(adev), nb, bl, &adev->EMPTIES, dir)){
		return -1;
	}else{
		struct HBM* cursor;
		int ix = 0;
		adev->hb = kmalloc(nb*sizeof(struct HBM*), GFP_KERNEL);
		list_for_each_entry(cursor, &adev->EMPTIES, list){
			WARN_ON(cursor->ix != ix);
			adev->hb[cursor->ix] = cursor;
			ix++;
		}
		dev_info(DEVP(adev), "setting nbuffers %d\n", ix);
		adev->nbuffers = ix;
		adev->bufferlen = bl;
		return 0;
	}
}

static int acq400_probe(struct platform_device *pdev)
{
        int rc;
        struct resource *acq400_resource;
        struct acq400_dev* adev = acq400_allocate_dev(pdev);

        if (!adev){
        	dev_err(DEVP(adev), "unable to allocate device structure\n");
        	return -ENOMEM;
        }
        pdev->dev.id = ndevices;
        /* Get our platform device resources */
        dev_dbg(DEVP(adev), "id:%d We have %d resources\n", pdev->id, pdev->num_resources);
        acq400_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (acq400_resource == NULL) {
                dev_err(&pdev->dev, "No resources found\n");
                return -ENODEV;
        }

        if (acq400_device_tree_init(adev)){
        	rc = -ENODEV;
        	goto remove;
        }

        adev->dev_physaddr = acq400_resource->start;
        adev->dev_addrsize = acq400_resource->end -
                acq400_resource->start + 1;
        if (!request_mem_region(adev->dev_physaddr,
                adev->dev_addrsize, acq400_devnames[adev->of_prams.site])) {
                dev_err(&pdev->dev, "can't reserve i/o memory at 0x%08X\n",
                        adev->dev_physaddr);
                rc = -ENODEV;
                goto fail;
        }
        adev->dev_virtaddr =
        	ioremap(adev->dev_physaddr, adev->dev_addrsize);
        dev_dbg(DEVP(adev), "acq400: mapped 0x%0x to 0x%0x\n",
        	adev->dev_physaddr, (unsigned int)adev->dev_virtaddr);

        acq400_devices[ndevices++] = adev;
        acq400_getID(adev);

        if (IS_DUMMY(adev)){
        	acq400_createSysfs(&pdev->dev);
        	dev_info(DEVP(adev), "DUMMY device detected, quitting\n");
        	return 0;
        }else if (IS_ACQ2006SC(adev) || IS_ACQ1001SC(adev)){
               	acq2006_createDebugfs(adev);
               	acq400_createSysfs(&pdev->dev);
               	return 0;
        }

        rc = alloc_chrdev_region(&adev->devno, ACQ420_MINOR_0,
        		ACQ420_MINOR_MAX, acq400_devnames[adev->of_prams.site]);
        //status = register_chrdev_region(acq420_dev->devno, 1, MODULE_NAME);
        if (rc < 0) {
                dev_err(&pdev->dev, "unable to register chrdev\n");
                goto fail;
        }

        /* Register with the kernel as a character device */
        cdev_init(&adev->cdev, &acq400_fops);
        adev->cdev.owner = THIS_MODULE;
        rc = cdev_add(&adev->cdev, adev->devno, ACQ420_MINOR_MAX);
        if (rc < 0){
        	goto fail;
        }
        if (IS_AO420(adev)){
        	rc = devm_request_threaded_irq(
        	          	DEVP(adev), adev->of_prams.irq,
        	          	ao400_isr, ao420_dma,
        	          	IRQF_SHARED, acq400_devnames[adev->of_prams.site],
        	          	adev);
        }else{
        	rc = devm_request_irq(DEVP(adev), adev->of_prams.irq, acq400_isr,
        			IRQF_SHARED, acq400_devnames[adev->of_prams.site], adev);

        }

  	if (rc){
  		dev_err(DEVP(adev),"unable to get IRQ%d\n",adev->of_prams.irq);
  		goto fail;
  	}

        if (allocate_hbm(adev,
        		IS_AO420(adev)? AO420_NBUFFERS: nbuffers,
        		IS_AO420(adev)? AO420_BUFFERLEN: bufferlen,
        		IS_AO420(adev)? DMA_TO_DEVICE: DMA_FROM_DEVICE)){
        	dev_err(&pdev->dev, "failed to allocate buffers");
        	goto fail;
        }

        if (IS_ACQ420(adev)){
        	unsigned rev = adev->mod_id&MOD_ID_REV_MASK;
        	if (rev < 3){
        		dev_err(DEVP(adev),
        		  "OBSOLETE FPGA IMAGE %u < 3, please update", rev);
        	}else{
        		dev_info(DEVP(adev), "FPGA image %u >= 3: OK", rev);
        	}
                acq420_init_defaults(adev);
        }else if (IS_ACQ43X(adev)){
        	acq43X_init_defaults(adev);
        }else if (IS_AO420(adev)){
        	ao420_init_defaults(adev);
        }else{
        	dev_warn(DEVP(adev), "no custom init for module type %x",
        			(adev)->mod_id>>MOD_ID_TYPE_SHL);
        }
        acq400_createSysfs(&pdev->dev);
        acq400_init_proc(adev);
        acq400_createDebugfs(adev);

        return 0;

 fail:
 	--ndevices;
       	dev_err(&pdev->dev, "Bailout!\n");
 remove:
        acq400_remove(pdev);
        return rc;
}

static int acq400_remove(struct platform_device *pdev)
/* undo all the probe things in reverse */
{
	if (pdev->id == -1){
		return -1;
	}else{
		struct acq400_dev* adev = acq400_devices[pdev->id];

		if (adev == 0){
			return -1;
		}

		acq400_removeDebugfs(adev);
		hbm_free(&pdev->dev, &adev->EMPTIES);
		hbm_free(&pdev->dev, &adev->REFILLS);
		acq400_delSysfs(&adev->pdev->dev);
		acq400_del_proc(adev);
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
static struct platform_driver acq400_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = xfifodma_of_match,
        },
        .probe = acq400_probe,
        .remove = acq400_remove,
};

static void __exit acq400_exit(void)
{
	platform_driver_unregister(&acq400_driver);
	acq400_module_remove_proc();

}

static int __init acq400_init(void)
{
        int status;

	printk("D-TACQ ACQ400 FMC Driver %s\n", REVID);
	DMA_NS_TEST;
	acq400_module_init_proc();
        status = platform_driver_register(&acq400_driver);

        return status;
}

module_init(acq400_init);
module_exit(acq400_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ ACQ400_FMC Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);




