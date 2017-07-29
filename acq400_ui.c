/* ------------------------------------------------------------------------- */
/* acq400_ui.c  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 29 Jul 2017  			/ User: pgm
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2017 Peter Milne, D-TACQ Solutions Ltd         *
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                *
 *
 * TODO 
 * TODO
 * ------------------------------------------------------------------------- */

#include "acq400.h"
#include "bolo.h"
#include "hbm.h"

#include "acq400_lists.h"
#include "acq400_ui.h"

int xo400_awg_open(struct inode *inode, struct file *file)
/* if write mode, reset length */
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	if ( (file->f_flags & O_ACCMODE) == O_WRONLY) {
		xo400_reset_playloop(adev, adev->AO_playloop.length = 0);
	}
	return 0;
}
int xo400_awg_release(struct inode *inode, struct file *file)
/* if it was a write, commit to memory and set length */
{
	struct acq400_dev* adev = ACQ400_DEV(file);

	if ( (file->f_flags & O_ACCMODE) != O_RDONLY) {
		adev->AO_playloop.oneshot =
			iminor(inode) == AO420_MINOR_HB0_AWG_ONCE? AO_oneshot:
			iminor(inode) == AO420_MINOR_HB0_AWG_ONCE_RETRIG? AO_oneshot_rearm:
					AO_continuous;
		xo400_reset_playloop(adev, adev->AO_playloop.length);
	}
	return 0;
}

ssize_t xo400_awg_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int pll = AOSAMPLES2BYTES(adev, adev->AO_playloop.length);
	unsigned len = adev->cursor.hb[0]->len;

	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	unsigned ib = bcursor/len;
	unsigned offset = bcursor - ib*len;
	int rc;

	if (bcursor >= pll){
		return 0;
	}else{
		int headroom = min(len-offset, pll-bcursor);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, (char*)adev->cursor.hb[ib]->va + offset, count);
	if (rc){
		return -1;
	}


	*f_pos += count;
	return count;
}

void flip_x16_sign_bit(unsigned short* bp, int nwords)
{
	const unsigned short sbit = 1<<15;

	while(nwords--){
		*bp++ ^= sbit;
	}
}
ssize_t xo400_awg_write(
	struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	unsigned len = adev->cursor.hb[0]->len;

	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	unsigned ib = bcursor/len;
	/** @@todo check ib within bounds .. what bounds? */
	unsigned offset = bcursor - ib*len;
	int headroom = (len - offset);
	char* dst = (char*)adev->cursor.hb[ib]->va + offset;
	int rc;

	if (count > headroom){
		count = headroom;
	}

	rc = copy_from_user(dst, buf, count);

	if (rc){
		return -1;
	}
	if (IS_AO424(adev) && SPAN_IS_BIPOLAR(adev)){
		flip_x16_sign_bit((unsigned short*)dst, count/sizeof(unsigned short));
	}

	dev_dbg(DEVP(adev),
		"dma_sync_single_for_device() %08x %d len:%d demand:%d %d (DMA_TO_DEVICE=%d)",
		adev->cursor.hb[ib]->pa + offset, count,
		len, bcursor+count,
		adev->cursor.hb[ib]->dir, DMA_TO_DEVICE);

	dma_sync_single_for_device(DEVP(adev),
			adev->cursor.hb[ib]->pa + offset, count, adev->cursor.hb[ib]->dir);

	*f_pos += count;
	adev->AO_playloop.length += AOBYTES2SAMPLES(adev, count);
	return count;
}

int xo400_open_awg(struct inode *inode, struct file *file)
{
	static struct file_operations fops = {
			.open = xo400_awg_open,
			.write = xo400_awg_write,
			.read = xo400_awg_read,
			.release = xo400_awg_release,
	};
	file->f_op = &fops;
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



int acq400_open_histo(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_histo = {
			.read = acq400_histo_read,
	};
	acq400_fops_histo.release = file->f_op->release;
	file->f_op = &acq400_fops_histo;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
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

ssize_t acq400_hb_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ibuf = BUFFER(PD(file)->minor);
	struct HBM *hb = adev->hb[ibuf];
	unsigned cursor = *f_pos;	/* f_pos counts in bytes */
	int rc;

	if (cursor >= hb->len){
		return 0;
	}else{
		int headroom = hb->len - cursor;
		if (count > headroom){
			count = headroom;
		}
	}
	dma_sync_single_for_cpu(DEVP(adev), hb->pa + cursor, count, hb->dir);

	rc = copy_to_user(buf, (char*)hb->va + cursor, count);
	if (rc){
		return -1;
	}

	*f_pos += count;
	return count;
}

ssize_t acq400_hb_write(
	struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ibuf = BUFFER(PD(file)->minor);
	struct HBM *hb = adev->hb[ibuf];
	unsigned cursor = *f_pos;	/* f_pos counts in bytes */
	int rc;

	if (cursor >= hb->len){
		return 0;
	}else{
		int headroom = hb->len - cursor;
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_from_user((char*)hb->va+cursor, buf, count);

	if (rc){
		return -1;
	}

	dma_sync_single_for_device(DEVP(adev),	hb->pa + cursor, count, hb->dir);

	*f_pos += count;
	return count;
}


int acq400_hb_release(struct inode *inode, struct file *file)
{
        struct acq400_dev *adev = ACQ400_DEV(file);
        int ibuf = BUFFER(PD(file)->minor);
        struct HBM *hbm = adev->hb[ibuf];

        /* Manage writes via reference counts */
        switch (file->f_flags & O_ACCMODE) {
        case O_WRONLY:
        case O_RDWR:
        	dev_dbg(DEVP(adev),
        	"acq400_hb_release() mode %x, dma_sync_single_for_device: pa:0x%08x len:%d dir:%d",
		file->f_flags & O_ACCMODE, hbm->pa, hbm->len, hbm->dir);
        	dma_sync_single_for_device(DEVP(adev), hbm->pa, hbm->len, hbm->dir);
        default:
        	;
        }

        return acq400_release(inode, file);
}



int acq400_open_hb(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_fops_dma = {
		.open = acq420_dma_open,
		.mmap = acq400_dma_mmap_host,
		.read = acq400_hb_read,
		.write = acq400_hb_write,
		.release = acq400_hb_release,
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
	sprintf(HB0, "%d %d\n", adev->rt.hb0_ix[0], adev->rt.hb0_ix[1]);
	count = min(count, strlen(HB0)+1);
	rc = copy_to_user(buf, HB0, count);
	if (rc){
		return -1;
	}

	*f_pos += count;
	return count;
}

int acq420_open_hb0(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_hb0 = {
			.read = acq400_hb0_read,
			.release = acq400_release
	};
	file->f_op = &acq400_fops_hb0;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}



int acq400_gpgmem_open(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	if (file->f_flags & O_WRONLY) {
		int iw;
		for (iw = 0; iw < adev->gpg_cursor; ++iw){
			iowrite32(0, adev->gpg_base+iw);
		}
		adev->gpg_cursor = 0;
	}
	return 0;
}
ssize_t acq400_gpgmem_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int len = adev->gpg_cursor*sizeof(u32);
	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	int rc;

	if (bcursor >= len){
		return 0;
	}else{
		int headroom = (len - bcursor);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, adev->gpg_buffer+bcursor, count);
	if (rc){
		return -1;
	}

	*f_pos += count;
	return count;
}

ssize_t acq400_gpgmem_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int len = GPG_MEM_ACTUAL;
	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	int rc;

	if (bcursor >= len){
		return 0;
	}else{
		int headroom = (len - bcursor);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_from_user(adev->gpg_buffer+bcursor, buf, count);
	if (rc){
		return -1;
	}
	*f_pos += count;
	adev->gpg_cursor += count/sizeof(u32);
	return count;

}

int set_gpg_top(struct acq400_dev* adev, u32 gpg_count)
{
	if (gpg_count >= 2){
		u32 gpg_ctrl = acq400rd32(adev, GPG_CTRL);
		u32 gpg_top = gpg_count - 1		// was count, not address
					 -1;		// GPG_2ND_LAST_ADDR
		gpg_top <<= GPG_CTRL_TOPADDR_SHL;
		gpg_top &= GPG_CTRL_TOPADDR;
		gpg_ctrl &= ~GPG_CTRL_TOPADDR;
		gpg_ctrl |= gpg_top;
		acq400wr32(adev, GPG_CTRL, gpg_ctrl);
		return 0;
	}else{
		dev_err(DEVP(adev), "set_gpg_top() ERROR: must have 2 or more entries");
		return -1;
	}
}

int acq400_gpgmem_release(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	unsigned* src = (unsigned *)adev->gpg_buffer;
	int iw;
	int rc;

	for (iw = 0; iw < adev->gpg_cursor; ++iw){
		iowrite32(src[iw], adev->gpg_base+iw);
	}
	dev_dbg(DEVP(adev), "acq400_gpgmem_release() %d\n", iw);
	rc = set_gpg_top(adev, adev->gpg_cursor);
	acq400_release(inode, file);
	return rc;
}

int acq420_open_gpgmem(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_gpgmem = {
			.open = acq400_gpgmem_open,
			.read = acq400_gpgmem_read,
			.write = acq400_gpgmem_write,
			.release = acq400_gpgmem_release
	};
	file->f_op = &acq400_fops_gpgmem;
	return file->f_op->open(inode, file);
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


int acq400_mmap_bar_page(struct file* file, struct vm_area_struct* vma)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = 0x1000;
	unsigned long pa = adev->dev_physaddr + ACQ400_MINOR_MAP_PAGE_OFFSET(PD(file)->minor);
	unsigned pfn = pa>> PAGE_SHIFT;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	dev_dbg(DEVP(adev), "acq400_mmap_bar_page pa:0x%08lx vsize %lu psize %lu %s",
		pa, vsize, psize, vsize>psize? "EINVAL": "OK");

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

int acq400_map_page_open(struct inode* inode, struct file* file)
{
	static struct file_operations acq400_map_page_fops = {
	        .owner = THIS_MODULE,
	        .release = acq400_release,
	        .mmap = acq400_mmap_bar_page
	};
	file->f_op = &acq400_map_page_fops;
	return 0;
}

int acq400_mmap_bar_atd(struct file* file, struct vm_area_struct* vma)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = AXI_ATD_LEN;
	unsigned pfn = (adev->dev_physaddr + AXI_ATD_RAM)>> PAGE_SHIFT;

	dev_dbg(DEVP(adev), "%c vsize %lu psize %lu %s",
		'D', vsize, psize, vsize>psize? "EINVAL": "OK");

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
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

int acq400_atd_open(struct inode* inode, struct file* file)
{
	static struct file_operations acq400_atd_fops = {
	        .owner = THIS_MODULE,
	        .release = acq400_release,
	        .mmap = acq400_mmap_bar_atd
	};
	file->f_op = &acq400_atd_fops;
	return 0;
}

ssize_t acq420_sew_fifo_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ix = PD(file)->minor - ACQ420_MINOR_SEW1_FIFO;

	int rc = acq400_sew_fifo_write_bytes(adev, ix, buf, count);

	if (rc > 0){
		*f_pos += rc;
	}
	return rc;
}
int acq420_sew_fifo_release(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ix = PD(file)->minor - ACQ420_MINOR_SEW1_FIFO;
	return acq400_sew_fifo_destroy(adev, ix);
}

int acq420_sew_fifo_open(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_sew_fifo = {
			.write = acq420_sew_fifo_write,
			.release = acq420_sew_fifo_release,
	};
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ix = PD(file)->minor - ACQ420_MINOR_SEW1_FIFO;
	int busy;

	if (mutex_lock_interruptible(&adev->awg_mutex)) {
		return -EINTR;
	}
	busy = adev->sewFifo[ix].sf_task != 0;
	if (!busy){
		acq400_sew_fifo_init(adev, ix);
	}
	mutex_unlock(&adev->awg_mutex);
	if (busy){
		return -EBUSY;
	}

	file->f_op = &acq420_sew_fifo;
	return 0;
}




int acq420_reserve_release(struct inode *inode, struct file *file)
/* if it was a write, commit to memory and set length */
{
	replace_all(PD(file));
	return acq400_release(inode, file);
}

ssize_t acq420_reserve_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	wait_queue_head_t waitq;	/* no one gonna wake this sucker */

	init_waitqueue_head(&waitq);

	wait_event_interruptible(waitq, 0);  /* but it will accept a signal .. */
	return -EINTR;
}

int acq420_reserve_dist_open(struct inode *inode, struct file *file)
/* simply reserve block 0 on open. Can get clever later */
{
	static struct file_operations fops = {
		.open = acq420_reserve_dist_open,
		.read = acq420_reserve_read,
		.release = acq420_reserve_release,
	};

	file->f_op = &fops;
	return acq400_reserve_dist_buffers(PD(file));
}
int acq420_reserve_open(struct inode *inode, struct file *file)
/* simply reserve block 0 on open. Can get clever later */
{
	static struct file_operations fops = {
		.open = acq420_reserve_open,
		.read = acq420_reserve_read,
		.release = acq420_reserve_release,
	};
	file->f_op = &fops;
	return reserve(PD(file), 0);
}




