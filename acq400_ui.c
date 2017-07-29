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
	rc = copy_to_user(buf, (char*)hb->va + cursor, count);
	if (rc){
		return -1;
	}

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

