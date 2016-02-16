/* ------------------------------------------------------------------------- */
/* acq400_xilinx_axidma.c ACQ420_FMC					     */
/*
 * acq400_xilinx_axidma.c
 *
 *  Created on: 10 Nov 2015
 *      Author: pgm
 */

/* ------------------------------------------------------------------------- */
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
#include "bolo.h"
#include "hbm.h"
#include "acq400_debugfs.h"
#include "acq400_lists.h"

#include "dmaengine.h"
#include <linux/amba/xilinx_dma.h>
#include "xilinx_axidma.h"

#include <linux/dmapool.h>

extern int AXI_BUFFER_COUNT;
extern int AXI_ONESHOT;
extern int bufferlen;

extern unsigned AXI_HEAD_DESCR_PA;
extern unsigned AXI_TAIL_DESCR_PA;

#define S2MM_DMACR_CYC 0x10



struct AxiDescrWrapper {
	struct xilinx_dma_desc_hw* va;
	unsigned pa;
};
#define AXI_DESCR_WRAPPER_SZ	sizeof(struct AxiDescrWrapper)

struct SEGMENT {
	enum { DUMP, STORE } mode;
	unsigned short nblocks;
};
typedef struct SEGMENT Segment;

#define ACQ400_AXIPOOL_SZ	4096

struct ACQ400_AXIPOOL {
	int ndescriptors;
	char pool_name[16];
	struct dma_pool *pool;
	struct AxiDescrWrapper *wrappers;
	unsigned dump_pa;
	int active_descriptors;
	int segment_cursor;
	Segment segments[1];		/* first segment */
};

#define GET_ACQ400_AXIPOOL(adev) \
	((struct ACQ400_AXIPOOL*)(adev)->axi_private)
#define MAX_SEGMENTS \
	((ACQ400_AXIPOOL_SZ-sizeof(struct ACQ400_AXIPOOL))/sizeof(Segment))

#define SEGDEF_LEN	5	/* [+-]NNN\n */

static void *acq400axi_proc_seq_start(struct seq_file *s, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;

        if (*pos == 0) {
        	seq_printf(s, "# AXI DESCRIPTORS %d/%d\n",
        			apool->active_descriptors, apool->ndescriptors);
        }
        if (*pos < apool->active_descriptors){
        	return &apool->wrappers[*pos];
        }

        return NULL;
}


static void *acq400axi_proc_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;

	if (++(*pos) < apool->active_descriptors){
		return &apool->wrappers[*pos];
	}else{
		return NULL;
	}
}

int pa2index(struct ACQ400_AXIPOOL* apool, unsigned pa)
{
	int ii;
	for (ii = 0; ii < apool->active_descriptors; ++ii){
		struct AxiDescrWrapper * cursor = apool->wrappers+ii;
		if (cursor->pa == pa){
			return ii;
		}
	}

	return -1;
}

static int acq400axi_proc_seq_show_descr(struct seq_file *s, void *v)
{
        struct acq400_dev *adev = s->private;
        struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;
        struct AxiDescrWrapper * cursor = v;


        seq_printf(s, "i:%08x,%03d, n:0x%08x,%03d b:%08x l:%08x %c\n",
        		cursor->pa, pa2index(apool, cursor->pa),
        		cursor->va->next_desc, pa2index(apool, cursor->va->next_desc),
        		cursor->va->buf_addr, cursor->va->control,
			apool->dump_pa==0? ' ':
				cursor->va->buf_addr==apool->dump_pa? '-': '+');

        return 0;
}

static int _initDevFromProcFile(struct file* file, struct seq_operations *seq_ops)
{
	// @@todo hack .. assumes parent is the id .. could do better?
	const char* dname = file->f_path.dentry->d_parent->d_iname;
	struct acq400_dev* adev = acq400_lookupSite(dname[0] -'0');
	if (adev->axi_private == 0){
		return -ENODEV;
	}
	((struct seq_file*)file->private_data)->private = adev;
	printk("initDevFromProcFile() 99 site:%d adev %p\n", dname[0] -'0', adev);
	return 0;
}
static int initDevFromProcFile(struct file* file, struct seq_operations *seq_ops)
{
	printk("initDevFromProcFile() 01 private %p\n", file->private_data);
	seq_open(file, seq_ops);
	printk("initDevFromProcFile() 10 private %p\n", file->private_data);
	return _initDevFromProcFile(file, seq_ops);
}

static void acq400_proc_seq_stop(struct seq_file *s, void *v)
{
}



static int acq400_proc_open_axi_descr(struct inode *inode, struct file *file)
{
	static struct seq_operations acq400_proc_seq_ops_channel_mapping = {
	        .start = acq400axi_proc_seq_start,
	        .next = acq400axi_proc_seq_next,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400axi_proc_seq_show_descr
	};

	printk("acq400_proc_open_axi_descr() 01 \n");
	return initDevFromProcFile(file, &acq400_proc_seq_ops_channel_mapping);
}
static struct file_operations acq400_proc_ops_axi_descr = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_axi_descr,
	.read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};

static void *axi_seg_proc_seq_start(struct seq_file *s, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;

        if (*pos == 0) {
        	seq_printf(s, "# DMA SEGMENTS\n");
        }
        if (*pos < apool->segment_cursor){
        	return &apool->segments[*pos];
        }
        return NULL;
}

static void *axi_seg_proc_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;
	Segment *seg = (Segment*)v;

	if (++seg - apool->segments < apool->segment_cursor){
		++(*pos);
		return (void*)seg;
	}else{
		return NULL;
	}
}

static int axi_seg_proc_seq_show(struct seq_file *s, void *v)
{
        Segment *seg = (Segment*)v;

        seq_printf(s, "%c%03d\n", seg->mode==DUMP? '-':'+', seg->nblocks);
        return 0;
}


static int acq400_proc_open_axi_segments(struct inode *inode, struct file *file)
{
	static struct seq_operations acq400_proc_seq_seg = {
	        .start = axi_seg_proc_seq_start,
	        .next = axi_seg_proc_seq_next,
	        .stop = acq400_proc_seq_stop,
	        .show = axi_seg_proc_seq_show
	};

	printk("acq400_proc_open_axi_descr() 01 \n");
	return initDevFromProcFile(file, &acq400_proc_seq_seg);
}

int addLoop(struct acq400_dev *adev, int iseg, int loopstart, int loopcount)
{
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	int iloop;
	int loop_cursor;
	int seg = iseg;

	for (iloop = 0; iloop < loopcount; ++iloop){
		for (loop_cursor = loopstart; loop_cursor < iseg;
						++seg, ++loop_cursor){
			if (seg >= MAX_SEGMENTS){
				dev_err(DEVP(adev),
					"addLoop() MAX_SEGMENTS %d exceeded",
					seg);
				return seg;
			}
			apool->segments[seg] = apool->segments[loop_cursor];
		}
	}

	return seg;
}
int parse_user_segments(struct acq400_dev *adev, char* ubuf, int count)
{
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	char *cursor = ubuf;
	char *eol;
	int iseg = 0;


	for (; cursor - ubuf < count; cursor = eol + 1){
		char skip_or_store;
		int blocks;
		int loopcount = 0;
		int loopstart = 0;

		eol = strchr(cursor, '\n');
		if (eol == NULL){
			eol = cursor+strlen(cursor);
		}else{
			*eol = '\0';
		}

		if (strlen(cursor) == 0) continue;
		if (cursor[0] == '#') continue;


		dev_dbg(DEVP(adev), "parse_user_segments() [%d] \"%s\"",
				iseg, cursor);

		if (sscanf(cursor, "loop to=%d count=%d", &loopstart, &loopcount) == 2){
			if (loopstart >= iseg){
				dev_err(DEVP(adev), "loop to %d not available", loopstart);
				return -EINVAL;
			}
			iseg = addLoop(adev, iseg, loopstart, loopcount);
			continue;
		}

		if (sscanf(cursor, "%c%d", &skip_or_store, &blocks) == 2){
			if (iseg < MAX_SEGMENTS){
				switch(skip_or_store){
				case '+':
					apool->segments[iseg].mode = STORE;
					break;
				case '-':
					apool->segments[iseg].mode = DUMP;
					break;
				default:
					dev_err(DEVP(adev), "[%d] bad mode %c\n",
							iseg, skip_or_store);
					return -EINVAL;
				}
				apool->segments[iseg++].nblocks = blocks;
			}else{
				dev_err(DEVP(adev), "[%d] no space", iseg);
				return -ENOSPC;
			}
		}else{
			dev_err(DEVP(adev),
					"[%d] bad spec \"%s\"\n", iseg, cursor);
			return -EINVAL;
		}
	}
	apool->segment_cursor = iseg;
	dev_dbg(DEVP(adev), "parse_user_segments() OK %d", iseg);
	return count;
}
/**
 * This function is called with the /proc file is written
 *
 */

ssize_t acq400_proc_segments_write(
	struct file *file, const char *buffer, size_t count, loff_t *data)
{
	const char* dname = file->f_path.dentry->d_parent->d_iname;
	struct acq400_dev *adev = acq400_lookupSite(dname[0] -'0');
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	char* lbuf;
	int rc;

	if (apool == 0){
		return -ENODEV;
	}
	if (count > MAX_SEGMENTS*SEGDEF_LEN) {
		return -EFBIG;
	}

	lbuf = kmalloc(count+1, GFP_KERNEL);
	/* write data to the buffer */
	if (copy_from_user(lbuf, buffer, count) ) {
		rc = -EFAULT;
	}else{
		lbuf[count] = '\0';
		rc = parse_user_segments(adev, lbuf, count);
	}

	kfree(lbuf);

	return rc;
}
static struct file_operations acq400_proc_ops_axi_segments = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_axi_segments,
        .read = seq_read,
	.write = acq400_proc_segments_write,
        .llseek = seq_lseek,
        .release = seq_release
};

extern int AXI_DEBUG_LOOPBACK_INDEX;


static void _finalize_descriptor_chain(struct acq400_dev *adev, int ndescriptors)
{
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	int ii;
	struct AxiDescrWrapper * cursor;

	for (ii = 0; ii < ndescriptors-1; ++ii){
		cursor = apool->wrappers+ii;
		cursor->va->next_desc = cursor[1].pa;
	}

	if (AXI_DEBUG_LOOPBACK_INDEX > 0){
		dev_info(DEVP(adev), "AXI_DEBUG_LOOPBACK_INDEX %d", AXI_DEBUG_LOOPBACK_INDEX);
		apool->wrappers[ii].va->next_desc =
				apool->wrappers[AXI_DEBUG_LOOPBACK_INDEX].pa;
	}else{
		apool->wrappers[ii].va->next_desc = apool->wrappers[0].pa;
	}
	apool->active_descriptors = ndescriptors;
}
static void init_descriptor_cache_nonseg(struct acq400_dev *adev, int ndescriptors)
{
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	int ii;
	struct AxiDescrWrapper * cursor;

	for (ii = 0; ii < ndescriptors; ++ii){
		cursor = apool->wrappers+ii;
		cursor->va = dma_pool_alloc(apool->pool, GFP_KERNEL, &cursor->pa);
		BUG_ON(cursor->va == 0);
		memset(cursor->va, 0, sizeof(struct xilinx_dma_desc_hw));
		cursor->va->buf_addr = adev->axi64_hb[ii]->pa;
		cursor->va->control = bufferlen;
	}
	_finalize_descriptor_chain(adev, ndescriptors);
}

static void init_descriptor_cache_segmented(struct acq400_dev *adev, int ndescriptors)
{
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	struct AxiDescrWrapper * wrapper = apool->wrappers;
	int last_buffer = AXI_BUFFER_COUNT-1;
	int ihb = 0;
	int iseg;

	dev_dbg(DEVP(adev), "init_descriptor_cache_segmented() 01 %d", ndescriptors);
	apool->dump_pa = adev->axi64_hb[last_buffer]->pa;

	for (iseg = 0 ; iseg < apool->segment_cursor; ++iseg){
		Segment* segment = &apool->segments[iseg];
		int ib;
		for (ib = 0; ib < segment->nblocks; ++ib, ++wrapper){
			wrapper->va =
				dma_pool_alloc(apool->pool, GFP_KERNEL, &wrapper->pa);
			memset(wrapper->va, 0, sizeof(struct xilinx_dma_desc_hw));

			if (segment->mode == STORE){
				if (ihb >= last_buffer){
					dev_info(DEVP(adev), "DONE: out of host buffers");
					break;
				}
				wrapper->va->buf_addr = adev->axi64_hb[ihb++]->pa;
			}else{
				wrapper->va->buf_addr = apool->dump_pa;
			}
			wrapper->va->control = adev->bufferlen;
		}
	}
	dev_dbg(DEVP(adev), "init_descriptor_cache_segmented() 88 %d", wrapper-apool->wrappers);
	_finalize_descriptor_chain(adev, wrapper-apool->wrappers);
}

int get_segmented_descriptor_total(struct acq400_dev *adev)
{
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	int ii;
	int ndesc = 0;

	for (ii = 0; ii < apool->segment_cursor; ++ii){
		ndesc += apool->segments[ii].nblocks;
	}
	return ndesc;
}

static void delete_pool(struct ACQ400_AXIPOOL* apool)
{
	struct AxiDescrWrapper * cursor = apool->wrappers;
	int iw = 0;
	for (; iw < apool->ndescriptors; ++iw, ++cursor){
		dma_pool_free(apool->pool, cursor->va, cursor->pa);
	}
	kfree(apool->wrappers);
	apool->ndescriptors = 0;
}
static void init_descriptor_cache(struct acq400_dev *adev)
{
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	int ndescriptors;

	if (apool->segment_cursor){
		ndescriptors = get_segmented_descriptor_total(adev);
	}else{
		ndescriptors = AXI_BUFFER_COUNT;
	}
	if (apool->ndescriptors && apool->ndescriptors < ndescriptors){
		delete_pool(apool);
	}
	if (apool->ndescriptors == 0){
		apool->wrappers =
			kzalloc(AXI_DESCR_WRAPPER_SZ*ndescriptors, GFP_KERNEL);
		apool->ndescriptors = ndescriptors;
	}
	if (apool->segment_cursor){
		init_descriptor_cache_segmented(adev, ndescriptors);
	}else{
		init_descriptor_cache_nonseg(adev, ndescriptors);
	}

}
void axi64_arm_dmac(struct xilinx_dma_chan *xchan, unsigned headpa, unsigned tailpa, unsigned oneshot)
{
	unsigned cr = dma_read(xchan, XILINX_DMA_CONTROL_OFFSET);
	unsigned halted, not_halted;
	unsigned rs_check;

	dma_write(xchan, XILINX_DMA_CONTROL_OFFSET, cr = XILINX_DMA_CR_RESET_MASK);
	dma_write(xchan, XILINX_DMA_CONTROL_OFFSET, cr = 0);
	halted = dma_read(xchan, XILINX_DMA_STATUS_OFFSET);
	dma_write(xchan, XILINX_DMA_CDESC_OFFSET, headpa);
	if (!oneshot){
		dma_write(xchan, XILINX_DMA_CONTROL_OFFSET, cr = S2MM_DMACR_CYC);
	}
	dma_write(xchan, XILINX_DMA_CONTROL_OFFSET, cr|XILINX_DMA_CR_RUNSTOP_MASK);
	rs_check = dma_read(xchan, XILINX_DMA_CONTROL_OFFSET);
	not_halted = dma_read(xchan, XILINX_DMA_STATUS_OFFSET);
	if ((rs_check&XILINX_DMA_CR_RUNSTOP_MASK) == 0){
		dev_err(xchan->dev, "NOT ENABLED CR=%08x", rs_check);
	}
	if ((not_halted&XILINX_DMA_SR_HALTED_MASK) != 0){
		dev_err(xchan->dev, "HALTED: but we wanted to GO! SR=%08x", not_halted);
	}
	dma_write(xchan, XILINX_DMA_TDESC_OFFSET, tailpa);
}

// put marker in reg to show we were there ...
#define SHOTID(adev)	(((adev->stats.shot&0x00ff)<<8)|0xcafe0000)

extern int wimp_out;

int _axi64_load_dmac(struct acq400_dev *adev)
{
	struct xilinx_dma_chan *xchan = to_xilinx_chan(adev->dma_chan[0]);
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);

	u32 head_pa = apool->wrappers[0].pa;
	u32 tail_pa = AXI_ONESHOT? apool->wrappers[apool->ndescriptors-1].pa:
			SHOTID(adev);

	if (!wimp_out){
		axi64_arm_dmac(xchan, head_pa, tail_pa, AXI_ONESHOT);
	}
	return 0;
}



int axi64_init_dmac(struct acq400_dev *adev)
{
	struct ACQ400_AXIPOOL* apool = kzalloc(ACQ400_AXIPOOL_SZ, GFP_KERNEL);

	dev_dbg(DEVP(adev), "axi64_init_dmac() 01");
	snprintf(apool->pool_name, 16, "acq400_axi_pool");
	apool->pool = dma_pool_create(apool->pool_name, DEVP(adev),
		sizeof(struct xilinx_dma_desc_hw), 0x40, 0);

	BUG_ON(apool->pool == 0);
	adev->axi_private = apool;

	proc_create("AXIDESCR", 0, adev->proc_entry, &acq400_proc_ops_axi_descr);
	proc_create("SEGMENTS", 0, adev->proc_entry, &acq400_proc_ops_axi_segments);
	dev_dbg(DEVP(adev), "axi64_init_dmac() 99");
	return 0;
}

int axi64_load_dmac(struct acq400_dev *adev)
{
	if (adev->axi_private == 0){
		axi64_init_dmac(adev);
	}
	init_descriptor_cache(adev);
	return _axi64_load_dmac(adev);
}


int axi64_free_dmac(struct acq400_dev *adev)
{
	struct ACQ400_AXIPOOL* apool = GET_ACQ400_AXIPOOL(adev);
	int ii;
	struct AxiDescrWrapper * cursor;

	adev->axi_private = 0;

	for (ii = 0; ii < apool->ndescriptors; ++ii){
		cursor = apool->wrappers+ii;
		dma_pool_free(apool->pool, cursor->va, cursor->pa);
	}
	kfree(apool->wrappers);
	dma_pool_destroy(apool->pool);
	kfree(apool);
	return 0;
}
