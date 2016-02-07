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

struct ACQ400_AXIPOOL {
	int ndescriptors;
	char pool_name[16];
	struct dma_pool *pool;
	struct AxiDescrWrapper *wrappers;
};


static void *acq400axi_proc_seq_start(struct seq_file *s, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;

        if (*pos == 0) {
        	seq_printf(s, "AXI DESCRIPTORS\n");
        }
        if (*pos < apool->ndescriptors){
        	return &apool->wrappers[*pos];
        }

        return NULL;
}


static void *acq400axi_proc_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;

	if (++(*pos) < apool->ndescriptors){
		return &apool->wrappers[*pos];
	}else{
		return NULL;
	}
}

int pa2index(struct ACQ400_AXIPOOL* apool, unsigned pa)
{
	int ii;
	for (ii = 0; ii < apool->ndescriptors; ++ii){
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


        seq_printf(s, "i:%08x,%03d, n:0x%08x,%03d b:%08x l:%08x\n",
        		cursor->pa, pa2index(apool, cursor->pa),
        		cursor->va->next_desc, pa2index(apool, cursor->va->next_desc),
        		cursor->va->buf_addr, cursor->va->control);

        return 0;
}

static int intDevFromProcFile(struct file* file, struct seq_operations *seq_ops)
{
	printk("intDevFromProcFile() 01 private %p\n", file->private_data);
	seq_open(file, seq_ops);
	printk("intDevFromProcFile() 10 private %p\n", file->private_data);

	{
		// @@todo hack .. assumes parent is the id .. could do better?
		const char* dname = file->f_path.dentry->d_parent->d_iname;
		void* adev = acq400_lookupSite(dname[0] -'0');
		((struct seq_file*)file->private_data)->private = adev;
		printk("intDevFromProcFile() 99 site:%d adev %p\n", dname[0] -'0', adev);
	}

	return 0;
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
	return intDevFromProcFile(file, &acq400_proc_seq_ops_channel_mapping);
}
static struct file_operations acq400_proc_ops_axi_descr = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_axi_descr,
	.read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};



static struct file_operations acq400_proc_ops_axi_segments = {
        .owner = THIS_MODULE,
/*
        .open = acq400_proc_open_axi_segments,
        .read = acq400_seq_segments_read,
	.write = acq400_seq_segments_write,
        .llseek = seq_lseek,
        .release = seq_release
*/
};

extern int AXI_DEBUG_LOOPBACK_INDEX;
static void init_descriptor_cache(struct acq400_dev *adev)
{
	struct ACQ400_AXIPOOL* apool = kzalloc(sizeof(struct ACQ400_AXIPOOL), GFP_KERNEL);
	int ii;
	struct AxiDescrWrapper * cursor;

	apool->ndescriptors = AXI_BUFFER_COUNT;
	snprintf(apool->pool_name, 16, "acq400_axi_pool");
	apool->pool = dma_pool_create(apool->pool_name, DEVP(adev),
		sizeof(struct xilinx_dma_desc_hw), 0x40, 0);

	BUG_ON(apool->pool == 0);

	apool->wrappers = kzalloc(sizeof(struct AxiDescrWrapper)*apool->ndescriptors, GFP_KERNEL);

	for (ii = 0; ii < apool->ndescriptors; ++ii){
		cursor = apool->wrappers+ii;
		cursor->va = dma_pool_alloc(apool->pool, GFP_KERNEL, &cursor->pa);
		BUG_ON(cursor->va == 0);
		memset(cursor->va, 0, sizeof(struct xilinx_dma_desc_hw));
		cursor->va->buf_addr = adev->axi64_hb[ii]->pa;
		cursor->va->control = bufferlen;
	}
	for (ii = 0; ii < apool->ndescriptors-1; ++ii){
		cursor = apool->wrappers+ii;
		cursor->va->next_desc = cursor[1].pa;
	}

	if (AXI_DEBUG_LOOPBACK_INDEX > 0){
		dev_info(DEVP(adev), "AXI_DEBUG_LOOPBACK_INDEX %d", AXI_DEBUG_LOOPBACK_INDEX);
		apool->wrappers[ii].va->next_desc = apool->wrappers[AXI_DEBUG_LOOPBACK_INDEX].pa;
	}else{
		apool->wrappers[ii].va->next_desc = apool->wrappers[0].pa;
	}
	adev->axi_private = apool;

	proc_create("AXIDESCR", 0, adev->proc_entry, &acq400_proc_ops_axi_descr);
	proc_create("SEGMENTS", 0, adev->proc_entry, &acq400_proc_ops_axi_segments);
}
void axi64_arm_dmac(struct xilinx_dma_chan *xchan, unsigned headpa, unsigned tailpa, unsigned oneshot)
{
	unsigned cr = dma_read(xchan, XILINX_DMA_CONTROL_OFFSET);
	unsigned halted, not_halted;
	unsigned rs_check;
	dev_dbg(xchan->dev, "axi64_arm_dmac() 01");
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
	dev_dbg(xchan->dev, "axi64_arm_dmac() 99");
}

// put marker in reg to show we were there ...
#define SHOTID(adev)	(((adev->stats.shot&0x00ff)<<8)|0xcafe0000)

int _axi64_load_dmac(struct acq400_dev *adev)
{
	struct xilinx_dma_chan *xchan = to_xilinx_chan(adev->dma_chan[0]);
	struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;
	u32 head_pa = apool->wrappers[0].pa;
	u32 tail_pa = AXI_ONESHOT? apool->wrappers[apool->ndescriptors-1].pa:
			SHOTID(adev);

	axi64_arm_dmac(xchan, head_pa, tail_pa, AXI_ONESHOT);
	return 0;
}

int axi64_load_dmac(struct acq400_dev *adev)
{

	if (adev->axi_private == 0){
		init_descriptor_cache(adev);
	}
	return _axi64_load_dmac(adev);
}

int axi64_free_dmac(struct acq400_dev *adev)
{
	struct ACQ400_AXIPOOL* apool = (struct ACQ400_AXIPOOL*)adev->axi_private;
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
