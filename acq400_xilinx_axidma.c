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


/*
 * seq_files have a buffer which can may overflow. When this happens a larger
 * buffer is reallocated and all the data will be printed again.
 * The overflow state is true when m->count == m->size.
 */
static bool seq_overflow(struct seq_file *m)
{
	return m->count == m->size;
}

static int traverse(struct seq_file *m, loff_t offset)
{
	loff_t pos = 0, index;
	int error = 0;
	void *p;

	m->version = 0;
	index = 0;
	m->count = m->from = 0;
	if (!offset) {
		m->index = index;
		return 0;
	}
	if (!m->buf) {
		m->buf = kmalloc(m->size = PAGE_SIZE, GFP_KERNEL);
		if (!m->buf)
			return -ENOMEM;
	}
	p = m->op->start(m, &index);
	while (p) {
		error = PTR_ERR(p);
		if (IS_ERR(p))
			break;
		error = m->op->show(m, p);
		if (error < 0)
			break;
		if (unlikely(error)) {
			error = 0;
			m->count = 0;
		}
		if (seq_overflow(m))
			goto Eoverflow;
		if (pos + m->count > offset) {
			m->from = offset - pos;
			m->count -= m->from;
			m->index = index;
			break;
		}
		pos += m->count;
		m->count = 0;
		if (pos == offset) {
			index++;
			m->index = index;
			break;
		}
		p = m->op->next(m, p, &index);
	}
	m->op->stop(m, p);
	m->index = index;
	return error;

Eoverflow:
	m->op->stop(m, p);
	kfree(m->buf);
	m->count = 0;
	m->buf = kmalloc(m->size <<= 1, GFP_KERNEL);
	return !m->buf ? -ENOMEM : -EAGAIN;
}


/**
 *	seq_read -	->read() method for sequential files.
 *	@file: the file to read from
 *	@buf: the buffer to read to
 *	@size: the maximum number of bytes to read
 *	@ppos: the current position in the file
 *
 *	Ready-made ->f_op->read()
 */
ssize_t acq400_seq_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	size_t copied = 0;
	loff_t pos;
	size_t n;
	void *p;
	int err = 0;

	mutex_lock(&m->lock);

	/*
	 * seq_file->op->..m_start/m_stop/m_next may do special actions
	 * or optimisations based on the file->f_version, so we want to
	 * pass the file->f_version to those methods.
	 *
	 * seq_file->version is just copy of f_version, and seq_file
	 * methods can treat it simply as file version.
	 * It is copied in first and copied out after all operations.
	 * It is convenient to have it as  part of structure to avoid the
	 * need of passing another argument to all the seq_file methods.
	 */
	m->version = file->f_version;

	/* Don't assume *ppos is where we left it */
	if (unlikely(*ppos != m->read_pos)) {
		while ((err = traverse(m, *ppos)) == -EAGAIN)
			;
		if (err) {
			/* With prejudice... */
			m->read_pos = 0;
			m->version = 0;
			m->index = 0;
			m->count = 0;
			goto Done;
		} else {
			m->read_pos = *ppos;
		}
	}

	/* grab buffer if we didn't have one */
	if (!m->buf) {
		m->buf = kmalloc(m->size = PAGE_SIZE, GFP_KERNEL);
		if (!m->buf)
			goto Enomem;
	}
	/* if not empty - flush it first */
	if (m->count) {
		n = min(m->count, size);
		err = copy_to_user(buf, m->buf + m->from, n);
		if (err)
			goto Efault;
		m->count -= n;
		m->from += n;
		size -= n;
		buf += n;
		copied += n;
		if (!m->count)
			m->index++;
		if (!size)
			goto Done;
	}
	/* we need at least one record in buffer */
	pos = m->index;
	p = m->op->start(m, &pos);
	while (1) {
		err = PTR_ERR(p);
		if (!p || IS_ERR(p))
			break;
		err = m->op->show(m, p);
		if (err < 0)
			break;
		if (unlikely(err))
			m->count = 0;
		if (unlikely(!m->count)) {
			p = m->op->next(m, p, &pos);
			m->index = pos;
			continue;
		}
		if (m->count < m->size)
			goto Fill;
		m->op->stop(m, p);
		kfree(m->buf);
		m->count = 0;
		m->buf = kmalloc(m->size <<= 1, GFP_KERNEL);
		if (!m->buf)
			goto Enomem;
		m->version = 0;
		pos = m->index;
		p = m->op->start(m, &pos);
	}
	m->op->stop(m, p);
	m->count = 0;
	goto Done;
Fill:
	/* they want more? let's try to get some more */
	while (m->count < size) {
		size_t offs = m->count;
		loff_t next = pos;
		p = m->op->next(m, p, &next);
		if (!p || IS_ERR(p)) {
			err = PTR_ERR(p);
			break;
		}
		err = m->op->show(m, p);
		if (seq_overflow(m) || err) {
			m->count = offs;
			if (likely(err <= 0))
				break;
		}
		pos = next;
	}
	m->op->stop(m, p);
	n = min(m->count, size);
	err = copy_to_user(buf, m->buf, n);
	if (err)
		goto Efault;
	copied += n;
	m->count -= n;
	if (m->count)
		m->from = n;
	else
		pos++;
	m->index = pos;
Done:
	if (!copied)
		copied = err;
	else {
		*ppos += copied;
		m->read_pos += copied;
	}
	file->f_version = m->version;
	mutex_unlock(&m->lock);
	return copied;
Enomem:
	err = -ENOMEM;
	goto Done;
Efault:
	err = -EFAULT;
	goto Done;
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
        .read = acq400_seq_read,
        .llseek = seq_lseek,
        .release = seq_release
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
