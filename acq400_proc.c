/* ------------------------------------------------------------------------- */
/* acq400_proc.c  		                     	 */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2012 pgm, D-TACQ Solutions Ltd                    *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
 *   Created on: Mar 24, 2013
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
 * acq400_proc.c
 *
 *  Created on: Mar 24, 2013
 *      Author: pgm
 */

#include "acq400.h"
#include "hbm.h"

static struct proc_dir_entry *acq400_proc_root;

/* proc_seq_single: single call to show only ..  bad use of proc_seq..*/
static void *acq400_proc_seq_single_start(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
                return s->private;
        }

        return NULL;
}

static void *acq400_proc_seq_single_next(struct seq_file *s, void *v, loff_t *pos)
{
        (*pos)++;
        return NULL;
}

static void acq400_proc_seq_stop(struct seq_file *s, void *v)
{
}

static int acq400_proc_seq_show_dma(struct seq_file *s, void *v)
{
        struct acq400_dev *adev = v;

        if (mutex_lock_interruptible(&adev->mutex)) {
                return -EINTR;
        }

        seq_printf(s, "\nFIFO DMA Test:\n\n");
        seq_printf(s, "Device Physical Address: 0x%0x\n", adev->dev_physaddr);
        seq_printf(s, "Device Virtual Address:  0x%0x\n",
                (u32)adev->dev_virtaddr);
        seq_printf(s, "Device Address Space:    %d bytes\n", adev->dev_addrsize);
        seq_printf(s, "DMA Channel:             %d\n", adev->of_prams.dma_channel);
        seq_printf(s, "FIFO Depth:              %d bytes\n", adev->of_prams.fifo_depth);
        seq_printf(s, "Burst Length:            %d words\n", adev->of_prams.burst_length);
        seq_printf(s, "\n");
        seq_printf(s, "Opens:                   %d\n", adev->stats.opens);
        seq_printf(s, "Writes:                  %d\n", adev->stats.writes);
        seq_printf(s, "Bytes Written:           %d\n", adev->stats.bytes_written);
        seq_printf(s, "Closes:                  %d\n", adev->stats.closes);
        seq_printf(s, "Errors:                  %d\n", adev->stats.errors);
        seq_printf(s, "Busy:                    %d\n", adev->busy);
        seq_printf(s, "\n");

        mutex_unlock(&adev->mutex);
        return 0;
}

static int acq400_proc_seq_show_qstats(struct seq_file *s, void *v)
{
        struct acq400_dev *adev = v;
        int stats[_BS_MAX] = {};
        int ibuf;
        int istat;

        mutex_lock(&adev->list_mutex);
        for (ibuf = 0; ibuf < adev->nbuffers; ++ibuf){
        	stats[adev->hb[ibuf]->bstate]++;
        }
        mutex_unlock(&adev->list_mutex);

        for(istat = 0; istat < _BS_MAX; ++istat){
        	seq_printf(s, "%d%c", stats[istat], istat+1==_BS_MAX? '\n':',');
        }
        return 0;
}
static int acq400_proc_seq_show_stats(struct seq_file *s, void *v)
{
        struct acq400_dev *adev = v;

        if (mutex_lock_interruptible(&adev->mutex)) {
                return -EINTR;
        }

        seq_printf(s, "ngets=%d\nnputs=%d\n", adev->rt.nget, adev->rt.nput);
        seq_printf(s, "refill_error=%d\nplease_stop=%d\n",
        		adev->rt.refill_error, adev->rt.please_stop);
        mutex_unlock(&adev->mutex);
        return 0;
}


static int intDevFromProcFile(struct file* file, struct seq_operations *seq_ops)
{
	seq_open(file, seq_ops);
	// @@todo hack .. assumes parent is the id .. could do better?
	((struct seq_file*)file->private_data)->private =
	            acq400_devices[file->f_path.dentry->d_parent->d_iname[0] -'0'];
	return 0;
}
static int acq400_proc_open_dmac(struct inode *inode, struct file *file)
{
	static struct seq_operations acq400_proc_seq_ops_dma = {
	        .start = acq400_proc_seq_single_start,
	        .next = acq400_proc_seq_single_next,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400_proc_seq_show_dma
	};

	return intDevFromProcFile(file, &acq400_proc_seq_ops_dma);
}

static int acq400_proc_open_qstats(struct inode *inode, struct file *file)
{
	static struct seq_operations acq400_proc_seq_ops_qstats = {
	        .start = acq400_proc_seq_single_start,
	        .next = acq400_proc_seq_single_next,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400_proc_seq_show_qstats
	};

	return intDevFromProcFile(file, &acq400_proc_seq_ops_qstats);
}
static int acq400_proc_open_stats(struct inode *inode, struct file *file)
{
	static struct seq_operations acq400_proc_seq_ops_dma = {
	        .start = acq400_proc_seq_single_start,
	        .next = acq400_proc_seq_single_next,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400_proc_seq_show_stats
	};

	return intDevFromProcFile(file, &acq400_proc_seq_ops_dma);
}

static void *acq400_proc_seq_start_buffers(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
        	struct acq400_dev *adev = s->private;
        	seq_printf(s, "Buffers\n");
        	return adev->hb[0];
        }

        return NULL;
}


static void *acq400_proc_seq_next_buffers(struct seq_file *s, void *v, loff_t *pos)
{
	struct acq400_dev *adev = s->private;

	if (++*pos < adev->nbuffers){
		return adev->hb[*pos];
	}else{
		return NULL;
	}
}


static int acq400_proc_seq_show_buffers(struct seq_file *s, void *v)
{
        struct acq400_dev *dev = s->private;
        struct HBM * hbm = v;

        if (mutex_lock_interruptible(&dev->mutex)) {
                return -EINTR;
        }

        seq_printf(s, "[%02d] va:%p pa:0x%08x len:%d dir:%d state:%d\n",
        		hbm->ix, hbm->va, hbm->pa, hbm->len, hbm->dir, hbm->bstate);
        mutex_unlock(&dev->mutex);
        return 0;
}


static int acq400_proc_open_buffers(struct inode *inode, struct file *file)
{
	/* SEQ operations for /proc */
	static struct seq_operations acq400_proc_seq_ops_buffers = {
	        .start = acq400_proc_seq_start_buffers,
	        .next = acq400_proc_seq_next_buffers,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400_proc_seq_show_buffers
	};

	return intDevFromProcFile(file, &acq400_proc_seq_ops_buffers);
}



#define DEF_PROC_OPSQ(QNAME) \
static void *acq400_proc_seq_start_##QNAME(struct seq_file *s, loff_t *pos)	\
{										\
        if (*pos == 0) {							\
        	struct acq400_dev *adev = s->private;				\
        	if (!list_empty(&adev->QNAME)){					\
        		return list_first_entry(&adev->QNAME, struct HBM, list);\
        	}								\
        }									\
        									\
        return NULL;								\
}										\
static void *acq400_proc_seq_next_##QNAME(struct seq_file *s, void *v, loff_t *pos) \
{										\
	struct acq400_dev *adev = s->private;					\
	int tpos = 0;								\
	struct HBM *hbm;							\
										\
	++*pos;									\
										\
	list_for_each_entry(hbm, &adev->QNAME, list){				\
		if (tpos++ == *pos){						\
			return hbm;						\
		}								\
	}									\
										\
	return NULL;								\
}										\
static int acq400_proc_open_##QNAME(struct inode *inode, struct file *file)	\
{										\
	/* SEQ operations for /proc */						\
	static struct seq_operations acq400_proc_seq_ops_buffers = {		\
	        .start = acq400_proc_seq_start_##QNAME,				\
	        .next = acq400_proc_seq_next_##QNAME,				\
	        .stop = acq400_proc_seq_stop,					\
	        .show = acq400_proc_seq_show_buffers				\
	};									\
										\
	return intDevFromProcFile(file, &acq400_proc_seq_ops_buffers);		\
}										\
										\
static struct file_operations acq400_proc_ops_##QNAME = {			\
        .owner = THIS_MODULE,							\
        .open = acq400_proc_open_##QNAME,					\
        .read = seq_read,							\
        .llseek = seq_lseek,							\
        .release = seq_release							\
}

DEF_PROC_OPSQ(EMPTIES);
DEF_PROC_OPSQ(INFLIGHT);
DEF_PROC_OPSQ(REFILLS);
DEF_PROC_OPSQ(OPENS);


static struct file_operations acq400_proc_ops_dmac = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_dmac,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};

static struct file_operations acq400_proc_ops_qstats = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_qstats,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};


static struct file_operations acq400_proc_ops_buffers = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_buffers,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};


static struct file_operations acq400_proc_ops_stats = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_stats,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};



void acq400_init_proc(struct acq400_dev* acq400_dev)
/* create unique stats entry under /proc/acq420/ */
{
	acq400_dev->proc_entry = proc_mkdir(
		acq400_names[acq400_dev->of_prams.site], acq400_proc_root);

	proc_create("dmac", 0, acq400_dev->proc_entry, &acq400_proc_ops_dmac);
	proc_create("buffers", 0, acq400_dev->proc_entry, &acq400_proc_ops_buffers);
	proc_create("EMPTIES", 0, acq400_dev->proc_entry, &acq400_proc_ops_EMPTIES);
	proc_create("INFLIGHT", 0, acq400_dev->proc_entry, &acq400_proc_ops_INFLIGHT);
	proc_create("REFILLS", 0, acq400_dev->proc_entry, &acq400_proc_ops_REFILLS);
	proc_create("OPENS", 0, acq400_dev->proc_entry, &acq400_proc_ops_OPENS);
	proc_create("stats", 0, acq400_dev->proc_entry, &acq400_proc_ops_stats);
	proc_create("Qstats", 0, acq400_dev->proc_entry, &acq400_proc_ops_qstats);
}

void acq400_del_proc(struct acq400_dev* adev)
{
	remove_proc_entry("dmac", adev->proc_entry);
	remove_proc_entry("buffers", adev->proc_entry);
	remove_proc_entry(acq400_names[adev->pdev->id], acq400_proc_root);
}

void acq400_module_init_proc(void)
{
	acq400_proc_root = proc_mkdir("driver/acq400", 0);
}
void acq400_module_remove_proc()
{
	remove_proc_entry("driver/acq400", NULL);
}
