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

/* Driver /proc filesystem operations so that we can show some statistics */
static void *acq400_proc_seq_start(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
                return s->private;
        }

        return NULL;
}

static void *acq400_proc_seq_next(struct seq_file *s, void *v, loff_t *pos)
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
	/* SEQ operations for /proc */
	static struct seq_operations acq400_proc_seq_ops_dma = {
	        .start = acq400_proc_seq_start,
	        .next = acq400_proc_seq_next,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400_proc_seq_show_dma
	};

	return intDevFromProcFile(file, &acq400_proc_seq_ops_dma);
}

static int acq400_proc_open_stats(struct inode *inode, struct file *file)
{
	/* SEQ operations for /proc */
	static struct seq_operations acq400_proc_seq_ops_dma = {
	        .start = acq400_proc_seq_start,
	        .next = acq400_proc_seq_next,
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

static void *acq400_proc_seq_start_EMPTIES(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
        	struct acq400_dev *adev = s->private;
        	if (!list_empty(&adev->EMPTIES)){
        		return list_first_entry(&adev->EMPTIES, struct HBM, list);
        	}
        }

        return NULL;
}

static void *acq400_proc_seq_next_EMPTIES(struct seq_file *s, void *v, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	int tpos = 0;
	struct HBM *hbm;

	++*pos;

	list_for_each_entry(hbm, &adev->EMPTIES, list){
		if (tpos++ == *pos){
			return hbm;
		}
	}

	return NULL;
}

static void *acq400_proc_seq_start_REFILLS(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
        	struct acq400_dev *adev = s->private;
        	if (!list_empty(&adev->REFILLS)){
        		return list_first_entry(&adev->REFILLS, struct HBM, list);
        	}
        }

        return NULL;
}

static void *acq400_proc_seq_next_REFILLS(struct seq_file *s, void *v, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	int tpos = 0;
	struct HBM *hbm;

	++*pos;

	list_for_each_entry(hbm, &adev->REFILLS, list){
		if (tpos++ == *pos){
			return hbm;
		}
	}

	return NULL;
}

static void *acq400_proc_seq_start_OPENS(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
        	struct acq400_dev *adev = s->private;
        	if (!list_empty(&adev->OPENS)){
        		return list_first_entry(&adev->OPENS, struct HBM, list);
        	}
        }

        return NULL;
}

static void *acq400_proc_seq_next_OPENS(struct seq_file *s, void *v, loff_t *pos)
{
	struct acq400_dev *adev = s->private;
	int tpos = 0;
	struct HBM *hbm;

	++*pos;

	list_for_each_entry(hbm, &adev->OPENS, list){
		if (tpos++ == *pos){
			return hbm;
		}
	}

	return NULL;
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

static int acq400_proc_open_EMPTIES(struct inode *inode, struct file *file)
{
	/* SEQ operations for /proc */
	static struct seq_operations acq400_proc_seq_ops_buffers = {
	        .start = acq400_proc_seq_start_EMPTIES,
	        .next = acq400_proc_seq_next_EMPTIES,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400_proc_seq_show_buffers
	};

	return intDevFromProcFile(file, &acq400_proc_seq_ops_buffers);
}

static int acq400_proc_open_REFILLS(struct inode *inode, struct file *file)
{
	/* SEQ operations for /proc */
	static struct seq_operations acq400_proc_seq_ops_buffers = {
	        .start = acq400_proc_seq_start_REFILLS,
	        .next = acq400_proc_seq_next_REFILLS,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400_proc_seq_show_buffers
	};

	return intDevFromProcFile(file, &acq400_proc_seq_ops_buffers);
}

static int acq400_proc_open_OPENS(struct inode *inode, struct file *file)
{
	/* SEQ operations for /proc */
	static struct seq_operations acq400_proc_seq_ops_buffers = {
	        .start = acq400_proc_seq_start_OPENS,
	        .next = acq400_proc_seq_next_OPENS,
	        .stop = acq400_proc_seq_stop,
	        .show = acq400_proc_seq_show_buffers
	};

	return intDevFromProcFile(file, &acq400_proc_seq_ops_buffers);
}
static struct file_operations acq400_proc_ops_dmac = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_dmac,
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

static struct file_operations acq400_proc_ops_EMPTIES = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_EMPTIES,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};

static struct file_operations acq400_proc_ops_REFILLS = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_REFILLS,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};

static struct file_operations acq400_proc_ops_OPENS = {
        .owner = THIS_MODULE,
        .open = acq400_proc_open_OPENS,
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


static int dma0_proc_read(
	char *buf, char **start, off_t offset, int len, int* eof, void* data )
{
#ifdef PGMCOMOUT
	unsigned nbytes;
	char *prog = get_pl330_dma_program(0, &nbytes);

	if (prog != NULL && nbytes > 0){
		memcpy(buf, prog, nbytes);
		return nbytes;
	}else{
		return -1;
	}
#else
	return -1;
#endif
}
void acq400_init_proc(struct acq400_dev* acq400_dev)
/* create unique stats entry under /proc/acq420/ */
{
	struct proc_dir_entry *proc_entry;
	acq400_dev->proc_entry = proc_mkdir(
		acq400_names[acq400_dev->of_prams.site], acq400_proc_root);

	proc_entry = create_proc_entry("dmac", 0, acq400_dev->proc_entry);
	if (proc_entry) {
		proc_entry->proc_fops = &acq400_proc_ops_dmac;
	}
	proc_entry = create_proc_entry("buffers", 0, acq400_dev->proc_entry);
	if (proc_entry) {
		proc_entry->proc_fops = &acq400_proc_ops_buffers;
	}
	proc_entry = create_proc_entry("EMPTIES", 0, acq400_dev->proc_entry);
	if (proc_entry) {
		proc_entry->proc_fops = &acq400_proc_ops_EMPTIES;
	}
	proc_entry = create_proc_entry("REFILLS", 0, acq400_dev->proc_entry);
	if (proc_entry) {
		proc_entry->proc_fops = &acq400_proc_ops_REFILLS;
	}
	proc_entry = create_proc_entry("OPENS", 0, acq400_dev->proc_entry);
	if (proc_entry) {
		proc_entry->proc_fops = &acq400_proc_ops_OPENS;
	}
	proc_entry = create_proc_entry("stats", 0, acq400_dev->proc_entry);
	if (proc_entry) {
		proc_entry->proc_fops = &acq400_proc_ops_stats;
	}

	create_proc_read_entry("dmac0", 0, acq400_dev->proc_entry, dma0_proc_read, 0);
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
