/* ------------------------------------------------------------------------- */
/* acq420_proc.c  		                     	 */
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
 * acq420_proc.c
 *
 *  Created on: Mar 24, 2013
 *      Author: pgm
 */

#include "acq420FMC.h"
#include "hbm.h"

static struct proc_dir_entry *acq400_proc_root;

/* Driver /proc filesystem operations so that we can show some statistics */
static void *acq420_proc_seq_start(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
                return s->private;
        }

        return NULL;
}

static void *acq420_proc_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
        (*pos)++;
        return NULL;
}

static void acq420_proc_seq_stop(struct seq_file *s, void *v)
{
}

static int acq420_proc_seq_show_dma(struct seq_file *s, void *v)
{
        struct acq420_dev *dev;

        dev = v;
        if (mutex_lock_interruptible(&dev->mutex)) {
                return -EINTR;
        }

        seq_printf(s, "\nFIFO DMA Test:\n\n");
        seq_printf(s, "Device Physical Address: 0x%0x\n", dev->dev_physaddr);
        seq_printf(s, "Device Virtual Address:  0x%0x\n",
                (u32)dev->dev_virtaddr);
        seq_printf(s, "Device Address Space:    %d bytes\n", dev->dev_addrsize);
        seq_printf(s, "DMA Channel:             %d\n", dev->dma_channel);
        seq_printf(s, "FIFO Depth:              %d bytes\n", dev->fifo_depth);
        seq_printf(s, "Burst Length:            %d words\n", dev->burst_length);
        seq_printf(s, "\n");
        seq_printf(s, "Opens:                   %d\n", dev->opens);
        seq_printf(s, "Writes:                  %d\n", dev->writes);
        seq_printf(s, "Bytes Written:           %d\n", dev->bytes_written);
        seq_printf(s, "Closes:                  %d\n", dev->closes);
        seq_printf(s, "Errors:                  %d\n", dev->errors);
        seq_printf(s, "Busy:                    %d\n", dev->busy);
        seq_printf(s, "\n");

        mutex_unlock(&dev->mutex);
        return 0;
}



static int intDevFromProcFile(struct file* file, struct seq_operations *seq_ops)
{
	seq_open(file, seq_ops);
	// @@todo hack .. assumes parent is the id .. could do better?
	((struct seq_file*)file->private_data)->private =
	            acq420_devices[file->f_path.dentry->d_parent->d_iname[0] -'0'];
	return 0;
}
static int acq420_proc_open_dmac(struct inode *inode, struct file *file)
{
	/* SEQ operations for /proc */
	static struct seq_operations acq420_proc_seq_ops_dma = {
	        .start = acq420_proc_seq_start,
	        .next = acq420_proc_seq_next,
	        .stop = acq420_proc_seq_stop,
	        .show = acq420_proc_seq_show_dma
	};

	return intDevFromProcFile(file, &acq420_proc_seq_ops_dma);
}

static void *acq420_proc_seq_start_buffers(struct seq_file *s, loff_t *pos)
{
        if (*pos == 0) {
        	struct acq420_dev *dev = s->private;
        	seq_printf(s, "Buffers\n");
        	return dev->buffers.next;
        }

        return NULL;
}

static void *acq420_proc_seq_next_buffers(struct seq_file *s, void *v, loff_t *pos)
{
	struct acq420_dev *dev = s->private;
	struct list_head* list = v;
	if (list->next != &dev->buffers){
		(*pos)++;
		return list->next;
	}else{
		return NULL;
	}
}



static int acq420_proc_seq_show_buffers(struct seq_file *s, void *v)
{
        struct acq420_dev *dev = s->private;
        struct HBM * hbm = list_entry(v, struct HBM, list);

        if (mutex_lock_interruptible(&dev->mutex)) {
                return -EINTR;
        }

        seq_printf(s, "[%02d] va:%p pa:0x%08x len:%d dir:%d state:%d\n",
        		hbm->ix, hbm->va, hbm->pa, hbm->len, hbm->dir, hbm->bstate);
        mutex_unlock(&dev->mutex);
        return 0;
}


static int acq420_proc_open_buffers(struct inode *inode, struct file *file)
{
	/* SEQ operations for /proc */
	static struct seq_operations acq420_proc_seq_ops_buffers = {
	        .start = acq420_proc_seq_start_buffers,
	        .next = acq420_proc_seq_next_buffers,
	        .stop = acq420_proc_seq_stop,
	        .show = acq420_proc_seq_show_buffers
	};

	return intDevFromProcFile(file, &acq420_proc_seq_ops_buffers);
}
static struct file_operations acq420_proc_ops_dmac = {
        .owner = THIS_MODULE,
        .open = acq420_proc_open_dmac,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};

static struct file_operations acq420_proc_ops_buffers = {
        .owner = THIS_MODULE,
        .open = acq420_proc_open_buffers,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release
};


void acq420_init_proc(struct acq420_dev* acq420_dev, int idev)
/* create unique stats entry under /proc/acq420/ */
{
	struct proc_dir_entry *proc_entry;
	acq420_dev->proc_entry = proc_mkdir(acq420_names[idev], acq400_proc_root);

	proc_entry = create_proc_entry("dmac", 0, acq420_dev->proc_entry);
	if (proc_entry) {
		proc_entry->proc_fops = &acq420_proc_ops_dmac;
	}
	proc_entry = create_proc_entry("buffers", 0, acq420_dev->proc_entry);
	if (proc_entry) {
		proc_entry->proc_fops = &acq420_proc_ops_buffers;
	}
}

void acq420_del_proc(struct acq420_dev* acq420_dev)
{
	remove_proc_entry("dmac", acq420_dev->proc_entry);
	remove_proc_entry("buffers", acq420_dev->proc_entry);
	remove_proc_entry(acq420_names[acq420_dev->pdev->id], acq400_proc_root);
}

void acq420_module_init_proc(void)
{
	acq400_proc_root = proc_mkdir("driver/acq420", 0);
}
void acq420_module_remove_proc()
{
	remove_proc_entry("driver/acq420", NULL);
}
