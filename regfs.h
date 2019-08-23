/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

    http://www.d-tacq.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of Version 2 of the GNU General Public License
    as published by the Free Software Foundation;

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/** @file regfs.h DESCR
 * 
 *  Created on: Feb 10, 2012
 *      Author: pgm
 */

#ifndef REGFS_DEV_H_
#define REGFS_DEV_H_

#define MAXSTACK 4


struct REGFS_DEV {
	void* va;
	struct platform_device* pdev;
	struct resource *mem;

	int istack;
	struct dentry *dstack[MAXSTACK];
	struct dentry *top;
	struct dentry *create_hook;


	struct cdev cdev;
	struct list_head list;
	wait_queue_head_t w_waitq;

	unsigned ints;
	unsigned status;
	unsigned sample_count;
	unsigned latch_count;

	void* client;		/* stash subclass data here */
};

extern irqreturn_t (*regfs_isr)(int irq, void *dev_id);
extern int regfs_probe(struct platform_device *pdev);
extern int regfs_remove(struct platform_device *pdev);

#endif /* REGFS_DEV_H_ */
