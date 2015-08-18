/* ------------------------------------------------------------------------- */
/* acq400_dspfs.c ACQ420_FMC						     */
/*
 * acq400_dspfs.c
 *
 *  Created on: 18 Aug 2015
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



#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "regfs.h"

#define REVID	"0.1"
#define MAXDEV 2

ulong acq400_dsp_sites[2] = { 0x80000000, 0x80040000 };
int acq400_dsp_sites_count = 2;
module_param_array(acq400_dsp_sites, ulong, &acq400_dsp_sites_count, 0644);

ulong acq400_dsp_len[2] = { 0x00040000, 0x00040000 };
int acq400_dsp_len_count = 2;
module_param_array(acq400_dsp_len, ulong, &acq400_dsp_len_count, 0644);

static int n_acq400_dsp;
module_param(n_acq400_dsp, int, 0444);

struct platform_device* devices[MAXDEV];

char* acq400_dsp_names[MAXDEV] = { "dsp0", "dsp1" };
int n_names = 2;
module_param_array(acq400_dsp_names, charp, &n_names, 0644);

void acq400_dsp_init_site(int site)
{
	struct platform_device *dev =
			kzalloc(sizeof(struct platform_device), GFP_KERNEL);

	dev->name = MODULE_NAME;
	dev->id = site;
	dev->id_auto = 0;
	dev->num_resources = 1;
	dev->resource = kzalloc(sizeof(struct resource), GFP_KERNEL);
	dev->resource->start = acq400_dsp_sites[site];
	dev->resource->end = acq400_dsp_sites[site]+acq400_dsp_len[site]-1;
	dev->resource->flags = IORESOURCE_MEM;
	dev->resource->name = acq400_dsp_names[site];

	platform_device_register(dev);
	devices[site] = dev;
}

void acq400_dsp_remove_site(int site)
{
	struct platform_device *dev = devices[site];
	platform_device_unregister(dev);
	kfree(dev->resource);
	kfree(dev);
	devices[site] = 0;
}

static void __exit acq400_dsp_exit(void)
{
	for (; n_acq400_dsp--;){
		acq400_dsp_remove_site(acq400_dsp_sites[n_acq400_dsp]);
	}
}



static int __init acq400_dsp_init(void)
{
        int status = 0;


	printk("D-TACQ ACQ480 Driver %s\n", REVID);

	for (n_acq400_dsp = 0; n_acq400_dsp < acq400_dsp_sites_count; ++n_acq400_dsp){
		acq400_dsp_init_site(n_acq400_dsp);
	}
        return status;
}

module_init(acq400_dsp_init);
module_exit(acq400_dsp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ DSP filesystem device descriptions");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
