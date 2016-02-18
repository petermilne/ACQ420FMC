/* ------------------------------------------------------------------------- */
/* dmadescfs.c ACQ420_FMC						     */
/*
 * dmadescfs.c
 *  N device nodes : 0..N-1
 *  open(),
 *  mmap() : first mmap() on device allocates a buffer.
 *  ioctl() :
 *   	DD_GETPA : returns buffer PA
 *   	DD_TOCPU : invalidate cache for cpu read after device write
 *   	DD_TODEV : flush cache for device use after cpu write
 *
 *
 *  Created on: 18 Feb 2016
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
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include "hbm.h"


#define REVID 		"1"
#define MODULE_NAME	"dmadescfs"

static int dmadescfs_probe(struct platform_device *pdev)
{
	return 0;
}

static int dmadescfs_remove(struct platform_device *pdev)
{

	return 0;
}


static struct platform_driver dmadescfs_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
        },
        .probe = dmadescfs_probe,
        .remove = dmadescfs_remove,
};

static void __exit dmadescfs_exit(void)
{
	platform_driver_unregister(&dmadescfs_driver);
}

static int __init dmadescfs_init(void)
{
        int status = 0;
        struct platform_device* pdev;

	printk("D-TACQ dmadescfs Driver %s\n", REVID);

	platform_driver_register(&dmadescfs_driver);

	pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	pdev->name = MODULE_NAME;
	pdev->id = 0;
	platform_device_register(pdev);

        return status;
}

module_init(dmadescfs_init);
module_exit(dmadescfs_exit);



MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ dmadescfs Driver");
MODULE_AUTHOR("D-TACQ Solutions");
MODULE_VERSION(REVID);
