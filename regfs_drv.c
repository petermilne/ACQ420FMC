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

/** @file regfs_fs.c DESCR
 * 
 *  Created on: August 18, 2015
 *      Author: pgm
 *

# cpsc register definition file
# formatting: NO TABS, ONLY SPACES
# "TABSTOPS" at 0, 32, 48, 64, 72
# By all means use a tabbing editor, but tabs will be replaced by spaces:
# ./fixup-tabs cpsc_regdef

cd PEX
PEX_CSR                         0x000           0xffffffff      r       %08x
PEX_MWE                         0x004           0x00000001      r
PEX_DEBUG                       0x008           0xffffffff      rw      %08x
PEX_INT                         0x00c           0xffffffff      r       %08x

 */

#include <asm/io.h>

#include <asm/types.h>

#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <linux/slab.h>

#include <linux/device.h>
#include <linux/platform_device.h>

#include "debugfs2.h"

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <linux/slab.h>
#include <linux/kdev_t.h>


#include <linux/moduleparam.h>

#include "regfs.h"

#define MAXSTACK 4

struct REGFS_DEV {
	void* va;
	struct platform_device* pdev;

	int istack;
	struct dentry *dstack[MAXSTACK];
	struct dentry *top;
	struct dentry *create_hook;
};

#define DEVP(dev)	(&(dev)->pdev->dev)

#define REVID "regfs_fs B1004"

#define LO32(addr) (((unsigned)(addr) & 4) == 0)




static void cd(struct REGFS_DEV* dev, char* dir)
{
	if (strcmp(dir, "/") == 0){
		dev->istack = 0;
	}else if (strcmp(dir, "..") == 0){
		if (dev->istack > 0){
			dev->istack -= 1;
		}
	}else{
		if (dev->istack+1 >= MAXSTACK){
			dev_err(DEVP(dev), "stack depth %d reached", MAXSTACK);
		}else{
			struct dentry *cwd = debugfs_create_dir(
					dir, dev->dstack[dev->istack]);
			if (cwd == 0){
				dev_err(DEVP(dev), "failed to create subdir");
			}else{
				dev->dstack[++dev->istack] = cwd;
				dev_dbg(DEVP(dev), "cd %s OK", dir);
			}
		}
	}
}

static struct dentry *cwd(struct REGFS_DEV* dev)
{
	return dev->dstack[dev->istack];
}


#define CD "cd "

static struct DebugFs2NodeInfo* nodeCreate(const struct DebugFs2NodeInfo *def){
	struct DebugFs2NodeInfo* nodeInfo = kmalloc(DBGFS2_NI_SZ, GFP_KERNEL);
	memcpy(nodeInfo, def, DBGFS2_NI_SZ);
	return nodeInfo;
}
static ssize_t regfs_direct_write(struct file *file,
				   const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct DebugFs2NodeInfo defaultNodeInfo = { 0, };
	char myline[80];
	ssize_t rc;

	struct REGFS_DEV* dev = (struct REGFS_DEV*)file->private_data;

	defaultNodeInfo.pread =
		defaultNodeInfo.pwrite = defaultNodeInfo.pcache = dev->va;

	rc = debugfs2_write_line(file, user_buf,
				 min(count, sizeof(myline)-1), ppos, myline);
	myline[79] = '\0';

	if (rc > 0 && myline[0] != '#' && strlen(myline) > 3){
		if (strncmp(myline, CD, strlen(CD)) == 0){
			cd(dev, myline+strlen(CD));
		}else{
			struct DebugFs2NodeInfo* nodeInfo =
				nodeCreate(&defaultNodeInfo);
			debugfs2_create_file_def(
					cwd(dev), nodeInfo, myline, (int)*ppos);
		}
		/* @@todo: newfile -> list */
	}

	return rc;
}



static int regfs_direct_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

const struct file_operations regfs_direct_fops = {
	.write =        regfs_direct_write,
	.open =		regfs_direct_open
};


struct resource* regfs_device_tree_init(struct REGFS_DEV* dev)
{
	return 0;
}
int regfs_init_fs(struct REGFS_DEV* dev)
{
	struct resource* mem;
	int rc = 0;
	dev_info(DEVP(dev), REVID);

	dev_dbg(DEVP(dev), "nres = %d [0].type: %x",
			dev->pdev->num_resources, dev->pdev->resource[0].flags);

	if (dev->pdev->dev.of_node != 0){
		if ((mem = regfs_device_tree_init(dev)) == 0){
			dev_err(DEVP(dev), "regfs_device_tree_init() failed");
			rc = -ENODEV;
			goto init99;
		}
	}

	mem = platform_get_resource(dev->pdev, IORESOURCE_MEM, 0);
	if (mem == NULL){
	       	dev_err(DEVP(dev), "No resources found");
	        rc = -ENODEV;
	        goto init99;
	}

	dev_dbg(DEVP(dev), "request_mem_region()");
	if (!request_mem_region(mem->start, mem->end-mem->start+1, mem->name)) {
		dev_err(DEVP(dev), "can't reserve i/o memory at 0x%08X\n",
	                        mem->start);
	        rc = -ENODEV;
	        goto init99;
	}
	dev_dbg(DEVP(dev), "ioremap() %08x %08x",
			mem->start, mem->end-mem->start+1);

	dev->va = ioremap(mem->start, mem->end-mem->start+1);

	dev_dbg(DEVP(dev), "create_dir %s", mem->name);

	dev->dstack[dev->istack = 0] = dev->top =
			debugfs_create_dir(mem->name, NULL);
	dev->create_hook = debugfs_create_file(".create", S_IWUGO,
				  dev->top, dev, &regfs_direct_fops);

	dev_dbg(DEVP(dev), "all good");
init99:
	return rc;
}

void regfs_remove_fs(struct REGFS_DEV* dev)
{
	dev_info(DEVP(dev), REVID);
	debugfs_remove_recursive(dev->top);
}

static int regfs_probe(struct platform_device *pdev)
{
	struct REGFS_DEV* rdev = kzalloc(sizeof(struct REGFS_DEV), GFP_KERNEL);
	rdev->pdev = pdev;

	return regfs_init_fs(rdev);
}

static int regfs_remove(struct platform_device *pdev)
{
	struct REGFS_DEV* rdev = 0;		// TODO LOOKUP
	regfs_remove_fs(rdev);
	return 0;
}

#define regfs_of_match 0

static struct platform_driver regfs_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = regfs_of_match,
        },
        .probe = regfs_probe,
        .remove = regfs_remove,
};

static void __exit regfs_exit(void)
{
	platform_driver_unregister(&regfs_driver);
}

static int __init regfs_init(void)
{
	return platform_driver_register(&regfs_driver);
}
module_init(regfs_init);
module_exit(regfs_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ regfs driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
