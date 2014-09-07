/* ------------------------------------------------------------------------- */
/* acq400_fs.c  D-TACQ ACQ400 FMC  DRIVER		                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                    *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
 * Copyright 2002, 2003 Jonathan Corbet <corbet@lwn.net>
 * This file may be redistributed under the terms of the GNU GPL.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pagemap.h> 	/* PAGE_CACHE_SIZE */
#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <asm/atomic.h>
#include <asm/uaccess.h>	/* copy_to_user */

#include "acq400.h"

#define ACQ400_FS_MAGIC	0xd1ac0400


static struct inode *a400fs_make_inode(struct super_block *sb, int mode)
{
	struct inode *ret = new_inode(sb);

	if (ret) {
		ret->i_ino = get_next_ino();
		ret->i_mode = mode;
		ret->i_uid = KUIDT_INIT(0);
		ret->i_gid = KGIDT_INIT(0);
		ret->i_blocks = 0;
		ret->i_atime = ret->i_mtime = ret->i_ctime = CURRENT_TIME;
	}
	return ret;
}

/*
 * Open a file.  All we have to do here is to copy over a
 * copy of the counter pointer so it's easier to get at.
 */
static int a400fs_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

#define TMPSIZE 20
/*
 * Read a file.  Here we increment and read the counter, then pass it
 * back to the caller.  The increment only happens if the read is done
 * at the beginning of the file (offset = 0); otherwise we end up counting
 * by twos.
 */
static ssize_t a400fs_read_file(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	atomic_t *counter = (atomic_t *) filp->private_data;
	int v, len;
	char tmp[TMPSIZE];
/*
 * Encode the value, and figure out how much of it we can pass back.
 */
	v = atomic_read(counter);
	if (*offset > 0)
		v -= 1;  /* the value returned when offset was zero */
	else
		atomic_inc(counter);
	len = snprintf(tmp, TMPSIZE, "%d\n", v);
	if (*offset > len)
		return 0;
	if (count > len - *offset)
		count = len - *offset;
/*
 * Copy it back, increment the offset, and we're done.
 */
	if (copy_to_user(buf, tmp + *offset, count))
		return -EFAULT;
	*offset += count;
	return count;
}

/*
 * Write a file.
 */
static ssize_t a400fs_write_file(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	atomic_t *counter = (atomic_t *) filp->private_data;
	char tmp[TMPSIZE];
/*
 * Only write from the beginning.
 */
	if (*offset != 0)
		return -EINVAL;
/*
 * Read the value from the user.
 */
	if (count >= TMPSIZE)
		return -EINVAL;
	memset(tmp, 0, TMPSIZE);
	if (copy_from_user(tmp, buf, count))
		return -EFAULT;
/*
 * Store it in the counter and we are done.
 */
	atomic_set(counter, simple_strtol(tmp, NULL, 10));
	return count;
}


/*
 * Now we can put together our file operations structure.
 */
static struct file_operations a400fs_file_ops = {
	.open	= a400fs_open,
	.read 	= a400fs_read_file,
	.write  = a400fs_write_file,
};


/*
 * Create a file mapping a name to a counter.
 */
static struct dentry *a400fs_create_file (struct super_block *sb,
		struct dentry *dir, const char *name,
		atomic_t *counter)
{
	struct dentry *dentry;
	struct inode *inode;
	struct qstr qname;
/*
 * Make a hashed version of the name to go with the dentry.
 */
	qname.name = name;
	qname.len = strlen (name);
	qname.hash = full_name_hash(name, qname.len);
/*
 * Now we can create our dentry and the inode to go with it.
 */

	dentry = d_alloc(dir, &qname);
	if (! dentry)
		goto out;
	inode = a400fs_make_inode(sb, S_IFREG | 0644);
	if (! inode)
		goto out_dput;
	inode->i_fop = &a400fs_file_ops;
	inode->i_private = counter;
/*
 * Put it all into the dentry cache and we're done.
 */
	d_add(dentry, inode);
	return dentry;
/*
 * Then again, maybe it didn't work.
 */
  out_dput:
	dput(dentry);
  out:
	return 0;
}


/*
 * Create a directory which can be used to hold files.  This code is
 * almost identical to the "create file" logic, except that we create
 * the inode with a different mode, and use the libfs "simple" operations.
 */
static struct dentry *a400fs_create_dir (struct super_block *sb,
		struct dentry *parent, const char *name)
{
	struct dentry *dentry;
	struct inode *inode;
	struct qstr qname;

	qname.name = name;
	qname.len = strlen (name);
	qname.hash = full_name_hash(name, qname.len);
	dentry = d_alloc(parent, &qname);
	if (! dentry)
		goto out;

	inode = a400fs_make_inode(sb, S_IFDIR | 0644);
	if (! inode)
		goto out_dput;
	inode->i_op = &simple_dir_inode_operations;
	inode->i_fop = &simple_dir_operations;

	d_add(dentry, inode);
	return dentry;

  out_dput:
	dput(dentry);
  out:
	return 0;
}

#define a400fs_create_raw_file a400fs_create_file
#define a400fs_create_channel_file a400fs_create_file

/*
 * OK, create the files that we export.
 */
static atomic_t counter, subcounter;

struct FS_NODES {
	struct super_block *sb;
	struct inode *root;
	struct dentry *root_dentry;
	struct dentry *rawdir;
	struct dentry *chandata[MAXDEVICES+1]; /* index from 1 */
} FSN;


static const char* sites[] = {
	"0", "1", "2", "3", "4", "5", "6"
};
static const char* channels[] = {
	"00error",
	"01", "02", "03", "04", "05", "06", "07", "08",
	"09", "10", "11", "12", "13", "14", "15", "16",
	"17", "18", "19", "20", "21", "22", "23", "24",
	"25", "26", "27", "28", "29", "30", "31", "32"
};

void a400fs_add_site(int site, struct acq400_dev *adev, struct FS_NODES *fsn)
{
	int ic;

	dev_info(DEVP(adev), "a400fs_add_site() 01 site:%d sb:%p",
			site, fsn->sb);

	a400fs_create_raw_file(fsn->sb, fsn->rawdir, sites[site], &counter);
	dev_info(DEVP(adev), "a400fs_add_site() 99 site:%d", site);

	fsn->chandata[site] =
		a400fs_create_dir(fsn->sb, fsn->root_dentry, sites[site]);

	if(fsn->chandata[site] == 0){
		dev_warn(DEVP(adev), "site:%d failed to create_dir", site);
		return;
	}

	for (ic = 1; ic <= adev->nchan_enabled; ++ic){
		struct dentry *dentry =
			a400fs_create_channel_file(
				fsn->sb, fsn->chandata[site],
				channels[ic], &counter);
		if (dentry == 0){
			dev_err(DEVP(adev), "ERROR failed to create channel file");
		}
	}
	dev_info(DEVP(adev), "a400fs_add_site() 99 site:%d", site);
}


static void a400fs_create_files (struct super_block *sb, struct dentry *root)
{
	int dev;
	FSN.root_dentry = root;
	FSN.sb = sb;
/*
 * One counter in the top-level directory.
 */
	atomic_set(&counter, 0);
	a400fs_create_file(sb, root, "counter", &counter);
/*
 * And one in a subdirectory.
 */
	atomic_set(&subcounter, 0);
	FSN.rawdir = a400fs_create_dir(sb, root, "raw");
	if (FSN.rawdir)
		a400fs_create_file(sb, FSN.rawdir, "subcounter", &subcounter);

	for (dev = 0; dev < MAXDEVICES; ++dev){
		struct acq400_dev *adev = acq400_devices[dev];
		if (adev){
			a400fs_add_site(adev->of_prams.site, adev, &FSN);
		}
	}
}


/*
 * Superblock stuff.  This is all boilerplate to give the vfs something
 * that looks like a filesystem to work with.
 */

/*
 * Our superblock operations, both of which are generic kernel ops
 * that we don't have to write ourselves.
 */
static struct super_operations a400fs_s_ops = {
	.statfs		= simple_statfs,
	.drop_inode	= generic_delete_inode,
};

/*
 * "Fill" a superblock with mundane stuff.
 */
static int a400fs_fill_super (struct super_block *sb, void *data, int silent)
{
/*
 * Basic parameters.
 */
	sb->s_blocksize = PAGE_CACHE_SIZE;
	sb->s_blocksize_bits = PAGE_CACHE_SHIFT;
	sb->s_magic = ACQ400_FS_MAGIC;
	sb->s_op = &a400fs_s_ops;
/*
 * We need to conjure up an inode to represent the root directory
 * of this filesystem.  Its operations all come from libfs, so we
 * don't have to mess with actually *doing* things inside this
 * directory.
 */
	FSN.root = a400fs_make_inode (sb, S_IFDIR | 0755);
	if (!FSN.root)
		goto out;
	FSN.root->i_op = &simple_dir_inode_operations;
	FSN.root->i_fop = &simple_dir_operations;
/*
 * Get a dentry to represent the directory in core.
 */
	FSN.root_dentry = d_make_root(FSN.root);
	if (! FSN.root_dentry)
		goto out_iput;
	sb->s_root = FSN.root_dentry;
/*
 * Make up the files which will be in this filesystem, and we're done.
 */
	a400fs_create_files (sb, FSN.root_dentry);

	return 0;

  out_iput:
	iput(FSN.root);
  out:
	return -ENOMEM;
}


/*
 * Stuff to pass in when registering the filesystem.
 */
static struct dentry *a400fs_get_super(struct file_system_type *fst,
		int flags, const char *devname, void *data)
{
	return mount_nodev(fst, flags, data, a400fs_fill_super);
}

static struct file_system_type a400fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "acq400fs",
	.mount		= a400fs_get_super,
	.kill_sb	= kill_litter_super,
};


int a400fs_init(void)
{
	return register_filesystem(&a400fs_type);
}

void a400fs_exit(void)
{
	unregister_filesystem(&a400fs_type);
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("peter.milne@d-tacq.com");
