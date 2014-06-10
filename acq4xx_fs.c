/* ------------------------------------------------------------------------- *
 * acq4xx_fs.c  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 25 May 2014  
 *    Author: pgm                                                         
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
 * libfs filesystem to access all the HBM's
 *
 * Copyright 2002, 2003 Jonathan Corbet <corbet@lwn.net>
 * This file may be redistributed under the terms of the GNU GPL.
 *
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pagemap.h> 	/* PAGE_CACHE_SIZE */
#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <asm/atomic.h>
#include <asm/uaccess.h>	/* copy_to_user */

#include <linux/moduleparam.h>

int file_count = 512;
module_param(file_count, int, 0444);

#define ACQ4XX_MAGIC 0xac40ac40

struct BufferInfo {
	char name[4];
	int ix;
};

/*
 * Anytime we make a file or directory in our filesystem we need to
 * come up with an inode to represent it internally.  This is
 * the function that does that job.  All that's really interesting
 * is the "mode" parameter, which says whether this is a directory
 * or file, and gives the permissions.
 */
static struct inode *a4fs_make_inode(struct super_block *sb, int mode)
{
	struct inode *ret = new_inode(sb);

	if (ret) {
		ret->i_mode = mode;
		ret->i_uid = ret->i_gid = 0;
		ret->i_blocks = 0;
		ret->i_atime = ret->i_mtime = ret->i_ctime = CURRENT_TIME;
	}
	return ret;
}


static int a4fs_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

#define TMPSIZE 20



static ssize_t a4fs_read_file(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	struct BufferInfo * bi = (struct BufferInfo *) filp->private_data;
	int len;
	char tmp[TMPSIZE];

	len = snprintf(tmp, TMPSIZE, "%d\n", bi->ix);
	if (*offset > len)
		return 0;
	if (count > len - *offset)
		count = len - *offset;

	if (copy_to_user(buf, tmp + *offset, count))
		return -EFAULT;
	*offset += count;
	return count;
}

/*
 * Write a file.
 */
static ssize_t a4fs_write_file(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	return -1;
}


/*
 * Now we can put together our file operations structure.
 */
static struct file_operations a4fs_file_ops = {
	.open	= a4fs_open,
	.read 	= a4fs_read_file,
	.write  = a4fs_write_file,
};


/*
 * Create a file mapping a name to a counter.
 */
static struct dentry *a4fs_create_file (struct super_block *sb,
		struct dentry *dir, const char *name,
		struct BufferInfo *info)
{
	struct dentry *dentry;
	struct inode *inode;
	struct qstr qname;
	qname.name = name;
	qname.len = strlen (name);
	qname.hash = full_name_hash(name, qname.len);

	dentry = d_alloc(dir, &qname);
	if (! dentry)
		goto out;
	inode = a4fs_make_inode(sb, S_IFREG | 0644);
	if (! inode)
		goto out_dput;
	inode->i_fop = &a4fs_file_ops;
	inode->i_private = info;

	d_add(dentry, inode);

	pr_info("created \"%s\" %p\n", name, dentry);
	return dentry;
/*
 * Then again, maybe it didn't work.
 */
  out_dput:
  	pr_err("failed to create inode \"%s\"\n", name);
	dput(dentry);
  out:
	return 0;
}


/*
 * Create a directory which can be used to hold files.  This code is
 * almost identical to the "create file" logic, except that we create
 * the inode with a different mode, and use the libfs "simple" operations.
 */

static struct dentry *a4fs_create_dir (struct super_block *sb,
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

	inode = a4fs_make_inode(sb, S_IFDIR | 0644);
	if (! inode)
		goto out_dput;
	inode->i_op = &simple_dir_inode_operations;
	inode->i_fop = &simple_dir_operations;

	pr_info("created dir \"%s\"\n", name);
	d_add(dentry, inode);
	return dentry;

  out_dput:
  	pr_err("failed to create dir \"%s\"\n", name);
	dput(dentry);
  out:
	return 0;
}



static void a4fs_create_files (struct super_block *sb, struct dentry *root)
{
	int ifile = 0;
	struct dentry *dir = root;
	for (ifile = 0; ifile < file_count; ++ifile){
		struct BufferInfo *bi = kmalloc(sizeof(struct BufferInfo), GFP_KERNEL);
		struct dentry *dentry;

		if ((ifile&0x1f) == 0){
			char* dname = kmalloc(4, GFP_KERNEL);
			snprintf(dname, 4, "%c", ifile/0x20+'A');
			dir = a4fs_create_dir(sb, root, dname);
			if (dir == 0){
				pr_err("ERROR: failed to create sub dir, use root\n");
				dir = root;
			}
		}

		bi->ix = ifile;
		sprintf(bi->name, "%03d", ifile);

		dentry =  a4fs_create_file (sb, dir, bi->name, bi);

		pr_info("dentry %p inode %p\n", dentry, dentry? dentry->d_inode: 0x0);
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
static struct super_operations a4fs_s_ops = {
	.statfs		= simple_statfs,
	.drop_inode	= generic_delete_inode,
};

/*
 * "Fill" a superblock with mundane stuff.
 */
static int a4fs_fill_super (struct super_block *sb, void *data, int silent)
{
	struct inode *root;
	struct dentry *root_dentry;
/*
 * Basic parameters.
 */
	sb->s_blocksize = PAGE_CACHE_SIZE;
	sb->s_blocksize_bits = PAGE_CACHE_SHIFT;
	sb->s_magic = ACQ4XX_MAGIC;
	sb->s_op = &a4fs_s_ops;
/*
 * We need to conjure up an inode to represent the root directory
 * of this filesystem.  Its operations all come from libfs, so we
 * don't have to mess with actually *doing* things inside this
 * directory.
 */
	root = a4fs_make_inode (sb, S_IFDIR | 0755);
	if (! root)
		goto out;
	root->i_op = &simple_dir_inode_operations;
	root->i_fop = &simple_dir_operations;
/*
 * Get a dentry to represent the directory in core.
 */
	root_dentry = d_make_root(root);
	if (! root_dentry)
		goto out_iput;
	sb->s_root = root_dentry;
/*
 * Make up the files which will be in this filesystem, and we're done.
 */
	a4fs_create_files (sb, root_dentry);
	return 0;

  out_iput:
	iput(root);
  out:
	return -ENOMEM;
}


/*
 * Stuff to pass in when registering the filesystem.
 */
static struct dentry *a4fs_get_super(struct file_system_type *fst,
		int flags, const char *devname, void *data)
{
	return mount_nodev(fst, flags, data, a4fs_fill_super);
}

static struct file_system_type a4fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "acq4xx_fs",
	.mount		= a4fs_get_super,
	.kill_sb	= kill_litter_super,
};




/*
 * Get things set up.
 */
static int __init a4fs_init(void)
{
	return register_filesystem(&a4fs_type);
}

static void __exit a4fs_exit(void)
{
	unregister_filesystem(&a4fs_type);
}

module_init(a4fs_init);
module_exit(a4fs_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Milne");
