/* ------------------------------------------------------------------------- *
 * acq480_drv.c
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 16 October 2014
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
 * instantiates ACQ480 i2c device (50R termination)
 * also creates a device driver hook for the spi buffer
 * acq480_knobs app mmaps this buffer and updates it.
 * a task on a poll loop in this driver monitors changes and sends them to SPI
 */
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/i2c.h>
//#include <linux/i2c/pca953x.h>
#include <linux/platform_device.h>
#include <linux/platform_data/pca953x.h>

#define REVID 		"1"
#define MODULE_NAME	"acq480"

int acq480_sites[6] = { 0,  };
int acq480_sites_count = 0;
module_param_array(acq480_sites, int, &acq480_sites_count, 0644);

static int n_acq480;
module_param(n_acq480, int, 0444);


#define I2C_CHAN(site) 	((site)+1)
#define NGPIO_CHIP	8

struct acq480_dev {
	dev_t devno;
	struct cdev cdev;
	struct platform_device *pdev;
	struct i2c_adapter *i2c_adapter;
};

static struct acq480_dev* acq480_devs[7];	/* 6 sites index from 1 */

static struct i2c_client* new_device(
		struct i2c_adapter *adap,
		const char* name, unsigned short addr, int gpio_base)
{
	struct pca953x_platform_data pca_data = {};
	struct i2c_board_info info = {};

	strlcpy(info.type, name, I2C_NAME_SIZE);
	info.addr = addr;
	pca_data.gpio_base = gpio_base;
	pca_data.irq_base = -1;
	info.platform_data = &pca_data;
	return i2c_new_device(adap, &info);
}


#define acq480_of_match 0

struct acq480_dev* acq480_allocate_dev(struct platform_device *pdev)
{
	int site = pdev->id;
	struct acq480_dev* adev =
		kzalloc(sizeof(struct acq480_dev), GFP_KERNEL);

	if (adev == NULL){
		return NULL;
	}
	acq480_devs[site] = adev;

	adev->pdev = pdev;
	adev->i2c_adapter = i2c_get_adapter(I2C_CHAN(site));
	if (new_device(adev->i2c_adapter, "pca9534", 0x20, -1) == 0){
		printk("acq480_init_site(%d) PGA1 NOT found\n", site);
	}

	return adev;
}

static int acq480_probe(struct platform_device *pdev)
{
	struct acq480_dev* adev = acq480_allocate_dev(pdev);
	return 0;
}

static int acq480_remove(struct platform_device *pdev)
{
	int site = pdev->id;
	i2c_put_adapter(acq480_devs[site]->i2c_adapter);
	return -1;
}

static void __init acq480_init_site(int site)
{
	struct platform_device* pdev =
			kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	pdev->name = MODULE_NAME;
	pdev->id = site;
	platform_device_register(pdev);
}

static void __init acq480_remove_site(int site)
{
	printk("acq480_remove_site %d ERROR removal not supported\n", site);
}
static struct platform_driver acq480_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = acq480_of_match,
        },
        .probe = acq480_probe,
        .remove = acq480_remove,
};

static void __exit acq480_exit(void)
{
	for (; n_acq480--;){
		acq480_remove_site(acq480_sites[n_acq480]);
	}
	platform_driver_unregister(&acq480_driver);
}



static int __init acq480_init(void)
{
        int status = 0;


	printk("D-TACQ ACQ425 i2c Driver %s\n", REVID);

	platform_driver_register(&acq480_driver);

	for (n_acq480 = 0; n_acq480 < acq480_sites_count; ++n_acq480){
		acq480_init_site(acq480_sites[n_acq480]);
	}
        return status;
}

module_init(acq480_init);
module_exit(acq480_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ ACQ480ELF i2c Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
