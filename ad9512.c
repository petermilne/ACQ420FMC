/* ------------------------------------------------------------------------- */
/* ad9512.c ACQ420_FMC						     */
/* ------------------------------------------------------------------------- */
/* ad9512.c  clock distributor driver
 * Project: ACQ420_FMC
 * Created: 4 Aug 2016  			/ User: pgm
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2016 Peter Milne, D-TACQ Solutions Ltd         *
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                *
 *
 * TODO 
 * TODO
 * ------------------------------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/spi/spi.h>

#define REVID	"1"


static int ad9512_probe(struct spi_device *spi)
{
	return 0;
}
static int ad9512_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver ad98512_spi_driver = {
	.driver = {
		.name	= "ad9512",
		.owner	= THIS_MODULE,
	},
	//.id_table = m25p_ids,
	.probe	= ad9512_probe,
	.remove	= ad9512_remove,
};


static void __exit ad9512_exit(void)
{
	spi_unregister_driver(&ad98512_spi_driver);
}


static int __init ad9512_init(void)
{
        int status = 0;

	printk("D-TACQ ACQ480 Driver %s\n", REVID);

	spi_register_driver(&ad98512_spi_driver);

        return status;
}

module_init(ad9512_init);
module_exit(ad9512_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ AD9512 spi Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
