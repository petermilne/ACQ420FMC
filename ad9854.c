/* ------------------------------------------------------------------------- */
/* ad9854.c  DDS  DRIVER
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

/**
 *
 *

NB: with the serial interface, addresses are by REGISTER, not by BYTE

SO: Serial offset or register offset
Len: length in bytes
Knob: linux virtual file "knob name"
Name: ADI literature name

SO | Len | Knob| Name
---|-----|-----|------
0  | 2   |POTW1|Phase Offset Tuning Word Register #1
1  | 2   |POTW2|Phase Offset Tuning Word Register #2
2  | 6   |FTW1 |Frequency Tuning Word #1
3  | 6   |FTW2 |Frequency Tuning Word #2
4  | 6   |DFR  |Delta Frequency Register
5  | 4   |UCR  |Update Clock Rate Register
6  | 3   |RRCR |Ramp Rate Clock Register
7  | 4   |CR   |Control Register
8  | 2   |IPDMR|I Path Digital Multiplier Register
9  | 2   |QPDMR|Q Path Digital Multiplier Register
A  | 1   |SKRR |Shaped On/Off Keying Ramp Rate Register
B  | 2   |QDACR|Q DAC Register 2 Bytes
 */

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

static int ad9854_probe(struct spi_device *spi)
{
	return 0;
}
static int ad9854_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver ad9854_spi_driver = {
	.driver = {
		.name	= "ad9854",
		.owner	= THIS_MODULE,
	},
	//.id_table = m25p_ids,
	.probe	= ad9854_probe,
	.remove	= ad9854_remove,
};


static void __exit ad9854_exit(void)
{
	spi_unregister_driver(&ad9854_spi_driver);
}

extern void ad9854_hook_spi(void);

static int __init ad9854_init(void)
{
        int status = 0;

	printk("D-TACQ ACQ480 Driver %s\n", REVID);

	spi_register_driver(&ad9854_spi_driver);

        return status;
}

module_init(ad9854_init);
module_exit(ad9854_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ AD9854 spi Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
