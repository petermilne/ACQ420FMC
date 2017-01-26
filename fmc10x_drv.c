/* ------------------------------------------------------------------------- */
/* fmc10x_drv.c ACQ420_FMC						     */
/* ------------------------------------------------------------------------- */
/* fmc10x_drv.c  D-TACQ ACQ400 FMC  DRIVER
 * Project: ACQ420_FMC
 * Created: 25 JAN 2017  	/ User: pgm
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2017 Peter Milne, D-TACQ Solutions Ltd         *
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

/*
 * spi:
 * ad9510 = SPI device 0 (or 1 depending on how you've defined it    )
 * ads62p49 = SPI device 1
 *
 * i2c:
 * 0x48 : adt7411	1001 000 or normalised 0100 1000
 * 0x4a : adt7411       1001 010 or normalised 0100 1010
 */
#define REVID	"1"

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

#include "acq400.h"

#define M1	1000000

static int fmc10x_init_spi(void)
{
	static struct spi_board_info spi_devs [2] = {
	{
		.modalias = "ads62p49",
		.max_speed_hz = 10*M1,
		.bus_num = 1,
		.chip_select = 0,
		.platform_data = NULL,
		.irq = 0,
	},
	{
		.modalias = "ad9510",
		.max_speed_hz = 10*M1,
		.bus_num = 1,
		.chip_select = 1,
		.platform_data = NULL,
		.irq = 0,
	},

	};

	return spi_register_board_info(spi_devs, 2);
}

extern void acq480_hook_spi(void);
extern int zynq_spi_goslow_usec_kludge;

static int __init fmc10x_init(void)
{
        int status = 0;

	printk("D-TACQ FMC10x Driver %s\n", REVID);

	acq480_hook_spi();

	status = fmc10x_init_spi();
        return status;
}

static void __exit fmc10x_exit(void)
{

}
module_init(fmc10x_init);
module_exit(fmc10x_exit);



MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ FMC10x Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
