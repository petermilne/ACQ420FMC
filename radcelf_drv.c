/* ------------------------------------------------------------------------- */
/* radcelf_drv.c ACQ420_FMC						     */
/* ------------------------------------------------------------------------- */
/* radcelf_drv.c  D-TACQ ACQ400 FMC  DRIVER
 * Project: ACQ420_FMC
 * Created: 4 Aug 2016  	/ User: pgm
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

/*
 * spi:
 * DDS A = SPI device 0 (or 1 depending on how you've defined it    )
 * DDS B = SPI device 1
 * DDS C = SPI device 2
 * AD9512A =  SPI device 3
 * AD9512B =  SPI device 4
 *
 * i2c:
 * 0x20 : tca6408
 * 0x28 : ad7417
 * 0x29 : ad7417
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

//#include <linux/i2c/pca953x.h>
#include <linux/platform_device.h>
#include <linux/platform_data/pca953x.h>

#include <linux/dma-mapping.h>
#include "hbm.h"

#include "acq400.h"
#include "ad9854.h"

int radcelf_gpio_base = 128;
module_param(radcelf_gpio_base, int, 0644);

int goslow_usec_kludge;
module_param(goslow_usec_kludge, int, 0644);

struct radcelf_dev {
	struct platform_device *pdev;
	struct i2c_adapter *i2c_adapter;
	char devname[16];

	struct spi_device *spi;
};

static int radcelf_init_spi(void)
{
	static struct AD9854_PlatformData pd;

	static struct spi_board_info spi_devs [5] = {
	{
		.modalias = "ad9854",
		.max_speed_hz = 1000000,
		.bus_num = 1,
		.chip_select = 0,
		.platform_data = &pd, /* No spi_driver specific config */
		.irq = 0,
	},
	{
		.modalias = "ad9854",
		.max_speed_hz = 1000000,
		.bus_num = 1,
		.chip_select = 1,
		.platform_data = &pd, /* No spi_driver specific config */
		.irq = 0,
	},
	{
		.modalias = "ad9854",
		.max_speed_hz = 1000000,
		.bus_num = 1,
		.chip_select = 2,
		.platform_data = &pd, /* No spi_driver specific config */
		.irq = 0,
	},
	{
		.modalias = "ad9512",
		.max_speed_hz = 1000000,
		.bus_num = 1,
		.chip_select = 3,
		.platform_data = NULL, /* No spi_driver specific config */
		.irq = 0,
	},
	{
		.modalias = "ad9512",
		.max_speed_hz = 1000000,
		.bus_num = 1,
		.chip_select = 4,
		.platform_data = NULL, /* No spi_driver specific config */
		.irq = 0,
	}
	};

	pd.dev_private = acq400_devices[2];
	pd.strobe = acq400_spi_strobe;
	pd.strobe_mode = SPI_STROBE_NONE;


	return spi_register_board_info(spi_devs, 5);
}

static struct i2c_client* new_device(
		struct i2c_adapter *adap,
		const char* name, unsigned short addr, void* platform_data)
{
	struct i2c_board_info info = {};

	strlcpy(info.type, name, I2C_NAME_SIZE);
	info.addr = addr;
	info.platform_data = platform_data;
	return i2c_new_device(adap, &info);
}


#define I2C_CHAN         (4)      /* CELF BUS = 4 */

static int radcelf_init_i2c(void)
{
	struct i2c_adapter *i2c_adap = i2c_get_adapter(I2C_CHAN);
	{
		struct pca953x_platform_data pca_data = {};
		pca_data.gpio_base = radcelf_gpio_base;
		pca_data.irq_base = -1;
		if (new_device(i2c_adap, "tca6408", 0x20, &pca_data) == 0){
			printk("radcelf_init_i2c dio NOT found\n");
		}
	}
	if (new_device(i2c_adap, "ad7417", 0x28, 0) == 0){
		printk("radcelf_init_i2c ad7417#1 NOT found\n");
	}
	if (new_device(i2c_adap, "ad7417", 0x29, 0) == 0){
		printk("radcelf_init_i2c ad7417#2 NOT found\n");
	}
	return 0;
}

extern void acq480_hook_spi(void);
extern int zynq_spi_goslow_usec_kludge;

static int __init radcelf_init(void)
{
        int status = 0;

	printk("D-TACQ RADCELF Driver %s\n", REVID);

	if (goslow_usec_kludge){
		zynq_spi_goslow_usec_kludge = goslow_usec_kludge;
		printk("zynq_spi_goslow_usec_kludge %d", zynq_spi_goslow_usec_kludge);
	}
	acq480_hook_spi();

	status = radcelf_init_spi();
	if (status == 0){
		status = radcelf_init_i2c();
	}
        return status;
}

static void __exit radcelf_exit(void)
{

}
module_init(radcelf_init);
module_exit(radcelf_exit);



MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ RADCELF Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
