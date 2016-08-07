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


#define POTW1_OFF	0
#define POTW2_OFF	1
#define FTW1_OFF	2
#define FTW2_OFF	3
#define DFR_OFF		4
#define UCR_OFF		5
#define RRCR_OFF	6
#define CR_OFF		7
#define IPDMR_OFF	8
#define QPDMR_OFF	9
#define SKRR_OFF	10
#define QDACR_OFF	11

#define POTW1_LEN	2
#define POTW2_LEN	2
#define FTW1_LEN	6
#define FTW2_LEN	6
#define DFR_LEN		6
#define UCR_LEN		4
#define RRCR_LEN	3
#define CR_LEN		4
#define IPDMR_LEN	2
#define QPDMR_LEN	2
#define SKRR_LEN	1
#define QDACR_LEN	2



static ssize_t store_multibytes(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count,
	const int REG, const int LEN)
{
	dev_info(dev, "store_multibytes REG:%d LEN:%d", REG, LEN);
	return 0;
}
static ssize_t show_multibytes(	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	const int REG, const int LEN)
{
	dev_info(dev, "show_multibytes REG:%d LEN:%d", REG, LEN);
	return 1;
}

#define AD9854_REG(name) 						\
static ssize_t show_##name(						\
	struct device *dev,						\
	struct device_attribute *attr,					\
	char* buf)							\
{									\
	return show_multibytes(dev, attr, buf, name##_OFF, name##_LEN);	\
}									\
static ssize_t store_##name(						\
	struct device *dev,						\
	struct device_attribute *attr,					\
	const char* buf,						\
	size_t count)							\
{									\
	return store_multibytes(dev, attr, buf, count, name##_OFF, name##_LEN);\
}									\
static DEVICE_ATTR(name, S_IRUGO|S_IWUGO, show_##name, store_##name)

AD9854_REG(POTW1);
AD9854_REG(POTW2);
AD9854_REG(FTW1);
AD9854_REG(FTW2);
AD9854_REG(DFR);
AD9854_REG(UCR);
AD9854_REG(RRCR);
AD9854_REG(CR);
AD9854_REG(IPDMR);
AD9854_REG(QPDMR);
AD9854_REG(SKRR);
AD9854_REG(QDACR);




const struct attribute *ad9854_attrs[] = {
	&dev_attr_POTW1.attr,
	&dev_attr_POTW2.attr,
	&dev_attr_FTW1.attr,
	&dev_attr_FTW2.attr,
	&dev_attr_DFR.attr,
	&dev_attr_UCR.attr,
	&dev_attr_RRCR.attr,
	&dev_attr_CR.attr,
	&dev_attr_IPDMR.attr,
	&dev_attr_QPDMR.attr,
	&dev_attr_SKRR.attr,
	&dev_attr_QDACR.attr,
	0
};
static int ad9854_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	if (sysfs_create_files(&dev->kobj, ad9854_attrs)){
		dev_err(dev, "ad9854_probe() failed to create knobs");
		return -1;
	}
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
