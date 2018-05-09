/* ------------------------------------------------------------------------- */
/* acq423_sysfs.c ACQ420_FMC						     */
/* ------------------------------------------------------------------------- */
/* acq423_sysfs.c  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 3 May 2018  			/ User: pgm
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2018 Peter Milne, D-TACQ Solutions Ltd         *
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

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/wait.h>

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */

#include "lk-shim.h"


#include <linux/device.h>
#include <linux/module.h>
#include <linux/user.h>

#include "acq400.h"
#include "hbm.h"

#include "acq400_sysfs.h"

static int odd_channels_status(struct acq400_dev *adev)
{
	u32 br = acq400rd32(adev, ACQ423_BANK);
	return (br&ACQ423_BANK_ODD_CHAN) != 0;
}

static ssize_t show_span(
	int ch, struct device * dev, struct device_attribute *attr,
	char * buf)
{
	int ch0 = ch - 1;
	int offsetw = ch0/ACQ423_SPAN_FPR;
	int offsetb = offsetw*sizeof(int);
	int shl = (ch0%ACQ423_SPAN_FPR) * ACQ423_SPAN_FW;
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 reg = acq400rd32(adev, ACQ423_SPAN_A+offsetb);
	return (reg >> shl) &ACQ423_SPAN_MASK;
}

static ssize_t store_span(
	int ch, struct device * dev, struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ch0 = ch - 1;
	int offsetw = ch0/ACQ423_SPAN_FPR;
	int offsetb = offsetw*sizeof(int);
	int shl = (ch0%ACQ423_SPAN_FPR) * ACQ423_SPAN_FW;

	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 span;
	u32 reg = acq400rd32(adev, ACQ423_SPAN_A+offsetb);

	if (sscanf(buf, "0x%x", &span) == 0 &&
	    sscanf(buf, "%d", &span) == 0){
		    return -1;
	}

	reg &= ~(ACQ423_SPAN_MASK<<shl);
	reg |= (span&ACQ423_SPAN_MASK) << shl;
	if (odd_channels_status(adev)){
		/* ALL EVENS MUST BE ZERO */
		reg &= ~ACQ423_SPAN_MASK_ODDS;
	}
	acq400wr32(adev, ACQ423_SPAN_A+offsetb, reg);
	return count;
}
#define MAKE_ACQ423_SPAN(NAME, CH)				\
static ssize_t show_span_ch##NAME(				\
	struct device * dev,					\
	struct device_attribute *attr,				\
	char * buf)						\
{								\
	return show_span(CH, dev, attr, buf);			\
}								\
								\
static ssize_t store_span_ch##NAME(				\
	struct device * dev,					\
	struct device_attribute *attr,				\
	const char * buf,					\
	size_t count)						\
{								\
	return store_span(CH, dev, attr, buf, count);		\
}								\
static DEVICE_ATTR(span_ch##NAME, S_IRUGO|S_IWUGO,		\
		show_span_ch##NAME, store_span_ch##NAME)

MAKE_ACQ423_SPAN(01, 1);
MAKE_ACQ423_SPAN(02, 2);
MAKE_ACQ423_SPAN(03, 3);
MAKE_ACQ423_SPAN(04, 4);
MAKE_ACQ423_SPAN(05, 5);
MAKE_ACQ423_SPAN(06, 6);
MAKE_ACQ423_SPAN(07, 7);
MAKE_ACQ423_SPAN(08, 8);

MAKE_ACQ423_SPAN(09, 9);
MAKE_ACQ423_SPAN(10,10);
MAKE_ACQ423_SPAN(11,11);
MAKE_ACQ423_SPAN(12,12);
MAKE_ACQ423_SPAN(13,13);
MAKE_ACQ423_SPAN(14,14);
MAKE_ACQ423_SPAN(15,15);
MAKE_ACQ423_SPAN(16,16);

MAKE_ACQ423_SPAN(17,17);
MAKE_ACQ423_SPAN(18,18);
MAKE_ACQ423_SPAN(19,19);
MAKE_ACQ423_SPAN(20,20);
MAKE_ACQ423_SPAN(21,21);
MAKE_ACQ423_SPAN(22,22);
MAKE_ACQ423_SPAN(23,23);
MAKE_ACQ423_SPAN(24,24);

MAKE_ACQ423_SPAN(25,25);
MAKE_ACQ423_SPAN(26,26);
MAKE_ACQ423_SPAN(27,27);
MAKE_ACQ423_SPAN(28,28);
MAKE_ACQ423_SPAN(29,29);
MAKE_ACQ423_SPAN(30,30);
MAKE_ACQ423_SPAN(31,31);
MAKE_ACQ423_SPAN(32,32);


static ssize_t show_odd_channels(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];

	return sprintf(buf, "%d\n", odd_channels_status(adev));
}
static ssize_t store_odd_channels(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
/* user mask: 1=enabled. Compute nchan_enabled BEFORE inverting MASK */
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	int odd_chan_en;

	if (sscanf(buf, "%d", &odd_chan_en) == 1){
		u32 br = acq400rd32(adev, ACQ423_BANK);

		if (odd_chan_en){
			br |= ACQ423_BANK_ODD_CHAN;
			adev->nchan_enabled = 16;
		}else{
			br &= ~ACQ423_BANK_ODD_CHAN;
			adev->nchan_enabled = 32;
		}

		acq400wr32(adev, ACQ423_BANK, br);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(odd_channels,
		S_IRUGO|S_IWUGO, show_odd_channels, store_odd_channels);


const struct attribute *acq423_attrs[] = {
	&dev_attr_odd_channels.attr,
	&dev_attr_span_ch01.attr,
	&dev_attr_span_ch02.attr,
	&dev_attr_span_ch03.attr,
	&dev_attr_span_ch04.attr,
	&dev_attr_span_ch05.attr,
	&dev_attr_span_ch06.attr,
	&dev_attr_span_ch07.attr,
	&dev_attr_span_ch08.attr,
	&dev_attr_span_ch09.attr,
	&dev_attr_span_ch10.attr,
	&dev_attr_span_ch11.attr,
	&dev_attr_span_ch12.attr,
	&dev_attr_span_ch13.attr,
	&dev_attr_span_ch14.attr,
	&dev_attr_span_ch15.attr,
	&dev_attr_span_ch16.attr,
	&dev_attr_span_ch17.attr,
	&dev_attr_span_ch18.attr,
	&dev_attr_span_ch19.attr,
	&dev_attr_span_ch20.attr,
	&dev_attr_span_ch21.attr,
	&dev_attr_span_ch22.attr,
	&dev_attr_span_ch23.attr,
	&dev_attr_span_ch24.attr,
	&dev_attr_span_ch25.attr,
	&dev_attr_span_ch26.attr,
	&dev_attr_span_ch27.attr,
	&dev_attr_span_ch28.attr,
	&dev_attr_span_ch29.attr,
	&dev_attr_span_ch30.attr,
	&dev_attr_span_ch31.attr,
	&dev_attr_span_ch32.attr,
	NULL
};
