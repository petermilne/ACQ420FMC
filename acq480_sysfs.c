/* ------------------------------------------------------------------------- */
/* acq480_sysfs.c ACQ420_FMC						     */
/*
 * acq480_sysfs.c
 *
 *  Created on: 27 Jul 2015
 *      Author: pgm
 */

/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2015 Peter Milne, D-TACQ Solutions Ltd                    *
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
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

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */

#include "lk-shim.h"


#include <linux/device.h>
#include <linux/module.h>
#include <linux/user.h>

#include "acq400.h"

/* acq480 training
 * 0: RESET	set CR=0x09, reset
 * 1: START	set CR=0x01, enable
 * 						acq480_knobs : Set ADC DESKEW
 * 2: DESKEW	set DESKEW, wait done
 * 3:           DESKEW DONE			acq480_knobs : Set ADC SYNC
 * 4: SYNC	set SYNC, wait done
 * 5:		SYNC DONE			acq480_knobs : Clear SYNC
 * 4: ACTIVATE 	set CR=0x15 : go
 */
static ssize_t show_train(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 acc_dec = acq400rd32(adev, ADC_ACC_DEC);
	unsigned shift = (acc_dec&ADC_ACC_DEC_SHIFT_MASK)>>
					getSHL(ADC_ACC_DEC_SHIFT_MASK);

	return sprintf(buf, "%u,%u\n",
			(acc_dec&ADC_ACC_DEC_LEN)+1, shift);
}

static ssize_t store_train(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	int train;
	int shift = 0;

	if (sscanf(buf, "%u,%u", &train, &shift) >= 1){
		u32 acdc;

		train = max(train, 1);
		train = min(train, ADC_MAX_NACC);
		shift = min(shift, ADC_ACC_DEC_SHIFT_MAX);

		acdc = train-1;
		acdc |= shift<<getSHL(ADC_ACC_DEC_SHIFT_MASK);
		acq400wr32(adev, ADC_ACC_DEC, acdc);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(train, S_IRUGO|S_IWUGO, show_train, store_train);

const struct attribute *acq480_attrs[] = {
	&dev_attr_train.attr,
	NULL
};
