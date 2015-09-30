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

void acq480wr32(struct acq400_dev *adev, int offset, u32 value)
{
	if (adev->RW32_debug){
		dev_info(DEVP(adev), "acq400wr32 %p [0x%02x] = %08x\n",
				adev->dev_virtaddr + offset, offset, value);
	}else{
		dev_dbg(DEVP(adev), "acq400wr32 %p [0x%02x] = %08x\n",
				adev->dev_virtaddr + offset, offset, value);
	}

	iowrite32(value, adev->dev_virtaddr + offset);
}

u32 acq480rd32(struct acq400_dev *adev, int offset)
{
	u32 rc = ioread32(adev->dev_virtaddr + offset);
	if (adev->RW32_debug){
		dev_info(DEVP(adev), "acq400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	}else{
		dev_dbg(DEVP(adev), "acq400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	}
	return rc;
}

#define acq400wr32 acq480wr32
#define acq400rd32 acq480rd32

enum ACQ480_TRAINING {
	ACQ480_RESET,
	ACQ480_START,
	ACQ480_DESKEW,
	ACQ480_DESKEW_DONE,
	ACQ480_SYNC,
	ACQ480_SYNC_DONE,
	ACQ480_ACTIVATE
};

static ssize_t show_train_states(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	char* pb = buf;
#define ST_APPEND(ST) pb += sprintf(pb, "%s=%d\n", #ST, ST)
	ST_APPEND(ACQ480_RESET);
	ST_APPEND(ACQ480_START);
	ST_APPEND(ACQ480_DESKEW);
	ST_APPEND(ACQ480_DESKEW_DONE);
	ST_APPEND(ACQ480_SYNC);
	ST_APPEND(ACQ480_SYNC_DONE);
	ST_APPEND(ACQ480_ACTIVATE);
	return pb - buf;
#undef ST_APPEND
}
static DEVICE_ATTR(train_states, S_IRUGO, show_train_states, 0);

static ssize_t show_train(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];

	return sprintf(buf, "%u\n", adev->acq480.train);
}

static ssize_t acq480_reset(struct acq400_dev *adev, int goodrc)
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);
	ctrl &= ~(ADC_CTRL_ADC_EN|ADC_CTRL_FIFO_EN);
	ctrl &= ~(ADC480_CTRL_DESKEW_TRAIN|ADC480_CTRL_SYNC_TRAIN);
	acq400wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_ADC_RST);

	adev->acq480.train = ACQ480_RESET;
	return goodrc;
}

static ssize_t acq480_start(struct acq400_dev *adev, int goodrc)
{
	if (adev->acq480.train != ACQ480_RESET){
		dev_err(DEVP(adev), "acq480_start not in ACQ480_RESET");
		return -1;
	}else{
		u32 ctrl = acq400rd32(adev, ADC_CTRL);
		acq400wr32(adev, ADC_CTRL, ctrl &~ ADC_CTRL_ADC_RST);
		adev->acq480.train = ACQ480_START;
		return goodrc;
	}
}

static int training_done(unsigned stat, unsigned shl)
// DONE when ALL bits HI
{
	return ((stat>>shl) & ADC480_FIFO_STA_DONE_MASK) == ADC480_FIFO_STA_DONE_MASK;
}
static ssize_t acq480_deskew(struct acq400_dev *adev, int goodrc)
{
	if (adev->acq480.train != ACQ480_START){
		dev_err(DEVP(adev), "acq480_deskew not in ACQ480_START");
		return -1;
	}else{
		u32 ctrl = acq400rd32(adev, ADC_CTRL);
		u32 stat;
		acq400wr32(adev, ADC_CTRL, ctrl | ADC480_CTRL_DESKEW_TRAIN);
		adev->acq480.train = ACQ480_DESKEW;

		stat = acq400rd32(adev, ADC_FIFO_STA);
		msleep(20);
		stat = acq400rd32(adev, ADC_FIFO_STA);
		if (!training_done(stat, ADC480_FIFO_STA_DESKEW_DONE_SHL)){
			dev_err(DEVP(adev), "deskew failed to complete %08x", stat);
			return -1;
		}else{
			adev->acq480.train = ACQ480_DESKEW_DONE;
			return goodrc;
		}
	}
}

static ssize_t acq480_sync(struct acq400_dev *adev, int goodrc)
{
	if (adev->acq480.train != ACQ480_DESKEW_DONE){
		dev_err(DEVP(adev), "acq480_sync not in ACQ480_DESKEW_DONE");
		return -1;
	}else{
		u32 ctrl = acq400rd32(adev, ADC_CTRL);
		u32 stat;
		acq400wr32(adev, ADC_CTRL, ctrl | ADC480_CTRL_SYNC_TRAIN);
		adev->acq480.train = ACQ480_SYNC;

		stat = acq400rd32(adev, ADC_FIFO_STA);
		msleep(20);
		stat = acq400rd32(adev, ADC_FIFO_STA);
		if (!training_done(stat, ADC480_FIFO_STA_SYNC_DONE_SHL)){
			dev_err(DEVP(adev), "deskew failed to complete %08x", stat);
			return -1;
		}else{
			acq400wr32(adev, ADC_CTRL, ctrl &
				~(ADC480_CTRL_DESKEW_TRAIN|ADC480_CTRL_SYNC_TRAIN));
			adev->acq480.train = ACQ480_SYNC_DONE;
			return goodrc;
		}
	}
}
static ssize_t acq480_activate(struct acq400_dev *adev, int goodrc)
{
	if (adev->acq480.train != ACQ480_SYNC_DONE){
		dev_err(DEVP(adev), "acq480_activate not in ACQ480_SYNC_DONE");
		return -1;
	}else{
		u32 ctrl = acq400rd32(adev, ADC_CTRL);
		acq400wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_ADC_EN|ADC_CTRL_FIFO_EN);
		adev->acq480.train = ACQ480_ACTIVATE;
		return goodrc;
	}
}


static ssize_t store_train(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	int train;

	if (sscanf(buf, "%u", &train) == 1){
		dev_dbg(DEVP(adev), "store_train() %u", train);
		switch(train){
		case ACQ480_RESET:
			return acq480_reset(adev, count);
		case ACQ480_START:
			return acq480_start(adev, count);
		case ACQ480_DESKEW:
			return acq480_deskew(adev, count);
		case ACQ480_SYNC:
			return acq480_sync(adev, count);
		case ACQ480_ACTIVATE:
			return acq480_activate(adev, count);
		case ACQ480_DESKEW_DONE:
		case ACQ480_SYNC_DONE:
			dev_err(DEVP(adev), "do not set DONE state %u", train);
			return -1;
		default:
			dev_err(DEVP(adev), "unrecognised state %u", train);
		}

		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(train, S_IRUGO|S_IWUGO, show_train, store_train);


const struct attribute *acq480_attrs[] = {
	&dev_attr_train.attr,
	&dev_attr_train_states.attr,
	NULL
};
