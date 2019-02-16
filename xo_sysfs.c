/* ------------------------------------------------------------------------- */
/* xo_sysfs.c ACQ420_FMC						     */
/* ------------------------------------------------------------------------- */
/* xo_sysfs.c  D-TACQ ACQ400 FMC  DRIVER
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

static ssize_t show_playloop_length(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	return sprintf(buf, "%u %u %s\n",
			xo_dev->AO_playloop.length,
			xo_dev->AO_playloop.oneshot,
			xo_dev->AO_playloop.oneshot == AO_oneshot? "ONESHOT":
				xo_dev->AO_playloop.oneshot == AO_oneshot_rearm? "ONESHOT-REARM": "");
}

static ssize_t store_playloop_length(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	unsigned playloop_length;
	unsigned one_shot;
	int rc;

	dev_dbg(DEVP(adev), "store_playloop_length \"%s\"", buf);

	switch(sscanf(buf, "%u %u", &playloop_length, &one_shot)){
	case 2:
		switch(one_shot){
		default:
			return -1;
		case AO_continuous:
		case AO_oneshot:
		case AO_oneshot_rearm:
			xo_dev->AO_playloop.oneshot = one_shot; /* fall thru */
		}
	case 1:
		if (xo_dev->AO_playloop.length == 0 || playloop_length < xo_dev->AO_playloop.length){
			rc = xo400_reset_playloop(adev, playloop_length);
		}else{
			/* append in shot possible */
			xo_dev->AO_playloop.length = playloop_length;
		}
		if (xo_dev->AO_playloop.oneshot != AO_continuous){
			xo_dev->AO_playloop.repeats = 0;
		}
		if (rc == 0){
			return count;
		}else{
			return rc;
		}
	default:
		return -1;
	}
}

static DEVICE_ATTR(playloop_length,
		S_IRUGO|S_IWUSR, show_playloop_length, store_playloop_length);




static ssize_t show_playloop_oneshot(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	return sprintf(buf, "%u\n", xo_dev->AO_playloop.oneshot);
}

static ssize_t store_playloop_oneshot(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	if (sscanf(buf, "%u", &xo_dev->AO_playloop.oneshot) == 1){
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(playloop_oneshot,
		S_IRUGO|S_IWUSR, show_playloop_oneshot, store_playloop_oneshot);


static ssize_t show_playloop_maxlen(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	return sprintf(buf, "%u\n", xo_dev->AO_playloop.maxlen);
}

static ssize_t store_playloop_maxlen(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	if (sscanf(buf, "%u", &xo_dev->AO_playloop.maxlen) == 1){
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(playloop_maxlen,
		S_IRUGO|S_IWUSR, show_playloop_maxlen, store_playloop_maxlen);

static ssize_t show_playloop_maxshot(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	return sprintf(buf, "%u\n", xo_dev->AO_playloop.maxshot);
}

static ssize_t store_playloop_maxshot(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	if (sscanf(buf, "%u", &xo_dev->AO_playloop.maxshot) == 1){
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(playloop_maxshot,
		S_IRUGO|S_IWUSR, show_playloop_maxshot, store_playloop_maxshot);

static ssize_t show_playloop_cursor(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	return sprintf(buf, "%u\n", xo_dev->AO_playloop.cursor);
}

static ssize_t store_playloop_cursor(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	if (sscanf(buf, "%u", &xo_dev->AO_playloop.cursor) == 1){
		return count;
	}else{
		return -1;
	}
}




static DEVICE_ATTR(playloop_cursor,
		S_IRUGO|S_IWUSR, show_playloop_cursor, store_playloop_cursor);

static ssize_t show_task_active(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->task_active);
}

static DEVICE_ATTR(task_active, S_IRUGO, show_task_active, 0);

static ssize_t show_pull_buf(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	return sprintf(buf, "%u\n", xo_dev->AO_playloop.pull_buf);
}

static DEVICE_ATTR(playloop_pull_buf, S_IRUGO, show_pull_buf, 0);


static ssize_t show_push_buf(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	return sprintf(buf, "%u\n", xo_dev->AO_playloop.push_buf);
}

static ssize_t store_push_buf(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	if (sscanf(buf, "%u", &xo_dev->AO_playloop.push_buf) == 1){
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(playloop_push_buf,
		S_IRUGO|S_IWUSR, show_push_buf, store_push_buf);

static ssize_t show_playloop_repeats(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	return sprintf(buf, "%u\n", xo_dev->AO_playloop.repeats);
}

static ssize_t store_playloop_repeats(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	struct XO_dev* xo_dev = container_of(adev, struct XO_dev, adev);
	if (sscanf(buf, "%u", &xo_dev->AO_playloop.repeats) == 1){
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(playloop_repeats,
		S_IRUGO|S_IWUSR, show_playloop_repeats, store_playloop_repeats);


static ssize_t show_xo_buffers(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u %u\n", adev->stats.xo.dma_buffers_out,
						adev->stats.xo.dma_buffers_in);
}
static DEVICE_ATTR(xo_buffers, S_IRUGO, show_xo_buffers, 0);


const struct attribute *playloop_attrs[] = {
	&dev_attr_playloop_length.attr,
	&dev_attr_playloop_oneshot.attr,
	&dev_attr_playloop_maxshot.attr,
	&dev_attr_playloop_cursor.attr,
	&dev_attr_playloop_repeats.attr,
	&dev_attr_playloop_maxlen.attr,
	&dev_attr_playloop_push_buf.attr,
	&dev_attr_playloop_pull_buf.attr,
	&dev_attr_xo_buffers.attr,
	&dev_attr_task_active.attr,
	NULL
};
