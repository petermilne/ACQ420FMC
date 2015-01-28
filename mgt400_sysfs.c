/* ------------------------------------------------------------------------- */
/* acq400_drv.c  D-TACQ ACQ400 FMC  DRIVER   
 *
 * mgt400_sysfs.c
 *
 *  Created on: 13 Jan 2015
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

#include "acq400.h"
#include "mgt400.h"


static ssize_t show_heartbeat(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	return sprintf(buf, "%u\n", mgt400rd32(mdev, HEART));
}
static DEVICE_ATTR(heartbeat, S_IRUGO, show_heartbeat, 0);

static ssize_t show_name(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	return sprintf(buf, "%s\n", mdev->devname);
}
static DEVICE_ATTR(name, S_IRUGO, show_name, 0);


static ssize_t show_site(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	return sprintf(buf, "%u\n", mdev->of_prams.site);
}
static DEVICE_ATTR(site, S_IRUGO, show_site, 0);

static const struct attribute *sysfs_base_attrs[] = {
	&dev_attr_heartbeat.attr,
	&dev_attr_name.attr,
	&dev_attr_site.attr,
	NULL
};
void mgt400_createSysfs(struct device *dev)
{
	//struct mgt400_dev *mdev = mgt400_devices[dev->id];

	dev_info(dev, "mgt400_createSysfs()");
	if (sysfs_create_files(&dev->kobj, sysfs_base_attrs)){
		dev_err(dev, "failed to create sysfs");
	}

}
