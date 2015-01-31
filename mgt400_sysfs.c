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


#define DMA_STATUS(DIR, REG, SHL) 					\
static ssize_t show_dma_stat_##DIR(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	struct mgt400_dev *mdev = mgt400_devices[dev->id];		\
	u32 sta = mgt400rd32(mdev, REG) >> SHL;				\
	u32 count = (sta&DMA_DATA_FIFO_COUNT) >> DMA_DATA_FIFO_COUNT_SHL; \
	u32 flags = sta&DMA_DATA_FIFO_FLAGS;				\
	return sprintf(buf, "%d %d \n", count, flags);			\
}									\
									\
static DEVICE_ATTR(dma_stat_##DIR, S_IRUGO, show_dma_stat_##DIR, 0)	\

DMA_STATUS(desc_pull, DESC_FIFO_SR, DMA_DATA_PULL_SHL);
DMA_STATUS(desc_push, DESC_FIFO_SR, DMA_DATA_PUSH_SHL);
DMA_STATUS(data_pull, DMA_FIFO_SR,  DMA_DATA_PULL_SHL);
DMA_STATUS(data_push, DMA_FIFO_SR,  DMA_DATA_PUSH_SHL);

static ssize_t show_buffer_counts(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	return sprintf(buf, "%lu,%lu\n",
			mdev->push.buffer_count, mdev->pull.buffer_count);
}

static DEVICE_ATTR(buffer_counts, S_IRUGO, show_buffer_counts, 0);

static ssize_t show_enable(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	return sprintf(buf, "%u\n", mgt400rd32(mdev, ZDMA_CR)&ZDMA_CR_ENABLE);
}
static DEVICE_ATTR(enable, S_IRUGO, show_enable, 0);

static ssize_t store_aurora_enable(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	int enable;

	if (sscanf(buf, "%d", &enable) == 1){
		u32 cr = mgt400rd32(mdev, AURORA_CR);
		if (enable){
			cr |= AURORA_CR_ENA;
		}else{
			cr &= ~AURORA_CR_ENA;
		}
		mgt400wr32(mdev, AURORA_CR, cr);
		return strlen(buf);
	}else{
		return -1;
	}
}
static ssize_t show_aurora_enable(
		struct device * dev,
		struct device_attribute *attr,
		char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	return sprintf(buf, "%u\n", (mgt400rd32(mdev, AURORA_CR)&AURORA_CR_ENA)!=0);
}
static DEVICE_ATTR(aurora_enable, S_IRUGO|S_IWUGO, show_aurora_enable, store_aurora_enable);


static ssize_t show_aurora_lane_up(
		struct device * dev,
		struct device_attribute *attr,
		char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	return sprintf(buf, "%u\n", AURORA_SR_UP(mgt400rd32(mdev, AURORA_SR)));
}
static DEVICE_ATTR(aurora_lane_up, S_IRUGO, show_aurora_lane_up, 0);

static ssize_t show_aurora_errors(
		struct device * dev,
		struct device_attribute *attr,
		char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	unsigned errs = AURORA_SR_ERR(mgt400rd32(mdev, AURORA_SR));
	if (errs){
		u32 cr = mgt400rd32(mdev, AURORA_CR);
		mgt400wr32(mdev, AURORA_CR, cr | AURORA_CR_CLR);
		mgt400wr32(mdev, AURORA_CR, cr);
	}
	return sprintf(buf, "%u\n", errs);
}
static DEVICE_ATTR(aurora_errors, S_IRUGO, show_aurora_errors, 0);

static ssize_t show_heartbeat(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	/* EPICS DS can't handle a full u32 */
	return sprintf(buf, "%u\n", mgt400rd32(mdev, HEART) >> 1);
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

static ssize_t show_dev(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	return sprintf(buf, "%u\n", MAJOR(mdev->cdev.dev));
}
static DEVICE_ATTR(dev, S_IRUGO, show_dev, 0);

/* echo 0 1 2 3 > clear_histo : clears everything */
static ssize_t store_clear_histo(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	struct mgt400_dev *mdev = mgt400_devices[dev->id];
	int minor[4];
	int nf = sscanf(buf, "%d %d %d %d", minor+0, minor+1, minor+2, minor+3);

	if (nf) {
		while(nf-- > 0){
			int rc = mgt400_clear_histo(mdev, minor[nf]);
			if (rc){
				return rc;
			}
		}
		return strlen(buf);
	}else{
		return -1;
	}
}
static DEVICE_ATTR(clear_histo, S_IWUGO, 0, store_clear_histo);

static const struct attribute *sysfs_base_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_aurora_enable.attr,
	&dev_attr_aurora_lane_up.attr,
	&dev_attr_aurora_errors.attr,
	&dev_attr_heartbeat.attr,
	&dev_attr_name.attr,
	&dev_attr_site.attr,
	&dev_attr_dev.attr,
	&dev_attr_dma_stat_desc_pull.attr,
	&dev_attr_dma_stat_desc_push.attr,
	&dev_attr_dma_stat_data_pull.attr,
	&dev_attr_dma_stat_data_push.attr,
	&dev_attr_buffer_counts.attr,
	&dev_attr_clear_histo.attr,
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
