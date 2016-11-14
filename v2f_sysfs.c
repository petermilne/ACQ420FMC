/*
 * v2f_sysfs.c
 *
 *  Created on: 12 Apr 2016
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

static ssize_t show_chan_sel(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 cs = acq400rd32(adev, V2F_CHAN_SEL);
	return sprintf(buf, "%d,%d,%d,%d\n",
			V2F_CHAN_SEL_DEC(cs, 1), V2F_CHAN_SEL_DEC(cs, 2),
			V2F_CHAN_SEL_DEC(cs, 3), V2F_CHAN_SEL_DEC(cs, 4));
}
static ssize_t store_chan_sel(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	int sel[4];
	if (sscanf(buf, "%u,%u,%u,%u", sel+0, sel+1, sel+2, sel+3) == 4){
		u32 cs = V2F_CHAN_SEL_ENC(1, sel[0]) +
			 V2F_CHAN_SEL_ENC(2, sel[1]) +
			 V2F_CHAN_SEL_ENC(3, sel[2]) +
			 V2F_CHAN_SEL_ENC(4, sel[3]);
		acq400wr32(adev, V2F_CHAN_SEL, cs);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(chan_sel,
		S_IRUGO|S_IWUGO, show_chan_sel, store_chan_sel);

static ssize_t show_hf(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 ctrl = acq400rd32(adev, V2F_CTRL);
	return sprintf(buf, "%d\n", (ctrl&V2F_CTRL_RANGE_HI) != 0);
}
static ssize_t store_hf(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned hf;
	if (sscanf(buf, "%u", &hf) == 1){
		u32 ctrl = acq400rd32(adev, V2F_CTRL);
		if (hf){
			ctrl |= V2F_CTRL_RANGE_HI;
		}else{
			ctrl &= ~V2F_CTRL_RANGE_HI;
		}
		acq400wr32(adev, V2F_CTRL, ctrl);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(hf, S_IRUGO|S_IWUGO, show_hf, store_hf);

/*
#define V2F_OFFSET_PACKED_1M 	3277
#define V2F_OFFSET_UNPACKED_1M 	838861

static ssize_t show_freq_offset(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 freq_off = acq400rd32(adev, V2F_FREQ_OFF);
	unsigned mstep = adev->data32? V2F_OFFSET_UNPACKED_1M: V2F_OFFSET_PACKED_1M;

	return sprintf(buf, "%u\n", freq_off/mstep);
}


static ssize_t store_freq_offset(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned mstep = adev->data32? V2F_OFFSET_UNPACKED_1M: V2F_OFFSET_PACKED_1M;
	u32 freq_off;

	if (sscanf(buf, "%u", &freq_off) == 1){
		if (freq_off > 4) freq_off = 4;

		acq400wr32(adev, V2F_FREQ_OFF, freq_off*mstep);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(freq_offset,
		S_IRUGO|S_IWUGO, show_freq_offset, store_freq_offset);
*/
static ssize_t show_freq_offset_raw(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 freq_off = acq400rd32(adev, V2F_FREQ_OFF);

	return sprintf(buf, "%u\n", freq_off);
}


static ssize_t store_freq_offset_raw(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 freq_off;

	if (sscanf(buf, "%u", &freq_off) == 1){
		if (freq_off > V2F_FREQ_OFF_MAX) freq_off = V2F_FREQ_OFF_MAX;
		acq400wr32(adev, V2F_FREQ_OFF, freq_off);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(freq_offset_raw,
		S_IRUGO|S_IWUGO, show_freq_offset_raw, store_freq_offset_raw);


static ssize_t show_reg(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	const int reg_off)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 reg = acq400rd32(adev, reg_off);
	return sprintf(buf, "%u\n", reg);
}

static ssize_t store_reg(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count,
	const int reg_off,
	const unsigned RMAX)
{
	u32 reg;
	if (sscanf(buf, "0x%x", &reg) == 1 || sscanf(buf, "%u", &reg) == 1){
		struct acq400_dev* adev = acq400_devices[dev->id];
		if (reg > RMAX) reg = RMAX;
		acq400wr32(adev, reg_off, reg);
		return count;
	}else{
		return -1;
	}
}

#define MAKE_REG_CAL(kname, REG, MAX)							\
static ssize_t show_reg_##kname(							\
	struct device *dev,								\
	struct device_attribute *attr,							\
	char* buf)									\
{											\
	return show_reg(dev, attr, buf, REG);						\
}											\
static ssize_t store_reg_##kname(							\
	struct device *dev, 								\
	struct device_attribute *attr,							\
	const char* buf,								\
	size_t count)									\
{											\
	return store_reg(dev, attr, buf, count, REG, MAX);				\
}											\
static DEVICE_ATTR(kname, S_IRUGO|S_IWUGO, show_reg_##kname, store_reg_##kname)


MAKE_REG_CAL(v2f_freq_off_1, V2F_FREQ_OFF+0x0, V2F_FREQ_OFF_MAX);
MAKE_REG_CAL(v2f_freq_off_2, V2F_FREQ_OFF+0x4, V2F_FREQ_OFF_MAX);
MAKE_REG_CAL(v2f_freq_off_3, V2F_FREQ_OFF+0x8, V2F_FREQ_OFF_MAX);
MAKE_REG_CAL(v2f_freq_off_4, V2F_FREQ_OFF+0xc, V2F_FREQ_OFF_MAX);

MAKE_REG_CAL(v2f_freq_slo_1, V2F_FREQ_SLO+0x0, V2F_FREQ_SLO_MAX);
MAKE_REG_CAL(v2f_freq_slo_2, V2F_FREQ_SLO+0x4, V2F_FREQ_SLO_MAX);
MAKE_REG_CAL(v2f_freq_slo_3, V2F_FREQ_SLO+0x8, V2F_FREQ_SLO_MAX);
MAKE_REG_CAL(v2f_freq_slo_4, V2F_FREQ_SLO+0xc, V2F_FREQ_SLO_MAX);

const struct attribute *sysfs_v2f_attrs[] = {
	&dev_attr_hf.attr,
	//&dev_attr_freq_offset.attr,
	&dev_attr_freq_offset_raw.attr,
	&dev_attr_chan_sel.attr,

	&dev_attr_v2f_freq_off_1.attr,
	&dev_attr_v2f_freq_off_2.attr,
	&dev_attr_v2f_freq_off_3.attr,
	&dev_attr_v2f_freq_off_4.attr,

	&dev_attr_v2f_freq_slo_1.attr,
	&dev_attr_v2f_freq_slo_2.attr,
	&dev_attr_v2f_freq_slo_3.attr,
	&dev_attr_v2f_freq_slo_4.attr,
	NULL
};

static ssize_t show_qen_count(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	int count = acq400rd32(adev, QEN_ENC_COUNT);
	return sprintf(buf, "%d\n", count);
}

static ssize_t store_qen_count(							\
	struct device *dev, 								\
	struct device_attribute *attr,							\
	const char* buf,								\
	size_t count)									\
{											\
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 ctrl = acq400rd32(adev, QEN_CTRL);
	acq400wr32(adev, QEN_CTRL, ctrl | QEN_CTRL_RESET);
	acq400wr32(adev, QEN_CTRL, ctrl);
	return count;
}
static DEVICE_ATTR(qen_count, S_IRUGO|S_IWUGO, show_qen_count, store_qen_count);


const struct attribute *sysfs_qen_attrs[] = {
	&dev_attr_qen_count.attr,
	NULL
};
