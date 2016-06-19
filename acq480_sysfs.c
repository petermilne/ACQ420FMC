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
	ACQ480_ACTIVATE,
	ACQ480_FAIL
};

static ssize_t show_train_states(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	char* pb = buf;
#define ST_APPEND(ST) pb += sprintf(pb, "%s=%d\n", #ST, ST)
	ST_APPEND(ACQ480_RESET);
	ST_APPEND(ACQ480_START);
	ST_APPEND(ACQ480_DESKEW);
	ST_APPEND(ACQ480_DESKEW_DONE);
	ST_APPEND(ACQ480_SYNC);
	ST_APPEND(ACQ480_SYNC_DONE);
	ST_APPEND(ACQ480_ACTIVATE);
	ST_APPEND(ACQ480_FAIL);
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
		case ACQ480_FAIL:
			adev->acq480.train = ACQ480_FAIL;
			break;
		default:
			dev_err(DEVP(adev), "unrecognised state %u", train);
		}

		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(train, S_IRUGO|S_IWUGO, show_train, store_train);


static ssize_t store_dclock_reset(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 reset;

	if (sscanf(buf, "%u", &reset) == 1 && reset == 1){
		u32 ctrl = acq400rd32(adev, ADC_CTRL);
		if (ctrl & ADC_CTRL_ENABLE_CAPTURE){
			dev_err(dev, "resetting DCLK, but ADC_ENABLED: 0x%08x", ctrl);
		}
		if (ctrl & ADC_CTRL_480_DCLK_SYNC){
			dev_err(dev, "resetting DCLK, but it was already set: 0x%08x", ctrl);
		}
		ctrl &= ~ADC_CTRL_480_DCLK_SYNC;
		dev_dbg(dev, "setting ADC_CTRL %08x", ctrl|ADC_CTRL_480_DCLK_SYNC);
		acq400wr32(adev, ADC_CTRL, ctrl|ADC_CTRL_480_DCLK_SYNC);
		msleep(1);
		dev_dbg(dev, "clear ADC_CTRL %08x", ctrl);
		acq400wr32(adev, ADC_CTRL, ctrl);
		return count;
	}else{
		return -1;
	}
}


static DEVICE_ATTR(dclock_reset, S_IWUGO, 0, store_dclock_reset);

static ssize_t show_acq480_train_ctrl(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 tc = acq400rd32(adev, ACQ480_TRAIN_CTRL);

	return sprintf(buf, "0x%08x\n", tc);
}

#define V2F_FREQ_OFF_MAX ((1<<22)-1)

static ssize_t store_acq480_train_ctrl(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 tc;

	if (sscanf(buf, "%x", &tc) == 1){
		acq400wr32(adev, ACQ480_TRAIN_CTRL, tc);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(acq480_train_ctrl,
		S_IRUGO|S_IWUGO, show_acq480_train_ctrl, store_acq480_train_ctrl);

static ssize_t show_acq480_two_lane_mode(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 adc_ctrl = acq400rd32(adev, ADC_CTRL);

	return sprintf(buf, "%d\n", (adc_ctrl&ADC_CTRL_480_TWO_LANE_MODE) != 0);
}


static ssize_t store_acq480_two_lane_mode(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 tc;

	if (sscanf(buf, "%x", &tc) == 1){
		u32 adc_ctrl = acq400rd32(adev, ADC_CTRL);
		if (tc){
			adc_ctrl |= ADC_CTRL_480_TWO_LANE_MODE;
		} else {
			adc_ctrl &+ ~ADC_CTRL_480_TWO_LANE_MODE;
		}
		acq400wr32(adev, ADC_CTRL, adc_ctrl);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(acq480_two_lane_mode,
		S_IRUGO|S_IWUGO, show_acq480_two_lane_mode, store_acq480_two_lane_mode);

static ssize_t show_acq480_train_xx_val(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	int data_reg, int frame_shl)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 tc = acq400rd32(adev, data_reg);
	u32 ctrl = acq400rd32(adev, ADC_CTRL);

	return sprintf(buf, "0x%08x %x\n", tc,
			(ctrl>>frame_shl)&ADC480_CTRL_FRAME_MASK);
}


static ssize_t show_acq480_train_hi_val(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	return show_acq480_train_xx_val(dev,attr, buf,
			ACQ480_TRAIN_HI_VAL, ADC480_CTRL_FRAME_HI_SHL);
}


static DEVICE_ATTR(acq480_train_hi_val, S_IRUGO, show_acq480_train_hi_val, 0);

static ssize_t show_acq480_train_lo_val(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	return show_acq480_train_xx_val(dev,attr, buf,
			ACQ480_TRAIN_LO_VAL, ADC480_CTRL_FRAME_LO_SHL);
}


static DEVICE_ATTR(acq480_train_lo_val, S_IRUGO, show_acq480_train_lo_val, 0);



static ssize_t show_acq480_loti(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];

	return sprintf(buf, "%d\n", acq480_train_fail(adev));
}


static DEVICE_ATTR(acq480_loti, S_IRUGO, show_acq480_loti, 0);


const struct attribute *acq480_attrs[] = {
	&dev_attr_train.attr,
	&dev_attr_train_states.attr,
	&dev_attr_dclock_reset.attr,
	&dev_attr_acq480_train_ctrl.attr,
	&dev_attr_acq480_train_hi_val.attr,
	&dev_attr_acq480_train_lo_val.attr,
	&dev_attr_acq480_loti.attr,
	&dev_attr_acq480_two_lane_mode.attr,
	NULL
};



