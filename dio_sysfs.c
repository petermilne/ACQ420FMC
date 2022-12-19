/* ------------------------------------------------------------------------- */
/* dio_sysfs.c D-TACQ ACQ400 series driver, sysfs (knobs)	             */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>
    This program is free software; you can redistribute it and/or modify
    it under the terms of Version 2 of the GNU General Public License
    as published by the Free Software Foundation;

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */


#include <linux/ctype.h>

#include "acq400.h"
#include "acq400_sysfs.h"

#include "sysfs_attrs.h"


static ssize_t show_DO32_reg(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned reg)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "0x%08x\n", acq400rd32(adev, reg));
}

/*
 * DO32=value  :: DO32 = value
 * DO32=+value :: DO32 |= value   	set bits
 * DO32=-value :: DO32 &= ~value        clear bits
 * DO32=+dvalue:: DO32 |= 1<<value      set bit
 * DO32=-dvalue:: DO32 &= ~(1<<value)   clear bit
 */
static ssize_t store_DO32_reg(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count,
	unsigned reg)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned DO32 = 0;
	enum BITMODE { CLR=-1, CPY=0, SET=1 } mode = CPY;		/* -1: CLR, 0: OVERWRITE, 1: SET */
	int is_bit = 0;

	if (buf[0] == '+'){
		mode = SET;
		buf++;
	}else if (buf[0] == '-'){
		mode = CLR;
		buf++;
	}
	if (mode != CPY && buf[0] == 'd'){
		is_bit = 1;
		buf++;
	}
	if (sscanf(buf, "0x%x", &DO32) == 1 || sscanf(buf, "%u", &DO32) == 1){
		if (mode != CPY){
			unsigned old = acq400rd32(adev, reg);
			if (is_bit){
				DO32 = 1<<DO32;
			}
			if (mode == SET){
				DO32 = old | DO32;
			}else if (mode == CLR){
				DO32 = old & ~DO32;
			}else{
				return -1;
			}
		}
		acq400wr32(adev, reg, DO32);
		return count;
	}else{
		return -1;
	}
}


#define MAKE_STORE_DO32(name, wreg, rreg) 					\
static ssize_t show_##name(							\
	struct device * dev,							\
	struct device_attribute *attr,						\
	char * buf)								\
{										\
	return show_DO32_reg(dev, attr, buf, rreg);				\
}										\
static ssize_t store_##name(							\
	struct device * dev,							\
	struct device_attribute *attr,						\
	const char * buf,							\
	size_t count)								\
{										\
	return store_DO32_reg(dev, attr, buf, count, wreg);			\
}										\
static DEVICE_ATTR(name, S_IRUGO|S_IWUSR, show_##name, store_##name)

MAKE_STORE_DO32(DO32, 			DIO432_FIFO, DIO432_DI_SNOOP);
MAKE_STORE_DO32(DO32_immediate_mask, 	DIO482_PG_IMM_MASK, DIO482_PG_IMM_MASK);

extern const struct device_attribute dev_attr_byte_is_output;

SCOUNT_KNOB(FPTRG, DIO482_PG_FPTRG_COUNT);

static ssize_t show_num_pg(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 ctrl = acq400rd32(adev, DIO432_CTRL);

	return sprintf(buf, "%d\n", ctrl&DIO432_CTRL_PG_CLK_IS_DO? 5: 4);
}

static ssize_t store_num_pg(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 ctrl = acq400rd32(adev, DIO432_CTRL);
	unsigned num_pg;

	if (sscanf(buf, "%u", &num_pg) == 1){
		if (num_pg == 5){
			ctrl |= DIO432_CTRL_PG_CLK_IS_DO;
		}else{
			ctrl &= ~DIO432_CTRL_PG_CLK_IS_DO;
		}
		acq400wr32(adev, DIO432_CTRL, ctrl);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(num_pg, S_IRUGO|S_IWUSR, show_num_pg, store_num_pg);

MAKE_BITS(chain_PG,   DIO432_CTRL, MAKE_BITS_FROM_MASK, DIO432_CTRL_CHAIN_PG);
MAKE_BITS(trgout_PG4, DIO432_CTRL, MAKE_BITS_FROM_MASK, DIO432_CTRL_CHAIN_PG);
MAKE_DNUM(wdt, 	      DIO482_PG_WDT, DIO482_PG_WDT_MASK);

MAKE_BITS(bypass_trg_debounce, DIO432_CTRL, MAKE_BITS_FROM_MASK, DIO432_CTRL_BYPASS_TRG);


const struct attribute *dio482_pg_attrs[] = {
		&dev_attr_DO32.attr,
		&dev_attr_DO32_immediate_mask.attr,
		&dev_attr_scount_FPTRG.attr,
		&dev_attr_trgout_PG4.attr,
		&dev_attr_chain_PG.attr,
		&dev_attr_num_pg.attr,
		&dev_attr_wdt.attr,
		&dev_attr_bypass_trg_debounce.attr,
		NULL
};

const struct attribute *dio482_pg32_attrs[] = {
		&dev_attr_DO32.attr,
		&dev_attr_DO32_immediate_mask.attr,
		&dev_attr_wdt.attr,
		&dev_attr_bypass_trg_debounce.attr,
		NULL
};

