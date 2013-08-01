/* ------------------------------------------------------------------------- */
/* rtm-t-sysfs.c RTM-T PCIe Host Side driver, sysfs (knobs)	             */
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


/** @file rtm-t-sysfs.c D-TACQ PCIe RTM_T driver, sysfs (knobs) */



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

#include "acq420FMC.h"

#define DEVICE_CREATE_FILE(dev, attr) 							\
	do {										\
		if (device_create_file(dev, attr)){ 					\
			dev_warn(dev, "%s:%d device_create_file", __FILE__, __LINE__); 	\
		} 									\
	} while (0)


static ssize_t show_clkdiv(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 clkdiv = acq420rd32(acq420_devices[dev->id], ADC_CLKDIV);
	return sprintf(buf, "%u\n", clkdiv);
}

static ssize_t store_clkdiv(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	u32 clkdiv;
	if (sscanf(buf, "%u", &clkdiv) == 1 &&
	    clkdiv > 1 &&
	    clkdiv <= ADC_CLK_DIV_MASK){
		acq420wr32(acq420_devices[dev->id], ADC_CLKDIV, clkdiv);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(clkdiv, S_IRUGO|S_IWUGO, show_clkdiv, store_clkdiv);

unsigned get_gain(u32 gains, int chan)
{
	return (gains >> chan*2) & 0x3;
}

unsigned set_gain(u32 gains, int chan, u32 value)
{
	u32 mask = 0x3 << chan*2;
	gains &= ~mask;
	gains |= (value&0x3) << chan*2;
	return gains;
}
static ssize_t show_gains(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 gains = acq420rd32(acq420_devices[dev->id], ADC_GAIN);
	return sprintf(buf, "%u%u%u%u\n",
			get_gain(gains, 0), get_gain(gains, 1),
			get_gain(gains, 2), get_gain(gains, 3));
}

static ssize_t store_gains(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	char gx[4];

	if (sscanf(buf, "%c%c%c%c", gx+0, gx+1, gx+2, gx+3) == 4){
		u32 gains = acq420rd32(acq420_devices[dev->id], ADC_GAIN);
		int ic;
		for (ic = 0; ic < 4; ++ic){
			if (gx[ic] >= '0' && gx[ic] <= '3'){
				gains = set_gain(gains, ic, gx[ic]-'0');
			}else{
				switch(gx[ic]) {
				case '-':
				case 'x':
				case 'X':
					break;
				default:
					dev_warn(dev, "bad value at %c at %d", gx[ic], ic);
				}
			}
		}
		dev_dbg(dev, "set gains: %02x", gains);
		acq420wr32(acq420_devices[dev->id], ADC_GAIN, gains);
		return count;
	}else{
		dev_warn(dev, "rejecting input args != 4");
		return -1;
	}
}

static DEVICE_ATTR(gains, S_IRUGO|S_IWUGO, show_gains, store_gains);


static ssize_t show_gain(
	int chan,
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 gains = acq420rd32(acq420_devices[dev->id], ADC_GAIN);

	return sprintf(buf, "%u\n", get_gain(gains, chan));
}

static ssize_t store_gain(
		int chan,
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count)
{
	char gx;

	if (sscanf(buf, "%c", &gx) == 1){
		u32 gains = acq420rd32(acq420_devices[dev->id], ADC_GAIN);
		if (gx >= '0' && gx <= '3'){
			gains = set_gain(gains, chan, gx-'0');
		}else{
			switch(gx) {
			case '-':
			case 'x':
			case 'X':
				break;
			default:
				dev_warn(dev, "bad value at %c at %d", gx, chan);
			}
		}

		dev_dbg(dev, "set gain: %02x", gains);
		acq420wr32(acq420_devices[dev->id], ADC_GAIN, gains);
		return count;
	}else{
		dev_warn(dev, "rejecting input args != 4");
		return -1;
	}
}

#define MAKE_GAIN(CH)							\
static ssize_t show_gain##CH(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_gain(CH - 1, dev, attr, buf);			\
}									\
									\
static ssize_t store_gain##CH(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_gain(CH - 1, dev, attr, buf, count);		\
}									\
static DEVICE_ATTR(gain##CH, S_IRUGO|S_IWUGO, show_gain##CH, store_gain##CH)

MAKE_GAIN(1);
MAKE_GAIN(2);
MAKE_GAIN(3);
MAKE_GAIN(4);

static ssize_t show_signal(
	int shl,
	int mbit, const char* mbit_hi, const char* mbit_lo,
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 adc_ctrl = acq420rd32(acq420_devices[dev->id], ADC_CTRL);
	if (adc_ctrl&mbit){
		u32 sel = (adc_ctrl >> shl) & ADC_CTRL_SIG_MASK;

		int rising = ((adc_ctrl >> shl) & ADC_CTRL_SIG_RISING) != 0;

		return sprintf(buf, "%s d%u %s\n",
				mbit_hi, sel, rising? "RISING": "FALLING");
	}else{
		return sprintf(buf, "%s\n", mbit_lo);
	}
}

static ssize_t store_signal(
		int shl,
		int mbit, const char* mbit_hi, const char* mbit_lo,
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count)
{
	unsigned dx;
	char sense;
	char mode[16];
	u32 adc_ctrl = acq420rd32(acq420_devices[dev->id], ADC_CTRL);
	int nscan = sscanf(buf, "%10s d%u %c", mode, &dx, &sense);

	switch(nscan){
	case 1:
		if (strcmp(mode, mbit_lo) == 0){
			adc_ctrl &= ~mbit;
			break;
		}
		dev_warn(dev, "single arg must be:\"%s\"", mbit_lo);
		return -1;
	case 3:
		if (strcmp(mode, mbit_hi) == 0){
			int rising 	= strchr("Rr+Pp", sense) != NULL;
			int falling 	= strchr("Ff-Nn", sense) != NULL;

			if (dx > 7){
				dev_warn(dev, "rejecting \"%s\" dx > 7", buf);
				return -1;
			}
			if (!rising && !falling){
				dev_warn(dev,
					"rejecting \"%s\" sense must be R or F", buf);
				return -1;
			}
			adc_ctrl &= ~(ADC_CTRL_SIG_MASK << shl);
			adc_ctrl |=  (dx|(rising? ADC_CTRL_SIG_RISING:0))<<shl;
			adc_ctrl |= mbit;
			break;
		}
		/* fall thru */
	default:
		dev_warn(dev, "%s|%s dX R|F", mbit_lo, mbit_hi);
		return -1;
	}
	/* here with success */
	acq420wr32(acq420_devices[dev->id], ADC_CTRL, adc_ctrl);
	return count;
}

#define MAKE_SIGNAL(SIGNAME, shl, mbit, HI, LO)							\
static ssize_t show_##SIGNAME(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_signal(shl, mbit, HI, LO, dev, attr, buf);		\
}									\
									\
static ssize_t store_##SIGNAME(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_signal(shl, mbit, HI, LO, dev, attr, buf, count);	\
}									\
static DEVICE_ATTR(SIGNAME, S_IRUGO|S_IWUGO, 				\
		show_##SIGNAME, store_##SIGNAME)

#define ENA	"enable"
#define DIS	"disable"
#define EXT	"external"
#define SOFT	"soft"
#define INT	"internal"
MAKE_SIGNAL(event1, ADC_CTRL_EVENT1_SHL, ADC_CTRL_MODE_EV1_EN, ENA, DIS);
MAKE_SIGNAL(event2, ADC_CTRL_EVENT2_SHL, ADC_CTRL_MODE_EV2_EN, ENA, DIS);
MAKE_SIGNAL(trg,    ADC_CTRL_TRIG_SHL,	 ADC_CTRL_MODE_HW_TRG, EXT, SOFT);
MAKE_SIGNAL(clk,    ADC_CTRL_CLK_SHL,	 ADC_CTRL_MODE_HW_CLK, EXT, INT);





static ssize_t show_simulate(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 simulate = acq420_devices[dev->id]->ramp_en != 0;
	return sprintf(buf, "%u\n", simulate);
}

static ssize_t store_simulate(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	u32 simulate;
	if (sscanf(buf, "%u", &simulate) == 1){
		acq420_devices[dev->id]->ramp_en = simulate != 0;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(simulate, S_IRUGO|S_IWUGO, show_simulate, store_simulate);

static ssize_t show_data32(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%u\n", acq420_devices[dev->id]->data32);
}

static ssize_t store_data32(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	u32 data32;
	if (sscanf(buf, "%u", &data32) == 1){
		acq420_devices[dev->id]->data32 = data32 != 0;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(data32, S_IRUGO|S_IWUGO, show_data32, store_data32);


static ssize_t show_stats(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct STATS *stats = &acq420_devices[dev->id]->stats;
	return sprintf(buf, "fifo_ints=%u dma_transactions=%u\n",
			stats->fifo_interrupts, stats->dma_transactions);
}

static ssize_t store_stats(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct STATS *stats = &acq420_devices[dev->id]->stats;
	u32 clr;

	if (sscanf(buf, "%u", &clr) == 1){
		if (clr){
			memset(stats, 0, sizeof(struct STATS));
		}
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(stats, S_IRUGO|S_IWUGO, show_stats, store_stats);









static ssize_t show_clk_count(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 counter = acq420rd32(acq420_devices[dev->id], ADC_CLK_CTR);
	return sprintf(buf, "%u\n", counter&ADC_SAMPLE_CTR_MASK);
}

static DEVICE_ATTR(clk_count, S_IRUGO|S_IWUGO, show_clk_count, 0);

static ssize_t show_sample_count(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 count = acq420rd32(acq420_devices[dev->id], ADC_SAMPLE_CTR);
	return sprintf(buf, "%u\n", count&ADC_SAMPLE_CTR_MASK);
}

static DEVICE_ATTR(sample_count, S_IRUGO|S_IWUGO, show_sample_count, 0);

static ssize_t show_clk_counter_src(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 counter = acq420rd32(acq420_devices[dev->id], ADC_CLK_CTR);
	return sprintf(buf, "%u\n", counter>>ADC_CLK_CTR_SRC_SHL);
}

static ssize_t store_clk_counter_src(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	u32 clk_counter_src;
	if (sscanf(buf, "%u", &clk_counter_src) == 1){
		clk_counter_src &= ADC_CLK_CTR_SRC_MASK;
		acq420wr32(acq420_devices[dev->id], ADC_CLK_CTR,
				clk_counter_src << ADC_CLK_CTR_SRC_SHL);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(clk_counter_src,
		S_IRUGO|S_IWUGO, show_clk_counter_src, store_clk_counter_src);



static const struct attribute *sysfs_attrs[] = {
	&dev_attr_clkdiv.attr,
	&dev_attr_gains.attr,
	&dev_attr_simulate.attr,
	&dev_attr_stats.attr,
	&dev_attr_gain1.attr,
	&dev_attr_gain2.attr,
	&dev_attr_gain3.attr,
	&dev_attr_gain4.attr,
	&dev_attr_event1.attr,
	&dev_attr_event2.attr,
	&dev_attr_trg.attr,
	&dev_attr_clk.attr,
	&dev_attr_data32.attr,
	&dev_attr_clk_count.attr,
	&dev_attr_clk_counter_src.attr,
	&dev_attr_sample_count.attr,
	NULL,
};
void acq420_createSysfs(struct device *dev)
{
	if (sysfs_create_files(&dev->kobj, sysfs_attrs)){
		dev_err(dev, "failed to create sysfs");
	}
}

void acq420_delSysfs(struct device *dev)
{
	sysfs_remove_files(&dev->kobj, sysfs_attrs);
}


