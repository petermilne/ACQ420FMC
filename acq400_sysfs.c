/* ------------------------------------------------------------------------- */
/* acq400_sysfs.c D-TACQ ACQ400 series driver, sysfs (knobs)	             */
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

#include "acq400.h"
#include "bolo.h"

#define MODIFY_REG_ACCESS_READ_BEFORE	0x1
#define MODIFY_REG_ACCESS_READ_AFTER	0x2
int modify_reg_access;
module_param(modify_reg_access, int, 0644);
MODULE_PARM_DESC(modify_spad_access,
"1: force read before, 2: force read after");

#define DEVICE_CREATE_FILE(dev, attr) 							\
	do {										\
		if (device_create_file(dev, attr)){ 					\
			dev_warn(dev, "%s:%d device_create_file", __FILE__, __LINE__); 	\
		} 									\
	} while (0)


/* generic show_bits, store_bits */

static ssize_t show_bits(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned REG,
	unsigned SHL,
	unsigned MASK)
{
	u32 regval = acq400rd32(acq400_devices[dev->id], REG);
	u32 field = (regval>>SHL)&MASK;

	return sprintf(buf, "%x\n", field);
}

static ssize_t store_bits(
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count,
		unsigned REG,
		unsigned SHL,
		unsigned MASK)
{
	u32 field;
	if (sscanf(buf, "%x", &field) == 1){
		u32 regval = acq400rd32(acq400_devices[dev->id], REG);
		regval &= ~(MASK << SHL);
		regval |= (field&MASK) << SHL;
		acq400wr32(acq400_devices[dev->id], REG, regval);
		return count;
	}else{
		return -1;
	}
}

#define MAKE_BITS_FROM_MASK	0xdeadbeef

int getSHL(unsigned mask)
/* converts mask to shift */
{
	int shl;
	for (shl = 0; (mask&1) == 0; ++shl, mask >>= 1){
		;
	}
	return shl;
}
#define MAKE_BITS_RO(NAME, REG, SHL, MASK)				\
static ssize_t show_bits##NAME(						\
	struct device *dev,						\
	struct device_attribute *attr,					\
	char *buf)							\
{									\
	unsigned shl = getSHL(MASK);					\
	if (shl){							\
		return show_bits(dev, attr, buf, REG, shl, (MASK)>>shl);\
	}else{								\
		return show_bits(dev, attr, buf, REG, SHL, MASK);	\
	}								\
}									\
static DEVICE_ATTR(NAME, S_IRUGO, show_bits##NAME, 0)

/* active low. Valid for single bit only */
#define MAKE_BIT_RON(NAME, REG, SHL, MASK)				\
static ssize_t show_bits##NAME(						\
	struct device *dev,						\
	struct device_attribute *attr,					\
	char *buf)							\
{									\
	unsigned shl = getSHL(MASK);					\
	if (shl){							\
		return !show_bits(dev, attr, buf, REG, shl, (MASK)>>shl);\
	}else{								\
		return !show_bits(dev, attr, buf, REG, SHL, MASK);	\
	}								\
}									\
static DEVICE_ATTR(NAME, S_IRUGO, show_bits##NAME, 0)

#define MAKE_BITS(NAME, REG, SHL, MASK)					\
static ssize_t show_bits##NAME(						\
	struct device *dev,						\
	struct device_attribute *attr,					\
	char *buf)							\
{									\
	unsigned shl = getSHL(MASK);					\
	if (shl){							\
		return show_bits(dev, attr, buf, REG, shl, (MASK)>>shl);\
	}else{								\
		return show_bits(dev, attr, buf, REG, SHL, MASK);	\
	}								\
}									\
static ssize_t store_bits##NAME(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	unsigned shl = getSHL(MASK);					\
	if (shl){							\
		return store_bits(dev, attr, buf, count, REG, shl, (MASK)>>shl);\
	}else{								\
		return store_bits(dev, attr, buf, count, REG, SHL, MASK);\
	}								\
}									\
static DEVICE_ATTR(NAME, S_IRUGO|S_IWUGO, show_bits##NAME, store_bits##NAME)

MAKE_BITS(gate_sync,    ADC_CTRL,      MAKE_BITS_FROM_MASK,	ADC_CTRL_435_GATE_SYNC);
MAKE_BITS(sync_in_clk,  HDMI_SYNC_DAT, HDMI_SYNC_IN_CLKb, 	0x1);
MAKE_BITS(sync_in_sync, HDMI_SYNC_DAT, HDMI_SYNC_IN_SYNCb, 	0x1);
MAKE_BITS(sync_in_trg,  HDMI_SYNC_DAT, HDMI_SYNC_IN_TRGb, 	0x1);
MAKE_BITS(sync_in_gpio, HDMI_SYNC_DAT, HDMI_SYNC_IN_GPIOb, 	0x1);


MAKE_BITS(di4_3, 	HDMI_SYNC_DAT, HDMI_SYNC_IN_CLKb, 	0x1);
MAKE_BITS(di4_2, 	HDMI_SYNC_DAT, HDMI_SYNC_IN_SYNCb, 	0x1);
MAKE_BITS(di4_1, 	HDMI_SYNC_DAT, HDMI_SYNC_IN_TRGb, 	0x1);
MAKE_BITS(di4_0, 	HDMI_SYNC_DAT, HDMI_SYNC_IN_GPIOb, 	0x1);

MAKE_BIT_RON(sync_out_cable_det, HDMI_SYNC_DAT, HDMI_SYNC_OUT_CABLE_DETNb, 0x1);

MAKE_BITS(sync_out_clk,  HDMI_SYNC_DAT, HDMI_SYNC_OUT_CLKb, 	0x1);
MAKE_BITS(sync_out_sync, HDMI_SYNC_DAT, HDMI_SYNC_OUT_SYNCb, 	0x1);
MAKE_BITS(sync_out_trg,  HDMI_SYNC_DAT, HDMI_SYNC_OUT_TRGb, 	0x1);
MAKE_BITS(sync_out_gpio, HDMI_SYNC_DAT, HDMI_SYNC_OUT_GPIOb, 	0x1);


MAKE_BITS(do4_3, 	HDMI_SYNC_DAT, HDMI_SYNC_OUT_CLKb, 	0x1);
MAKE_BITS(do4_2, 	HDMI_SYNC_DAT, HDMI_SYNC_OUT_SYNCb, 	0x1);
MAKE_BITS(do4_1, 	HDMI_SYNC_DAT, HDMI_SYNC_OUT_TRGb, 	0x1);
MAKE_BITS(do4_0, 	HDMI_SYNC_DAT, HDMI_SYNC_OUT_GPIOb, 	0x1);


MAKE_BITS(sync_out_src_sync, HDMI_SYNC_OUT_SRC, HSO_SYNC_SHL+HSO_SS_SEL_SHL, HSO_XX_SEL_MASK);
MAKE_BITS(sync_out_src_trg,  HDMI_SYNC_OUT_SRC, HSO_TRG_SHL+HSO_SS_SEL_SHL,  HSO_XX_SEL_MASK);
MAKE_BITS(sync_out_src_gpio, HDMI_SYNC_OUT_SRC, HSO_GPIO_SHL+HSO_SS_SEL_SHL, HSO_XX_SEL_MASK);
MAKE_BITS(sync_out_src_clk,  HDMI_SYNC_OUT_SRC, HSO_CLK_SHL+HSO_SS_SEL_SHL,  HSO_XX_SEL_MASK);

MAKE_BITS(sync_out_src_sync_dx, HDMI_SYNC_OUT_SRC, HSO_SYNC_SHL, HSO_DX_MASK);
MAKE_BITS(sync_out_src_trg_dx,  HDMI_SYNC_OUT_SRC, HSO_TRG_SHL,  HSO_DX_MASK);
MAKE_BITS(sync_out_src_gpio_dx, HDMI_SYNC_OUT_SRC, HSO_GPIO_SHL, HSO_DX_MASK);
MAKE_BITS(sync_out_src_clk_dx,  HDMI_SYNC_OUT_SRC, HSO_CLK_SHL,  HSO_DX_MASK);

MAKE_BITS(evt_src_d7, EVT_BUS_SRC, EBS_7_SHL, EBS_MASK);
MAKE_BITS(evt_src_d6, EVT_BUS_SRC, EBS_6_SHL, EBS_MASK);
MAKE_BITS(evt_src_d5, EVT_BUS_SRC, EBS_5_SHL, EBS_MASK);
MAKE_BITS(evt_src_d4, EVT_BUS_SRC, EBS_4_SHL, EBS_MASK);
MAKE_BITS(evt_src_d3, EVT_BUS_SRC, EBS_3_SHL, EBS_MASK);
MAKE_BITS(evt_src_d2, EVT_BUS_SRC, EBS_2_SHL, EBS_MASK);
MAKE_BITS(evt_src_d1, EVT_BUS_SRC, EBS_1_SHL, EBS_MASK);
MAKE_BITS(evt_src_d0, EVT_BUS_SRC, EBS_0_SHL, EBS_MASK);

MAKE_BITS(sig_src_route_sync_d1, SIG_SRC_ROUTE, SSR_SYNC_1_SHL, SSR_MASK);
MAKE_BITS(sig_src_route_sync_d0, SIG_SRC_ROUTE, SSR_SYNC_0_SHL, SSR_MASK);
MAKE_BITS(sig_src_route_trg_d1,  SIG_SRC_ROUTE, SSR_TRG_1_SHL,  SSR_MASK);
MAKE_BITS(sig_src_route_trg_d0,  SIG_SRC_ROUTE, SSR_TRG_0_SHL,  SSR_MASK);
MAKE_BITS(sig_src_route_clk_d1,  SIG_SRC_ROUTE, SSR_CLK_1_SHL,  SSR_MASK);
MAKE_BITS(sig_src_route_clk_d0,  SIG_SRC_ROUTE, SSR_CLK_0_SHL,  SSR_MASK);

MAKE_BITS(current_adc_enable, B8_CAD_CON, MAKE_BITS_FROM_MASK, B8_CAD_CON_ENABLE);
MAKE_BITS(offset_dac_enable, B8_ODA_CON, MAKE_BITS_FROM_MASK, B8_ODA_CON_ENABLE);


static const struct attribute *hdmi_sync_attrs[] = {
	&dev_attr_sync_in_clk.attr,
	&dev_attr_sync_in_sync.attr,
	&dev_attr_sync_in_trg.attr,
	&dev_attr_sync_in_gpio.attr,

	&dev_attr_di4_3.attr,
	&dev_attr_di4_2.attr,
	&dev_attr_di4_1.attr,
	&dev_attr_di4_0.attr,

	&dev_attr_sync_out_cable_det.attr,

	&dev_attr_sync_out_clk.attr,
	&dev_attr_sync_out_sync.attr,
	&dev_attr_sync_out_trg.attr,
	&dev_attr_sync_out_gpio.attr,

	&dev_attr_do4_3.attr,
	&dev_attr_do4_2.attr,
	&dev_attr_do4_1.attr,
	&dev_attr_do4_0.attr,

	&dev_attr_sync_out_src_sync.attr,
	&dev_attr_sync_out_src_trg.attr,
	&dev_attr_sync_out_src_gpio.attr,
	&dev_attr_sync_out_src_clk.attr,
	&dev_attr_sync_out_src_sync_dx.attr,
	&dev_attr_sync_out_src_trg_dx.attr,
	&dev_attr_sync_out_src_gpio_dx.attr,
	&dev_attr_sync_out_src_clk_dx.attr,

	&dev_attr_evt_src_d7.attr,
	&dev_attr_evt_src_d6.attr,
	&dev_attr_evt_src_d5.attr,
	&dev_attr_evt_src_d4.attr,
	&dev_attr_evt_src_d3.attr,
	&dev_attr_evt_src_d2.attr,
	&dev_attr_evt_src_d1.attr,
	&dev_attr_evt_src_d0.attr,

	&dev_attr_sig_src_route_sync_d1.attr,
	&dev_attr_sig_src_route_sync_d0.attr,
	&dev_attr_sig_src_route_trg_d1.attr,
	&dev_attr_sig_src_route_trg_d0.attr,
	&dev_attr_sig_src_route_clk_d1.attr,
	&dev_attr_sig_src_route_clk_d0.attr,

};
static ssize_t show_clkdiv(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 clkdiv = acq400rd32(acq400_devices[dev->id], ADC_CLKDIV);
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
	    clkdiv >= 1 &&
	    clkdiv <= ADC_CLK_DIV_MASK){
		acq400wr32(acq400_devices[dev->id], ADC_CLKDIV, clkdiv);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(clkdiv, S_IRUGO|S_IWUGO, show_clkdiv, store_clkdiv);

static ssize_t show_adc_conv_time(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 adc_conv_time = acq400rd32(acq400_devices[dev->id], ADC_CONV_TIME);
	return sprintf(buf, "%u\n", adc_conv_time);
}

static ssize_t store_adc_conv_time(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	u32 adc_conv_time;
	if (sscanf(buf, "%u", &adc_conv_time) == 1 &&
	    adc_conv_time >= 1 &&
	    adc_conv_time <= ADC_CLK_DIV_MASK){
		acq400wr32(acq400_devices[dev->id], ADC_CONV_TIME, adc_conv_time);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(adc_conv_time, S_IRUGO|S_IWUGO, show_adc_conv_time, store_adc_conv_time);



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
	u32 gains = acq400rd32(acq400_devices[dev->id], ADC_GAIN);
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
		u32 gains = acq400rd32(acq400_devices[dev->id], ADC_GAIN);
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
		acq400wr32(acq400_devices[dev->id], ADC_GAIN, gains);
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
	u32 gains = acq400rd32(acq400_devices[dev->id], ADC_GAIN);

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
		u32 gains = acq400rd32(acq400_devices[dev->id], ADC_GAIN);
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
		acq400wr32(acq400_devices[dev->id], ADC_GAIN, gains);
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
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned REG,
	int shl, int mbit,
	const char*signame, const char* mbit_hi, const char* mbit_lo)
{
	u32 adc_ctrl = acq400rd32(acq400_devices[dev->id], REG);
	if (adc_ctrl&mbit){
		u32 sel = (adc_ctrl >> shl) & CTRL_SIG_MASK;
		unsigned dx = sel&CTRL_SIG_SEL;
		int rising = ((adc_ctrl >> shl) & CTRL_SIG_RISING) != 0;

		return sprintf(buf, "%s=%d,%d,%d %s d%u %s\n",
				signame, (adc_ctrl&mbit)>>getSHL(mbit), dx, rising,
				mbit_hi, dx, rising? "RISING": "FALLING");
	}else{
		return sprintf(buf, "%s=0,0,0 %s\n", signame, mbit_lo);
	}
}

int store_signal3(struct device* dev,
		unsigned REG,
		int shl, int mbit,
		unsigned imode, unsigned dx, unsigned rising)
{
	struct acq400_dev* adev = acq400_devices[dev->id];

	if (adev->busy){
		return -EBUSY;
	}else{
		u32 adc_ctrl = acq400rd32(adev, REG);

		adc_ctrl &= ~mbit;
		if (imode != 0){
			if (dx > 7){
				dev_warn(dev, "rejecting \"%u\" dx > 7", dx);
				return -1;
			}
			adc_ctrl &= ~(CTRL_SIG_MASK << shl);
			adc_ctrl |=  (dx|(rising? CTRL_SIG_RISING:0))<<shl;
			adc_ctrl |= imode << getSHL(mbit);
		}
		acq400wr32(adev, REG, adc_ctrl);
		return 0;
	}
}
static ssize_t store_signal(
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count,
		unsigned REG,
		int shl, int mbit, const char* mbit_hi, const char* mbit_lo)
{
	char sense;
	char mode[16];
	unsigned imode, dx, rising;

	/* first form: imode,dx,rising : easiest with auto eg StreamDevice */
	int nscan = sscanf(buf, "%u,%u,%u", &imode, &dx, &rising);
	if (nscan == 3){
		if (store_signal3(dev, REG, shl, mbit, imode, dx, rising)){
			return -1;
		}else{
			return count;
		}
	}

	/* second form: mode dDX sense : better for human scripting */
	nscan = sscanf(buf, "%10s d%u %c", mode, &dx, &sense);

	switch(nscan){
	case 1:
		if (strcmp(mode, mbit_lo) == 0){
			return store_signal3(dev, REG, shl, mbit, 0, 0, 0);
		}
		dev_warn(dev, "single arg must be:\"%s\"", mbit_lo);
		return -1;
	case 3:
		if (strcmp(mode, mbit_hi) == 0){
			unsigned falling = strchr("Ff-Nn", sense) != NULL;
			rising 	= strchr("Rr+Pp", sense) != NULL;

			if (!rising && !falling){
				dev_warn(dev,
				 "rejecting \"%s\" sense must be R or F", buf);
				return -1;
			}else if (store_signal3(dev, REG, shl, mbit, 1, dx, rising)){
				return -1;
			}else{
				return count;
			}
		}
		/* else fall thru */
	default:
		dev_warn(dev, "%s|%s dX R|F", mbit_lo, mbit_hi);
		return -1;
	}
}

#define MAKE_SIGNAL(SIGNAME, REG, shl, mbit, HI, LO)			\
static ssize_t show_##SIGNAME(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_signal(dev, attr, buf, REG, shl, mbit, #SIGNAME, HI, LO);\
}									\
									\
static ssize_t store_##SIGNAME(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_signal(dev, attr, buf, count, REG, shl, mbit, HI, LO);\
}									\
static DEVICE_ATTR(SIGNAME, S_IRUGO|S_IWUGO, 				\
		show_##SIGNAME, store_##SIGNAME)

#define ENA	"enable"
#define DIS	"disable"
#define EXT	"external"
#define SOFT	"soft"
#define INT	"internal"
MAKE_SIGNAL(event1, TIM_CTRL, TIM_CTRL_EVENT1_SHL, TIM_CTRL_MODE_EV1_EN, ENA, DIS);
MAKE_SIGNAL(event0, TIM_CTRL, TIM_CTRL_EVENT0_SHL, TIM_CTRL_MODE_EV0_EN, ENA, DIS);
MAKE_SIGNAL(trg,    TIM_CTRL, TIM_CTRL_TRIG_SHL,   TIM_CTRL_MODE_HW_TRG, EXT, SOFT);
MAKE_SIGNAL(clk,    TIM_CTRL, TIM_CTRL_CLK_SHL,	   TIM_CTRL_MODE_HW_CLK, EXT, INT);
MAKE_SIGNAL(sync,   TIM_CTRL, TIM_CTRL_SYNC_SHL,   TIM_CTRL_MODE_SYNC,   EXT, INT);


MAKE_SIGNAL(gpg_trg, GPG_CONTROL, GPG_CTRL_TRG_SHL, GPG_CTRL_EXT_TRG, EXT, SOFT);
MAKE_SIGNAL(gpg_clk, GPG_CONTROL, GPG_CTRL_CLK_SHL, GPG_CTRL_EXTCLK,  EXT, INT);
MAKE_SIGNAL(gpg_sync,GPG_CONTROL, GPG_CTRL_SYNC_SHL, GPG_CTRL_EXT_SYNC, EXT, SOFT);

/* MUST set RGM as well */
MAKE_SIGNAL(rgm, ADC_CTRL, ADC_CTRL_RGM_GATE_SHL,
		ADC_CTRL_RGM_MODE_MASK<<ADC_CTRL_RGM_MODE_SHL, ENA, DIS);


static ssize_t show_gpg_top(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 gpg_ctrl = acq400rd32(adev, GPG_CONTROL);
	u32 gpg_top = (gpg_ctrl&GPG_CTRL_TOPADDR) >> GPG_CTRL_TOPADDR_SHL;
	return sprintf(buf, "%u\n", gpg_top);
}

static ssize_t store_gpg_top(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 gpg_top;

	if (sscanf(buf, "%u", &gpg_top) == 1){
		set_gpg_top(adev, gpg_top);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(gpg_top, S_IRUGO|S_IWUGO, show_gpg_top, store_gpg_top);


static ssize_t show_gpg_mode(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 gpg_ctrl = acq400rd32(adev, GPG_CONTROL);
	u32 gpg_mode = (gpg_ctrl&GPG_CTRL_MODE) >> GPG_CTRL_MODE_SHL;
	return sprintf(buf, "%u\n", gpg_mode);
}

static ssize_t store_gpg_mode(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 gpg_mode;

	if (sscanf(buf, "%u", &gpg_mode) == 1){
		u32 gpg_ctrl = acq400rd32(adev, GPG_CONTROL);
		gpg_mode <<= GPG_CTRL_MODE_SHL;
		gpg_mode &= GPG_CTRL_MODE;
		gpg_ctrl &= ~GPG_CTRL_MODE;
		gpg_ctrl |= gpg_mode;

		acq400wr32(adev, GPG_CONTROL, gpg_ctrl);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(gpg_mode, S_IRUGO|S_IWUGO, show_gpg_mode, store_gpg_mode);

static ssize_t show_gpg_enable(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 gpg_ctrl = acq400rd32(adev, GPG_CONTROL);
	return sprintf(buf, "%u\n", (gpg_ctrl&GPG_CTRL_ENABLE) != 0);
}

static ssize_t store_gpg_enable(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 gpg_enable;

	if (sscanf(buf, "%u", &gpg_enable) == 1){
		u32 gpg_ctrl = acq400rd32(adev, GPG_CONTROL);
		if (gpg_enable){
			gpg_ctrl |= GPG_CTRL_ENABLE;
		}else{
			gpg_ctrl &= ~GPG_CTRL_ENABLE;
		}
		acq400wr32(adev, GPG_CONTROL, gpg_ctrl);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(gpg_enable, S_IRUGO|S_IWUGO, show_gpg_enable, store_gpg_enable);

static ssize_t show_simulate(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 simulate = acq400_devices[dev->id]->ramp_en != 0;
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
		acq400_devices[dev->id]->ramp_en = simulate != 0;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(simulate, S_IRUGO|S_IWUGO, show_simulate, store_simulate);

static ssize_t show_spad(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	return sprintf(buf, "%u,%u,%u\n",
		adev->spad.spad_en,
		adev->spad.spad_en==SP_EN? adev->spad.len: 0,
		adev->spad.spad_en!=SP_OFF? adev->spad.diX: 0);
}

static ssize_t store_spad(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	struct Spad spad = { 1, 8, 0 };
	if (sscanf(buf, "%u,%u,%u", &spad.spad_en, &spad.len, &spad.diX) > 0){
		if (spad.diX > SD_DI32) spad.diX = SD_SEW;
		if (spad.len > 8) spad.len = 8;
		switch(spad.spad_en){
		default:
			spad.spad_en = SP_OFF;	/* fall thru */
		case SP_OFF:
			spad.len = 0; spad.diX = SD_SEW; break;
			break;
		case SP_EN:
			if (spad.len < 1) spad.len = 8;
			break;
		case SP_FRAME:
			spad.len = 0;
			break;
		}
		adev->spad = spad;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(spad, S_IRUGO|S_IWUGO, show_spad, store_spad);

static ssize_t show_spadN(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	const int ix)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 spad = get_spadN(adev, ix);
	return sprintf(buf, "0x%08x\n", spad);
}

static ssize_t store_spadN(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count,
	const int ix)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	int rc = count;
	u32 spad;
	enum { ASSIGN, SETBITS, CLRBITS } mode;


	if (sscanf(buf, "+%x", &spad) == 1){
		mode = SETBITS;
	}else if (sscanf(buf, "-%x", &spad) == 1){
		mode = CLRBITS;
	}else if (sscanf(buf, "%x", &spad) == 1){
		mode = ASSIGN;
	}else{
		return -1;
	}

	if (mutex_lock_interruptible(&adev->mutex)) {
		return -EINTR;
	}
	if (mode != ASSIGN){
		u32 rspad = get_spadN(adev, ix);
		switch(mode){
		case SETBITS:
			rspad |= spad; spad = rspad; break;
		case CLRBITS:
			rspad &= ~spad; spad = rspad; break;
		default:
			dev_err(DEVP(adev), "BAD mode %s %d", __FILE__, __LINE__);
			goto store_spadN_99;
		}
	}
	set_spadN(adev, ix, spad);

store_spadN_99:
	mutex_unlock(&adev->mutex);
	return rc;
}


#define MAKE_SPAD(IX)							\
static ssize_t show_spad##IX(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_spadN(dev, attr, buf, IX);				\
}									\
									\
static ssize_t store_spad##IX(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_spadN(dev, attr, buf, count, IX);			\
}									\
static DEVICE_ATTR(spad##IX, S_IRUGO|S_IWUGO, 				\
		show_spad##IX, store_spad##IX)

MAKE_SPAD(0);
MAKE_SPAD(1);
MAKE_SPAD(2);
MAKE_SPAD(3);
MAKE_SPAD(4);
MAKE_SPAD(5);
MAKE_SPAD(6);
MAKE_SPAD(7);

static ssize_t show_reg(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	const int reg_off,
	const char* fmt)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 reg = acq400rd32(adev, reg_off);
	return sprintf(buf, fmt, reg);
}

static ssize_t store_reg(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count,
	const int reg_off)
{
	u32 reg;
	if (sscanf(buf, "0x%x", &reg) == 1 || sscanf(buf, "%u", &reg) == 1){
		struct acq400_dev* adev = acq400_devices[dev->id];
		if ((modify_reg_access&MODIFY_REG_ACCESS_READ_BEFORE) != 0){
			acq400rd32(adev, reg_off);
		}
		acq400wr32(adev, reg_off, reg);
		if ((modify_reg_access&MODIFY_REG_ACCESS_READ_AFTER) != 0){
			u32 regr = acq400rd32(adev, reg_off);
			if (regr != reg){
				dev_warn(DEVP(adev),
					"reg:%04x mismatch w:%08x r:%08x",
					reg_off, reg, regr);
			}
		}
		return count;
	}else{
		return -1;
	}
}


#define MAKE_REG_RO(kname, REG, FMT)					\
static ssize_t show_reg_##REG(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_reg(dev, attr, buf, REG, FMT);			\
}									\
									\
static DEVICE_ATTR(kname, S_IRUGO, show_reg_##REG, 0)

#define MAKE_REG_RW(kname, REG, FMT)					\
static ssize_t show_reg_##REG(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_reg(dev, attr, buf, REG, FMT);			\
}									\
									\
static ssize_t store_reg_##REG(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_reg(dev, attr, buf, count, REG);	\
}									\
static DEVICE_ATTR(kname, S_IRUGO|S_IWUGO, show_reg_##REG, store_reg_##REG)

MAKE_REG_RW(sw_emb_word1, SW_EMB_WORD1, "0x%08x\n");
MAKE_REG_RW(sw_emb_word2, SW_EMB_WORD2, "0x%08x\n");
MAKE_REG_RO(evt_sc_latch, EVT_SC_LATCH,	"%u\n");

MAKE_REG_RW(rtm_translen, ADC_TRANSLEN, "%u\n");


static ssize_t show_nbuffers(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->nbuffers);
}

static DEVICE_ATTR(nbuffers, S_IRUGO, show_nbuffers, 0);


static ssize_t store_bufferlen(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	u32 _bufferlen;
	if (sscanf(buf, "%u", &_bufferlen) == 1){
		struct acq400_dev *adev = acq400_devices[dev->id];

		if (acq400_set_bufferlen(adev, _bufferlen) == _bufferlen){
			return count;
		}
	}

	return -1;
}
static ssize_t show_bufferlen(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->bufferlen);
}

static DEVICE_ATTR(bufferlen, S_IRUGO, show_bufferlen, 0);

static ssize_t show_hitide(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->hitide);
}

static DEVICE_ATTR(hitide, S_IRUGO, show_hitide, 0);

static ssize_t show_lotide(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->lotide);
}

static ssize_t store_lotide(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 lotide;
	if (sscanf(buf, "%u", &lotide) == 1 && lotide <= MAX_LOTIDE(adev)){
		adev->lotide = lotide;
		return count;
	}else{
		return -1;
	}
}
static DEVICE_ATTR(lotide, S_IRUGO|S_IWUGO, show_lotide, store_lotide);


static ssize_t show_adc_18b(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->adc_18b);
}

static ssize_t store_adc_18b(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 adc_18b;
	if (!IS_ACQ43X(adev) && sscanf(buf, "%u", &adc_18b) == 1){
		adev->adc_18b = adc_18b != 0;
		adev->word_size = adc_18b? 4: 2;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(adc_18b, S_IRUGO|S_IWUGO, show_adc_18b, store_adc_18b);

static ssize_t show_data32(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->data32);
}

static ssize_t store_data32(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 data32;
	if (!IS_ACQ43X(adev) && sscanf(buf, "%u", &data32) == 1){
		adev->data32 = data32 != 0;
		adev->word_size = data32? 4: 2;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(data32, S_IRUGO|S_IWUGO, show_data32, store_data32);

static ssize_t show_bitslice_frame(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 ctrl = acq400rd32(adev, ADC_CTRL);
	return sprintf(buf, "%u\n", (ctrl&ADC_CTRL_435_EMBED_STR) != 0);
}

static ssize_t store_bitslice_frame(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 bitslice_frame;
	if (sscanf(buf, "%u", &bitslice_frame) == 1){
		u32 ctrl = acq400rd32(adev, ADC_CTRL);
		if (bitslice_frame){
			ctrl |= ADC_CTRL_435_EMBED_STR;
		}else{
			ctrl &= ~ADC_CTRL_435_EMBED_STR;
		}
		acq400wr32(adev, ADC_CTRL, ctrl);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(bitslice_frame, S_IRUGO|S_IWUGO, show_bitslice_frame, store_bitslice_frame);


static ssize_t show_stats(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct STATS *stats = &acq400_devices[dev->id]->stats;
	return sprintf(buf, "fifo_ints=%u dma_transactions=%u fifo_errs=%d\n",
			stats->fifo_interrupts, stats->dma_transactions,
			stats->fifo_errors);
}

static ssize_t store_stats(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct STATS *stats = &acq400_devices[dev->id]->stats;
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

static ssize_t show_shot(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%d\n", adev->stats.shot);
}

static ssize_t store_shot(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 set;

	if (sscanf(buf, "%u", &set) == 1){
		adev->stats.shot = set;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(shot, S_IRUGO|S_IWUGO, show_shot, store_shot);

static ssize_t show_run(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%d\n", adev->stats.run);
}


static DEVICE_ATTR(run, S_IRUGO, show_run, 0);


static ssize_t show_clk_count(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 counter = acq400rd32_upcount(acq400_devices[dev->id], ADC_CLK_CTR);
	return sprintf(buf, "%u\n", counter&ADC_SAMPLE_CTR_MASK);
}

static DEVICE_ATTR(clk_count, S_IRUGO, show_clk_count, 0);


static ssize_t show_sample_count(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 count = acq400rd32_upcount(acq400_devices[dev->id], ADC_SAMPLE_CTR);
	return sprintf(buf, "%u\n", count&ADC_SAMPLE_CTR_MASK);
}

static DEVICE_ATTR(sample_count, S_IRUGO, show_sample_count, 0);



static ssize_t show_clk_counter_src(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 counter = acq400rd32(acq400_devices[dev->id], ADC_CLK_CTR);
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
		acq400wr32(acq400_devices[dev->id], ADC_CLK_CTR,
				clk_counter_src << ADC_CLK_CTR_SRC_SHL);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(clk_counter_src,
		S_IRUGO|S_IWUGO, show_clk_counter_src, store_clk_counter_src);


static ssize_t show_hi_res_mode(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 mode = acq400rd32(adev, ACQ435_MODE);
	return sprintf(buf, "%u\n", (mode&ACQ435_MODE_HIRES_512)? 1: 0);
}

static ssize_t store_hi_res_mode(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 hi_res_mode;
	if (sscanf(buf, "%u", &hi_res_mode) == 1){
		u32 mode = acq400rd32(adev, ACQ435_MODE);
		if (hi_res_mode == 0){
			mode &= ~ACQ435_MODE_HIRES_512;
		}else{
			mode |= ACQ435_MODE_HIRES_512;
		}
		hi_res_mode &= ADC_CLK_CTR_SRC_MASK;
		acq400wr32(adev, ACQ435_MODE, mode);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(hi_res_mode,
		S_IRUGO|S_IWUGO, show_hi_res_mode, store_hi_res_mode);

/** NB inverted to 1: enabled */

#define BADBANKMASK 0xffffffff

unsigned bank_mask_txt2int(const char* txt)
{
	const char* cursor;
	unsigned mask = 0;

	for (cursor = txt; *cursor && cursor-txt < 4; ++cursor){
		switch(*cursor){
		case 'A':	mask |= 1<<0; break;
		case 'B':	mask |= 1<<1; break;
		case 'C':	mask |= 1<<2; break;
		case 'D':	mask |= 1<<3; break;
		default:	return BADBANKMASK;
		}
	}
	return mask;
}

char* bank_mask_int2txt(unsigned mask, char txt[])
{
	char* pt = txt;
	int bit = 0;
	for (bit = 0; bit < 4; ++bit, mask >>= 1){
		if ((mask&1) != 0){
			switch(bit){
			case 0:		*pt++ = 'A'; break;
			case 1:		*pt++ = 'B'; break;
			case 2:		*pt++ = 'C'; break;
			case 3:		*pt++ = 'D'; break;
			}
		}
	}
	*pt++ ='\0';
	return txt;
}
static ssize_t show_bank_mask(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 mask = ~acq400rd32(adev, ACQ435_MODE) & ACQ435_MODE_BXDIS;
	char txt[8];
	return sprintf(buf, "%s\n",  bank_mask_int2txt(mask, txt));
}

unsigned nbits(unsigned mask)
{
	unsigned nb;
	for(nb = 0; mask != 0; mask >>= 1){
		if (mask&1){
			++nb;
		}
	}

	return nb;
}

static ssize_t store_bank_mask(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
/* user mask: 1=enabled. Compute nchan_enabled BEFORE inverting MASK */
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 bank_mask = bank_mask_txt2int(buf);
	if (bank_mask != BADBANKMASK){
		u32 mode = acq400rd32(adev, ACQ435_MODE);

		mode &= ~ACQ435_MODE_BXDIS;
		adev->nchan_enabled = 8 * nbits(bank_mask);
		bank_mask = ~bank_mask&ACQ435_MODE_BXDIS;
		mode |= bank_mask;

		acq400wr32(adev, ACQ435_MODE, mode);
		if (IS_AO424(adev)){
			measure_ao_fifo(adev);
		}
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(bank_mask,
		S_IRUGO|S_IWUGO, show_bank_mask, store_bank_mask);


static ssize_t show_active_chan(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%d\n", adev->nchan_enabled);
}

static DEVICE_ATTR(active_chan,
		S_IRUGO|S_IWUGO, show_active_chan, 0);

static ssize_t show_module_type(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%X\n", adev->mod_id>>MOD_ID_TYPE_SHL);
}


static DEVICE_ATTR(module_type, S_IRUGO, show_module_type, 0);

static ssize_t show_module_role(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%s\n",
		(adev->mod_id&MOD_ID_IS_SLAVE) ? "SLAVE": "MASTER");
}


static DEVICE_ATTR(module_role, S_IRUGO, show_module_role, 0);


static ssize_t show_module_name(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	const char* name;

	switch(adev->mod_id>>MOD_ID_TYPE_SHL){
	case MOD_ID_ACQ1001SC:
		name = "acq1001sc"; break;
	case MOD_ID_ACQ2006SC:
		name = "acq2006sc"; break;
	case MOD_ID_ACQ2106SC:
		name = "acq2106sc"; break;
	case MOD_ID_ACQ420FMC:
	case MOD_ID_ACQ420FMC_2000:
		name = "acq420fmc"; break;
	case MOD_ID_ACQ435ELF:
		name = "acq435elf"; break;
	case MOD_ID_ACQ430FMC:
		name = "acq430fmc"; break;
	case MOD_ID_ACQ480FMC:
		name = "acq480fmc"; break;
	case MOD_ID_ACQ425ELF:
	case MOD_ID_ACQ425ELF_2000:
		name = "acq425elf"; break;
	case MOD_ID_AO420FMC:
		name = "ao420fmc"; break;
	case MOD_ID_AO424ELF:
		name = "ao424elf"; break;
	default:
		name = "unknown";
		break;
	}
	return sprintf(buf, "%s\n", name);
}


static DEVICE_ATTR(module_name, S_IRUGO, show_module_name, 0);

static ssize_t show_site(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->of_prams.site);
}


static DEVICE_ATTR(site, S_IRUGO, show_site, 0);

static ssize_t show_sysclkhz(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->sysclkhz);
}

static DEVICE_ATTR(sysclkhz, S_IRUGO, show_sysclkhz, 0);

#define acq435_get_nacc_shift_exact 1
#define acq435_get_nacc_shift_ge    0
static u32 acq435_get_nacc_shift(unsigned divider, int exact)
{
#define EMATCH(x, y) (exact? (x) == (y): (x) >= (y))
	if (EMATCH(divider, 32)){
		return ADC_ACC_DEC_SHIFT_5;
	}else if (EMATCH(divider, 16)){
		return ADC_ACC_DEC_SHIFT_4;
	}else if (EMATCH(divider, 8)){
		return ADC_ACC_DEC_SHIFT_3;
	}else if (EMATCH(divider, 4)){
		return ADC_ACC_DEC_SHIFT_2;
	}else if (EMATCH(divider, 2)){
		return ADC_ACC_DEC_SHIFT_1;
	}else if (EMATCH(divider, 1)){
		return ADC_ACC_DEC_SHIFT_0;
	}else{
		return 0xffffffff;
	}
}

static ssize_t show_nacc(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 acc_dec = acq400rd32(adev, ADC_ACC_DEC);
	unsigned shift = (acc_dec&ADC_ACC_DEC_SHIFT_MASK)>>
					getSHL(ADC_ACC_DEC_SHIFT_MASK);
	unsigned divider;

	for (divider = 32; divider; divider >>= 1){
		if (acq435_get_nacc_shift(divider,acq435_get_nacc_shift_exact) == shift){
			break;
		}
	}

	return sprintf(buf, "%u,%u\n",
			(acc_dec&ADC_ACC_DEC_LEN)+1, divider);
}

static ssize_t store_nacc(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 nacc;
	u32 divider = 0;

	if (sscanf(buf, "%u,%u", &nacc, &divider) >= 1){
		u32 acc_dec = nacc;
		unsigned shift;

		if (divider == 0) divider = nacc;	/* most of the time, this is what you want! */
		if (acc_dec > 0) 	acc_dec -= 1;
		if (acc_dec == 0) 	divider = 1;
		acc_dec &= ADC_ACC_DEC_LEN;

		shift = acq435_get_nacc_shift(divider, acq435_get_nacc_shift_ge);
		acc_dec |= shift<<getSHL(ADC_ACC_DEC_SHIFT_MASK);
		acq400wr32(adev, ADC_ACC_DEC, acc_dec);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(nacc, S_IRUGO|S_IWUGO, show_nacc, store_nacc);


static const struct attribute *sysfs_base_attrs[] = {
	&dev_attr_module_type.attr,
	&dev_attr_module_role.attr,
	&dev_attr_module_name.attr,
	&dev_attr_nbuffers.attr,
	&dev_attr_bufferlen.attr,
	&dev_attr_site.attr,
	&dev_attr_data32.attr,
	NULL
};

static const struct attribute *sysfs_device_attrs[] = {
	&dev_attr_clkdiv.attr,
	&dev_attr_simulate.attr,
	&dev_attr_stats.attr,
	&dev_attr_event1.attr,
	&dev_attr_event0.attr,
	&dev_attr_sync.attr,
	&dev_attr_trg.attr,
	&dev_attr_clk.attr,
	&dev_attr_clk_count.attr,
	&dev_attr_clk_counter_src.attr,
	&dev_attr_sample_count.attr,
	&dev_attr_shot.attr,
	&dev_attr_run.attr,
	&dev_attr_hitide.attr,
	&dev_attr_lotide.attr,
	&dev_attr_sysclkhz.attr,
	&dev_attr_active_chan.attr,
	&dev_attr_rgm.attr,
	NULL,
};

static const struct attribute *acq420_attrs[] = {
	&dev_attr_adc_18b.attr,
	&dev_attr_gains.attr,
	&dev_attr_gain1.attr,
	&dev_attr_gain2.attr,
	&dev_attr_gain3.attr,
	&dev_attr_gain4.attr,
	&dev_attr_adc_conv_time.attr,
	NULL
};
static const struct attribute *acq435_attrs[] = {
	&dev_attr_hi_res_mode.attr,
	&dev_attr_bank_mask.attr,
	&dev_attr_bitslice_frame.attr,
	&dev_attr_sw_emb_word1.attr,
	&dev_attr_sw_emb_word2.attr,
	&dev_attr_evt_sc_latch.attr,
	&dev_attr_gate_sync.attr,
	&dev_attr_rtm_translen.attr,
	&dev_attr_nacc.attr,
	NULL
};


static ssize_t show_dac_headroom(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{

	struct acq400_dev *adev = acq400_devices[dev->id];
	int hr_samples = ao420_getFifoHeadroom(adev);
	return sprintf(buf, "%d samples %d bytes\n",
			hr_samples, AOSAMPLES2BYTES(adev, hr_samples));
}

static DEVICE_ATTR(dac_headroom, S_IRUGO, show_dac_headroom, 0);

static ssize_t show_dac_fifo_samples(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	int fifo_samples = adev->xo.getFifoSamples(adev);
	return sprintf(buf, "%d samples %d bytes\n",
			fifo_samples, AOSAMPLES2BYTES(adev, fifo_samples));
}

static DEVICE_ATTR(dac_fifo_samples, S_IRUGO, show_dac_fifo_samples, 0);

static ssize_t show_dac_range(
	int chan,
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 ranges = acq400rd32(acq400_devices[dev->id], AO420_RANGE);

	return sprintf(buf, "%u\n", ranges>>chan & 1);
}

static ssize_t store_dac_range(
		int shl,
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count)
{
	int gx;

	if (sscanf(buf, "%d", &gx) == 1){
		u32 ranges = acq400rd32(acq400_devices[dev->id], AO420_RANGE);
		unsigned bit = 1 << shl;
		if (gx){
			ranges |= bit;
		}else{
			ranges &= ~bit;
		}

		dev_dbg(dev, "set gain: %02x", ranges);
		acq400wr32(acq400_devices[dev->id], AO420_RANGE, ranges);
		return count;
	}else{
		dev_warn(dev, "rejecting input args != 4");
		return -1;
	}
}

#define MAKE_DAC_RANGE(NAME, SHL)					\
static ssize_t show_dac_range##NAME(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_dac_range(SHL, dev, attr, buf);			\
}									\
									\
static ssize_t store_dac_range##NAME(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_dac_range(SHL, dev, attr, buf, count);		\
}									\
static DEVICE_ATTR(dac_range_##NAME, S_IRUGO|S_IWUGO, 			\
		show_dac_range##NAME, store_dac_range##NAME)

MAKE_DAC_RANGE(01,  ao420_physChan(1));
MAKE_DAC_RANGE(02,  ao420_physChan(2));
MAKE_DAC_RANGE(03,  ao420_physChan(3));
MAKE_DAC_RANGE(04,  ao420_physChan(4));
MAKE_DAC_RANGE(REF, 4);

static void ao420_flushImmediate(struct acq400_dev *adev)
{
	unsigned *src = adev->AO_immediate._u.lw;
	void *fifo = adev->dev_virtaddr + AXI_FIFO;
	int imax = adev->nchan_enabled/2;
	int ii = 0;

	dev_dbg(DEVP(adev), "ao420_flushImmediate() 01 imax %d", imax);
	for (ii = 0; ii < imax; ++ii){
		dev_dbg(DEVP(adev), "fifo write: %p = 0x%08x\n",
				fifo + ii*sizeof(unsigned), src[ii]);
		iowrite32(src[ii], fifo + ii*sizeof(unsigned));
	}
	dev_dbg(DEVP(adev), "ao420_flushImmediate() 99 ii %d", ii);
}

static ssize_t show_dac_immediate(
	int chan,
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	short chx;
	int pchan = adev->xo.physchan(chan);

	chx = adev->AO_immediate._u.ch[pchan];
	if (IS_AO424(adev)){
		chx = ao424_fixEncoding(adev, pchan, chx);
	}

	return sprintf(buf, "0x%04x %d\n", chx, chx);
}

static ssize_t store_dac_immediate(
		int chan,
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	int chx;
	int pchan = adev->xo.physchan(chan);

	if (sscanf(buf, "0x%x", &chx) == 1 || sscanf(buf, "%d", &chx) == 1){
		unsigned cr = acq400rd32(adev, DAC_CTRL);
		if (IS_AO424(adev)){
			chx = ao424_fixEncoding(adev, pchan, chx);
		}
		adev->AO_immediate._u.ch[pchan] = chx;
		acq400wr32(adev, DAC_CTRL, cr|DAC_CTRL_LL|ADC_CTRL_ENABLE_ALL);
		ao420_flushImmediate(adev);
		return count;
	}else{
		dev_warn(dev, "rejecting input args != 0x%%04x or %%d");
		return -1;
	}
}

#define MAKE_DAC_IMMEDIATE(NAME, CH)					\
static ssize_t show_dac_immediate_##NAME(				\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_dac_immediate(CH, dev, attr, buf);			\
}									\
									\
static ssize_t store_dac_immediate_##NAME(				\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_dac_immediate(CH, dev, attr, buf, count);		\
}									\
static DEVICE_ATTR(AO_##NAME, S_IRUGO|S_IWUGO, 			\
		show_dac_immediate_##NAME, store_dac_immediate_##NAME)

MAKE_DAC_IMMEDIATE(01, 1);
MAKE_DAC_IMMEDIATE(02, 2);
MAKE_DAC_IMMEDIATE(03, 3);
MAKE_DAC_IMMEDIATE(04, 4);
MAKE_DAC_IMMEDIATE(05, 5);
MAKE_DAC_IMMEDIATE(06, 6);
MAKE_DAC_IMMEDIATE(07, 7);
MAKE_DAC_IMMEDIATE(08, 8);
MAKE_DAC_IMMEDIATE(09, 9);
MAKE_DAC_IMMEDIATE(10, 10);
MAKE_DAC_IMMEDIATE(11, 11);
MAKE_DAC_IMMEDIATE(12, 12);
MAKE_DAC_IMMEDIATE(13, 13);
MAKE_DAC_IMMEDIATE(14, 14);
MAKE_DAC_IMMEDIATE(15, 15);
MAKE_DAC_IMMEDIATE(16, 16);
MAKE_DAC_IMMEDIATE(17, 17);
MAKE_DAC_IMMEDIATE(18, 18);
MAKE_DAC_IMMEDIATE(19, 19);
MAKE_DAC_IMMEDIATE(20, 20);
MAKE_DAC_IMMEDIATE(21, 21);
MAKE_DAC_IMMEDIATE(22, 22);
MAKE_DAC_IMMEDIATE(23, 23);
MAKE_DAC_IMMEDIATE(24, 24);
MAKE_DAC_IMMEDIATE(25, 25);
MAKE_DAC_IMMEDIATE(26, 26);
MAKE_DAC_IMMEDIATE(27, 27);
MAKE_DAC_IMMEDIATE(28, 28);
MAKE_DAC_IMMEDIATE(29, 29);
MAKE_DAC_IMMEDIATE(30, 30);
MAKE_DAC_IMMEDIATE(31, 31);
MAKE_DAC_IMMEDIATE(32, 32);


static ssize_t show_playloop_length(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u %s\n",
			adev->AO_playloop.length,
			adev->AO_playloop.one_shot? "ONESHOT": "");
}

static ssize_t store_playloop_length(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned playloop_length;
	unsigned one_shot;

	switch(sscanf(buf, "%u %u", &playloop_length, &one_shot)){
	case 2:
		adev->AO_playloop.one_shot = one_shot != 0; /* fall thru */
	case 1:
		xo400_reset_playloop(adev, playloop_length);
		return count;
	default:
		return -1;
	}
}

static DEVICE_ATTR(playloop_length,
		S_IRUGO|S_IWUGO, show_playloop_length, store_playloop_length);

static ssize_t show_playloop_cursor(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%u\n", adev->AO_playloop.cursor);
}

static ssize_t store_playloop_cursor(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	if (sscanf(buf, "%u", &adev->AO_playloop.cursor) == 1){
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(playloop_cursor,
		S_IRUGO|S_IWUGO, show_playloop_cursor, store_playloop_cursor);


#define TIMEOUT 1000

#define DACSPI(adev) (IS_BOLO8(adev)? B8_DAC_SPI: AO420_DACSPI)

static int poll_dacspi_complete(struct acq400_dev *adev, u32 wv)
{
	unsigned pollcat = 0;
	while( ++pollcat < TIMEOUT){
		u32 rv = acq400rd32(adev, DACSPI(adev));
		if ((rv&AO420_DACSPI_WC) != 0){
			dev_dbg(DEVP(adev),
			"poll_dacspi_complete() success after %d\n", pollcat);
			wv &= ~(AO420_DACSPI_CW|AO420_DACSPI_WC);
			acq400wr32(adev, DACSPI(adev), wv);
			return 0;
		}
		if ((pollcat%100)==0){
			dev_warn(DEVP(adev),
					"poll_dacspi_complete %d %08x %08x\n",
					pollcat, wv, rv);
		}
	}
	dev_err(DEVP(adev), "poll_dacspi_complete() giving up, no completion");
	return -1;
}


static ssize_t show_dacspi(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 dacspi = acq400rd32(adev, DACSPI(adev));

	return sprintf(buf, "0x%08x\n", dacspi);
}

static ssize_t store_dacspi(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned dacspi;
	if (sscanf(buf, "0x%x", &dacspi) == 1 || sscanf(buf, "%d", &dacspi) == 1){
		dacspi |= AO420_DACSPI_CW;
		acq400wr32(adev, DACSPI(adev), dacspi);

		if ((dacspi&AO420_DACSPI_CW) != 0){
			if (poll_dacspi_complete(adev, dacspi)){
				return -1;
			}
		}
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(dacspi, S_IWUGO, show_dacspi, store_dacspi);

#define DAC_CTRL_RESET(adev)	(IS_BOLO8(adev)? B8_DAC_CON: DAC_CTRL)

static ssize_t show_dacreset(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned dac_ctrl = acq400rd32(adev, DAC_CTRL_RESET(adev));

	return sprintf(buf, "%u\n", (dac_ctrl&ADC_CTRL_ADC_RST) != 0);
}

static ssize_t store_dacreset(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned dac_ctrl = acq400rd32(adev, DAC_CTRL_RESET(adev));
	unsigned dacreset;

	dac_ctrl |= ADC_CTRL_MODULE_EN;

	if (sscanf(buf, "%d", &dacreset) == 1){
		if (dacreset){
			dac_ctrl |= ADC_CTRL_ADC_RST;
		}else{
			dac_ctrl &= ~ADC_CTRL_ADC_RST;
		}
		acq400wr32(adev, DAC_CTRL_RESET(adev), dac_ctrl);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(dacreset, S_IWUGO|S_IRUGO, show_dacreset, store_dacreset);

static ssize_t show_dacreset_device(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned dac_ctrl = acq400rd32(adev, DAC_CTRL_RESET(adev));

	return sprintf(buf, "%u\n", (dac_ctrl&ADC_CTRL_ADC_RST) != 0);
}

static ssize_t store_dacreset_device(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned dacreset_device;

	if (sscanf(buf, "%d", &dacreset_device) == 1){
		if (dacreset_device){
			if (ao424_set_spans(adev)){
				return -1;
			}
		}
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(dacreset_device, S_IWUGO, show_dacreset_device, store_dacreset_device);

static ssize_t show_dac_encoding(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];

	return sprintf(buf, "%s\n",
		IS_AO420(adev)? "signed": IS_AO424(adev)? "unsigned": "unknown");
}


static DEVICE_ATTR(dac_encoding, S_IRUGO, show_dac_encoding, 0);


static ssize_t show_odd_channels(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 cgen = acq400rd32(adev, DAC_424_CGEN);
	return sprintf(buf, "%d\n",  (cgen&DAC_424_CGEN_ODD_CHANS) != 0);
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
		u32 cgen = acq400rd32(adev, DAC_424_CGEN);
		cgen &= ~DAC_424_CGEN_DISABLE_X;

		if (odd_chan_en){
			cgen |= DAC_424_CGEN_ODD_CHANS;
			adev->nchan_enabled = 16;
		}else{
			cgen &= ~DAC_424_CGEN_ODD_CHANS;
			adev->nchan_enabled = 32;
		}

		acq400wr32(adev, DAC_424_CGEN, cgen);
		measure_ao_fifo(adev);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(odd_channels,
		S_IRUGO|S_IWUGO, show_odd_channels, store_odd_channels);


static const struct attribute *ao420_attrs[] = {
	&dev_attr_dac_range_01.attr,
	&dev_attr_dac_range_02.attr,
	&dev_attr_dac_range_03.attr,
	&dev_attr_dac_range_04.attr,
	&dev_attr_dac_range_REF.attr,
	&dev_attr_AO_01.attr,
	&dev_attr_AO_02.attr,
	&dev_attr_AO_03.attr,
	&dev_attr_AO_04.attr,
	&dev_attr_playloop_length.attr,
	&dev_attr_playloop_cursor.attr,
	&dev_attr_dacspi.attr,
	&dev_attr_dacreset.attr,
	&dev_attr_dacreset_device.attr,
	&dev_attr_dac_headroom.attr,
	&dev_attr_dac_fifo_samples.attr,
	&dev_attr_dac_encoding.attr,
	NULL
};


static const struct attribute *ao424_attrs[] = {
	&dev_attr_AO_01.attr,
	&dev_attr_AO_02.attr,
	&dev_attr_AO_03.attr,
	&dev_attr_AO_04.attr,
	&dev_attr_AO_05.attr,
	&dev_attr_AO_06.attr,
	&dev_attr_AO_07.attr,
	&dev_attr_AO_08.attr,
	&dev_attr_AO_09.attr,
	&dev_attr_AO_10.attr,
	&dev_attr_AO_11.attr,
	&dev_attr_AO_12.attr,
	&dev_attr_AO_13.attr,
	&dev_attr_AO_14.attr,
	&dev_attr_AO_15.attr,
	&dev_attr_AO_16.attr,
	&dev_attr_AO_17.attr,
	&dev_attr_AO_18.attr,
	&dev_attr_AO_19.attr,
	&dev_attr_AO_20.attr,
	&dev_attr_AO_21.attr,
	&dev_attr_AO_22.attr,
	&dev_attr_AO_23.attr,
	&dev_attr_AO_24.attr,
	&dev_attr_AO_25.attr,
	&dev_attr_AO_26.attr,
	&dev_attr_AO_27.attr,
	&dev_attr_AO_28.attr,
	&dev_attr_AO_29.attr,
	&dev_attr_AO_30.attr,
	&dev_attr_AO_31.attr,
	&dev_attr_AO_32.attr,
	&dev_attr_playloop_length.attr,
	&dev_attr_playloop_cursor.attr,
	&dev_attr_dacreset.attr,
	&dev_attr_dacreset_device.attr,
	&dev_attr_dac_headroom.attr,
	&dev_attr_dac_fifo_samples.attr,
	&dev_attr_dac_encoding.attr,
	&dev_attr_bank_mask.attr,
	&dev_attr_odd_channels.attr,
	NULL
};

static ssize_t show_offset_dacN(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	const int ix)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	u32 offset_dac = bolo8_get_offset_dacN(adev, ix);
	return sprintf(buf, "0x%08x\n", offset_dac);
}

static ssize_t store_offset_dacN(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count,
	const int ix)
{
	struct acq400_dev* adev = acq400_devices[dev->id];
	int rc = count;
	int offset;

	if (sscanf(buf, "++%d", &offset) == 1){
		offset = adev->bolo8.offset_dacs[ix] + offset;
	}else if (sscanf(buf, "--%d", &offset) == 1){
		offset = adev->bolo8.offset_dacs[ix] - offset;
	}else if (sscanf(buf, "0x%x", &offset) == 1){
		;
	}else if (sscanf(buf, "%d", &offset) == 1){
		;
	}else{
		return -1;
	}

	bolo8_set_offset_dacN(adev, ix, adev->bolo8.offset_dacs[ix] = offset);

	return rc;
}


#define MAKE_OFFSET_DAC(IX)						\
static ssize_t show_offset_dac##IX(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_offset_dacN(dev, attr, buf, IX-1);			\
}									\
									\
static ssize_t store_offset_dac##IX(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_offset_dacN(dev, attr, buf, count, IX-1);		\
}									\
static DEVICE_ATTR(offset_dac##IX, S_IRUGO|S_IWUGO, 			\
		show_offset_dac##IX, store_offset_dac##IX)


MAKE_OFFSET_DAC(1);
MAKE_OFFSET_DAC(2);
MAKE_OFFSET_DAC(3);
MAKE_OFFSET_DAC(4);
MAKE_OFFSET_DAC(5);
MAKE_OFFSET_DAC(6);
MAKE_OFFSET_DAC(7);
MAKE_OFFSET_DAC(8);

static ssize_t show_adc_sample_count(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 count = acq400rd32_upcount(acq400_devices[dev->id], B8_ADC_SAMPLE_CNT);
	return sprintf(buf, "%u\n", count&ADC_SAMPLE_CTR_MASK);
}

static DEVICE_ATTR(adc_sample_count, S_IRUGO, show_adc_sample_count, 0);

static ssize_t show_dac_sample_count(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	u32 count = acq400rd32_upcount(acq400_devices[dev->id], B8_DAC_SAMPLE_CNT);
	return sprintf(buf, "%u\n", count&ADC_SAMPLE_CTR_MASK);
}

static DEVICE_ATTR(dac_sample_count, S_IRUGO, show_dac_sample_count, 0);

static const struct attribute *bolo8_attrs[] = {
	&dev_attr_current_adc_enable.attr,
	&dev_attr_offset_dac_enable.attr,
	&dev_attr_dacspi.attr,
	&dev_attr_dacreset.attr,
	&dev_attr_offset_dac1.attr,
	&dev_attr_offset_dac2.attr,
	&dev_attr_offset_dac3.attr,
	&dev_attr_offset_dac4.attr,
	&dev_attr_offset_dac5.attr,
	&dev_attr_offset_dac6.attr,
	&dev_attr_offset_dac7.attr,
	&dev_attr_offset_dac8.attr,
	&dev_attr_adc_sample_count.attr,
	&dev_attr_dac_sample_count.attr,
	NULL
};


static ssize_t show_DO32(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "0x%08x\n", adev->dio432.DO32);
}

static ssize_t store_DO32(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned DO32 = 0;

	if (sscanf(buf, "0x%x", &DO32) == 1 || sscanf(buf, "%u", &DO32) == 1){
		adev->dio432.DO32 = DO32;
		wake_up_interruptible(&adev->w_waitq);
		yield();
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(DO32, S_IRUGO|S_IWUGO, show_DO32, store_DO32);

static ssize_t show_DI32(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "0x%08x\n", adev->dio432.DI32);
}

static ssize_t store_DI32(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned DI32 = 0;

	if (sscanf(buf, "%u", &DI32) == 1){
		adev->dio432.DI32 = DI32;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(DI32, S_IRUGO|S_IWUGO, show_DI32, store_DI32);

static ssize_t show_mode(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	return sprintf(buf, "%d %s\n", adev->dio432.mode,
			dio32mode2str(adev->dio432.mode));
}

static ssize_t store_mode(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned mode = 0;

	if (sscanf(buf, "%u", &mode) == 1){
		dio432_set_mode(adev, mode);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(mode, S_IRUGO|S_IWUGO, show_mode, store_mode);

static ssize_t show_ext_clk_from_sync(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 ctrl = acq400rd32(adev, DIO432_DIO_CTRL);
	return sprintf(buf, "%d\n",
			(ctrl&DIO432_CTRL_EXT_CLK_SYNC) != 0);

}

static ssize_t store_ext_clk_from_sync(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned ext_clk_from_sync = 0;

	if (sscanf(buf, "%u", &ext_clk_from_sync) == 1){
		u32 ctrl = acq400rd32(adev, DIO432_DIO_CTRL);
		if (ext_clk_from_sync){
			ctrl |= DIO432_CTRL_EXT_CLK_SYNC;
		}else{
			ctrl &= ~DIO432_CTRL_EXT_CLK_SYNC;
		}
		acq400wr32(adev, DIO432_DIO_CTRL, ctrl);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(ext_clk_from_sync, S_IRUGO|S_IWUGO, show_ext_clk_from_sync, store_ext_clk_from_sync);


static ssize_t show_byte_is_output(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 byte_is_output = adev->dio432.byte_is_output;

	return sprintf(buf, "%d,%d,%d,%d\n",
			byte_is_output&DIO432_CPLD_CTRL_OUTPUT(0),
			byte_is_output&DIO432_CPLD_CTRL_OUTPUT(1),
			byte_is_output&DIO432_CPLD_CTRL_OUTPUT(2),
			byte_is_output&DIO432_CPLD_CTRL_OUTPUT(3)
	);
}

static ssize_t store_byte_is_output(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	int bytes[4];
	unsigned byte_is_output = 0;

	if (sscanf(buf, "%d,%d,%d,%d", bytes+0, bytes+1, bytes+2, bytes+3) == 4){
		int ib = 0;
		for (ib = 0; ib <= 3; ++ib){
			if (bytes[ib]){
				byte_is_output |= DIO432_CPLD_CTRL_OUTPUT(ib);
			}
		}
		adev->dio432.byte_is_output = byte_is_output;
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(byte_is_output, S_IRUGO|S_IWUGO, show_byte_is_output, store_byte_is_output);



static const struct attribute *dio432_attrs[] = {
	&dev_attr_DI32.attr,
	&dev_attr_DO32.attr,
	&dev_attr_mode.attr,
	&dev_attr_byte_is_output.attr,
	&dev_attr_ext_clk_from_sync.attr,
	&dev_attr_playloop_length.attr,
	&dev_attr_playloop_cursor.attr,
	NULL
};



#define SCOUNT_KNOB(name, reg) 						\
static ssize_t show_clk_count_##name(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	u32 counter = acq400rd32_upcount(acq400_devices[dev->id], reg);	\
	return sprintf(buf, "%u\n", counter);				\
}									\
static DEVICE_ATTR(scount_##name, S_IRUGO, show_clk_count_##name, 0)

SCOUNT_KNOB(CLK_EXT, 	ACQ2006_CLK_COUNT(0));
SCOUNT_KNOB(CLK_MB, 	ACQ2006_CLK_COUNT(1));
SCOUNT_KNOB(CLK_S1,     ACQ2006_CLK_COUNT(SITE2DX(1)));
SCOUNT_KNOB(CLK_S2,     ACQ2006_CLK_COUNT(SITE2DX(2)));
SCOUNT_KNOB(CLK_S3,     ACQ2006_CLK_COUNT(SITE2DX(3)));
SCOUNT_KNOB(CLK_S4,     ACQ2006_CLK_COUNT(SITE2DX(4)));
SCOUNT_KNOB(CLK_S5,     ACQ2006_CLK_COUNT(SITE2DX(5)));
SCOUNT_KNOB(CLK_S6,     ACQ2006_CLK_COUNT(SITE2DX(6)));

SCOUNT_KNOB(TRG_EXT, 	ACQ2006_TRG_COUNT(0));
SCOUNT_KNOB(TRG_MB, 	ACQ2006_TRG_COUNT(1));
SCOUNT_KNOB(TRG_S1,     ACQ2006_TRG_COUNT(SITE2DX(1)));
SCOUNT_KNOB(TRG_S2,     ACQ2006_TRG_COUNT(SITE2DX(2)));
SCOUNT_KNOB(TRG_S3,     ACQ2006_TRG_COUNT(SITE2DX(3)));
SCOUNT_KNOB(TRG_S4,     ACQ2006_TRG_COUNT(SITE2DX(4)));
SCOUNT_KNOB(TRG_S5,     ACQ2006_TRG_COUNT(SITE2DX(5)));
SCOUNT_KNOB(TRG_S6,     ACQ2006_TRG_COUNT(SITE2DX(6)));

SCOUNT_KNOB(EVT_EXT, 	ACQ2006_EVT_COUNT(0));
SCOUNT_KNOB(EVT_MB, 	ACQ2006_EVT_COUNT(1));
SCOUNT_KNOB(EVT_S1,     ACQ2006_EVT_COUNT(SITE2DX(1)));
SCOUNT_KNOB(EVT_S2,     ACQ2006_EVT_COUNT(SITE2DX(2)));
SCOUNT_KNOB(EVT_S3,     ACQ2006_EVT_COUNT(SITE2DX(3)));
SCOUNT_KNOB(EVT_S4,     ACQ2006_EVT_COUNT(SITE2DX(4)));
SCOUNT_KNOB(EVT_S5,     ACQ2006_EVT_COUNT(SITE2DX(5)));
SCOUNT_KNOB(EVT_S6,     ACQ2006_EVT_COUNT(SITE2DX(6)));

SCOUNT_KNOB(SYN_EXT, 	ACQ2006_SYN_COUNT(0));
SCOUNT_KNOB(SYN_MB, 	ACQ2006_SYN_COUNT(1));
SCOUNT_KNOB(SYN_S1,     ACQ2006_SYN_COUNT(SITE2DX(1)));
SCOUNT_KNOB(SYN_S2,     ACQ2006_SYN_COUNT(SITE2DX(2)));
SCOUNT_KNOB(SYN_S3,     ACQ2006_SYN_COUNT(SITE2DX(3)));
SCOUNT_KNOB(SYN_S4,     ACQ2006_SYN_COUNT(SITE2DX(4)));
SCOUNT_KNOB(SYN_S5,     ACQ2006_SYN_COUNT(SITE2DX(5)));
SCOUNT_KNOB(SYN_S6,     ACQ2006_SYN_COUNT(SITE2DX(6)));


static ssize_t show_fan_percent(
	struct device * dev,
	struct device_attribute *attr,
	char * buf)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned pwm = acq400rd32(adev, MOD_CON)&ACQ1001_MCR_PWM_MASK;
	unsigned fan_percent;

	/* scale to 1..100 */
	pwm >>= ACQ1001_MCR_PWM_BIT;
	if (pwm > ACQ1001_MCR_PWM_MIN){
		pwm -= ACQ1001_MCR_PWM_MIN;
	}
	fan_percent = (pwm*100)/ACQ1001_MCR_PWM_MAX;
	return sprintf(buf, "%u\n", fan_percent);
}

static ssize_t store_fan_percent(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned mod_con = acq400rd32(adev, MOD_CON);
	unsigned fan_percent;

	if (sscanf(buf, "%u", &fan_percent) == 1){
		unsigned pwm;

		if (fan_percent > 100) fan_percent = 100;
		pwm = (fan_percent*ACQ1001_MCR_PWM_MAX)/100;
		pwm += ACQ1001_MCR_PWM_MIN;
		pwm <<= ACQ1001_MCR_PWM_BIT;
		if (pwm > ACQ1001_MCR_PWM_MASK){
			pwm = ACQ1001_MCR_PWM_MASK;
		}else{
			pwm &= ACQ1001_MCR_PWM_MASK;
		}
		mod_con &= ~ACQ1001_MCR_PWM_MASK;
		mod_con |= pwm;

		dev_info(DEVP(adev), "store_fan_percent:%d write %08x\n",
				fan_percent, mod_con);

		acq400wr32(adev, MOD_CON, mod_con);
		return count;
	}else{
		return -1;
	}
}

static DEVICE_ATTR(fan_percent, S_IWUGO|S_IRUGO, show_fan_percent, store_fan_percent);

static ssize_t show_acq0000_mod_con(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned mask)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 mod_con = acq400rd32(adev, MOD_CON);

	return sprintf(buf, "%u\n", (mod_con&mask) != 0);
}



static ssize_t store_acq0000_mod_con(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count,
	unsigned mask)
{
	unsigned enable;
	if (sscanf(buf, "%u", &enable) == 1){
		struct acq400_dev *adev = acq400_devices[dev->id];
		u32 mod_con = acq400rd32(adev, MOD_CON);

		if (enable){
			mod_con |= mask;
		}else{
			mod_con &= ~mask;
		}
		acq400wr32(adev, MOD_CON, mod_con);
		return count;
	}else{
		return -1;
	}
}

#define MODCON_KNOB(name, mask)						\
static ssize_t show_##name(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_acq0000_mod_con(dev, attr, buf, mask);		\
}									\
static ssize_t store_##name(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_acq0000_mod_con(dev, attr, buf, count, mask);	\
}									\
static DEVICE_ATTR(name, S_IRUGO|S_IWUGO, show_##name, store_##name)

MODCON_KNOB(mod_en, 	MCR_MOD_EN);
MODCON_KNOB(psu_sync, 	MCR_PSU_SYNC);
MODCON_KNOB(fan,	MCR_FAN_EN);
MODCON_KNOB(soft_trig,  MCR_SOFT_TRIG);
MODCON_KNOB(celf_power_en, ACQ1001_MCR_CELF_PSU_EN);

int get_agg_threshold_bytes(struct acq400_dev *adev, u32 agg)
{
	return ((agg>>AGG_SIZE_SHL)&AGG_SIZE_MASK)*AGG_SIZE_UNIT(adev);
}

#define AGG_SEL	"aggregator="
#define TH_SEL	"threshold="

static ssize_t show_agg_reg(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	const unsigned offset,
	const unsigned mshift)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	u32 regval = acq400rd32(adev, offset);
	char mod_group[80];
	int site;

	for (site = 1, mod_group[0] = '\0'; site <= 6; ++site){
		if ((regval & AGG_MOD_EN(site, mshift)) != 0){
			/* @@todo site 3/4 frig */
			int dsite = (IS_ACQ1001SC(adev) && site == 3)? 4: site;
			if (strlen(mod_group) == 0){
				strcat(mod_group, "sites=");
			}else{
				strcat(mod_group, ",");
			}

			sprintf(mod_group+strlen(mod_group), "%d", dsite);
		}
	}
	if (strlen(mod_group) == 0){
		sprintf(mod_group, "sites=none");
	}
	if (mshift == DATA_ENGINE_MSHIFT){
		sprintf(mod_group+strlen(mod_group), " %s%d", AGG_SEL,
				regval&DATA_ENGINE_SELECT_AGG? 1: 0);
	}else if(mshift == AGGREGATOR_MSHIFT){
		sprintf(mod_group+strlen(mod_group), " %s%d", TH_SEL,
				get_agg_threshold_bytes(adev, regval));
	}

	return sprintf(buf, "0x%08x %s %s\n", regval, mod_group,
			regval&DATA_MOVER_EN? "on": "off");
}

extern int good_sites[];
extern int good_sites_count;


int get_site(struct acq400_dev *adev, char s)
{
	int ii;
	int site = s-'0';

	for(ii = 0; ii < good_sites_count; ++ii){
		if (site == good_sites[ii]){
			if (IS_ACQ1001SC(adev) && site == 4){
				/* @@todo site 3/4 frig */
				return 3;
			}else{
				return site;
			}
		}
	}

	return -1;
}

int add_aggregator_set(struct acq400_dev *adev, int site)
{
	int ia = 0;
	struct acq400_dev *slave = acq400_lookupSite(site);
	for (ia = 0; ia < MAXSITES; ++ia){
		dev_dbg(DEVP(adev), "add_aggregator set [%d]=%p site:%d %p",
				ia, adev->aggregator_set[ia], site, slave);
		if (adev->aggregator_set[ia] == slave){
			return 0;
		}else if (adev->aggregator_set[ia] == 0){
			adev->aggregator_set[ia] = slave;
			return 0;
		}
	}
	dev_err(DEVP(adev), "ERROR: add_aggregator_set() failed");
	return 1;
}

void clear_aggregator_set(struct acq400_dev *adev)
{
	int ia = 0;
	for (ia = 0; ia < MAXSITES; ++ia){
		adev->aggregator_set[ia] = 0;
	}
}
static ssize_t store_agg_reg(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf,
	size_t count,
	const unsigned offset,
	const unsigned mshift)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned regval;
	char* match;
	int pass = 0;

	dev_dbg(DEVP(adev), "store_agg_reg \"%s\"", buf);

	if ((match = strstr(buf, "sites=")) != 0){
		unsigned regval = acq400rd32(adev, offset);
		char* cursor = match+strlen("sites=");
		int site;

		regval &= ~(AGG_SITES_MASK << mshift);

		if (strncmp(cursor, "none", 4) != 0){
			for (; *cursor && *cursor != ' '; ++cursor){
				switch(*cursor){
				case ',':
				case ' ':	continue;
				case '\n':  	break;
				default:
					site = get_site(adev, *cursor);
					if (site > 0){
						regval |= AGG_MOD_EN(site, mshift);
						add_aggregator_set(adev, site);
						break;
					}else{
						dev_err(dev, "bad site designator: %c", *cursor);
						return -1;
					}
				}
			}
		}else{
			clear_aggregator_set(adev);
		}
		acq400wr32(adev, offset, regval);
		pass = 1;
	}
	if (mshift == DATA_ENGINE_MSHIFT){
		char *agg_sel = strstr(buf, AGG_SEL);
		if (agg_sel){
			int include_agg = 0;
			if (sscanf(agg_sel, AGG_SEL"%d", &include_agg) == 1){
				unsigned regval = acq400rd32(adev, offset);
				if (include_agg){
					regval |= DATA_ENGINE_SELECT_AGG;
				}else{
					regval &= ~DATA_ENGINE_SELECT_AGG;
				}
				acq400wr32(adev, offset, regval);
				pass = 1;
			}
		}
	}else if (mshift == AGGREGATOR_MSHIFT){
		char *th_sel = strstr(buf, TH_SEL);
		if (th_sel){
			int thbytes = 0;
			if (sscanf(th_sel, TH_SEL"%d", &thbytes) == 1){
				unsigned regval = acq400rd32(adev, offset);
				unsigned th = thbytes/AGG_SIZE_UNIT(adev);
				if (th > AGG_SIZE_MASK) th = AGG_SIZE_MASK;
				regval &= ~(AGG_SIZE_MASK<<AGG_SIZE_SHL);
				regval |= th<<AGG_SIZE_SHL;
				acq400wr32(adev, offset, regval);
				pass = 1;
			}else{
				dev_err(dev, "arg not integer (%s)", th_sel);
				return -1;
			}
		}
	}
	if ((match = strstr(buf, "on")) != 0){
		unsigned regval = acq400rd32(adev, offset);
		regval |= DATA_MOVER_EN;
		acq400wr32(adev, offset, regval);
		pass = 1;
	}else if ((match = strstr(buf, "off")) != 0){
		unsigned regval = acq400rd32(adev, offset);
		regval &= ~DATA_MOVER_EN;
		acq400wr32(adev, offset, regval);
		pass = 1;
	}

	if (pass){
		return count;
	}else{
		/* aggregator= passes this paragraph, so put it LAST! */
		if (sscanf(buf, "%x", &regval) == 1){
			acq400wr32(adev, offset, regval);
			return count;
		}
		return -1;
	}
}


#define REG_KNOB(name, offset, mshift)					\
static ssize_t show_reg_##name(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_agg_reg(dev, attr, buf, offset, mshift);		\
}									\
static ssize_t store_reg_##name(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf,						\
	size_t count)							\
{									\
	return store_agg_reg(dev, attr, buf, count,  offset, mshift);	\
}									\
static DEVICE_ATTR(name, 						\
	S_IRUGO|S_IWUGO, show_reg_##name, store_reg_##name)

REG_KNOB(aggregator, AGGREGATOR,	AGGREGATOR_MSHIFT);
REG_KNOB(data_engine_0, DATA_ENGINE(0), DATA_ENGINE_MSHIFT);
REG_KNOB(data_engine_1, DATA_ENGINE(1), DATA_ENGINE_MSHIFT);
REG_KNOB(data_engine_2, DATA_ENGINE(2), DATA_ENGINE_MSHIFT);
REG_KNOB(data_engine_3, DATA_ENGINE(3), DATA_ENGINE_MSHIFT);



static ssize_t show_aggsta(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned mask)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	unsigned aggsta = acq400rd32(adev, AGGSTA);
	unsigned field = aggsta&mask;

	for (; (mask&0x1) == 0; mask >>=1, field >>= 1){
		;
	}

	return sprintf(buf, "%u\n", field);
}

#define SHOW_AGGSTA(name, mask) 					\
static ssize_t show_aggsta_##name(					\
	struct device *dev,						\
	struct device_attribute *attr,					\
	char* buf )							\
{									\
	return show_aggsta(dev, attr, buf, mask);			\
}									\
static DEVICE_ATTR(name, S_IRUGO, show_aggsta_##name, 0)

SHOW_AGGSTA(aggsta_fifo_count, AGGSTA_FIFO_COUNT);
SHOW_AGGSTA(aggsta_fifo_stat,  AGGSTA_FIFO_STAT);
SHOW_AGGSTA(aggsta_engine_stat, AGGSTA_ENGINE_STAT);

static const struct attribute *sc_common_attrs[] = {
	&dev_attr_aggregator.attr,
	&dev_attr_aggsta_fifo_count.attr,
	&dev_attr_aggsta_fifo_stat.attr,
	&dev_attr_aggsta_engine_stat.attr,
	&dev_attr_mod_en.attr,
	&dev_attr_psu_sync.attr,
	&dev_attr_soft_trig.attr,
	&dev_attr_celf_power_en.attr,
	&dev_attr_gpg_top.attr,
	&dev_attr_gpg_trg.attr,
	&dev_attr_gpg_clk.attr,
	&dev_attr_gpg_sync.attr,
	&dev_attr_gpg_mode.attr,
	&dev_attr_gpg_enable.attr,
	&dev_attr_spad.attr,
	&dev_attr_spad0.attr,
	&dev_attr_spad1.attr,
	&dev_attr_spad2.attr,
	&dev_attr_spad3.attr,
	&dev_attr_spad4.attr,
	&dev_attr_spad5.attr,
	&dev_attr_spad6.attr,
	&dev_attr_spad7.attr,
	NULL
};
static const struct attribute *acq2006sc_attrs[] = {
	&dev_attr_data_engine_0.attr,
	&dev_attr_data_engine_1.attr,
	&dev_attr_data_engine_2.attr,
	&dev_attr_data_engine_3.attr,

	&dev_attr_scount_CLK_EXT.attr,
	&dev_attr_scount_CLK_MB.attr,
	&dev_attr_scount_CLK_S1.attr,
	&dev_attr_scount_CLK_S2.attr,
	&dev_attr_scount_CLK_S3.attr,
	&dev_attr_scount_CLK_S4.attr,
	&dev_attr_scount_CLK_S5.attr,
	&dev_attr_scount_CLK_S6.attr,

	&dev_attr_scount_TRG_EXT.attr,
	&dev_attr_scount_TRG_MB.attr,
	&dev_attr_scount_TRG_S1.attr,
	&dev_attr_scount_TRG_S2.attr,
	&dev_attr_scount_TRG_S3.attr,
	&dev_attr_scount_TRG_S4.attr,
	&dev_attr_scount_TRG_S5.attr,
	&dev_attr_scount_TRG_S6.attr,

	&dev_attr_scount_EVT_EXT.attr,
	&dev_attr_scount_EVT_MB.attr,
	&dev_attr_scount_EVT_S1.attr,
	&dev_attr_scount_EVT_S2.attr,
	&dev_attr_scount_EVT_S3.attr,
	&dev_attr_scount_EVT_S4.attr,
	&dev_attr_scount_EVT_S5.attr,
	&dev_attr_scount_EVT_S6.attr,

	&dev_attr_scount_SYN_EXT.attr,
	&dev_attr_scount_SYN_MB.attr,
	&dev_attr_scount_SYN_S1.attr,
	&dev_attr_scount_SYN_S2.attr,
	&dev_attr_scount_SYN_S3.attr,
	&dev_attr_scount_SYN_S4.attr,
	&dev_attr_scount_SYN_S5.attr,
	&dev_attr_scount_SYN_S6.attr,
	NULL
};

static const struct attribute *acq1001sc_attrs[] = {
	&dev_attr_data_engine_0.attr,
	&dev_attr_fan.attr,
	&dev_attr_fan_percent.attr,

	&dev_attr_scount_CLK_EXT.attr,
	&dev_attr_scount_CLK_MB.attr,
	&dev_attr_scount_CLK_S1.attr,
	&dev_attr_scount_CLK_S2.attr,

	&dev_attr_scount_TRG_EXT.attr,
	&dev_attr_scount_TRG_MB.attr,
	&dev_attr_scount_TRG_S1.attr,
	&dev_attr_scount_TRG_S2.attr,

	&dev_attr_scount_EVT_EXT.attr,
	&dev_attr_scount_EVT_MB.attr,
	&dev_attr_scount_EVT_S1.attr,
	&dev_attr_scount_EVT_S2.attr,

	&dev_attr_scount_SYN_EXT.attr,
	&dev_attr_scount_SYN_MB.attr,
	&dev_attr_scount_SYN_S1.attr,
	&dev_attr_scount_SYN_S2.attr,
	NULL
};
void acq400_createSysfs(struct device *dev)
{
	struct acq400_dev *adev = acq400_devices[dev->id];
	const struct attribute **specials = 0;

	dev_info(dev, "acq400_createSysfs()");
	if (sysfs_create_files(&dev->kobj, sysfs_base_attrs)){
		dev_err(dev, "failed to create sysfs");
	}

	if (IS_DUMMY(adev)){
		return;
	}else if IS_ACQx00xSC(adev){
		if (sysfs_create_files(&dev->kobj, sc_common_attrs)){
			dev_err(dev, "failed to create sysfs");
		}
		if (HAS_HDMI_SYNC(adev)){
			if (sysfs_create_files(&dev->kobj, hdmi_sync_attrs)){
				dev_err(dev, "failed to create sysfs HDMI");
			}
		}
		if (IS_ACQ2X06SC(adev)){
			specials = acq2006sc_attrs;
		}else if (IS_ACQ1001SC(adev)){
			specials = acq1001sc_attrs;
		}
	}else{
		if (sysfs_create_files(&dev->kobj, sysfs_device_attrs)){
			dev_err(dev, "failed to create sysfs");
		}
		if (IS_ACQ42X(adev)){
			specials = acq420_attrs;
		}else if (IS_ACQ43X(adev)){
			specials = acq435_attrs;
		}else if (IS_AO420(adev)){
			specials = ao420_attrs;
		}else if (IS_AO424(adev)){
			specials = ao424_attrs;
		}else if (IS_BOLO8(adev)){
			specials = bolo8_attrs;
		}else if (IS_DIO432X(adev)){
			specials = dio432_attrs;
		}else{
			return;
		}
	}


	if (sysfs_create_files(&dev->kobj, specials)){
		dev_err(dev, "failed to create sysfs");
	}
}

void acq400_delSysfs(struct device *dev)
{
	sysfs_remove_files(&dev->kobj, sysfs_base_attrs);
	// @@todo .. undoing the rest will be interesting .. */
}


