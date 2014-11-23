/*                                                                           *
 * ao424_drv.c                                                              *
 *                                                                           *
 *  Created on: 24 Sep 2014                                                  *
 *      Author: pgm                                                          *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                    *
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
#include "acq400_lists.h"

#include "dmaengine.h"
#include <linux/jiffies.h>

typedef unsigned short ushort;

int set_spans_timeout = 1000;
module_param(set_spans_timeout, int, 0644);
MODULE_PARM_DESC(set_spans_timeout, "timeout on setting spans pollcount");

#define IS_UNIPOLAR(span) ((span) == 0 || (span) == 1)

#define UP_ZERO	0x0000
#define BP_ZERO 0x8000

static void _acq400wr32(struct acq400_dev *adev, int offset, u32 value)
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

static u32 _acq400rd32(struct acq400_dev *adev, int offset)
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

#define acq400rd32 _acq400rd32
#define acq400wr32 _acq400wr32

void ao424_init_on_setspan(struct acq400_dev* adev)
{
	struct AO424 *ao424ch = &adev->ao424_device_settings;
	int ic;

	for (ic = 0; ic < AO424_MAXCHAN; ++ic){
		ao424ch->u.ch.ao424_initvals[ic] =
		IS_UNIPOLAR(ao424ch->u.ch.ao424_spans[ic])? UP_ZERO: BP_ZERO;
		dev_dbg(DEVP(adev), "ao424_init_on_setspan() span:%d %s %04x",
				ao424ch->u.ch.ao424_spans[ic],
				IS_UNIPOLAR(ao424ch->u.ch.ao424_spans[ic])? "UP" : "BP",
				ao424ch->u.ch.ao424_initvals[ic]);
	}
}

int _ao424_set_spans(struct acq400_dev* adev, unsigned ctrl)
{
	unsigned stat1;
	unsigned stat2;
	unsigned fifo_before;
	unsigned fifo_during;
	unsigned fifo_after;
	int pollcat = 0;

	ao424_init_on_setspan(adev);

	ctrl &= ~ADC_CTRL_ADC_EN;
	ctrl |= ADC_CTRL_MODULE_EN;

	dev_dbg(DEVP(adev), "_ao424_set_spans: ctrl %x", ctrl);
	acq400wr32(adev, DAC_INT_CSR, 0);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_RST_ALL);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN);

	fifo_before = adev->xo.getFifoSamples(adev);
	write32(adev->dev_virtaddr+AXI_FIFO,
			adev->ao424_device_settings.u.lw, AO424_MAXCHAN);

	fifo_during = adev->xo.getFifoSamples(adev);

	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN|AO424_DAC_CTRL_SPAN);
	stat1 = acq400rd32(adev, DAC_FIFO_STA);

	while(((stat2 = acq400rd32(adev, DAC_FIFO_STA))&AO424_DAC_FIFO_STA_SWC) == 0){
		msleep(1);
		if (++pollcat > set_spans_timeout){
			acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN|AO424_DAC_CTRL_SPAN);
			dev_info(DEVP(adev), "SWC timeout at pollcat %d", pollcat);
			return -1;
		}
	}

	fifo_after = adev->xo.getFifoSamples(adev);


	acq400wr32(adev, DAC_FIFO_STA, acq400rd32(adev, DAC_FIFO_STA));
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN);

	dev_info(DEVP(adev), "AO424_DAC_CTRL_SPAN set SWC stat1=%08x  stat2=%08x now: %08x pollcat:%d",
			stat1, stat2, acq400rd32(adev, DAC_FIFO_STA), pollcat);
	dev_info(DEVP(adev), "fifo samples before:%u during:%u after:%u now:%u",
			fifo_before, fifo_during, fifo_after, adev->xo.getFifoSamples(adev));
	return 0;
}

int ao424_set_spans(struct acq400_dev* adev)
{
	unsigned ctrl = acq400rd32(adev, DAC_CTRL);
	int was_enabled = 0;
	int rc = 0;

	if ((ctrl&ADC_CTRL_ADC_EN) != 0){
		if (adev->AO_playloop.length > 0){
			dev_err(DEVP(adev), "ao424_set_spans ADC_CTRL_ADC_EN AND playloop_length no change");
			return -1;
		}else{
			dev_dbg(DEVP(adev), "ADC is enabled, clear defaults");
			memset(adev->ao424_device_settings.u.ch.ao424_initvals, 0,
					sizeof(adev->ao424_device_settings.u.ch.ao424_initvals));
			memset(adev->ao424_device_settings.ao424_immediates, 0,
					sizeof(adev->ao424_device_settings.ao424_immediates));
		}
		was_enabled = 1;
		dev_dbg(DEVP(adev), "clear ADC_CTRL_ADC_EN");
		acq400wr32(adev, DAC_CTRL, ctrl &~ADC_CTRL_ADC_EN);
	}

	rc = _ao424_set_spans(adev, ctrl);
	if (rc == 0){
		if (was_enabled){
			dev_dbg(DEVP(adev), "enable ADC_CTRL_ADC_EN");
			acq400wr32(adev, DAC_CTRL, ctrl);
		}
	}
	return rc;
}
