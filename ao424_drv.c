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

int set_spans_timeout = 8000;
module_param(set_spans_timeout, int, 0644);
MODULE_PARM_DESC(set_spans_timeout, "timeout on setting spans pollcount");



int _ao424_set_spans(struct acq400_dev* adev, unsigned ctrl)
{
	unsigned stat1;
	unsigned stat2;
	unsigned fifo_before;
	unsigned fifo_during;
	unsigned fifo_after;
	int pollcat = 0;

	ctrl |= ADC_CTRL_MODULE_EN;
	acq400wr32(adev, DAC_INT_CSR, 0);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_RST_ALL);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN);

	fifo_before = adev->xo.getFifoSamples(adev);
	write32(adev->dev_virtaddr+AXI_FIFO,
			adev->ao424_device_settings.u.lw, AO424_MAXCHAN);

	fifo_during = adev->xo.getFifoSamples(adev);

	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN|AO424_DAC_CTRL_SPAN);
	stat1 = acq400rd32(adev, DAC_FIFO_STA);

	while(((stat2 = acq400rd32(adev, DAC_FIFO_STA))&AO424_DAC_FIFO_STA_SWC) ==
			(stat1&AO424_DAC_FIFO_STA_SWC)){
		yield();
		if (++pollcat > set_spans_timeout){
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

	if ((ctrl&ADC_CTRL_ADC_EN) != 0){
		dev_err(DEVP(adev), "ao424_set_spans ADC_CTRL_ADC_EN");
		return -1;
	}else{
		return _ao424_set_spans(adev, ctrl);
	}
}
