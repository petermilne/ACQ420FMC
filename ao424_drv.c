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

int set_spans_timeout_ms = 4000;
module_param(set_spans_timeout_ms, int, 0644);
MODULE_PARM_DESC(set_spans_timeout_ms, "timeout on setting spans");


int _ao424_set_spans(struct acq400_dev* adev, unsigned ctrl)
{
	unsigned stat;
	unsigned long timeout;
	unsigned long j1;
	unsigned fifo_before;
	unsigned fifo_during;
	unsigned fifo_after;
	int pollcat = 0;
	int rc = 0;

	ctrl |= ADC_CTRL_MODULE_EN;
	acq400wr32(adev, DAC_INT_CSR, 0);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_RST_ALL);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN);

	fifo_before = ao420_getFifoSamples(adev);
	write32(adev->dev_virtaddr+AXI_FIFO,
			adev->ao424_device_settings.u.lw, AO424_MAXCHAN);

	fifo_during = ao420_getFifoSamples(adev);

	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN|AO424_DAC_CTRL_SPAN);

	fifo_after = ao420_getFifoSamples(adev);

	j1 = jiffies;
	timeout = j1 + msecs_to_jiffies(set_spans_timeout_ms);

	dev_dbg(DEVP(adev), "j1:%lu timeout would be at:%lu", j1, timeout);

	while(((stat = acq400rd32(adev, DAC_FIFO_STA))&AO424_DAC_FIFO_STA_SWC) == 0){
		if (jiffies != j1){
			dev_info(DEVP(adev), "stat:%08x waiting:%08x now:%lu timeout scheduled at:%lu",
				stat, AO424_DAC_FIFO_STA_SWC, jiffies, timeout);
			j1 = jiffies;
		}
		yield();
		if (time_is_after_jiffies(timeout)){
			dev_err(DEVP(adev), "timeout polling for spans done");
			rc = -1;
			break;
		}
		++pollcat;
	}

	dev_dbg(DEVP(adev), "after poll loop: [%d] status %08x", pollcat, stat);
	dev_dbg(DEVP(adev), "before:%u during:%u after:%u eng:%u",
				fifo_before, fifo_during, fifo_after, ao420_getFifoSamples(adev));
	acq400wr32(adev, DAC_FIFO_STA, stat);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN);
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
