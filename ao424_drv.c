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


typedef unsigned short ushort;


void ao424_set_spans(struct acq400_dev* adev)
{
	unsigned ctrl = ADC_CTRL_MODULE_EN;
	unsigned stat;
	acq400wr32(adev, DAC_INT_CSR, 0);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_RST);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN);
	write32(adev->dev_virtaddr+AXI_FIFO,
			adev->ao424_device_settings.u.lw, AO424_MAXCHAN);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN|AO424_DAC_CTRL_SPAN);

	while(((stat = acq400rd32(adev, DAC_FIFO_STA))&AO424_DAC_FIFO_STA_SWC) == 0){
		yield();
	}
	acq400wr32(adev, DAC_FIFO_STA, stat);
	acq400wr32(adev, DAC_CTRL, ctrl|ADC_CTRL_FIFO_EN);
}
