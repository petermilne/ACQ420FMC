/*                                                                           *
 * dio432_drv.c                                                              *
 *                                                                           *
 *  Created on: 21 Sep 2014                                                  *
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

void dio432_set_direction(struct acq400_dev *adev, unsigned byte_is_output)
{
	acq400wr32(adev, DIO432_DIO_CPLD_CTRL, byte_is_output);
	acq400wr32(adev, DIO432_DIO_CPLD_CTRL,
				DIO432_CPLD_CTRL_COMMAND_WRITE|byte_is_output);

	/* @@todo doxygen indicates poll for complete, but SR example sets it
	 * following the SR example
	 */
	acq400wr32(adev, DIO432_DIO_CPLD_CTRL,
			DIO432_CPLD_CTRL_COMMAND_COMPLETE|
			DIO432_CPLD_CTRL_COMMAND_WRITE|byte_is_output);
	acq400wr32(adev, DIO432_DIO_CPLD_CTRL, byte_is_output);
}

int dio32_immediate_loop(void *data)
{
	struct acq400_dev *adev = (struct acq400_dev *)data;
	unsigned syscon = DIO432_CTRL_LL;
	int nloop = 0;

	syscon |= IS_DIO432PMOD(adev)?
		DIO432_CTRL_SHIFT_DIV_PMOD: DIO432_CTRL_SHIFT_DIV_FMC;
	acq400wr32(adev, DIO432_DIO_CTRL, syscon);
	acq400wr32(adev, DIO432_DIO_CTRL, syscon |= DIO432_CTRL_MODULE_EN);
	acq400wr32(adev, DIO432_DIO_CTRL, syscon | DIO432_CTRL_ADC_RST|DIO432_CTRL_FIFO_RST);
	acq400wr32(adev, DIO432_DIO_CTRL, syscon);
	acq400wr32(adev, DIO432_DIO_CTRL, syscon|DIO432_CTRL_ADC_EN);
	dio432_set_direction(adev, adev->dio432_immediate.byte_is_output);

	acq400wr32(adev, DIO432_DI_FIFO_STATUS, DIO432_FIFSTA_CLR);
	acq400wr32(adev, DIO432_DO_FIFO_STATUS, DIO432_FIFSTA_CLR);

	for(; !kthread_should_stop(); ++nloop){
		acq400wr32(adev, DIO432_FIFO, adev->dio432_immediate.DO32);
		adev->dio432_immediate.DI32 = acq400rd32(adev, DIO432_FIFO);
		wait_event_interruptible_timeout(
			adev->DMA_READY, kthread_should_stop(),	1);
	}
}

void dio432_init_immediate(struct acq400_dev* adev)
{
	adev->w_task = kthread_run(dio32_immediate_loop, adev,
					"%s.dio32i", devname(adev));
}

void dio432_init_clocked(struct acq400_dev* adev)
{
	if (adev->w_task != 0){
		kthread_stop(adev->w_task);
	}
}
void dio432_set_immediate(struct acq400_dev* adev, int enable)
{
	if (enable){
		dio432_init_immediate(adev);
	}else{
		dio432_init_clocked(adev);
	}
}
