/* ------------------------------------------------------------------------- *
 * bolo8_core.c  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 26 Apr 2014  
 *    Author: pgm                                                         
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
#include "bolo.h"
#include "hbm.h"

int b8_adc_conv_time = B8_ADC_CONV_TIME_DEFAULT;
module_param(b8_adc_conv_time, int, 0644);
MODULE_PARM_DESC(b8_adc_conv_time, "Number of ticks of clk_100M before commencing read back");


void bolo8_onStart(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, B8_ADC_CON);
	unsigned scount;
	dev_dbg(DEVP(adev), "bolo8_onStart()");
	acq400wr32(adev, B8_ADC_HITIDE, 	adev->hitide);
	// set clkdiv (assume done)
	// set timing bus (assume done)
	/** clear FIFO flags .. workaround hw bug */
	acq400wr32(adev, B8_ADC_FIFO_STA, ADC_FIFO_FLAGS);

	acq400wr32(adev, B8_ADC_CON, ctrl | ADC_CTRL_FIFO_RST);
	acq400wr32(adev, B8_ADC_CON, ctrl);
	if ((scount = acq400rd32(adev, B8_ADC_SAMPLE_CNT)) > 0){
		dev_warn(DEVP(adev),
		"ERROR: reset fifo but it's not empty! :%08x", scount);
	}

	acq400wr32(adev, B8_ADC_CONV_TIME, b8_adc_conv_time);
	adev->fifo_isr_done = 0;
	//acq420_enable_interrupt(adev);
	acq400wr32(adev, B8_ADC_CON, ctrl  |= ADC_CTRL_ADC_EN);
	acq400wr32(adev, B8_ADC_CON, ctrl  |= ADC_CTRL_FIFO_EN);

	/* next: valid Master, Standalone only. @@todo slave? */
	acq400wr32(adev, B8_ADC_CON, ctrl | ADC_CTRL_ADC_RST);
	acq400wr32(adev, B8_ADC_CON, ctrl);
}

void bolo8_onStop(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, B8_ADC_CON);
	acq400wr32(adev, B8_ADC_CON, ctrl & ~ADC_CTRL_ENABLE_CAPTURE);
}

#define DAC_COMPLETE(adev) \
		((acq400rd32(adev, B8_ODA_DATA)&B8_ODA_DATA_COMPLETE) != 0)


short bolo8_get_offset_dacN(struct acq400_dev *adev, int ix)
{
	u32 regval = B8_ODA_DATA_ADDR(ix)|B8_ODA_DATA_RDnW;
	short result;
	int pollcat = 0;

	acq400wr32(adev, B8_ODA_DATA, B8_ODA_DATA_RUN|regval);
	while(!DAC_COMPLETE(adev)){
		yield();
		if ((++pollcat&0xffff) == 0){
			dev_warn(DEVP(adev), "polling for SPI dac");
		}
	}
	result = acq400rd32(adev, B8_ODA_DATA)&0x0ffff;
	acq400wr32(adev, B8_ODA_DATA, regval);
	return result;
}
void bolo8_set_offset_dacN(struct acq400_dev *adev, int ix, short offset)
{

	u32 regval = B8_ODA_DATA_ADDR(ix)|(offset&0x0ffff);
	int pollcat = 0;

	acq400wr32(adev, B8_ODA_DATA, B8_ODA_DATA_RUN|regval);
	while(!DAC_COMPLETE(adev)){
		yield();
		if ((++pollcat&0xffff) == 0){
			dev_warn(DEVP(adev), "polling for SPI dac");
		}
	}
	acq400wr32(adev, B8_ODA_DATA, regval);
}
