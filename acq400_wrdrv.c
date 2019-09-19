/* ------------------------------------------------------------------------- */
/* acq400_wrdrv.c  D-TACQ ACQ400 White Rabbit  DRIVER	                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2019 Peter Milne, D-TACQ Solutions Ltd                    *
 *  This program is free software; you can redistribute it and/or modify     *
 *  it under the terms of Version 2 of the GNU General Public License        *
 *  as published by the Free Software Foundation;                            *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program; if not, write to the Free Software              *
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/*
 * acq400_wrdrv.c
 *
 *  Created on: 19 Sep 2019
 *      Author: pgm
 */


#include "acq400.h"
#include "bolo.h"
#include "hbm.h"
#include "acq400_debugfs.h"
#include "acq400_lists.h"
#include "acq400_ui.h"



int wr_ts_inten = 1;
module_param(wr_ts_inten, int, 0444);
MODULE_PARM_DESC(nbuffers, "1: enable Time Stamp interrupts");

int wr_pps_inten = 1;
module_param(wr_pps_inten, int, 0444);
MODULE_PARM_DESC(nbuffers, "1: enable PPS interrupts");



static u32 wr_ctrl_set(struct acq400_dev *adev, unsigned bits){
	u32 int_sta = acq400rd32(adev, WR_CTRL);
	int_sta |= bits;
	acq400wr32(adev, WR_CTRL, int_sta);
}
static u32 wr_ctrl_clr(struct acq400_dev *adev, unsigned bits){
	u32 int_sta = acq400rd32(adev, WR_CTRL);
	int_sta &= ~bits;
	acq400wr32(adev, WR_CTRL, int_sta);
}


static irqreturn_t wr_ts_isr(int irq, void *dev_id)
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;
	struct acq400_sc_dev* sc_dev = container_of(adev, struct acq400_sc_dev, adev);

	u32 int_sta = acq400rd32(adev, WR_CTRL);
	u32 ts = acq400rd32(adev, WR_TAI_STAMP);

	acq400wr32(adev, WR_CTRL, int_sta);

	return IRQ_WAKE_THREAD;	/* canned */
}

static irqreturn_t wr_ts_kthread(int irq, void *dev_id)
/* keep the AO420 FIFO full. Recycle buffer only */
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;
	struct acq400_sc_dev* sc_dev = container_of(adev, struct acq400_sc_dev, adev);

	return IRQ_HANDLED;
}

static irqreturn_t wr_pps_isr(int irq, void *dev_id)
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;
	struct acq400_sc_dev* sc_dev = container_of(adev, struct acq400_sc_dev, adev);

	u32 int_sta = acq400rd32(adev, WR_CTRL);

	acq400wr32(adev, WR_CTRL, int_sta);

	return IRQ_WAKE_THREAD;	/* canned */
}

static irqreturn_t wr_pps_kthread(int irq, void *dev_id)
/* keep the AO420 FIFO full. Recycle buffer only */
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;
	struct acq400_sc_dev* sc_dev = container_of(adev, struct acq400_sc_dev, adev);

	return IRQ_HANDLED;
}



int acq400_wr_init_irq(struct acq400_dev* adev)
{
	int rc;
	int irq = platform_get_irq(adev->pdev, IRQ_REQUEST_OFFSET);
	if (irq <= 0){
		return 0;
	}
	dev_info(DEVP(adev), "acq400_wr_init_irq %d", irq);

	rc = devm_request_threaded_irq(
			DEVP(adev), irq, wr_ts_isr, wr_ts_kthread, IRQF_NO_THREAD,
			"wr_ts",	adev);
	if (rc){
		dev_err(DEVP(adev),"unable to get IRQ %d K414 KLUDGE IGNORE\n", irq);
		return 0;
	}

	irq = platform_get_irq(adev->pdev, IRQ_REQUEST_OFFSET+1);
	if (irq <= 0){
		return 0;
	}
	if (wr_ts_inten){
		wr_ctrl_set(adev, WR_CTRL_TS_INTEN);
	}
	dev_info(DEVP(adev), "acq400_wr_init_irq %d", irq);

	rc = devm_request_threaded_irq(
			DEVP(adev), irq, wr_pps_isr, wr_pps_kthread, IRQF_NO_THREAD,
			"wr_pps",	adev);
	if (rc){
		dev_err(DEVP(adev),"unable to get IRQ %d K414 KLUDGE IGNORE\n", irq);
		return 0;
	}
	if (wr_pps_inten){
		wr_ctrl_set(adev, WR_CTRL_PPS_INTEN);
	}
	return rc;
}
