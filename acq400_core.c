/* ------------------------------------------------------------------------- */
/* acq400_core.c  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 29 Jul 2017  			/ User: pgm
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2017 Peter Milne, D-TACQ Solutions Ltd         *
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                *
 *
 * TODO 
 * TODO
 * ------------------------------------------------------------------------- */

#include "acq400.h"
#include "bolo.h"
#include "hbm.h"
#include "acq400_debugfs.h"
#include "acq400_lists.h"

int debcount;
module_param(debcount, int, 0644);
MODULE_PARM_DESC(debcount, "NZ if counter debounce ever .. happened");

int modify_spad_access;
module_param(modify_spad_access, int, 0644);
MODULE_PARM_DESC(modify_spad_access,
"-1: access DRAM instead to trace spad fault, 1: force read before, 2: force read after");


void acq400wr32(struct acq400_dev *adev, int offset, u32 value)
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

u32 acq400rd32(struct acq400_dev *adev, int offset)
{
	u32 rc = ioread32(adev->dev_virtaddr + offset);
	if (adev->RW32_debug > 1){
		dev_info(DEVP(adev), "acq400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	}else{
		dev_dbg(DEVP(adev), "acq400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	}
	return rc;
}


void write32(volatile u32* to, volatile u32* from, int nwords)
{
	int ii;

	for (ii = 0; ii < nwords; ++ii){
		iowrite32(from[ii], to);
	}
}

int xo400_write_fifo(struct acq400_dev* adev, int frombyte, int bytes)
{
	unsigned len = adev->cursor.hb[0]->len;
	unsigned ib = frombyte/len;
	unsigned offset = frombyte - ib*len;
	/* WARNING: assumes NEVER overlaps HB end .. */
	dev_dbg(DEVP(adev), "write32(%p = %p+%d, %d",
			adev->dev_virtaddr+AXI_FIFO,
			adev->cursor.hb[ib]->va,
			offset/sizeof(u32),
			bytes/sizeof(u32)
	);

	write32(adev->dev_virtaddr+AXI_FIFO,
		adev->cursor.hb[ib]->va+offset/sizeof(u32),
		bytes/sizeof(u32));
	return bytes;
}

u32 acq400rd32_upcount(struct acq400_dev *adev, int offset)
{
	u32 c1 = acq400rd32(adev, offset);
	u32 c2;

	while((c2 = acq400rd32(adev, offset)) < c1){
		c1 = c2;
		++debcount;
	}
	return c2;
}

void set_spadN(struct acq400_dev* adev, int n, u32 value)
{
	if (modify_spad_access == -1){
		adev->fake_spad[n] = value;
	}else{
		if (modify_spad_access > 0){
			acq400rd32(adev, SPADN(n));
		}
		acq400wr32(adev, SPADN(n), value);
		if (modify_spad_access > 1){
			u32 v2 = acq400rd32(adev, SPADN(n));
			if (v2 != value){
				dev_warn(DEVP(adev),
					"spadN fail: w:%08x r:%08x", value, v2);
			}
		}
	}
}
u32 get_spadN(struct acq400_dev* adev, int n)
{
	if (modify_spad_access){
		return adev->fake_spad[n];
	}else{
		return acq400rd32(adev, SPADN(n));
	}
}

static bool filter_true(struct dma_chan *chan, void *param)
{
	return true;
}


static int _get_dma_chan(struct acq400_dev *adev, int ic)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	adev->dma_chan[ic] = dma_request_channel(mask, filter_true, NULL);
	if (adev->dma_chan[ic] == 0){
		dev_err(DEVP(adev), "%p id:%d dma_find_channel set zero",
					adev, adev->pdev->dev.id);
	}else{
		dev_dbg(DEVP(adev), "dma chan[%d] selected: %d",
				ic, adev->dma_chan[ic]->chan_id);
	}
	return adev->dma_chan[ic] == 0 ? -1: 0;
}

extern int xo_use_distributor;		/* REMOVE ME */

int get_dma_channels(struct acq400_dev *adev)
{
	if (IS_AXI64(adev)){
		return 0;
	}else if (IS_AO42X(adev) || IS_DIO432X(adev)){
		if (xo_use_distributor){
			int rc = _get_dma_chan(adev, 0) || _get_dma_chan(adev, 1);
			return rc;
		}else{
			return _get_dma_chan(adev, 0);
		}
	}else{
		int rc = _get_dma_chan(adev, 0) || _get_dma_chan(adev, 1);
		return rc;
	}
}

static void _release_dma_chan(struct acq400_dev *adev, int ic)
{
	if (adev->dma_chan[ic]){
		dma_release_channel(adev->dma_chan[ic]);
		adev->dma_chan[ic] = 0;
	}
}
void release_dma_channels(struct acq400_dev *adev)
{
	if (!IS_AXI64(adev)){
		_release_dma_chan(adev, 0);
		_release_dma_chan(adev, 1);
	}
}

void acq400_clear_histo(struct acq400_dev *adev)
{
	memset(adev->fifo_histo, 0, FIFO_HISTO_SZ*sizeof(u32));
}

void add_fifo_histo(struct acq400_dev *adev, u32 status)
{
	adev->fifo_histo[STATUS_TO_HISTO(status)]++;
}

void add_fifo_histo_ao42x(struct acq400_dev *adev, unsigned samples)
{
	(adev->fifo_histo[samples >> adev->xo.hshift])++;
}


int check_fifo_statuses(struct acq400_dev *adev)
{
	if (adev->is_sc){
		int islave = 0;
		for (islave = 0; islave < MAXSITES; ++islave){
			struct acq400_dev* slave = adev->aggregator_set[islave];
			if (!slave) break;

			if (slave->isFifoError(slave)){
				dev_err(DEVP(adev), "FIFO ERROR slave %d", SITE(*slave));
				slave->rt.refill_error = true;
				goto fail;
			} else if (IS_ACQ480(adev) && adev->rt.nget != 0 && acq480_train_fail(slave) == 1){
				dev_err(DEVP(adev), "LINK TRAINING ERROR slave %d", SITE(*slave));
				if (acq480_train_fail(slave) == 1){
					dev_err(DEVP(adev), "LINK TRAINING ERROR slave %d 2nd strike", SITE(*slave));
					slave->rt.refill_error = true;
					goto fail;
				}
			} else {
				continue;
			}
		}
	}else{
		return adev->isFifoError(adev);
	}
	return 0;
fail:
	adev->rt.refill_error = true;
	dev_err(DEVP(adev), "ERROR: quit on FIFERR set refill_error");
	wake_up_interruptible_all(&adev->refill_ready);
	return 1;
}


void go_rt(int prio)
{
	struct task_struct *task = current;
	struct sched_param param = { };

	param.sched_priority = prio;
	sched_setscheduler(task, SCHED_FIFO, &param);
}

void acq400_set_peripheral_SPI_CS(unsigned csword)
{
	struct acq400_dev* adev = acq400_devices[0];

	dev_dbg(DEVP(adev), "acq400_set_peripheral_SPI_CS() %08x\n", csword);
	acq400wr32(adev, SPI_PERIPHERAL_CS, csword);
}

EXPORT_SYMBOL_GPL(acq400_set_peripheral_SPI_CS);

