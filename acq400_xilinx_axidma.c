/* ------------------------------------------------------------------------- */
/* acq400_xilinx_axidma.c ACQ420_FMC					     */
/*
 * acq400_xilinx_axidma.c
 *
 *  Created on: 10 Nov 2015
 *      Author: pgm
 */

/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2015 Peter Milne, D-TACQ Solutions Ltd                    *
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
#include "bolo.h"
#include "hbm.h"
#include "acq400_debugfs.h"
#include "acq400_lists.h"

#include "dmaengine.h"
#include <linux/amba/xilinx_dma.h>
#include "xilinx_axidma.h"

extern int AXI_BUFFER_COUNT;
extern int AXI_ONESHOT;
extern int bufferlen;

extern unsigned AXI_HEAD_DESCR_PA;
extern unsigned AXI_TAIL_DESCR_PA;

#define S2MM_DMACR_CYC 0x10

void axi64_arm_dmac(struct xilinx_dma_chan *xchan, unsigned headpa, unsigned tailpa, unsigned oneshot)
{
	unsigned cr = dma_read(xchan, XILINX_DMA_CONTROL_OFFSET);
	dev_dbg(xchan->dev, "axi64_arm_dmac() 01");
	dma_write(xchan, XILINX_DMA_CONTROL_OFFSET, cr |= XILINX_DMA_CR_RESET_MASK);
	dma_write(xchan, XILINX_DMA_CONTROL_OFFSET, cr = 0);
	dma_write(xchan, XILINX_DMA_CDESC_OFFSET, headpa);
	if (!oneshot){
		dma_write(xchan, XILINX_DMA_CONTROL_OFFSET, cr = S2MM_DMACR_CYC);
	}
	dma_write(xchan, XILINX_DMA_CONTROL_OFFSET, cr|XILINX_DMA_CR_RUNSTOP_MASK);
	dma_write(xchan, XILINX_DMA_TDESC_OFFSET, tailpa);
	dev_dbg(xchan->dev, "axi64_arm_dmac() 99");
}
int axi64_load_dmac(struct acq400_dev *adev)
{
	char _nbuffers[8];
	char _bufferlen[16];
	char _oneshot[4];
	char *argv[] = {
		"/usr/local/bin/acq400_axi_dma_test_harness",
		NULL, NULL, NULL,
		NULL
	};
	static char *envp[] = {
			"HOME=/",
			"TERM=linux",
			"PATH=/sbin:/bin:/usr/sbin:/usr/bin", NULL
	};
	struct xilinx_dma_chan *xchan = to_xilinx_chan(adev->dma_chan[0]);
	u32* dregs = xchan->regs;
	int rc;

	dev_dbg(DEVP(adev), "regs:%p : %08x %08x %08x %08x\n", dregs,
			dma_read(xchan, 0), dma_read(xchan, 0x4),
			dma_read(xchan, 0x8), dma_read(xchan, 0xc)
			);
	sprintf(_nbuffers, "%d", AXI_BUFFER_COUNT);	argv[1] = _nbuffers;
	sprintf(_bufferlen, "%d", bufferlen);		argv[2] = _bufferlen;
	sprintf(_oneshot,   "%d", AXI_ONESHOT!=0);	argv[3] = _oneshot;

	AXI_HEAD_DESCR_PA = AXI_TAIL_DESCR_PA = 0;
	dev_info(DEVP(adev), "axi64_load_dmac() spawn %s %s %s %s",
					argv[0], argv[1], argv[2], argv[3]);
	rc = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);

	if (rc != 0){
		dev_warn(DEVP(adev), "helper function %s returned %d", argv[0], rc);
	}

	if (AXI_HEAD_DESCR_PA != 0 && AXI_TAIL_DESCR_PA != 0){
		axi64_arm_dmac(xchan, AXI_HEAD_DESCR_PA, AXI_TAIL_DESCR_PA, AXI_ONESHOT);
		return 0;
	}else{
		dev_err(DEVP(adev), "AXI_HEAD_DESCR_PA && AXI_TAIL_DESCR_PA not set");
		return -1;
	}
}
