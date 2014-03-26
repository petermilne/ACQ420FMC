/* ------------------------------------------------------------------------- *
 * acq400_debugfs.c  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 9 Mar 2014  
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
#include "acq400_debugfs.h"

struct dentry* acq400_debug_root;

#define DBG_REG_CREATE_NAME(name, reg) 				\
	sprintf(pcursor, "%s.0x%02x", name, reg);		\
	debugfs_create_x32(pcursor, S_IRUGO, 			\
		adev->debug_dir, adev->dev_virtaddr+(reg));     \
	pcursor += strlen(pcursor) + 1

#define DBG_REG_CREATE(reg) 					\
	sprintf(pcursor, "%s.0x%02x", #reg, reg);		\
	debugfs_create_x32(pcursor, S_IRUGO, 			\
		adev->debug_dir, adev->dev_virtaddr+(reg));     \
	pcursor += strlen(pcursor) + 1


void acq400_createDebugfs(struct acq400_dev* adev)
{
	char* pcursor;
	if (!acq400_debug_root){
		acq400_debug_root = debugfs_create_dir("acq400", 0);
		if (!acq400_debug_root){
			dev_warn(&adev->pdev->dev, "failed create dir acq420");
			return;
		}
	}
	pcursor = adev->debug_names = kmalloc(4096, GFP_KERNEL);


	adev->debug_dir = debugfs_create_dir(
			acq400_devnames[adev->of_prams.site], acq400_debug_root);

	if (!adev->debug_dir){
		dev_warn(&adev->pdev->dev, "failed create dir acq400.x");
		return;
	}
	DBG_REG_CREATE(MOD_ID);
	DBG_REG_CREATE(ADC_CTRL);
	DBG_REG_CREATE(TIM_CTRL);
	DBG_REG_CREATE(ADC_HITIDE);
	DBG_REG_CREATE(ADC_FIFO_SAMPLES);
	DBG_REG_CREATE(ADC_FIFO_STA);
	DBG_REG_CREATE(ADC_INT_CSR);
	DBG_REG_CREATE(ADC_CLK_CTR);
	DBG_REG_CREATE(ADC_SAMPLE_CTR);
	DBG_REG_CREATE(ADC_SAMPLE_CLK_CTR);

	DBG_REG_CREATE(ADC_CLKDIV);
	if (IS_ACQ420(adev)){
		DBG_REG_CREATE(ADC_GAIN);
		DBG_REG_CREATE(ADC_CONV_TIME);
	} else if (IS_ACQ43X(adev)){
		DBG_REG_CREATE(SW_EMB_WORD1);
		DBG_REG_CREATE(SW_EMB_WORD2);
		DBG_REG_CREATE(ACQ435_MODE);
	} else if (IS_AO420(adev)){
		DBG_REG_CREATE(AO420_RANGE);
		DBG_REG_CREATE(AO420_DACSPI);
	}


}

void acq400_removeDebugfs(struct acq400_dev* adev)
{
	debugfs_remove_recursive(adev->debug_dir);
	kfree(adev->debug_names);
}

void acq2006_createDebugfs(struct acq400_dev* adev)
{
	char* pcursor;
	int site;
	int sites = IS_ACQ2006SC(adev)? 6: IS_ACQ1001SC(adev)? 2: 0;
	if (!acq400_debug_root){
		acq400_debug_root = debugfs_create_dir("acq400", 0);
		if (!acq400_debug_root){
			dev_warn(&adev->pdev->dev, "failed create dir acq420");
			return;
		}
	}
	pcursor = adev->debug_names = kmalloc(4096, GFP_KERNEL);


	adev->debug_dir = debugfs_create_dir(
			acq400_devnames[adev->of_prams.site], acq400_debug_root);

	if (!adev->debug_dir){
		dev_warn(&adev->pdev->dev, "failed create dir acq400.x");
		return;
	}
	DBG_REG_CREATE(MOD_ID);
	DBG_REG_CREATE(MOD_CON);
	DBG_REG_CREATE(AGGREGATOR);
	DBG_REG_CREATE(AGGSTA);
	DBG_REG_CREATE(DATA_ENGINE_0);
	if (IS_ACQ2006SC(adev)){
		DBG_REG_CREATE(DATA_ENGINE_1);
		DBG_REG_CREATE(DATA_ENGINE_2);
		DBG_REG_CREATE(DATA_ENGINE_3);
	}
	if (IS_ACQ1001SC(adev)){
		DBG_REG_CREATE(GPG_CONTROL);
		DBG_REG_CREATE(HDMI_SYNC);
	}

	DBG_REG_CREATE_NAME("CLK_EXT", ACQ2006_CLK_COUNT(EXT_DX));
	DBG_REG_CREATE_NAME("CLK_MB",  ACQ2006_CLK_COUNT(MB_DX));
	for (site = 1; site <= sites; ++site){
		char name[20];
		sprintf(name, "CLK_%d", site);
		DBG_REG_CREATE_NAME(name, ACQ2006_CLK_COUNT(SITE2DX(site)));
	}

	DBG_REG_CREATE_NAME("TRG_EXT", ACQ2006_TRG_COUNT(EXT_DX));
	DBG_REG_CREATE_NAME("TRG_MB",  ACQ2006_TRG_COUNT(MB_DX));
	for (site = 1; site <= sites; ++site){
		char name[20];
		sprintf(name, "TRG_%d", site);
		DBG_REG_CREATE_NAME(name, ACQ2006_TRG_COUNT(SITE2DX(site)));
	}

	DBG_REG_CREATE_NAME("SYN_EXT", ACQ2006_SYN_COUNT(EXT_DX));
	DBG_REG_CREATE_NAME("SYN_MB",  ACQ2006_SYN_COUNT(MB_DX));
	for (site = 1; site <= sites; ++site){
		char name[20];
		sprintf(name, "SYN_%d", site);
		DBG_REG_CREATE_NAME(name, ACQ2006_SYN_COUNT(SITE2DX(site)));
	}

	DBG_REG_CREATE_NAME("EVT_EXT", ACQ2006_EVT_COUNT(EXT_DX));
	DBG_REG_CREATE_NAME("EVT_MB",  ACQ2006_EVT_COUNT(MB_DX));
	for (site = 1; site <= sites; ++site){
		char name[20];
		sprintf(name, "EVT_%d", site);
		DBG_REG_CREATE_NAME(name, ACQ2006_EVT_COUNT(SITE2DX(site)));
	}

	if (IS_ACQ1001SC(adev) || (IS_ACQ2006SC(adev)&&FPGA_REV(adev) >= 8)){
		char name[16];
		int ii;
		for (ii = 0; ii < SPADMAX; ++ii){
			snprintf(name, 16, "spad%d", ii);
			DBG_REG_CREATE_NAME(name, SPADN(ii));
		}
	}
}
