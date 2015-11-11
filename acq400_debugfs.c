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
#include "bolo.h"
#include "acq400_debugfs.h"
#include "acq400_debugfs_internal.h"


void bolo8_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	struct dentry* chdir = debugfs_create_dir("current", adev->debug_dir);

	DBG_REG_CREATE(B8_SYS_CON);
	DBG_REG_CREATE(B8_TIM_CON);
	DBG_REG_CREATE(B8_INT_CSR);
	DBG_REG_CREATE(B8_CLK_CTR);
	DBG_REG_CREATE(B8_CLKDIV);
	DBG_REG_CREATE(B8_ADC_CON);
	DBG_REG_CREATE(B8_ADC_HITIDE);
	DBG_REG_CREATE(B8_ADC_FIFO_CNT);
	DBG_REG_CREATE_RW(B8_ADC_FIFO_STA);
	DBG_REG_CREATE(B8_ADC_SAMPLE_CNT);
	DBG_REG_CREATE(B8_DAC_CON);
	DBG_REG_CREATE(B8_DAC_WAVE_TOP);
	DBG_REG_CREATE(B8_DAC_FIFO_STA);
	DBG_REG_CREATE(B8_DAC_SAMPLE_CNT);
	DBG_REG_CREATE(B8_DAC_SPI);
	DBG_REG_CREATE(B8_DAC_SPI_RBK);
/* Current ADc */
	DBG_REG_CREATE(B8_CAD_CON);
	DBG_REG_CREATE(B8_CAD_DELAY);

	CH_REG_CREATE(1, B8_CAD_1);
	CH_REG_CREATE(2, B8_CAD_2);
	CH_REG_CREATE(3, B8_CAD_3);
	CH_REG_CREATE(4, B8_CAD_4);
	CH_REG_CREATE(5, B8_CAD_5);
	CH_REG_CREATE(6, B8_CAD_6);
	CH_REG_CREATE(7, B8_CAD_7);
	CH_REG_CREATE(8, B8_CAD_8);
	CH_REG_CREATE(A1, B8_CAD_A1);
	CH_REG_CREATE(A2, B8_CAD_A2);
	CH_REG_CREATE(A3, B8_CAD_A3);
	CH_REG_CREATE(A4, B8_CAD_A4);
	CH_REG_CREATE(A5, B8_CAD_A5);
	CH_REG_CREATE(A6, B8_CAD_A6);
	CH_REG_CREATE(A7, B8_CAD_A7);
	CH_REG_CREATE(A8, B8_CAD_A8);
/* Offset DAc */
	DBG_REG_CREATE(B8_ODA_CON);
	DBG_REG_CREATE(B8_ODA_DATA);
}

/* ao424 ch knobs: working with a cache buffer, use regular accessor */
char * _ao424_create_ch_knobs(
	struct acq400_dev* adev, const char* name, u16 values[],
	char* pcursor)
{
	struct dentry* dir = debugfs_create_dir(name, adev->debug_dir);
	int ic;

	for (ic = 0; ic < AO424_MAXCHAN; ++ic){
		int nc = sprintf(pcursor, "%02d", ic+1);
		debugfs_create_x16(pcursor, S_IWUGO|S_IRUGO, dir, values+ic);
		pcursor += nc+1;
	}


	return pcursor;
}
void ao424_create_spans(struct acq400_dev* adev, char* pcursor)
{
	pcursor = _ao424_create_ch_knobs(adev, "spans",
			adev->ao424_device_settings.u.ch.ao424_spans, pcursor);
	pcursor = _ao424_create_ch_knobs(adev, "init",
			adev->ao424_device_settings.u.ch.ao424_initvals, pcursor);
}
void ao420_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	DBG_REG_CREATE(DAC_CTRL);
	DBG_REG_CREATE(TIM_CTRL);
	DBG_REG_CREATE(DAC_LOTIDE);
	DBG_REG_CREATE(DAC_FIFO_SAMPLES);
	DBG_REG_CREATE_RW(DAC_FIFO_STA);
	DBG_REG_CREATE(DAC_INT_CSR);
	DBG_REG_CREATE(DAC_CLK_CTR);
	DBG_REG_CREATE(DAC_SAMPLE_CTR);
	DBG_REG_CREATE(DAC_CLKDIV);
	if (IS_AO420(adev)){
		DBG_REG_CREATE(AO420_RANGE);
		DBG_REG_CREATE(AO420_DACSPI);
		DBG_REG_CREATE(DAC_GAIN_OFF(1));
		DBG_REG_CREATE(DAC_GAIN_OFF(2));
		DBG_REG_CREATE(DAC_GAIN_OFF(3));
		DBG_REG_CREATE(DAC_GAIN_OFF(4));
		DBG_REG_CREATE(DAC_GAIN_OFF(32));
	}
	if (IS_AO424(adev)){
		DBG_REG_CREATE(DAC_424_CGEN);
		DBG_REG_CREATE(AO424_DELAY);
		ao424_create_spans(adev, pcursor);
	}
}

void adc_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	DBG_REG_CREATE(ADC_CTRL);
	DBG_REG_CREATE(TIM_CTRL);
	DBG_REG_CREATE(ADC_HITIDE);
	DBG_REG_CREATE(ADC_FIFO_SAMPLES);
	DBG_REG_CREATE_RW(ADC_FIFO_STA);
	DBG_REG_CREATE(ADC_INT_CSR);
	DBG_REG_CREATE(ADC_CLK_CTR);
	DBG_REG_CREATE(ADC_SAMPLE_CTR);
	DBG_REG_CREATE(ADC_SAMPLE_CLK_CTR);
	DBG_REG_CREATE(ADC_CLKDIV);
	DBG_REG_CREATE(ADC_ACC_DEC);
}

void acq420_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	adc_createDebugfs(adev, pcursor);
	DBG_REG_CREATE(ADC_GAIN);
	DBG_REG_CREATE(ADC_CONV_TIME);
}

void pmodadc1_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	adc_createDebugfs(adev, pcursor);
}
void acq43x_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	adc_createDebugfs(adev, pcursor);
	DBG_REG_CREATE(SW_EMB_WORD1);
	DBG_REG_CREATE(SW_EMB_WORD2);
	DBG_REG_CREATE(EVT_SC_LATCH);
	DBG_REG_CREATE(ACQ435_MODE);
	DBG_REG_CREATE(ADC_TRANSLEN);
	DBG_REG_CREATE(ATD_TRIGGERED);
	DBG_REG_CREATE(ATD_MASK_AND);
	DBG_REG_CREATE(ATD_MASK_OR);
}

void dio432_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	DBG_REG_CREATE( DIO432_MOD_ID		);
	DBG_REG_CREATE( DIO432_DIO_CTRL		);
	DBG_REG_CREATE( DIO432_TIM_CTRL		);
	DBG_REG_CREATE( DIO432_DI_HITIDE	);
	DBG_REG_CREATE( DIO432_DI_FIFO_COUNT	);
	DBG_REG_CREATE_RW( DIO432_DI_FIFO_STATUS);
	DBG_REG_CREATE( DIO432_DIO_ICR		);
	DBG_REG_CREATE( DIO432_DIO_SAMPLE_COUNT );
	DBG_REG_CREATE( DIO432_DIO_CPLD_CTRL	);
	DBG_REG_CREATE( DIO432_DO_LOTIDE	);
	DBG_REG_CREATE( DIO432_DO_FIFO_COUNT	);
	DBG_REG_CREATE( DIO432_DO_FIFO_STATUS	);
}

void v2f_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	DBG_REG_CREATE( V2F_CTRL );
	DBG_REG_CREATE( V2F_STAT );
	DBG_REG_CREATE( V2F_CHAN_SEL );
	DBG_REG_CREATE( V2F_FREQ_OFF );
}
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

	if (IS_ACQ42X(adev)){
		acq420_createDebugfs(adev, pcursor);
	}else if (IS_DIO432X(adev)){
		dio432_createDebugfs(adev, pcursor);
	}else{
		switch(GET_MOD_ID(adev)){
		case MOD_ID_BOLO8:
		case MOD_ID_BOLO8B:
			bolo8_createDebugfs(adev, pcursor);
			break;
		case MOD_ID_AO420FMC:
		case MOD_ID_AO424ELF:
			ao420_createDebugfs(adev, pcursor);
			break;
		case MOD_ID_ACQ435ELF:
		case MOD_ID_ACQ437ELF:
		case MOD_ID_ACQ430FMC:
			acq43x_createDebugfs(adev, pcursor);
			break;
		case MOD_ID_ACQ480FMC:
			adc_createDebugfs(adev, pcursor);
			break;
		case MOD_ID_PMODADC1:
			pmodadc1_createDebugfs(adev, pcursor);
			break;
		case MOD_ID_ACQ400T_FMC:
		case MOD_ID_ACQ400T_ELF:
			break;
		case MOD_ID_V2F:
			v2f_createDebugfs(adev, pcursor);
			break;
		default:
			dev_warn(&adev->pdev->dev, "unsupported MOD_ID:%02x",
				GET_MOD_ID(adev));
		}
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
	int sites = IS_ACQ2X06SC(adev)? 6: IS_ACQ1001SC(adev)? 2: 0;
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
	if (IS_ACQ2X06SC(adev)){
		DBG_REG_CREATE(DATA_ENGINE_1);
		if (IS_AXI64(adev)){
			DBG_REG_CREATE(AXI_DMA_ENGINE_DATA);
		}else{
			DBG_REG_CREATE(DATA_ENGINE_2);
			DBG_REG_CREATE(DATA_ENGINE_3);
		}
	}

	DBG_REG_CREATE(GPG_CONTROL);
	DBG_REG_CREATE(HDMI_SYNC_DAT);
	DBG_REG_CREATE(HDMI_SYNC_OUT_SRC);
	DBG_REG_CREATE(EVT_BUS_SRC);
	DBG_REG_CREATE(SIG_SRC_ROUTE);
	DBG_REG_CREATE(FPCTL);
	DBG_REG_CREATE(GPG_DEBUG);

	DBG_REG_CREATE(DISTRIBUTOR);
	DBG_REG_CREATE(DISTRIBUTOR_STA);

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

	if (IS_ACQ1001SC(adev) || IS_ACQ2106SC(adev) || (IS_ACQ2006SC(adev)&&FPGA_REV(adev) >= 8)){
		char name[16];
		int ii;
		for (ii = 0; ii < SPADMAX; ++ii){
			snprintf(name, 16, "spad%d", ii);
			DBG_REG_CREATE_NAME(name, SPADN(ii));
		}
	}
}
