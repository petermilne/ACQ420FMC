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


static int axi_s16_set(void *data, u64 val)
/* we'd need a shadow for this .. ignore for now */
{
	return -1;
}
static int axi_s16_get(void *data, u64 *val)
{
	u32 addr = (u32)data;
	u32 v32 = *(u32*)(addr& ~3);
	if (addr&3){
		*val = v32 & 0x0000ffff;
	}else{
		*val = v32 >> 16;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_axi_s16, axi_s16_get, axi_s16_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_axi_s16_ro, axi_s16_get, NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_axi_s16_wo, NULL, axi_s16_set, "%lld\n");


/**
 * debugfs_create_u16 - create a debugfs file that is used to read and write an unsigned 16-bit value
 * @name: a pointer to a string containing the name of the file to create.
 * @mode: the permission that the file should have
 * @parent: a pointer to the parent dentry for this file.  This should be a
 *          directory dentry if set.  If this parameter is %NULL, then the
 *          file will be created in the root of the debugfs filesystem.
 * @value: a pointer to the variable that the file should read to and write
 *         from.
 *
 * This function creates a file in debugfs with the given name that
 * contains the value of the variable @value.  If the @mode variable is so
 * set, it can be read from, and written to.
 *
 * This function will return a pointer to a dentry if it succeeds.  This
 * pointer must be passed to the debugfs_remove() function when the file is
 * to be removed (no automatic cleanup happens if your module is unloaded,
 * you are responsible here.)  If an error occurs, %NULL will be returned.
 *
 * If debugfs is not enabled in the kernel, the value -%ENODEV will be
 * returned.  It is not wise to check for this value, but rather, check for
 * %NULL or !%NULL instead as to eliminate the need for #ifdef in the calling
 * code.
 */
struct dentry *debugfs_create_axi_s16(const char *name, umode_t mode,
				  struct dentry *parent, u16 *value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_axi_s16_ro);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_axi_s16_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_axi_s16);
}
EXPORT_SYMBOL_GPL(debugfs_create_axi_s16);


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

#define CH_REG_CREATE(name, reg)				\
	sprintf(pcursor, "%s", #name);				\
	debugfs_create_axi_s16(pcursor, S_IRUGO,		\
		chdir, adev->dev_virtaddr+(reg));     		\
	pcursor += strlen(pcursor) + 1

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
	DBG_REG_CREATE(B8_ADC_FIFO_STA);
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

void ao420_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	DBG_REG_CREATE(DAC_CTRL);
	DBG_REG_CREATE(TIM_CTRL);
	DBG_REG_CREATE(DAC_LOTIDE);
	DBG_REG_CREATE(DAC_FIFO_SAMPLES);
	DBG_REG_CREATE(DAC_FIFO_STA);
	DBG_REG_CREATE(DAC_INT_CSR);
	DBG_REG_CREATE(DAC_CLK_CTR);
	DBG_REG_CREATE(DAC_SAMPLE_CTR);
	DBG_REG_CREATE(DAC_CLKDIV);
	DBG_REG_CREATE(AO420_RANGE);
	DBG_REG_CREATE(AO420_DACSPI);
}

void adc_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
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
}

void acq420_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	adc_createDebugfs(adev, pcursor);
	DBG_REG_CREATE(ADC_GAIN);
	DBG_REG_CREATE(ADC_CONV_TIME);
}

void acq43x_createDebugfs(struct acq400_dev* adev, char* pcursor)
{
	adc_createDebugfs(adev, pcursor);
	DBG_REG_CREATE(SW_EMB_WORD1);
	DBG_REG_CREATE(SW_EMB_WORD2);
	DBG_REG_CREATE(EVT_SC_LATCH);
	DBG_REG_CREATE(ACQ435_MODE);
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

	switch(GET_MOD_ID(adev)){
	case MOD_ID_BOLO8:
		bolo8_createDebugfs(adev, pcursor);
		break;
	case MOD_ID_AO420FMC:
		ao420_createDebugfs(adev, pcursor);
		break;
	case MOD_ID_ACQ420FMC:
	case MOD_ID_ACQ420FMC_2000:
		acq420_createDebugfs(adev, pcursor);
		break;
	case MOD_ID_ACQ435ELF:
	case MOD_ID_ACQ430FMC:
		acq43x_createDebugfs(adev, pcursor);
		break;
	default:
		dev_warn(&adev->pdev->dev, "unsupported MOD_ID:%02x",
				GET_MOD_ID(adev));
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
//	if (IS_ACQ1001SC(adev)){
	DBG_REG_CREATE(GPG_CONTROL);
	DBG_REG_CREATE(HDMI_SYNC_DAT);
	DBG_REG_CREATE(HDMI_SYNC_OUT_SRC);
	DBG_REG_CREATE(EVT_BUS_SRC);
	DBG_REG_CREATE(SIG_SRC_ROUTE);
//	}

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
