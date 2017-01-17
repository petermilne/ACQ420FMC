/* ------------------------------------------------------------------------- */
/* acq400_drv.c  D-TACQ ACQ400 FMC  DRIVER		                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2013 Peter Milne, D-TACQ Solutions Ltd                    *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
 *                                                                           *
 *  This program is free software; you can redistribute it and/or modify     *
 *  it under the terms of Version 2 of the GNU General Public License        *
 *  as published by the Free Software Foundation;                            *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
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

#define REVID "3.150"

/* Define debugging for use during our driver bringup */
#undef PDEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO fmt, ## args)


/* MODULE PARAMETERS */


int ndevices;
module_param(ndevices, int, 0444);
MODULE_PARM_DESC(ndevices, "number of devices found in probe");

int adc_18b = 0;
module_param(adc_18b, int, 0444);
MODULE_PARM_DESC(adc_18b, "set TRUE on load if 18 bit devices fitted [1]");

int data_32b = 0;
module_param(data_32b, int, 0444);
MODULE_PARM_DESC(data_32b, "set TRUE on load if 32 bit data required [0]");

int adc_conv_time = ADC_CONV_TIME_1000;
module_param(adc_conv_time, int, 0444);
MODULE_PARM_DESC(adc_conv_time, "hardware tweak, change at load only");

int acq424_adc_conv_time = 0x2d;
module_param(acq424_adc_conv_time, int, 0444);

int nbuffers = 64;
module_param(nbuffers, int, 0444);
MODULE_PARM_DESC(nbuffers, "number of capture buffers");

int bufferlen = 0x10000;
module_param(bufferlen, int, 0444);
MODULE_PARM_DESC(bufferlen, "length of capture buffer");

int frontside_bufferlen = 0x10000;
module_param(frontside_bufferlen, int, 0444);

int hitide = HITIDE;
module_param(hitide, int, 0644);
MODULE_PARM_DESC(hitide, "hitide value (words)");

int lotide = HITIDE-4;
module_param(lotide, int, 0644);
MODULE_PARM_DESC(lotide, "lotide value (words)");


int FIFERR = ADC_FIFO_STA_ERR;
module_param(FIFERR, int, 0644);
MODULE_PARM_DESC(FIFERR, "fifo status flags considered ERROR");

int act_on_fiferr = 1;
module_param(act_on_fiferr, int, 0644);
MODULE_PARM_DESC(act_on_fiferr, "0: log, don't act. 1: abort on error");

int debcount;
module_param(debcount, int, 0644);
MODULE_PARM_DESC(debcount, "NZ if counter debounce ever .. happened");

int maxdma = MAXDMA;
module_param(maxdma, int, 0644);
MODULE_PARM_DESC(maxdma, "set maximum DMA len bytes");

int agg_reset_dbg = 0;
module_param(agg_reset_dbg, int, 0644);

int quit_on_buffer_exhaustion;
module_param(quit_on_buffer_exhaustion, int, 0644);
MODULE_PARM_DESC(quit_on_buffer_exhaustion, "abort capture when out of buffers");

int modify_spad_access;
module_param(modify_spad_access, int, 0644);
MODULE_PARM_DESC(modify_spad_access,
"-1: access DRAM instead to trace spad fault, 1: force read before, 2: force read after");

int dio432_rowback = 256/4;
module_param(dio432_rowback, int, 0644);
MODULE_PARM_DESC(dio432_rowback, "stop short filling FIFO by this much");


int AO420_MAX_FILL_BLOCK = 16384;
module_param(AO420_MAX_FILL_BLOCK, int, 0644);

int BQ_LEN_SHL = 2;
module_param(BQ_LEN_SHL, int, 0644);

int hb0_no_ratelimit = 0;
module_param(hb0_no_ratelimit, int, 0644);

/* PIG TEST. SLAVE has non zero settings */
int sideport_does_not_touch_trg = 0;
module_param(sideport_does_not_touch_trg, int, 0644);

int default_dma_direction = DMA_FROM_DEVICE;
module_param(default_dma_direction, int , 0644);
MODULE_PARM_DESC(default_dma_direction, "set=1 for XO only device");

int xo_use_bigbuf = 0;
module_param(xo_use_bigbuf, int, 0644);
MODULE_PARM_DESC(xo_use_bigbuf, "set=1 if ONLY XO in box, then use bb to load long AWG");

int xo_use_contiguous_pa_if_possible = 0;
module_param(xo_use_contiguous_pa_if_possible, int, 0644);
MODULE_PARM_DESC(xo_use_contiguous_pa_if_possible, "set=1 to roll forward into next HB if contiguous");

int measure_ao_fifo_ok = 1;
module_param(measure_ao_fifo_ok, int, 0644);
MODULE_PARM_DESC(measure_ao_fifo_ok, "stubs ao fifo measure, cause of blowout on ao428");

int distributor_first_buffer = 0;
module_param(distributor_first_buffer, int, 0644);
MODULE_PARM_DESC(distributor_first_buffer, "use in mixed aggregator/distributor systems to avoid buffer overlap");

/* GLOBALS */

/* driver supports multiple devices.
 * ideally we'd have no globals here at all, but it works, for now
 */

/* index from 0, including site 0 */
struct acq400_dev* acq400_devices[MAXDEVICES+1];

#define DMA_NS_MAX     40
int dma_ns_lines[DMA_NS_MAX];
int dma_ns[DMA_NS_MAX];
int dma_ns_num = DMA_NS_MAX;
module_param_array(dma_ns, int, &dma_ns_num, 0444);
module_param_array(dma_ns_lines, int, &dma_ns_num, 0444);


int good_sites[MAXDEVICES];
int good_sites_count = 0;
module_param_array(good_sites, int, &good_sites_count, 0444);

int ao420_dma_threshold = MIN_DMA_BYTES;
module_param(ao420_dma_threshold, int, 0644);
MODULE_PARM_DESC(ao420_dma_threshold, "use DMA for transfer to AO [set 999999 to disable]");

int ao420_mapping[AO_CHAN] = { 4, 3, 2, 1 };
int ao420_mapping_count = 4;
module_param_array(ao420_mapping, int, &ao420_mapping_count, 0644);


int event_isr_msec = 20;
module_param(event_isr_msec, int, 0644);
MODULE_PARM_DESC(event_isr_msec, "event isr poll rate (remove when actual interrupt works)");

#define AO420_BUFFERLEN	0x100000

int ao420_buffer_length = AO420_BUFFERLEN;
module_param(ao420_buffer_length, int, 0644);
MODULE_PARM_DESC(ao420_buffer_length, "AWG buffer length");

#define AO424_BUFFERLEN	0x400000		/* HACK biggest buffer we can get */
int ao424_buffer_length = AO424_BUFFERLEN;
module_param(ao424_buffer_length, int, 0644);
MODULE_PARM_DESC(ao424_buffer_length, "AWG buffer length");

//int event_to = HZ/2;
int event_to = 0x7fffffff;		// ie infinity
module_param(event_to, int, 0644);
MODULE_PARM_DESC(event_to, "backstop event time out should be one TBLOCK");

int acq400_event_count_limit = 1;
module_param(acq400_event_count_limit, int, 0644);
MODULE_PARM_DESC(acq400_event_count_limit, "limit number of events per shot 0: no limit");


int is_acq2106B = 0;
module_param(is_acq2106B, int, 0444);
MODULE_PARM_DESC(is_acq2106B, "boolean indicator set on load");


int AXI_BUFFER_COUNT = 500;
module_param(AXI_BUFFER_COUNT, int, 0644);
MODULE_PARM_DESC(AXI_BUFFER_COUNT, "number of buffers in AXI cycle");


int AXI_BUFFER_CHECK_TICKS = 100;
module_param(AXI_BUFFER_CHECK_TICKS, int , 0644);
MODULE_PARM_DESC(AXI_BUFFER_CHECK_TICKS, "AXI buffer poll timeout");

int AXI_CALL_HELPER = 1;
module_param(AXI_CALL_HELPER, int, 0644);
MODULE_PARM_DESC(AXI_CALL_HELPER, "call helper to set up DMA chain else, someone else did it");

int AXI_DMA_HAS_CATCHUP = 1;
module_param(AXI_DMA_HAS_CATCHUP, int, 0644);
MODULE_PARM_DESC(AXI_DMA_HAS_CATCHUP, "catchup backlog on same tick");

int sync_continuous = 1;
module_param(sync_continuous, int, 0644);
MODULE_PARM_DESC(sync_continuous, "set zero to stub dma sync on continuous read (experiment)");

int AXI_INIT_BUFFERS = 0;
module_param(AXI_INIT_BUFFERS, int, 0644);
MODULE_PARM_DESC(AXI_INIT_BUFFERS, "initialise buffers before start .. see exactly how far DMA got for debug");

int AXI_ONESHOT = 0;
module_param(AXI_ONESHOT, int, 0644);
MODULE_PARM_DESC(AXI_ONESHOT, "axi DMA once through");

unsigned AXI_HEAD_DESCR_PA = 0;
module_param(AXI_HEAD_DESCR_PA, uint, 0644);

unsigned AXI_TAIL_DESCR_PA = 0;
module_param(AXI_TAIL_DESCR_PA, uint, 0644);

unsigned AXI_POISON_OFFSET = 0;
module_param(AXI_POISON_OFFSET, uint, 0644);
MODULE_PARM_DESC(AXI_POISON_OFFSET, "DEBUG: locate POISON in buffer (0=END)");

int AXI_DEBUG_LOOPBACK_INDEX = 0;
module_param(AXI_DEBUG_LOOPBACK_INDEX, int, 0644);
MODULE_PARM_DESC(AXI_POISON_OFFSET, "DEBUG: set non zero to skip first buffers on loopback. so that we see first time contents..");

int dtd_pulse_nsec = 1000000;
module_param(dtd_pulse_nsec, int, 0644);

int dtd_display_pulse_nsec = 200000000;
module_param(dtd_display_pulse_nsec, int, 0644);


int wimp_out;
module_param(wimp_out, int, 0644);

int histo_poll_ms = 10;
module_param(histo_poll_ms, int, 0644);
MODULE_PARM_DESC(histo_poll_ms, "histogram poll rate msec");

int xo_use_distributor;
module_param(xo_use_distributor, int, 0644);
MODULE_PARM_DESC(xo_use_distributor, "use distributor and PRI for XO transfer";)

int xo_distributor_sample_size = sizeof(unsigned);
module_param(xo_distributor_sample_size, int, 0644);
MODULE_PARM_DESC(xo_distributor_sample_size, "sample size in distributor set");


int continuous_reader;
module_param(continuous_reader, int, 0444);
MODULE_PARM_DESC(continuous_reader, "bitmask shows which sites have a reader");

// @@todo pgm: crude: index by site, index from 0
const char* acq400_names[] = { "0", "1", "2", "3", "4", "5", "6" };
const char* acq400_devnames[] = {
	"acq400.0", "acq400.1", "acq400.2",
	"acq400.3", "acq400.4", "acq400.5", "acq400.6"
};

#define AO420_NBUFFERS 	2


#define ACQ420_NBUFFERS	16
#define ACQ420_BUFFERLEN frontside_bufferlen

/* SC uses DMA with peripheral control */
#define SC_PRI	0		/* use this peripheral 		*/
#define SC_EV	8		/* use this event (and +1) 	*/

#define DMA_SC_FLAGS \
	(SC_EV << DMA_CHANNEL_EV0_SHL        | \
	 DMA_SRC_NO_INCR | \
	 PRI_TO_CTRL_FLAGS(SC_PRI) << DMA_CHANNEL_STARTS_WFP_SHL | \
	 PRI_TO_CTRL_FLAGS(SC_PRI) << DMA_CHANNEL_ENDS_FLUSHP_SHL)

/* DS0 uses DMA with peripheral control */
#define DS0_PRI	1
#define DS0_EV	10

#define DMA_DS0_FLAGS \
	(DS0_EV << DMA_CHANNEL_EV0_SHL        | \
	 DMA_DST_NO_INCR | \
	 PRI_TO_CTRL_FLAGS(DS0_PRI) << DMA_CHANNEL_STARTS_WFP_SHL | \
	 PRI_TO_CTRL_FLAGS(DS0_PRI) << DMA_CHANNEL_ENDS_FLUSHP_SHL)


#define DMA_TIMEOUT 		msecs_to_jiffies(10000)
/* first time : infinite timeout .. we probably won't live this many jiffies */
#define START_TIMEOUT		0x7fffffff

int ai_data_loop(void *data);
int axi64_data_loop(void* data);
int fifo_monitor(void* data);

int fiferr;

void acq420_onStart(struct acq400_dev *adev);
void acq480_onStart(struct acq400_dev *adev);
void acq43X_onStart(struct acq400_dev *adev);
void ao420_onStart(struct acq400_dev *adev);
static void acq420_disable_fifo(struct acq400_dev *adev);

static void _ao420_onStart(struct acq400_dev *adev);
static void _ao420_onStop(struct acq400_dev *adev);
static void _dio432_DO_onStart(struct acq400_dev *adev);

void go_rt(int prio)
{
	struct task_struct *task = current;
	struct sched_param param = { };

	param.sched_priority = prio;
	sched_setscheduler(task, SCHED_FIFO, &param);
}

void acq400_enable_event0(struct acq400_dev *adev, int enable){
	u32 timcon = acq400rd32(adev, TIM_CTRL);
	if (enable){
		timcon |= TIM_CTRL_MODE_EV0_EN;
	}else{
		timcon &= ~TIM_CTRL_MODE_EV0_EN;
	}
	dev_dbg(DEVP(adev), "acq400_enable_event0(%d) 0x%08x", enable, timcon);
	acq400wr32(adev, TIM_CTRL, timcon);
}

const char* devname(struct acq400_dev *adev)
{
	return acq400_devnames[adev->of_prams.site];
}
/* correct for FPGA mismatch with front panel connectors */
int ao420_physChan(int lchan /* 1..4 */ )
{
	int ip;
	for (ip = 0; ip < AO_CHAN; ++ip){
		if (ao420_mapping[ip] == lchan){
			return ip;
		}
	}
	BUG();
}

int ao428_physChan(int lchan /* 1..8 */ )
{
	return lchan-1;
}
int ao424_physChan(int lchan /* 1..32 */ )
{
	return lchan-1;
}

int isGoodSite(int site)
{
	if (site == 0){
		return 1;
	}else if (site < 0 || site > MAXDEVICES){
		return 0;
	}else{
		int ii;
		for (ii = 0; ii < good_sites_count; ++ii){
			if (good_sites[ii] == site){
				return 1;
			}
		}
		return 0;
	}
}
int acq400_release(struct inode *inode, struct file *file);
int acq400_hb_release(struct inode *inode, struct file *file);


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
	if (adev->RW32_debug){
		dev_info(DEVP(adev), "acq400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	}else{
		dev_dbg(DEVP(adev), "acq400rd32 %p [0x%02x] = %08x\n",
			adev->dev_virtaddr + offset, offset, rc);
	}
	return rc;
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

void set_continuous_reader(struct acq400_dev *adev)
{
	adev->continuous_reader = task_pid_nr(current);
	continuous_reader |= 1 << adev->of_prams.site;
}

void clr_continuous_reader(struct acq400_dev *adev)
{
	adev->continuous_reader = 0;
	continuous_reader &= ~(1 << adev->of_prams.site);
}
u32 acq420_set_fmt(struct acq400_dev *adev, u32 adc_ctrl)
/* DOES NOT ACTUALLY WRITE HARDWARE! */
{
	if (adev->adc_18b){
		adc_ctrl |= ADC_CTRL_420_18B;
	}else{
		adc_ctrl &= ~ADC_CTRL_420_18B;
	}
	if (adev->data32){
		adc_ctrl |= ADC_CTRL32B_data;
	}else{
		adc_ctrl &= ~ADC_CTRL32B_data;
	}
	return adc_ctrl;
}

static void acq400sc_init_defaults(struct acq400_dev *adev)
{
	u32 mcr = acq400rd32(adev, MCR);
	if (IS_ACQ2006B(adev)||IS_ACQ1001SC(adev)){
		mcr |= MCR_PSU_SYNC;
	}
	acq400wr32(adev, MCR, mcr|MCR_MOD_EN);

	if (IS_AXI64(adev)){
		int databursts = bufferlen/0x800;
		acq400wr32(adev, AXI_DMA_ENGINE_DATA, databursts-1);
		sync_continuous = 0;
	}
}


int _ao420_getFifoSamples(struct acq400_dev* adev) {
	return acq400rd32(adev, DAC_FIFO_SAMPLES)&DAC_FIFO_SAMPLES_MASK;
}

int _dio432_DO_getFifoSamples(struct acq400_dev* adev) {
	return acq400rd32(adev, DIO432_DO_FIFO_COUNT)&DAC_FIFO_SAMPLES_MASK;
}

static void acq420_reset_fifo(struct acq400_dev *adev)
/* Raise and Release reset */
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);

	acq400wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_FIFO_RST);
	acq400wr32(adev, ADC_CTRL, ctrl);
	/** clear FIFO flags .. workaround hw bug */
	acq400wr32(adev, ADC_FIFO_STA, ADC_FIFO_STA_ERR);

}

static void acq420_enable_fifo(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);
	if (adev->ramp_en){
		ctrl |= ADC_CTRL_RAMP_EN;
	}else{
		ctrl &= ~ADC_CTRL_RAMP_EN;
	}
	if (IS_ACQ42X(adev)){
		ctrl = acq420_set_fmt(adev, ctrl);
	}
	acq400wr32(adev, ADC_CTRL, ctrl|ADC_CTRL_ENABLE_ALL);
}

void acq420_disable_fifo(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);
	dev_dbg(DEVP(adev), "acq420_disable_fifo() %08x -> %08x",
			ctrl, ctrl & ~ADC_CTRL_ENABLE_CAPTURE);
	acq400wr32(adev, ADC_CTRL, ctrl & ~ADC_CTRL_ENABLE_CAPTURE);
	acq420_reset_fifo(adev);
}


void acq400_enable_trg_if_master(struct acq400_dev *adev)
{
	dev_dbg(DEVP(adev), "acq400_enable_trg_if_master %d = %d",
			adev->of_prams.site, ((adev->mod_id&MOD_ID_IS_SLAVE) == 0));

	if ((adev->mod_id&MOD_ID_IS_SLAVE) == 0){
		acq400_enable_trg(adev, 1);
	}
}

void acq400_enable_trg(struct acq400_dev *adev, int enable){
	u32 timcon = acq400rd32(adev, TIM_CTRL);
	if (enable){
		timcon |= TIM_CTRL_MODE_HW_TRG_EN;
	}else{
		timcon &= ~TIM_CTRL_MODE_HW_TRG_EN;
	}
	acq400wr32(adev, TIM_CTRL, timcon);
}

static void acq420_init_defaults(struct acq400_dev *adev)
{
	u32 adc_ctrl = acq400rd32(adev, ADC_CTRL);

	dev_info(DEVP(adev), "ACQ420 device init");
	acq400wr32(adev, ADC_CONV_TIME, IS_ACQ424(adev)? acq424_adc_conv_time: adc_conv_time);
	adev->data32 = data_32b;
	adev->adc_18b = adc_18b;
	adc_ctrl |= acq420_set_fmt(adev, adc_ctrl);
	acq400wr32(adev, ADC_CTRL, ADC_CTRL_MODULE_EN|adc_ctrl);
	adev->nchan_enabled = IS_ACQ424(adev)? 32: IS_ACQ425(adev)? 16: 4;
	adev->word_size = adev->data32? 4: 2;
	adev->sysclkhz = SYSCLK_M100;
	adev->hitide = hitide;
	adev->lotide = lotide;
	adev->onStart = acq420_onStart;
	adev->onStop = acq420_disable_fifo;

	if (IS_ACQ424(adev)){
		acq400wr32(adev, ADC_CLKDIV, 66);
	}
}

int acq480_train_fail(struct acq400_dev *adev)
{
	if ((adev->mod_id&MOD_ID_REV_MASK) >= 0xa){
		return (acq400rd32(adev, ADC_CTRL)&ADC480_CTRL_TRAIN_OK) == 0;
	}else{
		return -1;
	}
}
static void acq480_init_defaults(struct acq400_dev *adev)
{
	dev_info(DEVP(adev), "ACQ480 device init: skeleton");

	adev->data32 = 0;
	adev->adc_18b = 0;

	acq400wr32(adev, ADC_CTRL, ADC_CTRL_MODULE_EN);
	adev->nchan_enabled = 8;
	adev->word_size = 2;
	adev->sysclkhz = SYSCLK_M100;
	adev->hitide = hitide;
	adev->lotide = lotide;
	adev->onStart = acq480_onStart;
	adev->onStop = acq420_disable_fifo;
	adev->clkdiv_mask = 0x0001;		/* disable clkdiv */
}


static u32 _v2f_init(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, V2F_CTRL);

	if (adev->data32){
		ctrl &= ~V2F_CTRL_DATA_PACKED;
	}else{
		ctrl |= V2F_CTRL_DATA_PACKED;
	}
	acq400wr32(adev, V2F_CTRL, ctrl&=~ V2F_CTRL_EN);
	acq400wr32(adev, V2F_CTRL, ctrl|V2F_CTRL_RST);
	acq400wr32(adev, V2F_CTRL, ctrl);
	return ctrl;
}
static void _v2f_onStop(struct acq400_dev *adev)
{
	_v2f_init(adev);
}

static void _v2f_onStart(struct acq400_dev *adev)
{
	u32 ctrl = _v2f_init(adev);
	acq400wr32(adev, V2F_CTRL, ctrl|V2F_CTRL_EN);
}

static void v2f_init_defaults(struct acq400_dev *adev)
{
	adev->data32 = 0;
	adev->word_size = 4;
	adev->nchan_enabled = 0;
	acq400wr32(adev, V2F_CHAN_SEL, 0x04030201);
	acq400wr32(adev, V2F_CTRL, ADC_CTRL_MODULE_EN|V2F_CTRL_EN);
	adev->onStart = _v2f_onStart;
	adev->onStop = _v2f_onStop;
}

static void _qen_onStop(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, QEN_CTRL);
	ctrl &= ~(QEN_CTRL_FIFO_EN|QEN_CTRL_EN);
	acq400wr32(adev, QEN_CTRL, ctrl);
}

static void _qen_onStart(struct acq400_dev *adev)
{
	acq420_enable_fifo(adev);
	acq420_reset_fifo(adev);
}

static void qen_init_defaults(struct acq400_dev *adev)
{
	struct acq400_dev *adev1 = acq400_devices[1];
	if (adev1 && adev1->data32 == 0){
		adev->data32 = 0;
		adev->word_size = 2;
		adev->nchan_enabled = 2;
	}else{
		adev->data32 = 1;
		adev->word_size = 1;
		adev->nchan_enabled = 1;
	}

	acq400wr32(adev, QEN_CTRL, QEN_CTRL_MODULE_EN);
	adev->onStart = _qen_onStart;
	adev->onStop = _qen_onStop;
}
static void pig_celf_init_defaults(struct acq400_dev *adev)
{
	adev->data32 = 0;
	/* data is 4 x 32 bits, but I+Q surely 16 bit each? */
	adev->word_size = 2;
	adev->nchan_enabled = 8;
	adev->clkdiv_mask = 0;	/* unfortunate alias of clkdiv reg. stub it */
	dev_info(DEVP(adev), "pig_celf_init_defaults()");
	acq400wr32(adev, MCR, MCR_MOD_EN);
	adev->onStart = acq420_onStart;
	adev->onStop = acq420_disable_fifo;
	acq400wr32(adev, PC_DDS_DAC_CLKDIV, 	0x5);
	acq400wr32(adev, PC_ADC_CLKDIV, 	0x14);
	acq400wr32(adev, PC_DDS_PHASE_INC, 	0x80000);
}

static void rad_celf_init_defaults(struct acq400_dev *adev)
{
	dev_info(DEVP(adev), "rad_celf_init_defaults()");
	acq400wr32(adev, MCR, MCR_MOD_EN);
}

static void pmodadc1_init_defaults(struct acq400_dev *adev)
{
	u32 adc_ctrl = acq400rd32(adev, ADC_CTRL);

	dev_info(DEVP(adev), "PMODADC1 device init");
	if (FPGA_REV(adev) < 5){
		dev_warn(DEVP(adev), "OUTDATED FPGA PERSONALITY, please update");
	}
	acq400wr32(adev, ADC_CONV_TIME, adc_conv_time);
	adev->data32 = 0;
	adev->adc_18b = 0;
	adc_ctrl |= PMODADC1_CTRL_DIV|PMODADC1_CTRL_EXT_CLK_FROM_SYNC;
	acq400wr32(adev, ADC_CTRL, ADC_CTRL_MODULE_EN|adc_ctrl);
	/** nchan_enabled @@todo fudge for use in D32 (acq435 systems) */
	adev->nchan_enabled = 1;
	adev->word_size = 4;
	adev->hitide = hitide;
	adev->lotide = lotide;
	adev->onStart = acq420_onStart;
	adev->onStop = acq420_disable_fifo;
}

int xtd_watchdog(void *data)
{
	struct acq400_dev *adev = (struct acq400_dev *)data;
	int nzcount;
	u32 trg_src0 = 0;
	u32 trg_src;

	while(1) {
		msleep(10);
		trg_src = acq400rd32(adev, ATD_TRIGGERED);

		if (trg_src == 0){
			nzcount = 0;
			trg_src0 = 0;
		}else if (trg_src0 != 0 && (trg_src&trg_src0) == trg_src0){
			if (++nzcount >= 3){
				dev_info(DEVP(adev), "xtd_watchdog clears %08x", trg_src);
				acq400_clearDelTrgEvent(adev);
				acq400wr32(adev, ATD_TRIGGERED, trg_src);
				trg_src0 = 0;
			}
		}else{
			trg_src0 = trg_src;
		}
	}
}


static void start_xtd_watchdog(struct acq400_dev *adev)
{
	kthread_run(xtd_watchdog, adev, "%s.xtd", devname(adev));
}
static void acq43X_init_defaults(struct acq400_dev *adev)
{
	int is_acq430 = IS_ACQ430(adev);
	u32 adc_ctrl = acq400rd32(adev, ADC_CTRL);

	dev_info(DEVP(adev), "%s device init", is_acq430? "ACQ430": "ACQ435");
	adev->data32 = 1;
	if (IS_ACQ430(adev)){
		adev->nchan_enabled = 8;
	}else if (IS_ACQ437(adev)){
		adev->nchan_enabled = 16;
	}else{
		adev->nchan_enabled = 32;	// 32 are you sure?.
	}
	adev->word_size = 4;
	if (is_acq430){
		acq400wr32(adev, ACQ435_MODE, ACQ430_BANKSEL);
	}
	// @@todo ACQ437_BANKSEL ?
	adev->hitide = 128;
	adev->lotide = adev->hitide - 4;
	adev->sysclkhz = SYSCLK_M100;
	acq400wr32(adev, ADC_CLKDIV, 16);
	acq400wr32(adev, ADC_CTRL, adc_ctrl|ADC_CTRL_MODULE_EN);
	adev->onStart = acq43X_onStart;
	adev->onStop = acq420_disable_fifo;
	if (HAS_XTD(adev)){
		start_xtd_watchdog(adev);
	}
}



static void ao428_init_defaults(struct acq400_dev *adev)
/* dac_celf mounts DAC20, assy known as "ao428elf" */
{
	u32 dac_ctrl = acq400rd32(adev, DAC_CTRL);
	dev_info(DEVP(adev), "AO420 device init");

	adev->data32 = 1;
	adev->nchan_enabled = 8;
	adev->word_size = 4;
	adev->cursor.hb = &adev->hb[0];

	adev->sysclkhz = SYSCLK_M66;
	adev->onStart = _ao420_onStart;
	adev->onStop = _ao420_onStop;
	adev->xo.physchan = ao428_physChan;
	adev->xo.getFifoSamples = _ao420_getFifoSamples;
	adev->xo.fsr = DAC_FIFO_STA;
	dac_ctrl |= ADC_CTRL_MODULE_EN;
	acq400wr32(adev, DAC_CTRL, dac_ctrl);
	if (measure_ao_fifo_ok){
		measure_ao_fifo(adev);
	}
}
static void ao420_init_defaults(struct acq400_dev *adev)
{
	u32 dac_ctrl = acq400rd32(adev, DAC_CTRL);
	dev_info(DEVP(adev), "AO420 device init");

	adev->data32 = 0;
	adev->nchan_enabled = 4;
	adev->word_size = 2;
	adev->cursor.hb = &adev->hb[0];

	adev->sysclkhz = SYSCLK_M66;
	adev->onStart = _ao420_onStart;
	adev->onStop = _ao420_onStop;
	adev->xo.physchan = ao420_physChan;
	adev->xo.getFifoSamples = _ao420_getFifoSamples;
	adev->xo.fsr = DAC_FIFO_STA;
	dac_ctrl |= ADC_CTRL_MODULE_EN;
	acq400wr32(adev, DAC_CTRL, dac_ctrl);
	measure_ao_fifo(adev);
}

static void ao424_init_defaults(struct acq400_dev *adev)
{
	u32 dac_ctrl = acq400rd32(adev, DAC_CTRL);

	/* default to +/-10V bipolar. That's what 99% of the people want .. */
	dev_info(DEVP(adev), "AO424 device init default SPAN=3");
	ao424_setspan_defaults(adev);
	adev->data32 = 0;
	adev->nchan_enabled = 32;
	adev->word_size = 2;
	adev->cursor.hb = &adev->hb[0];

	adev->sysclkhz = SYSCLK_M66;
	adev->onStart = _ao420_onStart;
	adev->xo.physchan = ao424_physChan;
	adev->xo.getFifoSamples = _ao420_getFifoSamples;
	adev->xo.fsr = DAC_FIFO_STA;

	dac_ctrl |= ADC_CTRL_MODULE_EN;
	adev->ao424_device_settings.encoded_twocmp = 0;
	acq400wr32(adev, DAC_CTRL, dac_ctrl);
	ao424_set_spans(adev);
	measure_ao_fifo(adev);
}

static void dio432_onStop(struct acq400_dev *adev);


int set_debugs(char* on)
{
	 char *argv[] = { "/mnt/local/set_debugs", on, NULL };
	 static char *envp[] = {
	        "HOME=/",
	        "TERM=linux",
	        "PATH=/sbin:/bin:/usr/sbin:/usr/bin", NULL };

	  return call_usermodehelper( argv[0], argv, envp, UMH_WAIT_PROC );
}

int dio432_isFifoError(struct acq400_dev *adev);

static void dio432_init_defaults(struct acq400_dev *adev)
{
	u32 dac_ctrl = acq400rd32(adev, DAC_CTRL);

	dev_info(DEVP(adev), "dio432_init_defaults() 01");
	adev->data32 = 1;
	adev->nchan_enabled = 1;
	adev->word_size = 4;
	adev->cursor.hb = &adev->hb[0];
	adev->hitide = 2048;
	adev->lotide = 0x1fff;
	adev->sysclkhz = SYSCLK_M66;
	adev->onStart = _dio432_DO_onStart;
	adev->onStop = dio432_onStop;
	adev->xo.getFifoSamples = _dio432_DO_getFifoSamples;
	adev->xo.fsr = DIO432_DO_FIFO_STATUS;
	adev->isFifoError = dio432_isFifoError;
	if (FPGA_REV(adev) < 5){
		dev_warn(DEVP(adev), "OUTDATED FPGA PERSONALITY, please update");
	}
	//set_debugs("on");
	dac_ctrl |= IS_DIO432PMOD(adev)?
		DIO432_CTRL_SHIFT_DIV_PMOD: DIO432_CTRL_SHIFT_DIV_FMC;
	dac_ctrl |= DIO432_CTRL_MODULE_EN | DIO432_CTRL_DIO_EN;
	acq400wr32(adev, DAC_CTRL, dac_ctrl);

	dev_info(DEVP(adev), "dio432_init_defaults %d dac_ctrl=%08x",
			__LINE__, acq400rd32(adev, DAC_CTRL));
	//acq400wr32(adev, DAC_INT_CSR, 0);
	acq400wr32(adev, DAC_CTRL, dac_ctrl|DIO432_CTRL_FIFO_RST|DIO432_CTRL_DIO_RST);
	acq400wr32(adev, DAC_CTRL, dac_ctrl|DIO432_CTRL_FIFO_EN);

	dev_info(DEVP(adev), "dio432_init_defaults() 60 measure_ao_fifo()");
	measure_ao_fifo(adev);
	dev_info(DEVP(adev), "dio432 max fifo samples %d", adev->xo.max_fifo_samples);
	if (dio432_rowback){
		adev->xo.max_fifo_samples -= dio432_rowback;
		dev_info(DEVP(adev), "dio432 max fifo samples %d dio432_rowback",
				adev->xo.max_fifo_samples);
	}
	acq400wr32(adev, DIO432_DI_FIFO_STATUS, DIO432_FIFSTA_CLR);
	acq400wr32(adev, DIO432_DO_FIFO_STATUS, DIO432_FIFSTA_CLR);
	//set_debugs("off");
	dev_info(DEVP(adev), "dio432_init_defaults %d dac_ctrl=%08x",
			__LINE__, acq400rd32(adev, DAC_CTRL));
	dev_info(DEVP(adev), "dio432_init_defaults() 99 cursor %d", adev->cursor.hb[0]->ix);
}

int bolo8_isFifoError(struct acq400_dev *adev);

static void bolo8_init_defaults(struct acq400_dev* adev)
{
	u32 syscon = acq400rd32(adev, B8_SYS_CON);
	syscon |= ADC_CTRL_MODULE_EN;

	acq400wr32(adev, B8_SYS_CON, syscon);
	acq400wr32(adev, B8_DAC_CON, B8_DAC_CON_INIT);
	dev_info(DEVP(adev), "bolo8_init_defaults() B8_DAC_CON set:%x get:%x",
			B8_DAC_CON_INIT, acq400rd32(adev, B8_DAC_CON));

	adev->data32 = data_32b;
	adev->nchan_enabled = 8;
	adev->word_size = adev->data32? 4: 2;
	adev->hitide = 128;
	adev->lotide = adev->hitide - 4;
	adev->sysclkhz = SYSCLK_M100;
	acq400wr32(adev, ADC_CLKDIV, 10);
	adev->onStart = bolo8_onStart;
	adev->onStop = bolo8_onStop;
	adev->isFifoError = bolo8_isFifoError;

	adev->bolo8.awg_buffer = kmalloc(4096, GFP_KERNEL);
	adev->bolo8.awg_buffer_max = 4096;
	adev->bolo8.awg_buffer_cursor = 0;

}
static u32 acq420_get_fifo_samples(struct acq400_dev *adev)
{
	return acq400rd32(adev, ADC_FIFO_SAMPLES);
}

static u32 aggregator_get_fifo_samples(struct acq400_dev *adev)
{
	return acq400rd32(adev, AGGSTA);
}

/* @@todo not used
static u32 acq420_samples2bytes(struct acq400_dev *adev, u32 samples)
{
	return samples * adev->nchan_enabled * adev->word_size;
}
*/




void acq2006_aggregator_reset(struct acq400_dev *adev)
{
	u32 agg = acq400rd32(adev, AGGREGATOR);
	acq400wr32(adev, AGGREGATOR, agg &= ~(AGG_FIFO_RESET|AGG_ENABLE));
	acq400wr32(adev, AGGREGATOR, agg | AGG_FIFO_RESET);
	acq400wr32(adev, AGGREGATOR, agg);
	acq400rd32(adev, AGGREGATOR);
}

void sc_data_engine_reset_enable(unsigned dex)
{
	struct acq400_dev *adev = acq400_devices[0];
	u32 DEX = acq400rd32(adev, dex);
	acq400wr32(adev, dex, DEX &= ~(DE_ENABLE));
	acq400wr32(adev, dex, DEX | DE_ENABLE);
}
void acq2006_aggregator_enable(struct acq400_dev *adev)
{
	u32 agg = acq400rd32(adev, AGGREGATOR);
	agg &= ~AGG_SPAD_ALL_MASK;

	switch(adev->spad.spad_en){
	default:
		goto agg99;
	case SP_EN:
		agg |= SET_AGG_SPAD_LEN(adev->spad.len);
		agg |= AGG_SPAD_EN;
		break;
	case SP_FRAME:
		agg |= AGG_SPAD_EN|AGG_SPAD_FRAME;
		break;
	}
	switch(adev->spad.diX){
	case SD_DI4:
		agg |= SET_AGG_SNAP_DIx(AGG_SNAP_DI_4); break;
	case SD_DI32:
		agg |= SET_AGG_SNAP_DIx(AGG_SNAP_DI_32); break;
	}
 agg99:
	acq400wr32(adev, AGGREGATOR, agg | AGG_ENABLE);
}
void acq2006_aggregator_disable(struct acq400_dev *adev)
{
	u32 agg = acq400rd32(adev, AGGREGATOR);
	acq400wr32(adev, AGGREGATOR, agg & ~AGG_ENABLE);
}

/** @todo not used
static void acq420_enable_interrupt(struct acq400_dev *adev)
{
	u32 int_ctrl = acq400rd32(adev, ADC_INT_CSR);
	acq400wr32(adev, ADC_INT_CSR,	int_ctrl|0x1);
}
*/




static void acq400_clear_histo(struct acq400_dev *adev)
{
	memset(adev->fifo_histo, 0, FIFO_HISTO_SZ*sizeof(u32));
}

int acq420_isFifoError(struct acq400_dev *adev)
{
	u32 fifsta = acq400rd32(adev, ADC_FIFO_STA);
	int err = (fifsta&fiferr) != 0;

	acq400wr32(adev, ADC_FIFO_STA, fifsta);

	if (err){
		u32 fifsta2 = acq400rd32(adev, ADC_FIFO_STA);
		adev->stats.fifo_errors++;
		dev_warn(DEVP(adev), "FIFERR after %d buffers, mask:%08x cmp:%08x actual:%08x %s\n",
				acq400_lookupSite(0)->rt.nput,
				FIFERR, fiferr, fifsta,
				(fifsta2&fiferr) != 0? "NOT CLEARED": "clear");
	}

	return act_on_fiferr? err: 0;
}

int dio432_isFifoError(struct acq400_dev *adev)
{
	u32 fifsta = acq400rd32(adev, ADC_FIFO_STA);
	int err = (fifsta&fiferr) != 0;
	int rc = 0;

	acq400wr32(adev, ADC_FIFO_STA, fifsta);

	if (err){
		u32 fifsta2 = acq400rd32(adev, ADC_FIFO_STA);
		int nbuf = acq400_lookupSite(0)->rt.nput;
		adev->stats.fifo_errors++;
		dev_warn(DEVP(adev), "FIFERR after %d buffers, mask:%08x cmp:%08x actual:%08x %s\n",
				nbuf, 				FIFERR, fiferr, fifsta,
				(fifsta2&fiferr) != 0? "NOT CLEARED": "clear");

		rc = nbuf>1 && act_on_fiferr? err: 0;
		if (err && !act_on_fiferr && rc == 0){
			dev_info(DEVP(adev), "initial error ignored");
		}
	}
	return rc;
}

int bolo8_isFifoError(struct acq400_dev *adev)
{
	u32 fifsta = acq400rd32(adev, B8_ADC_FIFO_STA);
	int err = (fifsta&fiferr) != 0;

	acq400wr32(adev, B8_ADC_FIFO_STA, fifsta);

	if (err){
		u32 fifsta2 = acq400rd32(adev, B8_ADC_FIFO_STA);
		adev->stats.fifo_errors++;
		dev_warn(DEVP(adev), "BOLO8 FIFERR after %d buffers, mask:%08x cmp:%08x actual:%08x %s\n",
				acq400_lookupSite(0)->rt.nput,
				FIFERR, fiferr, fifsta,
				(fifsta2&fiferr) != 0? "NOT CLEARED": "clear");
	}

	return act_on_fiferr? err: 0;
}

int acq420_convActive(struct acq400_dev *adev)
{
	u32 fifsta = acq400rd32(adev, ADC_FIFO_STA);
	u32 active = fifsta&ADC_FIFO_STA_ACTIVE;

	dev_dbg(DEVP(adev), "acq420_convActive() %08x %s",
			fifsta, active? "YES": "no");
	return active;
}
void acqXXX_onStartNOP(struct acq400_dev *adev)
{

}
void acqXXX_onStopNOP(struct acq400_dev *adev)
{

}
void acq43X_onStart(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, ADC_CTRL);

	dev_dbg(DEVP(adev), "acq435_onStart() ctrl=%08x", ctrl);

	acq400wr32(adev, ADC_HITIDE, 	adev->hitide);
	ctrl &= ~ADC_CTRL_FIFO_EN|ADC_CTRL_ADC_EN;
	acq400wr32(adev, ADC_CTRL, ctrl |= ADC_CTRL_MODULE_EN);

	if (HAS_DTD(adev)){
		acq400_clearDelTrgEvent(adev);
	}
	if (adev->ramp_en){
		ctrl |= ADC_CTRL_RAMP_EN;
	}else{
		ctrl &= ~ADC_CTRL_RAMP_EN;
	}

	// set mode (assume done)
	// set clkdiv (assume done)
	// set timing bus (assume done)
	/** clear FIFO flags .. workaround hw bug */
	acq400wr32(adev, ADC_FIFO_STA, ADC_FIFO_STA_ERR);

	acq400wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_FIFO_RST);
	acq400wr32(adev, ADC_CTRL, ctrl);
	if (acq420_get_fifo_samples(adev)){
		dev_warn(DEVP(adev), "ERROR: reset fifo but it's not empty!");
	}

	adev->fifo_isr_done = 0;
	//acq420_enable_interrupt(adev);
	acq400wr32(adev, ADC_CTRL, ctrl  |= ADC_CTRL_ADC_EN);
	acq400wr32(adev, ADC_CTRL, ctrl  |= ADC_CTRL_FIFO_EN);

	/* next: valid Master, Standalone only. @@todo slave? */
	acq400wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_ADC_RST);
	acq400wr32(adev, ADC_CTRL, ctrl);

	dev_dbg(DEVP(adev), "acq435_onStart() 99");
}

void ao420_clear_fifo_flags(struct acq400_dev *adev)
{
	acq400wr32(adev, ADC_FIFO_STA, ADC_FIFO_STA_EMPTY|ADC_FIFO_STA_ERR);
}
void ao420_reset_fifo(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, DAC_CTRL);
	acq400wr32(adev, DAC_CTRL, ctrl &= ~ADC_CTRL_FIFO_EN);
	acq400wr32(adev, DAC_CTRL, ctrl | ADC_CTRL_FIFO_RST);
	ao420_clear_fifo_flags(adev);
	acq400wr32(adev, DAC_CTRL, ctrl |= ADC_CTRL_FIFO_EN);
}

static void _ao420_onStart(struct acq400_dev *adev)
{
	u32 ctrl = acq400rd32(adev, DAC_CTRL);

	ctrl &= ~DAC_CTRL_LL;
	if (adev->AO_playloop.one_shot == 0 ||
			adev->AO_playloop.length > adev->lotide){
		dev_dbg(DEVP(adev), "_ao420_onStart() set lotide:%d", adev->lotide);
		acq400wr32(adev, DAC_LOTIDE, adev->lotide);
		x400_enable_interrupt(adev);
	}else{
		acq400wr32(adev, DAC_LOTIDE, 0);
	}
	acq400wr32(adev, DAC_CTRL, ctrl |= DAC_CTRL_DAC_EN);
}

void _ao420_stop(struct acq400_dev* adev);

static void _ao420_onStop(struct acq400_dev *adev)
{
	_ao420_stop(adev);
}



void _dio432_DO_onStart(struct acq400_dev *adev)
{
	if (xo_use_distributor){
		acq400wr32(adev, DIO432_DO_LOTIDE, adev->lotide);
		dev_dbg(DEVP(adev), "_dio432_DO_onStart() 05");
	}else if (adev->AO_playloop.one_shot == 0 ||
			adev->AO_playloop.length > adev->lotide){
		dev_dbg(DEVP(adev), "_dio432_DO_onStart() 10");
		acq400wr32(adev, DIO432_DO_LOTIDE, adev->lotide);
		x400_enable_interrupt(adev);
	}else{
		acq400wr32(adev, DIO432_DO_LOTIDE, 0);
		dev_dbg(DEVP(adev), "_dio432_DO_onStart() 20");
	}
	acq400wr32(adev, DIO432_DIO_CTRL, acq400rd32(adev, DIO432_DIO_CTRL)|DIO432_CTRL_DIO_EN);
	dev_dbg(DEVP(adev), "_dio432_DO_onStart() 99");
}

void dio432_onStop(struct acq400_dev *adev)
{
	x400_disable_interrupt(adev);
	dio432_disable(adev);
	//dio432_init_clocked(adev);
}

void acq420_onStart(struct acq400_dev *adev)
{
	u32 ctrl;
	u32 status = acq400rd32(adev, ADC_FIFO_STA);

	dev_dbg(DEVP(adev), "acq420_onStart()");

	if ((status&ADC_FIFO_STA_ACTIVE) != 0){
		dev_err(DEVP(adev), "ERROR: ADC_FIFO_STA_ACTIVE set %08x", status);
		acq420_disable_fifo(adev);
	}

	acq400wr32(adev, ADC_HITIDE, 	adev->hitide);
	acq420_enable_fifo(adev);
	ctrl = acq400rd32(adev, ADC_CTRL);
	acq400wr32(adev, ADC_CTRL, ctrl | ADC_CTRL_ADC_RST);
	acq420_reset_fifo(adev);
	acq400wr32(adev, ADC_CTRL, ctrl);
	adev->fifo_isr_done = 0;
	//acq420_enable_interrupt(adev);
}

void acq480_onStart(struct acq400_dev *adev)
{
	dev_dbg(DEVP(adev), "acq480_onStart()");
	acq400wr32(adev, ADC_HITIDE, 	adev->hitide);
	acq420_enable_fifo(adev);

	acq420_reset_fifo(adev);

	adev->fifo_isr_done = 0;
	//acq420_enable_interrupt(adev);
}


static void acq400_getID(struct acq400_dev *adev)
{
	u32 modid;

	dev_info(DEVP(adev), "About to read MODID from %p\n", adev->dev_virtaddr+MOD_ID);

	modid = acq400rd32(adev, MOD_ID);
	adev->mod_id = modid;

	dev_info(DEVP(adev), "Device MODID %08x", modid);
}
/*
static void acq420_force_interrupt(int interrupt)
{
	u32 status;
	status = ioread32(acq420_dev->dev_virtaddr + ALG_INT_FORCE);
	iowrite32((interrupt), acq420_dev->dev_virtaddr + ALG_INT_FORCE);
}
*/

unsigned long long t0;
int ins;

/** @@todo ... hook to ZYNQ timers how? */

extern unsigned long long otick(void);
extern unsigned delta_nsec(unsigned long long t0, unsigned long long t1);

#if 0
#define DMA_NS_INIT \
	do { 					\
		ins=0; t0 = otick(); 		\
	} while(0)

#define DMA_NS \
	do { 						\
		dma_ns_lines[ins] = __LINE__; 		\
		dma_ns[ins] = delta_nsec(t0, otick()); 	\
		ins++; 					\
		BUG_ON(ins >= DMA_NS_MAX);		\
	} while(0)


void timer_test(void)
{
	DMA_NS_INIT;

	while(ins+1 < DMA_NS_MAX){
		DMA_NS;
	}
}
#define DMA_NS_TEST	timer_test()
#else
#define DMA_NS_INIT
#define DMA_NS
#define DMA_NS_TEST
#endif


void acq400_dma_callback(void *param)
{
	struct acq400_dev* adev = (struct acq400_dev*)param;
	adev->dma_callback_done++;
	wake_up_interruptible(&adev->DMA_READY);
}

dma_cookie_t
dma_async_memcpy_callback(
	struct dma_chan *chan,
	dma_addr_t dma_dst, dma_addr_t dma_src,
	size_t len, unsigned long flags,
	dma_async_tx_callback callback,
	void *callback_param)
{
	struct dma_device *dev = chan->device;
	struct dma_async_tx_descriptor *tx;
	flags |=  DMA_CTRL_ACK;

	DMA_NS;
	dev_dbg(dev->dev, "dev->prep_dma_memcpy %d 0x%08x 0x%08x %d %08lx",
			chan->chan_id, dma_dst, dma_src, len, flags);
	tx = dev->device_prep_dma_memcpy(chan, dma_dst, dma_src, len, flags);

	DMA_NS;
	if (!tx) {
		dev_err(dev->dev, "prep failed");
		return -ENOMEM;
	} else{
		dma_cookie_t cookie;
		tx->callback = callback;
		tx->callback_param = callback_param;
		cookie = tx->tx_submit(tx);
		dev_dbg(dev->dev, "submit(%d) done %x", chan->chan_id, cookie);
		DMA_NS;

		preempt_disable();
		__this_cpu_add(chan->local->bytes_transferred, len);
		__this_cpu_inc(chan->local->memcpy_count);
		preempt_enable();
		return cookie;
	}
}

dma_cookie_t
dma_async_memcpy(
	struct dma_chan *chan, dma_addr_t src, 	dma_addr_t dest, size_t len)
{
	struct dma_device *dev = chan->device;
	struct dma_async_tx_descriptor *tx;
	unsigned long flags = DMA_DST_NO_INCR | DMA_CTRL_ACK;

	DMA_NS;
	tx = dev->device_prep_dma_memcpy(chan, dest, src, len, flags);

	DMA_NS;
	if (!tx) {
		return -ENOMEM;
	} else{
		dma_cookie_t cookie;
		tx->callback = NULL;
		cookie = tx->tx_submit(tx);

		DMA_NS;

		preempt_disable();
		__this_cpu_add(chan->local->bytes_transferred, len);
		__this_cpu_inc(chan->local->memcpy_count);
		preempt_enable();
		return cookie;
	}
}

int dma_memcpy(
	struct acq400_dev* adev, dma_addr_t dest, dma_addr_t src, size_t len)
{
	dma_cookie_t cookie;
	DMA_NS_INIT;
	DMA_NS;
	if (adev->dma_chan[0] == 0){
		dev_err(DEVP(adev), "%p id:%d dma_find_channel set zero",
				adev, adev->pdev->dev.id);
		return -1;
	}
	dev_dbg(DEVP(adev), "dma_memcpy() chan:%d src:%08x dest:%08x len:%d\n",
			adev->dma_chan[0]->chan_id, src, dest, len);
	cookie = dma_async_memcpy(adev->dma_chan[0], src, dest, len);
	dev_dbg(DEVP(adev), "dma_memcpy() wait cookie:%d\n", cookie);
	dma_sync_wait(adev->dma_chan[0], cookie);
	dev_dbg(DEVP(adev), "dma_memcpy() wait cookie:%d done\n", cookie);
	DMA_NS;
	return len;
}




/* File operations */
int acq400_open_main(struct inode *inode, struct file *file)
{
        int rc = 0;
        struct acq400_dev* adev = ACQ400_DEV(file);

        if (mutex_lock_interruptible(&adev->mutex)) {
                return -ERESTARTSYS;
        }

        /* We're only going to allow one write at a time, so manage that via
         * reference counts
         */
        switch (file->f_flags & O_ACCMODE) {
        case O_RDONLY:
                break;
        case O_WRONLY:
                if (adev->writers || adev->busy) {
                        rc = -EBUSY;
                        goto out;
                } else {
                        adev->writers++;
                }
                break;
        case O_RDWR:
        default:
                if (adev->writers || adev->busy) {
                        rc = -EBUSY;
                        goto out;
                } else {
                        adev->writers++;
                }
        }

        adev->stats.opens++;

out:
        mutex_unlock(&adev->mutex);
        return rc;
}

static int acq420_dma_open(struct inode *inode, struct file *file)
{
	return 0;
}

int acq400_dma_mmap_host(struct file* file, struct vm_area_struct* vma)
/**
 * mmap the host buffer.
 */
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ibuf = BUFFER(PD(file)->minor);
	struct HBM *hb = adev->hb[ibuf];
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = hb->len;
	unsigned pfn = hb->pa >> PAGE_SHIFT;

	if (!IS_BUFFER(PD(file)->minor)){
		dev_warn(DEVP(adev), "ERROR: device node not a buffer");
		return -1;
	}
	dev_dbg(&adev->pdev->dev, "%c vsize %lu psize %lu %s",
		'D', vsize, psize, vsize>psize? "EINVAL": "OK");

	if (vsize > psize){
		return -EINVAL;                   /* request too big */
	}
	if (remap_pfn_range(
		vma, vma->vm_start, pfn, vsize, vma->vm_page_prot)){
		return -EAGAIN;
	}else{
		return 0;
	}
}

ssize_t acq400_hb_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ibuf = BUFFER(PD(file)->minor);
	struct HBM *hb = adev->hb[ibuf];
	unsigned cursor = *f_pos;	/* f_pos counts in bytes */
	int rc;

	if (cursor >= hb->len){
		return 0;
	}else{
		int headroom = hb->len - cursor;
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, (char*)hb->va + cursor, count);
	if (rc){
		return -1;
	}

	*f_pos += count;
	return count;
}

int acq400_open_hb(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_fops_dma = {
		.open = acq420_dma_open,
		.mmap = acq400_dma_mmap_host,
		.read = acq400_hb_read,
		.release = acq400_hb_release,
		// sendfile method is no more.. it's probably not quite this easy ..
		// sure enough, it's not !
		// most likely the HB's have to be on a block device ..
		.splice_read	= generic_file_splice_read
	};
	file->f_op = &acq420_fops_dma;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}



ssize_t acq400_histo_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	unsigned *the_histo = ACQ400_DEV(file)->fifo_histo;
	int maxentries = FIFO_HISTO_SZ;
	unsigned cursor = *f_pos;	/* f_pos counts in entries */
	int rc;

	if (cursor >= maxentries){
		return 0;
	}else{
		int headroom = (maxentries - cursor) * sizeof(unsigned);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, the_histo+cursor, count);
	if (rc){
		return -1;
	}

	*f_pos += count/sizeof(unsigned);
	return count;
}



static bool filter_true(struct dma_chan *chan, void *param)
{
	return true;
}

static bool filter_axi(struct dma_chan *chan, void *param)
{
	struct acq400_dev *adev = (struct acq400_dev *)param;
	const char* dname = chan->device->dev->driver->name;
	dev_dbg(DEVP(adev), "filter_axi: %s\n", chan->device->dev->driver->name);

	if (dname != 0 && strcmp(dname, "xilinx-acq400-dma") == 0){
		return true;
	}else{
		return false;
	}
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


int get_dma_channels(struct acq400_dev *adev)
{
	if (IS_AXI64(adev)){
		dma_cap_mask_t mask;
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		adev->dma_chan[0] = dma_request_channel(mask, filter_axi, adev);
		dev_info(DEVP(adev), "axi_dma not using standard driver");
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
	_release_dma_chan(adev, 0);
	_release_dma_chan(adev, 1);
}


int _acq420_continuous_start_dma(struct acq400_dev *adev)
{
	int rc = 0;
	int pollcat = 0;
	if (mutex_lock_interruptible(&adev->mutex)) {
		return -EINTR;
	}

	if (adev->busy || get_dma_channels(adev)){
		if (adev->busy){
			dev_err(DEVP(adev), "BUSY");
		}else{
			dev_err(DEVP(adev), "no dma chan");
		}
		rc = -EBUSY;
		mutex_unlock(&adev->mutex);
		return rc;
	}
	adev->dma_callback_done = 0;
	adev->fifo_isr_done = 0;
	mutex_unlock(&adev->mutex);

	dev_dbg(DEVP(adev), "acq420_continuous_start() %p id:%d : dma_chan: %p",
			adev, adev->pdev->dev.id, adev->dma_chan);



	if (adev->w_task == 0){
		int retry = 0;
		for(; IS_ERR_OR_NULL(adev->w_task); ++retry){
			dev_dbg(DEVP(adev), "acq420_continuous_start() kthread_run()");
			adev->w_task = kthread_run(
				IS_AXI64(adev)? axi64_data_loop: ai_data_loop, adev,
				"%s.ai", devname(adev));
			if (IS_ERR_OR_NULL(adev->w_task)){
				dev_err(DEVP(adev), "ERROR: failed to start task %p", adev->w_task);
				if (++retry > 5){
					BUG();
				}
				msleep(10*retry);
			}else{
				break;
			}
		}
	}else{
		dev_warn(DEVP(adev),
				"acq420_continuous_start() task already running ?\n");
	}

	if (adev->h_task == 0){
		adev->h_task = kthread_run(fifo_monitor, adev, "%s.fm", devname(adev));
	}
	while(!adev->task_active){
		yield();
		if ((++pollcat&0x1ffff) == 0){
			dev_warn(DEVP(adev), "Polling for task active");
			if (pollcat > 0x60000){
				dev_err(DEVP(adev), "ERROR: task_active not happening");
				rc = -1;
				break;
			}
		}
	}
	adev->busy = 1;
	return rc;
}

int _onStart(struct acq400_dev *adev)
{
	if (mutex_lock_interruptible(&adev->mutex)) {
		return -EINTR;
	}
	adev->oneshot = 0;
	adev->stats.shot++;
	adev->stats.run = 1;
	adev->cursor.offset = 0;
	memset(&adev->rt, 0, sizeof(struct RUN_TIME));
	acq400_clear_histo(adev);
	mutex_unlock(&adev->mutex);
	return 0;
}
int _acq420_continuous_start(struct acq400_dev *adev, int dma_start)
{
	int rc;
	dev_dbg(DEVP(adev), "_acq420_continuous_start() 01 dma_start:%d", dma_start);
	rc = _onStart(adev);
	if (rc){
		return rc;
	}
	if (dma_start){
		int rc =_acq420_continuous_start_dma(adev);
		if (rc != 0){
			return rc;
		}
	}

	adev->onStart(adev);
	dev_dbg(DEVP(adev), "_acq420_continuous_start() 99");
	return 0;
}

int acq420_continuous_start(struct inode *inode, struct file *file)
{
	return _acq420_continuous_start(ACQ400_DEV(file), 1);
}


int acq400_set_bufferlen(struct acq400_dev *adev, int _bufferlen)
{
	/* client may request sub-buffer size eg to ensure data align
	 * 96 x 4 channels, best length = 1044480
	 */
	if (_bufferlen){
		adev->bufferlen = min(bufferlen, _bufferlen);
	}else{
		adev->bufferlen = bufferlen;
	}

	return adev->bufferlen;
}
int acq2006_continuous_start(struct inode *inode, struct file *file)
/* this sequence CRITICAL for a clean start
 * (1) reset the aggregator, leaving it disabled
 * (2) reset the data engine, leaving it enabled (0xf1000000 : waiting flush)
 * (3) enable the DMA, sends the flush, data engine => 0xf3000000
 * (4) enable the aggregator
 * (5) now enable the trigger.
 */
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	int rc;

	empty_lists(adev);
	dev_dbg(DEVP(adev), "acq2006_continuous_start() 01");
	rc = _onStart(adev);
	if (rc){
		return rc;
	}
	fiferr = FIFERR;
	_onStart(adev);
	adev->RW32_debug = agg_reset_dbg;
	acq2006_aggregator_reset(adev);				/* (1) */
	sc_data_engine_reset_enable(DATA_ENGINE_0);	/* (2) */

	if (agg_reset_dbg) dev_info(DEVP(adev), "start dma");
	rc =_acq420_continuous_start_dma(adev);			/* (3) */
	if (rc != 0){
		dev_info(DEVP(adev), "acq2006_continuous_start() 66");
		return rc;
	}


	acq2006_aggregator_enable(adev);			/* (4) */

	dev_dbg(DEVP(adev), "acq2006_continuous_start() acq400_enable_trg %d",
			adev->aggregator_set[0]->of_prams.site);

	/* (5) */
	acq400_visit_set(adev->aggregator_set, acq400_enable_trg_if_master);
/*
	acq400_visit_set()
	acq400_enable_trg(adev->aggregator_set[0], 1);
*/
	adev->RW32_debug = 0;
	dev_dbg(DEVP(adev), "acq2006_continuous_start() 99");
	return 0;
}


void acq400_bq_notify(struct acq400_dev *adev, struct HBM *hbm)
{
	struct acq400_path_descriptor *cur;
	struct acq400_path_descriptor *tmp;
	int ix = hbm->ix;
	int nelems = 0;
	int th = nbuffers - 2;

	mutex_lock(&adev->bq_clients_mutex);

	/* _safe should not be needed since we're mutexed, but .. */
	list_for_each_entry_safe(cur, tmp, &adev->bq_clients, bq_list){
		struct BQ* bq = &cur->bq;
		int nq = CIRC_CNT(bq->head, bq->tail, bq->bq_len);
		if (CIRC_SPACE(bq->head, bq->tail, bq->bq_len) < 1 || nq > th){
			bq->head = bq->tail = 0;
			++adev->bq_overruns;
		}
		bq->buf[bq->head] = ix;
		smp_store_release(&bq->head, (bq->head+1)&(bq->bq_len-1));
		wake_up_interruptible(&cur->waitq);
		++nelems;
		if (nq > adev->bq_max) adev->bq_max = nq;
	}
	mutex_unlock(&adev->bq_clients_mutex);
	dev_dbg(DEVP(adev), "acq400_bq_notify() nelems:%d", nelems);
}

ssize_t acq400_continuous_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
/* NB: waits for a full buffer to ARRIVE, but only returns the 2 char ID */
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	char* lbuf = PD(file)->lbuf;
	int rc = 0;
	struct HBM *hbm;
	int nread;
	unsigned long now;


	adev->stats.reads++;
	adev->count = count;
	adev->this_count = 0;
	set_continuous_reader(adev);

	if (adev->rt.please_stop){
		return -1;		/* EOF ? */
	}
	while (!list_empty(&adev->OPENS)){
		putEmpty(adev);
	}

	dev_dbg(DEVP(adev), "acq400_continuous_read():getFull()");

	switch((rc = getFull(adev, &hbm, GF_WAIT))){
	case GET_FULL_OK:
		break;
	case GET_FULL_DONE:
		dev_warn(DEVP(adev), "finished");
		return -1;		/* finished  ret 0 WBN? */
	case GET_FULL_REFILL_ERR:
		dev_warn(DEVP(adev), "refill error\n");
		return -1;
	default:
		dev_warn(DEVP(adev), "interrupted\n");
		return rc;
	}

	if (list_empty(&adev->OPENS)){
		dev_warn(DEVP(adev), "no buffer available");
		return -1;
	}


	dev_dbg(DEVP(adev), "acq400_continuous_read():getFull() : %d", hbm->ix);

	/* update every hb0 or at least once per second */
	now = get_seconds() + hb0_no_ratelimit;
	/* ratelimited to 1Hz - client gets current and previous hbm */
	if (adev->rt.hbm_m1 != 0 && now != adev->rt.hb0_last){
		adev->rt.hb0_count++;
		adev->rt.hb0_ix[0] = adev->rt.hbm_m1->ix;
		adev->rt.hb0_ix[1] = hbm->ix;
		adev->rt.hb0_last = now;
		dev_dbg(DEVP(adev), "hb0 %d", adev->rt.hb0_count);
		dma_sync_single_for_cpu(DEVP(adev), hbm->pa, hbm->len, hbm->dir);
		wake_up_interruptible(&adev->hb0_marker);
	}else if (sync_continuous){
		/* this operation is surprisingly expensive. Use sparingly */
		dma_sync_single_for_cpu(DEVP(adev), hbm->pa, hbm->len, hbm->dir);
	}

	nread = sprintf(lbuf, "%02d\n", hbm->ix);
	adev->rt.hbm_m1 = hbm;

	acq400_bq_notify(adev, hbm);
#if 0
	/* pick off any other queued buffers ..
	 * enable only when acq400_stream can handle multiple responses..
	 */
	while(getFull(adev, &hbm, GF_NOWAIT) == GET_FULL_OK){
		acq400_bq_notify(adev, hbm);
		nread += sprintf(lbuf+nread, "%02d\n", hbm->ix);
		if (nread + 10 > MAXLBUF){
			break;
		}
	}
#endif
	rc = copy_to_user(buf, lbuf, nread);
	return rc? -rc: nread;
}



void _acq420_continuous_dma_stop(struct acq400_dev *adev)
{
	unsigned long work_task_wait = 0;

	while(mutex_lock_interruptible(&adev->mutex)) {
		;
	}
	if (adev->task_active && !IS_ERR_OR_NULL(adev->w_task)){
		kthread_stop(adev->w_task);
	}

	if (adev->h_task != 0){
		kthread_stop(adev->h_task);
		adev->h_task = 0;
	}
	mutex_unlock(&adev->mutex);

	wake_up_interruptible(&adev->DMA_READY);
	wake_up_interruptible(&adev->w_waitq);

	while(adev->task_active){
		work_task_wait = jiffies + msecs_to_jiffies(1000);
		while (adev->task_active){
			yield();
			if (time_after_eq(jiffies, work_task_wait)){
				dev_warn(DEVP(adev), "WAITING for work task\n");
				break;
			}
		}
	}
	mutex_lock(&adev->mutex);
	adev->w_task = 0;
	mutex_unlock(&adev->mutex);

	move_list_to_empty(adev, &adev->OPENS);
	move_list_to_empty(adev, &adev->REFILLS);
	move_list_to_empty(adev, &adev->INFLIGHT);
	adev->busy = 0;

	release_dma_channels(adev);
	adev->stats.run = 0;
}

void _acq420_continuous_stop(struct acq400_dev *adev, int dma_stop)
{
	adev->onStop(adev);

	dev_dbg(DEVP(adev), "acq420_continuous_stop() kthread_stop called\n");

	if (dma_stop){
		_acq420_continuous_dma_stop(adev);
	}else{
		adev->stats.run = 0;
	}
	if (acq400_event_count_limit &&
		adev->rt.event_count >= acq400_event_count_limit){
		acq400_enable_event0(adev, 1);
	}
	dev_dbg(DEVP(adev), "acq420_continuous_stop(): quitting ctrl:%08x",
			acq400rd32(adev, ADC_CTRL));
}
int acq420_continuous_stop(struct inode *inode, struct file *file)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	fiferr = 0;				/* don't care about any errs now */
	dev_dbg(DEVP(adev), "acq420_continuous_stop() fiferr clr from %08x", FIFERR);
	clr_continuous_reader(adev);
	_acq420_continuous_stop(ACQ400_DEV(file), 1);

	return acq400_release(inode, file);
}

int acq2006_continuous_stop(struct inode *inode, struct file *file)
{
	struct acq400_dev *adev = ACQ400_DEV(file);

	fiferr = 0;				/* don't care about any errs now */
	dev_dbg(DEVP(adev), "acq2006_continuous_stop() fiferr clr from %08x", FIFERR);
	acq2006_aggregator_disable(adev);
	_acq420_continuous_dma_stop(adev);
	clr_continuous_reader(adev);

	if (adev->rt.event_count >= acq400_event_count_limit){
		dev_dbg(DEVP(adev), "acq2006_continuous_stop() restore event..");
		acq400_enable_event0(adev, 1);
	}
	return acq400_release(inode, file);
}

void acq2006_estop(struct acq400_dev *adev)
{
	fiferr = 0;				/* don't care about any errs now */

	dev_dbg(DEVP(adev), "acq2006_estop() fiferr clr from %08x", FIFERR);
	acq2006_aggregator_disable(adev);
	_acq420_continuous_dma_stop(adev);
	dev_dbg(DEVP(adev), "acq2006_estop() 99");
}

static unsigned int acq420_continuous_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	int rc = 0;
	int wait = 0;

	if (!list_empty(&adev->REFILLS)){
		rc = POLLIN|POLLRDNORM;
	}else if (adev->rt.refill_error){
		rc = POLLERR;
	}else if (adev->rt.please_stop){
		rc = POLLHUP;
	}else{
		wait = 1;
		poll_wait(file, &adev->refill_ready, poll_table);
		if (!list_empty(&adev->REFILLS)){
			rc = POLLIN|POLLRDNORM;
		}else if (adev->rt.refill_error){
			rc = POLLERR;
		}else if (adev->rt.please_stop){
			rc = POLLHUP;
		}
	}
	dev_dbg(DEVP(adev), "acq420_continuous_poll() %d return %d", wait, rc);
	return rc;
}
ssize_t acq400_hb0_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	char HB0[16];

	int rc;
	/* force wait until next .. this is very conservative, we could
	 * stash the hb0 in DESCR for use between calls.
	 * But this way is self-regulating. This is for human monitor,
	 * not an attempt to handle ALL the data
	 */
	unsigned hb0_count = adev->rt.hb0_count;

	if (wait_event_interruptible(
			adev->hb0_marker,
			adev->rt.hb0_count != hb0_count ||
			adev->rt.refill_error ||
			adev->rt.please_stop)){
		return -EINTR;
	} else if (adev->rt.please_stop){
		return -GET_FULL_DONE;
	} else if (adev->rt.refill_error){
		return -GET_FULL_REFILL_ERR;
	}
	sprintf(HB0, "%d %d\n", adev->rt.hb0_ix[0], adev->rt.hb0_ix[1]);
	count = min(count, strlen(HB0)+1);
	rc = copy_to_user(buf, HB0, count);
	if (rc){
		return -1;
	}

	*f_pos += count;
	return count;
}


int acq400_open_histo(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_histo = {
			.read = acq400_histo_read,
			.release = acq400_release
	};
	file->f_op = &acq400_fops_histo;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}



int acq420_open_continuous(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_continuous = {
			.open = acq420_continuous_start,
			.read = acq400_continuous_read,
			.release = acq420_continuous_stop,
			.poll = acq420_continuous_poll
	};
	static struct file_operations acq2006_fops_continuous = {
			.open = acq2006_continuous_start,
			.read = acq400_continuous_read,
			.release = acq2006_continuous_stop,
			.poll = acq420_continuous_poll
	};
	struct acq400_dev *adev = ACQ400_DEV(file);
	int rc = acq400_open_main(inode, file);
	if (rc){
		return rc;
	}
	if (IS_SC(adev)){
		file->f_op = &acq2006_fops_continuous;
	}else{
		file->f_op = &acq400_fops_continuous;
	}
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}

int acq420_sideported_start(struct inode *inode, struct file *file)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	dev_dbg(DEVP(adev), "acq420_sideported_start()");
	if (sideport_does_not_touch_trg == 0){
		acq400_enable_trg(adev, 0);
	}
	return _acq420_continuous_start(adev, 0);
}

ssize_t acq400_sideported_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	int rc;
	set_continuous_reader(adev);

	if (wait_event_interruptible(
			adev->refill_ready,
			!list_empty(&adev->REFILLS) ||
			adev->rt.refill_error ||
			adev->rt.please_stop)){
		return -EINTR;
	} else if (adev->rt.please_stop){
		rc = copy_to_user(buf, "D", 1);
		if (rc) {
			return -1;
		}
		return GET_FULL_DONE;
	} else if (adev->rt.refill_error){
		rc = copy_to_user(buf, "RE", 2);
		if (rc){
			return -1;
		}
		return GET_FULL_REFILL_ERR;
	}
	return 0;
}

int acq420_sideported_stop(struct inode *inode, struct file *file)
{
	struct acq400_dev *adev = ACQ400_DEV(file);

	clr_continuous_reader(adev);
	_acq420_continuous_stop(adev, 0);
	return acq400_release(inode, file);
}

static unsigned int acq420_sideported_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	unsigned int rc;

	if (!list_empty(&adev->REFILLS)){
		rc = POLLIN|POLLRDNORM;
	}else if (adev->rt.refill_error){
		rc = POLLERR;
	}else if (adev->rt.please_stop){
		rc = POLLHUP;
	}else{
		poll_wait(file, &adev->refill_ready, poll_table);
		if (!list_empty(&adev->REFILLS)){
			rc = POLLIN|POLLRDNORM;
		}else if (adev->rt.refill_error){
			rc = POLLERR;
		}else if (adev->rt.please_stop){
			rc = POLLHUP;
		}else{
			rc = 0;
		}
	}
	dev_dbg(DEVP(adev), "acq420_sideported_poll() ret %u\n", rc);
	return rc;
}
int acq420_open_sideported(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_sideported = {
			.open = acq420_sideported_start,
			.read = acq400_sideported_read,
			.release = acq420_sideported_stop,
			.poll = acq420_sideported_poll
	};
	int rc = acq400_open_main(inode, file);
	if (rc){
		return rc;
	}
	file->f_op = &acq400_fops_sideported;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}

int acq420_open_hb0(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_hb0 = {
			.read = acq400_hb0_read,
			.release = acq400_release
	};
	file->f_op = &acq400_fops_hb0;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}

int acq400_gpgmem_open(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	if (file->f_flags & O_WRONLY) {
		int iw;
		for (iw = 0; iw < adev->gpg_cursor; ++iw){
			iowrite32(0, adev->gpg_base+iw);
		}
		adev->gpg_cursor = 0;
	}
	return 0;
}
ssize_t acq400_gpgmem_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int len = adev->gpg_cursor*sizeof(u32);
	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	int rc;

	if (bcursor >= len){
		return 0;
	}else{
		int headroom = (len - bcursor);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, adev->gpg_buffer+bcursor, count);
	if (rc){
		return -1;
	}

	*f_pos += count;
	return count;
}

ssize_t acq400_gpgmem_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int len = GPG_MEM_ACTUAL;
	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	int rc;

	if (bcursor >= len){
		return 0;
	}else{
		int headroom = (len - bcursor);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_from_user(adev->gpg_buffer+bcursor, buf, count);
	if (rc){
		return -1;
	}
	*f_pos += count;
	adev->gpg_cursor += count/sizeof(u32);
	return count;

}

int set_gpg_top(struct acq400_dev* adev, u32 gpg_count)
{
	if (gpg_count >= 2){
		u32 gpg_ctrl = acq400rd32(adev, GPG_CTRL);
		u32 gpg_top = gpg_count - 1		// was count, not address
					 -1;		// GPG_2ND_LAST_ADDR
		gpg_top <<= GPG_CTRL_TOPADDR_SHL;
		gpg_top &= GPG_CTRL_TOPADDR;
		gpg_ctrl &= ~GPG_CTRL_TOPADDR;
		gpg_ctrl |= gpg_top;
		acq400wr32(adev, GPG_CTRL, gpg_ctrl);
		return 0;
	}else{
		dev_err(DEVP(adev), "set_gpg_top() ERROR: must have 2 or more entries");
		return -1;
	}
}

int acq400_gpgmem_release(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	unsigned* src = (unsigned *)adev->gpg_buffer;
	int iw;

	for (iw = 0; iw < adev->gpg_cursor; ++iw){
		iowrite32(src[iw], adev->gpg_base+iw);
	}
	dev_dbg(DEVP(adev), "acq400_gpgmem_release() %d\n", iw);
	return set_gpg_top(adev, adev->gpg_cursor);
}

int acq420_open_gpgmem(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_gpgmem = {
			.open = acq400_gpgmem_open,
			.read = acq400_gpgmem_read,
			.write = acq400_gpgmem_write,
			.release = acq400_gpgmem_release
	};
	file->f_op = &acq400_fops_gpgmem;
	return file->f_op->open(inode, file);
}

ssize_t acq420_sew_fifo_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ix = PD(file)->minor - ACQ420_MINOR_SEW1_FIFO;

	int rc = acq400_sew_fifo_write_bytes(adev, ix, buf, count);

	if (rc > 0){
		*f_pos += rc;
	}
	return rc;
}
int acq420_sew_fifo_release(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ix = PD(file)->minor - ACQ420_MINOR_SEW1_FIFO;
	return acq400_sew_fifo_destroy(adev, ix);
}

int acq420_sew_fifo_open(struct inode *inode, struct file *file)
{
	static struct file_operations acq420_sew_fifo = {
			.write = acq420_sew_fifo_write,
			.release = acq420_sew_fifo_release,
	};
	struct acq400_dev* adev = ACQ400_DEV(file);
	int ix = PD(file)->minor - ACQ420_MINOR_SEW1_FIFO;
	int busy;

	if (mutex_lock_interruptible(&adev->awg_mutex)) {
		return -EINTR;
	}
	busy = adev->sewFifo[ix].sf_task != 0;
	if (!busy){
		acq400_sew_fifo_init(adev, ix);
	}
	mutex_unlock(&adev->awg_mutex);
	if (busy){
		return -EBUSY;
	}

	file->f_op = &acq420_sew_fifo;
	return 0;
}

int streamdac_data_loop(void *data)
{
	struct acq400_dev *adev = (struct acq400_dev *)data;
	int rc = 0;
	struct HBM* hbm_full;

	dev_info(DEVP(adev), "streamdac_data_loop 01");

	while(1){
		dev_dbg(DEVP(adev), "streamdac_data_loop() wait_event");
		if (wait_event_interruptible(
				adev->w_waitq,
				!list_empty(&adev->REFILLS) ||
				adev->rt.please_stop)){
			rc = -EINTR;
			goto quit;
		}
		if (adev->rt.please_stop || kthread_should_stop()){
			goto quit;
		}

		/** @@todo : FIFO filling happens here */
		hbm_full = ao_getFull(adev);

		dev_dbg(DEVP(adev), "streamdac_data_loop() after ao_getFull(), hbm:%p", hbm_full);
		if (hbm_full == 0){
			goto quit;
		}
		dev_dbg(DEVP(adev), "streamdac_data_loop() ao_putEmpty %p", hbm_full);
		ao_putEmpty(adev, hbm_full);

		dev_dbg(DEVP(adev), "streamdac_data_loop() after ao_putEmpty(), hbm:%p", hbm_full);
		wake_up_interruptible(&adev->w_waitq);
	}
quit:
	dev_info(DEVP(adev), "streamdac_data_loop 99");
	move_list_to_empty(adev, &adev->REFILLS);
	move_list_to_empty(adev, &adev->INFLIGHT);
	return rc;
}
int acq400_streamdac_open(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	adev->stream_dac_consumer.hb = 0;
	adev->stream_dac_consumer.offset = 0;
	adev->w_task = kthread_run(
		streamdac_data_loop, adev, "%s.dac", devname(adev));
	return 0;
}


ssize_t acq400_streamdac_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int len = adev->bufferlen;
	char* va;
	int headroom;

	dev_dbg(DEVP(adev), "write 01");

	if (!adev->stream_dac_producer.hb){
		while((adev->stream_dac_producer.hb[0] = ao_getEmpty(adev)) == 0){
			dev_dbg(DEVP(adev), "ao_getEmpty() fail");
			if (wait_event_interruptible(
				adev->w_waitq,
				!list_empty(&adev->EMPTIES)) ){
				return -EINTR;
			}
		}
		adev->stream_dac_producer.offset = 0;
	}
	va = (char*)adev->stream_dac_producer.hb[0]->va;
	headroom = len - adev->stream_dac_producer.offset;
	if (count > headroom){
		count = headroom;
	}

	dev_dbg(DEVP(adev), "write copy %d", count);

	if(copy_from_user(va+adev->stream_dac_producer.offset, buf, count)){
		return -1;
	}
	adev->stream_dac_producer.offset += count;
	if (adev->stream_dac_producer.offset >= len){
		dev_dbg(DEVP(adev), "putFull %p", adev->stream_dac_producer.hb);
		ao_putFull(adev, adev->stream_dac_producer.hb[0]);
		adev->stream_dac_producer.hb = 0;
	}

	*f_pos += count;
	dev_dbg(DEVP(adev), "write 99 return %d", count);
	return count;
}


int acq400_streamdac_release(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	dev_info(DEVP(adev), "acq400_streamdac_release()");
	adev->rt.please_stop = 1;
	wake_up_interruptible(&adev->w_waitq);
	move_list_to_empty(adev, &adev->REFILLS);
	move_list_to_empty(adev, &adev->INFLIGHT);
	return 0;
}


int acq400_open_streamdac(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_streamdac = {
			.open = acq400_streamdac_open,
			.write = acq400_streamdac_write,
			.release = acq400_streamdac_release,
	};
	file->f_op = &acq400_fops_streamdac;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}


int acq400_event_open(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	u32 int_csr;

	if (mutex_lock_interruptible(&adev->bq_clients_mutex)) {
		return -ERESTARTSYS;
	}else{
		++adev->event_client_count;
		mutex_unlock(&adev->bq_clients_mutex);
	}
	int_csr = x400_get_interrupt(adev);
	x400_set_interrupt(adev, int_csr|ADC_INT_CSR_COS_EN_ALL);

	/* good luck using this in a 64-bit system ... */
	/*
	setup_timer( &adev->event_timer, event_isr, (unsigned)adev);
	mod_timer( &adev->event_timer, jiffies + msecs_to_jiffies(event_isr_msec));
	*/
	return 0;
}

void init_event_info(struct EventInfo *eventInfo)
{
	struct acq400_dev* adev0 = acq400_lookupSite(0);
	memset(eventInfo, 0, sizeof(struct EventInfo));

	mutex_lock(&adev0->list_mutex);
	/* event is somewhere between these two blocks */
	if (!list_empty(&adev0->REFILLS)){
		eventInfo->hbm0 = list_last_entry(&adev0->REFILLS, struct HBM, list);
	}else if (!list_empty(&adev0->OPENS)){
		eventInfo->hbm0 = list_last_entry(&adev0->OPENS, struct HBM, list);
	}
	if (!list_empty(&adev0->INFLIGHT)){
		eventInfo->hbm1 = list_first_entry(&adev0->INFLIGHT, struct HBM, list);
	}
	mutex_unlock(&adev0->list_mutex);
	eventInfo->pollin = 1;
}
ssize_t acq400_event_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	char lbuf[40];
	int nbytes;
	int rc;
	struct EventInfo eventInfo = PD(file)->eventInfo;
	struct acq400_dev* adev0 = acq400_lookupSite(0);
	u32 old_sample = adev->rt.samples_at_event;
	int timeout = 0;

	if (eventInfo.pollin){
		PD(file)->eventInfo.pollin = 0;
	}

	dev_dbg(DEVP(adev), "acq400_event_read() 01 pollin %d old_sample %d",
			eventInfo.pollin, old_sample);

	if (eventInfo.pollin == 0){
		/* force caller to wait fresh event. This is an auto-rate-limit
		 * it's also re-entrant (supports multiple clients each at own rate)
		 * NB: NO ATTEMPT to guarantee that all events processed. caveat emptor
		 */
		dev_dbg(DEVP(adev), "acq400_event_read() 10 wait event");

		rc = wait_event_interruptible_timeout(
				adev->event_waitq,
				adev->rt.samples_at_event != old_sample,
				event_to);
		if (rc < 0){
			return -EINTR;
		}else if (rc == 0){
			return -EAGAIN;
		}
		init_event_info(&eventInfo);
	}

	dev_dbg(DEVP(adev), "acq400_event_read() hbm0 %p hbm1 %p %s",
			eventInfo.hbm0, eventInfo.hbm1,
			eventInfo.hbm1? "must wait for hbm1 to complete": "ready");

	if (eventInfo.hbm0){
		dma_sync_single_for_cpu(DEVP(adev), eventInfo.hbm0->pa, eventInfo.hbm0->len, eventInfo.hbm0->dir);
	}
	if (eventInfo.hbm1){
		int rc = wait_event_interruptible_timeout(
			adev0->refill_ready,
			eventInfo.hbm1->bstate != BS_FILLING ||
				adev0->rt.refill_error ||
				adev0->rt.please_stop,
			event_to);
		if (rc < 0){
			return -EINTR;
		}else if (rc == 0){
			timeout = 1;
		}
		dma_sync_single_for_cpu(DEVP(adev), eventInfo.hbm1->pa, eventInfo.hbm1->len, eventInfo.hbm1->dir);
	}

	nbytes = snprintf(lbuf, sizeof(lbuf), "%u %d %d %s 0x%08x %d\n",
			adev->rt.samples_at_event,
			eventInfo.hbm0? eventInfo.hbm0->ix: -1,
			eventInfo.hbm1? eventInfo.hbm1->ix: -1, timeout? "TO": "OK",
			adev->atd.event_source,
			adev->rt.event_count);

	if (HAS_DTD(adev) && adev->atd.event_source){
		adev->atd.event_source = 0;
	}
	rc = copy_to_user(buf, lbuf, nbytes);
	if (rc != 0){
		rc = -1;
	}else{
		rc = nbytes;
	}
	dev_dbg(DEVP(adev), "acq400_event_read() 99 \"%s\" rc %d", lbuf, rc);
	return rc;
}


static unsigned int acq400_event_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	unsigned rc;

	if (adev->rt.samples_at_event){
		init_event_info(&PD(file)->eventInfo);
		rc = POLLIN;
	}else{
		poll_wait(file, &adev->event_waitq, poll_table);
		if (adev->rt.samples_at_event){
			init_event_info(&PD(file)->eventInfo);
			rc = POLLIN;
		}else{
			rc = 0;
		}
	}
	dev_dbg(DEVP(adev), "acq400_event_poll() return %u", rc);
	return rc;
}
int acq400_event_release(struct inode *inode, struct file *file)
{
	struct acq400_dev* adev = ACQ400_DEV(file);

	if (mutex_lock_interruptible(&adev->bq_clients_mutex)) {
		return -ERESTARTSYS;
	}else{
		if (--adev->event_client_count == 0){
			u32 int_csr = x400_get_interrupt(adev);

			int_csr &= ~(ADC_INT_CSR_EVENT1_EN|ADC_INT_CSR_EVENT0_EN);
			x400_set_interrupt(adev, int_csr);
		}
		mutex_unlock(&adev->bq_clients_mutex);
	}
	return 0;
}

int acq400_open_event(struct inode *inode, struct file *file)
{
	static struct file_operations acq400_fops_event = {
			.open = acq400_event_open,
			.read = acq400_event_read,
			.poll = acq400_event_poll,
			.release = acq400_event_release
	};
	file->f_op = &acq400_fops_event;

	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}

void bolo_awg_commit(struct acq400_dev* adev)
{
	int nwords = adev->bolo8.awg_buffer_cursor/sizeof(u32);
	u32 *src = (u32*)adev->bolo8.awg_buffer;
	int ii;
	for (ii = 0; ii < nwords; ++ii){
		acq400wr32(adev, B8_AWG_MEM+sizeof(u32)*ii, src[ii]);
	}
	/* wavetop in shorts, starting from zero */
	acq400wr32(adev, B8_DAC_WAVE_TOP, adev->bolo8.awg_buffer_cursor/sizeof(u16)-1);
	acq400wr32(adev, B8_DAC_CON, acq400rd32(adev, B8_DAC_CON)|B8_DAC_CON_ENA);
}
int bolo_awg_open(struct inode *inode, struct file *file)
/* if write mode, reset length */
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	if ( (file->f_flags & O_ACCMODE) == O_WRONLY) {
		adev->bolo8.awg_buffer_cursor = 0;
	}
	return 0;
}
int bolo_awg_release(struct inode *inode, struct file *file)
/* if it was a write, commit to memory and set length */
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	if ( (file->f_flags & O_ACCMODE) == O_WRONLY) {
		bolo_awg_commit(adev);
	}
	return 0;
}

ssize_t bolo_awg_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int len = adev->bolo8.awg_buffer_cursor;
	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	int rc;

	if (bcursor >= len){
		return 0;
	}else{
		int headroom = (len - bcursor);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, adev->bolo8.awg_buffer+bcursor, count);
	if (rc){
		return -1;
	}


	*f_pos += count;
	return count;
}

ssize_t bolo_awg_write(
	struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int len = adev->bolo8.awg_buffer_max;
	unsigned bcursor = adev->bolo8.awg_buffer_cursor;
	int rc;

	if (bcursor >= len){
		return 0;
	}else{
		int headroom = (len - bcursor);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_from_user(adev->bolo8.awg_buffer+bcursor, buf, count);
	if (rc){
		return -1;
	}
	*f_pos += count;
	adev->bolo8.awg_buffer_cursor += count;
	return count;
}

int bolo_open_awg(struct inode *inode, struct file *file)
{
	static struct file_operations fops = {
			.open = bolo_awg_open,
			.write = bolo_awg_write,
			.read = bolo_awg_read,
			.release = bolo_awg_release,
	};
	file->f_op = &fops;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}



int xo400_awg_open(struct inode *inode, struct file *file)
/* if write mode, reset length */
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	if ( (file->f_flags & O_ACCMODE) == O_WRONLY) {
		xo400_reset_playloop(adev, adev->AO_playloop.length = 0);
	}
	return 0;
}
int xo400_awg_release(struct inode *inode, struct file *file)
/* if it was a write, commit to memory and set length */
{
	struct acq400_dev* adev = ACQ400_DEV(file);

	if ( (file->f_flags & O_ACCMODE) != O_RDONLY) {
		adev->AO_playloop.one_shot =
			iminor(inode) == AO420_MINOR_HB0_AWG_ONCE? AO_oneshot:
			iminor(inode) == AO420_MINOR_HB0_AWG_ONCE_RETRIG? AO_oneshot_rearm:
					AO_continuous;
		xo400_reset_playloop(adev, adev->AO_playloop.length);
	}
	return 0;
}

ssize_t xo400_awg_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	int pll = AOSAMPLES2BYTES(adev, adev->AO_playloop.length);
	unsigned len = adev->cursor.hb[0]->len;

	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	unsigned ib = bcursor/len;
	unsigned offset = bcursor - ib*len;
	int rc;

	if (bcursor >= pll){
		return 0;
	}else{
		int headroom = min(len-offset, pll-bcursor);
		if (count > headroom){
			count = headroom;
		}
	}
	rc = copy_to_user(buf, (char*)adev->cursor.hb[ib]->va + offset, count);
	if (rc){
		return -1;
	}


	*f_pos += count;
	return count;
}

void flip_x16_sign_bit(unsigned short* bp, int nwords)
{
	const unsigned short sbit = 1<<15;

	while(nwords--){
		*bp++ ^= sbit;
	}
}
ssize_t xo400_awg_write(
	struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_dev* adev = ACQ400_DEV(file);
	unsigned len = adev->cursor.hb[0]->len;

	unsigned bcursor = *f_pos;	/* f_pos counts in bytes */
	unsigned ib = bcursor/len;
	/** @@todo check ib within bounds .. what bounds? */
	unsigned offset = bcursor - ib*len;
	int headroom = (len - offset);
	char* dst = (char*)adev->cursor.hb[ib]->va + offset;
	int rc;

	if (count > headroom){
		count = headroom;
	}

	rc = copy_from_user(dst, buf, count);

	if (rc){
		return -1;
	}
	if (IS_AO424(adev) && SPAN_IS_BIPOLAR(adev)){
		flip_x16_sign_bit((unsigned short*)dst, count/sizeof(unsigned short));
	}

	dev_dbg(DEVP(adev),
		"dma_sync_single_for_device() %08x %d len:%d demand:%d %d (DMA_TO_DEVICE=%d)",
		adev->cursor.hb[ib]->pa + offset, count,
		len, bcursor+count,
		adev->cursor.hb[ib]->dir, DMA_TO_DEVICE);

	dma_sync_single_for_device(DEVP(adev),
			adev->cursor.hb[ib]->pa + offset, count, adev->cursor.hb[ib]->dir);

	*f_pos += count;
	adev->AO_playloop.length += AOBYTES2SAMPLES(adev, count);
	return count;
}

int xo400_open_awg(struct inode *inode, struct file *file)
{
	static struct file_operations fops = {
			.open = xo400_awg_open,
			.write = xo400_awg_write,
			.read = xo400_awg_read,
			.release = xo400_awg_release,
	};
	file->f_op = &fops;
	if (file->f_op->open){
		return file->f_op->open(inode, file);
	}else{
		return 0;
	}
}

int acq420_reserve_release(struct inode *inode, struct file *file)
/* if it was a write, commit to memory and set length */
{
	replace_all(PD(file));
	return acq400_release(inode, file);
}

ssize_t acq420_reserve_read(
	struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	wait_queue_head_t waitq;	/* no one gonna wake this sucker */

	init_waitqueue_head(&waitq);

	wait_event_interruptible(waitq, 0);  /* but it will accept a signal .. */
	return -EINTR;
}

int acq420_reserve_dist_open(struct inode *inode, struct file *file)
/* simply reserve block 0 on open. Can get clever later */
{
	static struct file_operations fops = {
		.open = acq420_reserve_dist_open,
		.read = acq420_reserve_read,
		.release = acq420_reserve_release,
	};

	file->f_op = &fops;
	if (distributor_first_buffer > 0){
		int ib;
		int rc = 0;
		for (ib = distributor_first_buffer; ib < nbuffers; ++ib){
			rc = reserve(PD(file), ib);
			if (rc != 0){
				dev_err(PDEV(file), "failed to reserve buffer %d", ib);
				break;
			}
		}
		return rc;
	}else{
		return -1;
	}

}
int acq420_reserve_open(struct inode *inode, struct file *file)
/* simply reserve block 0 on open. Can get clever later */
{
	static struct file_operations fops = {
		.open = acq420_reserve_open,
		.read = acq420_reserve_read,
		.release = acq420_reserve_release,
	};
	file->f_op = &fops;
	return reserve(PD(file), 0);
}

int acq400_init_descriptor(void** pd)
{
	struct acq400_path_descriptor* pdesc = kzalloc(PDSZ, GFP_KERNEL);
	INIT_LIST_HEAD(&pdesc->bq_list);
	init_waitqueue_head(&pdesc->waitq);

	*pd = pdesc;
	return 0;
}

ssize_t acq400_bq_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
{
	struct acq400_path_descriptor* pdesc = PD(file);
	struct acq400_dev* adev = pdesc->dev;
	struct BQ* bq = &pdesc->bq;
	char lbuf[32];
	int bc;
	int rc;

	dev_dbg(DEVP(adev), "wait_event_interruptible(%p)", &pdesc->waitq);
	if (wait_event_interruptible(
		pdesc->waitq,
		CIRC_CNT(bq->head, bq->tail, bq->bq_len))){
		return -EINTR;
	}

	bc = snprintf(lbuf, 32, "%03d\n", bq->buf[bq->tail]);
	smp_store_release(&bq->tail, (bq->tail+1)&(bq->bq_len-1));

	if (bc > count){
		return -ENOSPC;
	}
	rc = copy_to_user(buf, lbuf, bc);
	if (rc){
		return -rc;
	}else{
		return bc;
	}
}

static unsigned int acq400_bq_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	struct acq400_path_descriptor* pdesc = PD(file);
	struct BQ* bq = &pdesc->bq;

	if (CIRC_SPACE(bq->head, bq->tail, bq->bq_len)){
		return POLLIN|POLLRDNORM;
	}else{
		poll_wait(file, &pdesc->waitq, poll_table);
		if (CIRC_SPACE(bq->head, bq->tail, bq->bq_len)){
			return POLLIN|POLLRDNORM;
		}else{
			return 0;
		}
	}
}

int acq400_bq_release(struct inode *inode, struct file *file)
{
	struct acq400_path_descriptor* pdesc = PD(file);
	struct acq400_dev* adev = pdesc->dev;

	struct acq400_path_descriptor *cur;
	int nelems = 0;


        if (mutex_lock_interruptible(&adev->bq_clients_mutex)) {
	       return -ERESTARTSYS;
	}
        list_del_init(&pdesc->bq_list);
        /* diagnostic */
        list_for_each_entry(cur, &adev->bq_clients, bq_list){
               	++nelems;
        }
        mutex_unlock(&adev->bq_clients_mutex);

        if (nelems){
        	dev_dbg(DEVP(adev), "nelems:%d", nelems);
        }else{
        	dev_dbg(DEVP(adev), "nelems:%d", nelems);
        }

        kfree(pdesc->bq.buf);
        return acq400_release(inode, file);
}

int acq400_bq_open(struct inode *inode, struct file *file, int backlog)
{
	static struct file_operations acq400_fops_bq = {
			.read = acq400_bq_read,
			.release = acq400_bq_release,
			.poll = acq400_bq_poll
	};

	struct acq400_path_descriptor* pdesc = PD(file);
	struct acq400_dev* adev = pdesc->dev;

	struct acq400_path_descriptor *cur;
	int nelems = 0;

        file->f_op = &acq400_fops_bq;

        INIT_LIST_HEAD(&pdesc->bq_list);
        pdesc->bq.bq_len = backlog;
        pdesc->bq.buf = kzalloc(pdesc->bq.bq_len*sizeof(unsigned), GFP_KERNEL);

        if (mutex_lock_interruptible(&adev->bq_clients_mutex)) {
	       return -ERESTARTSYS;
	}
        list_add_tail(&pdesc->bq_list, &adev->bq_clients);
        list_for_each_entry(cur, &adev->bq_clients, bq_list){
        	++nelems;
        }
        mutex_unlock(&adev->bq_clients_mutex);
        if (nelems > 1){
        	dev_dbg(DEVP(adev), "nelems:%d", nelems);
        }else{
        	dev_dbg(DEVP(adev), "nelems:%d", nelems);
        }
	return 0;
}

int acq400_mmap_bar_page(struct file* file, struct vm_area_struct* vma)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = 0x1000;
	unsigned long pa = adev->dev_physaddr + ACQ400_MINOR_MAP_PAGE_OFFSET(PD(file)->minor);
	unsigned pfn = pa>> PAGE_SHIFT;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	dev_dbg(DEVP(adev), "acq400_mmap_bar_page pa:0x%08lx vsize %lu psize %lu %s",
		pa, vsize, psize, vsize>psize? "EINVAL": "OK");

	if (vsize > psize){
		return -EINVAL;                   /* request too big */
	}
	if (io_remap_pfn_range(
		vma, vma->vm_start, pfn, vsize, vma->vm_page_prot)){
		return -EAGAIN;
	}else{
		return 0;
	}
}

int acq400_map_page_open(struct inode* inode, struct file* file)
{
	static struct file_operations acq400_map_page_fops = {
	        .owner = THIS_MODULE,
	        .release = acq400_release,
	        .mmap = acq400_mmap_bar_page
	};
	file->f_op = &acq400_map_page_fops;
	return 0;
}

int acq400_mmap_bar_atd(struct file* file, struct vm_area_struct* vma)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = AXI_ATD_LEN;
	unsigned pfn = (adev->dev_physaddr + AXI_ATD_RAM)>> PAGE_SHIFT;

	dev_dbg(DEVP(adev), "%c vsize %lu psize %lu %s",
		'D', vsize, psize, vsize>psize? "EINVAL": "OK");

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (vsize > psize){
		return -EINVAL;                   /* request too big */
	}
	if (io_remap_pfn_range(
		vma, vma->vm_start, pfn, vsize, vma->vm_page_prot)){
		return -EAGAIN;
	}else{
		return 0;
	}
}

int acq400_atd_open(struct inode* inode, struct file* file)
{
	static struct file_operations acq400_atd_fops = {
	        .owner = THIS_MODULE,
	        .release = acq400_release,
	        .mmap = acq400_mmap_bar_atd
	};
	file->f_op = &acq400_atd_fops;
	return 0;
}

int acq400_open(struct inode *inode, struct file *file)
{
        struct acq400_dev *adev;
        int minor;
        int rc;

        acq400_init_descriptor(&file->private_data);


        PD(file)->dev = adev = container_of(inode->i_cdev, struct acq400_dev, cdev);
        PD(file)->minor = minor = MINOR(inode->i_rdev);
        INIT_LIST_HEAD(&PD(file)->RESERVED);

        dev_dbg(DEVP(adev), "hello: minor:%d\n", minor);

        if (minor >= ACQ420_MINOR_BUF && minor <= ACQ420_MINOR_BUF2){
        	rc = acq400_open_hb(inode, file);
        } else if (minor >= ACQ420_MINOR_CHAN && minor <= ACQ420_MINOR_CHAN2){
        	rc = -ENODEV;  	// @@todo maybe later0
        } else {
        	switch(minor){
        	case ACQ420_MINOR_SIDEPORTED:
        		rc = acq420_open_sideported(inode, file);
        		break;
        	case ACQ420_MINOR_CONTINUOUS:
        		rc = acq420_open_continuous(inode, file);
        		break;
        	case ACQ420_MINOR_HISTO:
        		rc = acq400_open_histo(inode, file);
        		break;
        	case ACQ420_MINOR_HB0:
        		rc = acq420_open_hb0(inode, file);
        		break;
        	case ACQ420_MINOR_GPGMEM:
        		rc = acq420_open_gpgmem(inode, file);
        		break;
        	case ACQ420_MINOR_EVENT:
        		rc = acq400_open_event(inode, file);
        		break;
        	case ACQ420_MINOR_0:
        		rc = acq400_open_main(inode, file);
        		break;
        	case ACQ420_MINOR_STREAMDAC:
        		rc = acq400_open_streamdac(inode, file);
        		break;
        	case ACQ420_MINOR_BOLO_AWG:
        		rc = bolo_open_awg(inode, file);
        		break;
        	case AO420_MINOR_HB0_AWG_ONCE:
        	case AO420_MINOR_HB0_AWG_LOOP:
        	case AO420_MINOR_HB0_AWG_ONCE_RETRIG:
        		rc = xo400_open_awg(inode, file);
        		break;
        	case ACQ420_MINOR_RESERVE_BLOCKS:
        		rc = acq420_reserve_open(inode, file);
        		break;
        	case ACQ400_MINOR_RSV_DIST:
        		rc = acq420_reserve_dist_open(inode, file);
        		break;
        	case ACQ400_MINOR_BQ_NOWAIT:
        		rc = acq400_bq_open(inode, file, BQ_MIN_BACKLOG);
        		break;
        	case ACQ400_MINOR_BQ_FULL:
        		rc = acq400_bq_open(inode, file, BQ_MAX_BACKLOG);
        		break;
        	case ACQ420_MINOR_SEW1_FIFO:
        	case ACQ420_MINOR_SEW2_FIFO:
        		rc = acq420_sew_fifo_open(inode, file);
        		break;
        	case ACQ400_MINOR_ATD:
        		rc = acq400_atd_open(inode, file);
        		break;
        	default:
        		if (minor >= ACQ400_MINOR_MAP_PAGE &&
        		    minor < ACQ400_MINOR_MAP_PAGE+16  ){
        			rc = acq400_map_page_open(inode, file);
        		}else{
        			rc = -ENODEV;
        		}
        	}
        }

        if (rc != 0){
        	dev_err(DEVP(adev), "acq400_open FAIL minor:%d rc:%d", minor, rc);
        	if (PD(file)) kfree(PD(file));
        	SETPD(file, 0);
        }
        return rc;
}


int acq400_release(struct inode *inode, struct file *file)
{
        struct acq400_dev *adev = ACQ400_DEV(file);

        if (mutex_lock_interruptible(&adev->mutex)) {
                return -EINTR;
        }

        /* Manage writes via reference counts */
        switch (file->f_flags & O_ACCMODE) {
        case O_RDONLY:
                break;
        case O_WRONLY:
                adev->writers--;
                break;
        case O_RDWR:
        default:
                adev->writers--;
        }

        adev->stats.closes++;

        mutex_unlock(&adev->mutex);

        if (PD(file)) kfree(PD(file));
        return 0;
}

int acq400_hb_release(struct inode *inode, struct file *file)
{
        struct acq400_dev *adev = ACQ400_DEV(file);
        int ibuf = BUFFER(PD(file)->minor);
        struct HBM *hbm = adev->hb[ibuf];

        /* Manage writes via reference counts */
        switch (file->f_flags & O_ACCMODE) {
        case O_WRONLY:
        case O_RDWR:
        	dev_dbg(DEVP(adev),
        	"acq400_hb_release() mode %x, dma_sync_single_for_device: pa:0x%08x len:%d dir:%d",
		file->f_flags & O_ACCMODE, hbm->pa, hbm->len, hbm->dir);
        	dma_sync_single_for_device(DEVP(adev), hbm->pa, hbm->len, hbm->dir);
        default:
        	;
        }

        return acq400_release(inode, file);
}

ssize_t acq400_read(struct file *file, char __user *buf, size_t count,
        loff_t *f_pos)
{
	return -EINVAL;
}

ssize_t acq400_write(struct file *file, const char __user *buf, size_t count,
        loff_t *f_pos)
{
        return -EINVAL;
}

int acq400_mmap_bar(struct file* file, struct vm_area_struct* vma)
{
	struct acq400_dev *adev = ACQ400_DEV(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = adev->dev_addrsize;
	unsigned pfn = adev->dev_physaddr >> PAGE_SHIFT;

	dev_dbg(DEVP(adev), "%c vsize %lu psize %lu %s",
		'D', vsize, psize, vsize>psize? "EINVAL": "OK");

	if (vsize > psize){
		return -EINVAL;                   /* request too big */
	}
	if (io_remap_pfn_range(
		vma, vma->vm_start, pfn, vsize, vma->vm_page_prot)){
		return -EAGAIN;
	}else{
		return 0;
	}
}


static void add_fifo_histo(struct acq400_dev *adev, u32 status)
{
	adev->fifo_histo[STATUS_TO_HISTO(status)]++;
}

static void add_fifo_histo_ao42x(struct acq400_dev *adev, unsigned samples)
{
	(adev->fifo_histo[samples >> adev->xo.hshift])++;
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

/* minimum PL330 size
 * could be a physical limit, but also pragmatic given slow setup time
 */


static int xo400_write_fifo_dma(struct acq400_dev* adev, int frombyte, int bytes)
{
	/* index from cursor to find correct hb.
	 * HB's are NOT physically contiguous ..
	 * WARNING: assumes all HB's are the same length ..
	 * WARNING: assumes NEVER overlaps HB end ..
	 * This is OK provided xo400_write_fifo_dma is a factor of hb->len (it is).
	 * easy to catch anyway .. warn that we took the catch ..
	 */
	unsigned len = adev->cursor.hb[0]->len;
	unsigned ib = frombyte/len;
	unsigned offset = frombyte - ib*len;
	int rc;

	if (unlikely(offset + bytes > len)){
		if (xo_use_contiguous_pa_if_possible && adev->cursor.hb[ib]->pa + len == adev->cursor.hb[ib+1]->pa){
			dev_dbg(DEVP(adev), "xo400_write_fifo_dma() [%d] %d %d unlikely happened we have contiguous pa, keep going",
							ib, frombyte, bytes);
		}else{
			bytes = len - offset;		// buffer head room
			dev_dbg(DEVP(adev), "xo400_write_fifo_dma() [%d] %d %d unlikely happened",
				ib, frombyte, bytes);
		}
	}
	if (bytes >= MIN_DMA_BYTES){
		int rbytes = (bytes/MIN_DMA_BYTES)*MIN_DMA_BYTES;
		if (rbytes != bytes){
			dev_dbg(DEVP(adev), "xo400_write_fifo_dma() rounding bytes from %d to %d", bytes, rbytes);
			bytes = rbytes;
		}
	}else{
		return xo400_write_fifo(adev, frombyte, bytes);
	}

	rc = dma_memcpy(adev,
			adev->dev_physaddr+AXI_FIFO,
			adev->cursor.hb[ib]->pa+offset, bytes);

	if (rc != bytes){
		dev_err(DEVP(adev), "dma_memcpy FAILED :%d\n", rc);
	}
	return rc;
}


void _ao420_stop(struct acq400_dev* adev)
{
	unsigned cr = acq400rd32(adev, DAC_CTRL);
	x400_disable_interrupt(adev);

	dev_dbg(DEVP(adev), "_ao420_stop() reset FIFO and clear histo\n");

	cr &= ~DAC_CTRL_DAC_EN;
	if (IS_AO42X(adev)){
 		cr |= DAC_CTRL_LL;
		if (adev->data32){
			cr |= ADC_CTRL32B_data;
		}else{
			cr &= ~ADC_CTRL32B_data;
		}
	}
	//adev->AO_playloop.length = 0;
	adev->AO_playloop.cursor = 0;
	acq400wr32(adev, DAC_CTRL, cr);
	release_dma_channels(adev);

	ao420_reset_fifo(adev);
	acq400_clear_histo(adev);
}



void measure_ao_fifo(struct acq400_dev *adev)
{
	unsigned osam = 0xffffffff;
	unsigned sam;
	int values_per_lw = adev->data32? 1: 2;
	unsigned cr;
	int nblocks;

	dev_dbg(DEVP(adev), "measure_ao_fifo() 01");

	cr = acq400rd32(adev, DAC_CTRL);
	acq400wr32(adev, DAC_CTRL, cr &= ~(DAC_CTRL_LL|DAC_CTRL_DAC_EN));


	ao420_reset_fifo(adev);
	for (nblocks = 0; (sam = adev->xo.getFifoSamples(adev)) != osam;
			osam = sam, ++nblocks){
		dev_dbg(DEVP(adev), "xo400_write_fifo(16384)");
		xo400_write_fifo(adev, 0, 1024);
	}

	dev_info(DEVP(adev), "measure_ao_fifo() write %d k, measure:%d samples\n",
			nblocks, sam);

	if (IS_AO42X(adev)){
		u32 dac_fifo_samples = acq400rd32(adev, DAC_FIFO_SAMPLES);
		int lwords = (dac_fifo_samples>>16)&DAC_FIFO_SAMPLES_MASK;

	/* MATCH lwords..samples is not exact */
		if ((lwords - sam*adev->nchan_enabled/values_per_lw) < 32){
			dev_info(DEVP(adev), "MATCHES: %08x lwords:%d samples:%d nchan:%d values_per_lw:%d\n",
				dac_fifo_samples,
				lwords, sam, adev->nchan_enabled, values_per_lw);
		}else{
			dev_warn(DEVP(adev), "MISMATCH: %08x lwords:%d samples:%d nchan:%d values_per_lw:%d\n",
				dac_fifo_samples,
				lwords, sam, adev->nchan_enabled, values_per_lw);
		}
	}
	adev->xo.max_fifo_samples = sam;
	adev->hitide = sam - 32;
	if (adev->nchan_enabled > 8){
		adev->lotide = 9*sam/10;
	}else{
		adev->lotide = 8*sam/10;
	}

	for (adev->xo.hshift = 0;
		(sam >> adev->xo.hshift) > 256; ++adev->xo.hshift)
		;
	dev_info(DEVP(adev), "setting max_fifo_samples:%u hshift:%u lotide:%d",
			sam, adev->xo.hshift, adev->lotide);

	ao420_reset_fifo(adev);
}

#define DAC_FIFO_STA_ERR (ADC_FIFO_STA_EMPTY|ADC_FIFO_STA_ERR)

void check_fiferr(struct acq400_dev* adev, unsigned fsr)
{
	u32 fifo_sta = acq400rd32(adev, fsr);

	dev_dbg(DEVP(adev), "check_fiferr() fsr:%04x sta:%08x", fsr, fifo_sta);

	if ((fifo_sta&ADC_FIFO_STA_ACTIVE) == 0){
		return;
	}
	if ((fifo_sta&DAC_FIFO_STA_ERR) != 0){
		unsigned stat2  = acq400rd32(adev, fsr);

		acq400wr32(adev, fsr, fifo_sta&0x0000000f);

		if (++adev->stats.fifo_errors < 10){
			if ((fifo_sta & ADC_FIFO_STA_EMPTY) != 0){
				dev_err(DEVP(adev),
				"ERROR FIFO underrun at %d %08x samples:%d",
				adev->stats.fifo_interrupts, fifo_sta,
				adev->xo.getFifoSamples(adev));
			}else{
				dev_err(DEVP(adev), "ERROR FIFO at %d %08x samples:%d stat2:%08x",
					adev->stats.fifo_interrupts, fifo_sta,
					adev->xo.getFifoSamples(adev),
					stat2);
			}
		}

	}
}

static int ao_auto_rearm(void *clidat)
/* poll for AWG complete, then rearm it */
{
	struct acq400_dev* adev = (struct acq400_dev*)clidat;
	unsigned dio_sc;
	wait_queue_head_t wait;
	init_waitqueue_head(&wait);

	dev_dbg(DEVP(adev), "ao_auto_rearm() 01");

	while (adev->xo.getFifoSamples(adev)){
		wait_event_interruptible_timeout(wait, 0, 1);
	}
	dio_sc = acq400rd32(adev, DIO432_DIO_SAMPLE_COUNT);
	_ao420_stop(adev);
	if (IS_DIO432X(adev)){
		dev_info(DEVP(adev), "ao_auto_rearm FIFO drained count %u", dio_sc);
	}

	if (adev->AO_playloop.length > 0 &&
		adev->AO_playloop.one_shot == AO_oneshot_rearm){
		dev_dbg(DEVP(adev), "ao_auto_rearm() reset %d", adev->AO_playloop.length);
		xo400_reset_playloop(adev, adev->AO_playloop.length);
	}
	dev_dbg(DEVP(adev), "ao_auto_rearm() 99");
	return 0;
}
static int xo400_fill_fifo(struct acq400_dev* adev)
/* returns 1 if further interrupts are required */
{
	int headroom;
	int rc = 1;		/* assume more ints wanted unless complete */
	int maxiter = 1000;
	static int next_one_verbose;

	dev_dbg(DEVP(adev), "xo420_fill_fifo() 01\n");

	if (mutex_lock_interruptible(&adev->awg_mutex)) {
		return 0;
	}
	go_rt(adev->nchan_enabled>8? MAX_RT_PRIO: MAX_RT_PRIO-2);

	while(adev->AO_playloop.length != 0 &&
	      (headroom = ao420_getFifoHeadroom(adev)) > xo400_getFillThreshold(adev) &&
	      adev->AO_playloop.length > adev->AO_playloop.cursor			 ){

		int remaining = adev->AO_playloop.length - adev->AO_playloop.cursor;
		int headroom_lt_remaining = headroom < remaining;

		remaining = min(remaining, headroom);

		dev_dbg(DEVP(adev), "headroom:%d remaining:%d using:%d\n",
			headroom, adev->AO_playloop.length - adev->AO_playloop.cursor, remaining);

		if (remaining){
			int cursor = AOSAMPLES2BYTES(adev, adev->AO_playloop.cursor);
			int lenbytes = AOSAMPLES2BYTES(adev, remaining);

			lenbytes = min(lenbytes, AO420_MAX_FILL_BLOCK);

			if (adev->dma_chan[0] != 0 && lenbytes > max(MIN_DMA_BYTES, ao420_dma_threshold)){
				int nbuf = lenbytes/MIN_DMA_BYTES;
				int dma_bytes = nbuf*MIN_DMA_BYTES;

				dev_dbg(DEVP(adev), "dma: cursor:%5d lenbytes: %d", cursor, dma_bytes);
				lenbytes = xo400_write_fifo_dma(adev, cursor, dma_bytes);
				if (next_one_verbose || lenbytes != dma_bytes){
					dev_info(DEVP(adev), "dma: [%d] cursor:%5d dma_bytes %d lenbytes: %d",
							next_one_verbose, cursor, dma_bytes, lenbytes);
					next_one_verbose = !next_one_verbose;
				}
			}else if (headroom_lt_remaining){
				/* FIFO nearly full, catch it next time */
				break;
			}else{
				/* we really have to write the end of the buffer .. */
				dev_dbg(DEVP(adev), "pio: cursor:%5d lenbytes: %d", cursor, lenbytes);
				lenbytes = xo400_write_fifo_dma(adev, cursor, lenbytes);
			}
			/* UGLY: has divide. */
			if (lenbytes < 0){
				dev_err(DEVP(adev), "ERROR in DMA");
				break;
			}
			adev->AO_playloop.cursor += AOBYTES2SAMPLES(adev, lenbytes);
		}

		if (adev->AO_playloop.cursor >= adev->AO_playloop.length){
			if (adev->AO_playloop.one_shot &&
				(adev->AO_playloop.repeats==0 || --adev->AO_playloop.repeats==0)){
				dev_dbg(DEVP(adev), "ao420 oneshot done disable interrupt");
				x400_disable_interrupt(adev);
				rc = 0;
				kthread_run(ao_auto_rearm, adev,
							"%s.awgrearm", devname(adev));
				goto done_no_check;
			}else{
				adev->AO_playloop.cursor = 0;
			}
		}
		if (--maxiter == 0){
			dev_warn(DEVP(adev), "xo400_fill_fifo() working too hard breaking to release mutex");
			break;
		}
	}

	check_fiferr(adev, adev->xo.fsr);
done_no_check:
	mutex_unlock(&adev->awg_mutex);
	dev_dbg(DEVP(adev), "xo420_fill_fifo() done filling, samples:%08x headroom %d\n",
			adev->xo.getFifoSamples(adev), ao420_getFifoHeadroom(adev));
	return rc;
}
static irqreturn_t xo400_dma(int irq, void *dev_id)
/* keep the AO420 FIFO full. Recycle buffer only */
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;

	if (xo_use_distributor){
		dev_warn(DEVP(adev), "bogus interrupt");
	}else if (adev->AO_playloop.length){
		u32 start_samples = adev->xo.getFifoSamples(adev);
		dev_dbg(DEVP(adev), "xo400_dma() start_samples: %u, headroom %d\n",
					start_samples, ao420_getFifoHeadroom(adev));
		if (xo400_fill_fifo(adev)){
			x400_enable_interrupt(adev);
		}
		add_fifo_histo_ao42x(adev, start_samples);
	}

	return IRQ_HANDLED;
}

void xo400_getDMA(struct acq400_dev* adev)
{
	if (adev->dma_chan[0] == 0 &&
			ao420_dma_threshold < adev->xo.max_fifo_samples){
		if (get_dma_channels(adev)){
			dev_err(DEVP(adev), "no dma chan");
			ao420_dma_threshold = adev->xo.max_fifo_samples;
		}else{
			if (adev->dma_chan[0] == 0){
				dev_err(DEVP(adev), "BAD DMA CHAN!");
			}else{
				dev_dbg(DEVP(adev),
					"ao420_reset_playloop() channel %d",
					adev->dma_chan[0]->chan_id);
			}
		}
	}
}

int ao_samples_per_hb(struct acq400_dev *adev)
{
	// @@todo valid single DIO432 ONLY!
	return bufferlen / xo_distributor_sample_size;
}

#define XO_MAX_POLL 100

int waitXoFifoEmpty(struct acq400_dev *adev)
{
	int pollcat = 0;
	int s1, s0 = 0;
	while ((s1 = adev->xo.getFifoSamples(adev)) > 0){
		msleep(20);
		if (s1 == s0 || ++pollcat > XO_MAX_POLL){
			dev_err(DEVP(adev), "TIMEOUT waiting for XO FIFO EMPTY");
			return -1;
		}
		s0 = s1;
	}
	return 0;
}
int xo_data_loop(void *data)
/** xo_data_loop() : outputs using distributor and PRI on SC, but loop is
 * actually associated with the master site
 * (usually 1, where playloop_length is set)
 * hence: adev: master site, adev0: distributor access.
 */
{
	static const unsigned wflags[2] = { DMA_WAIT_EV0, DMA_WAIT_EV1 };
	static const unsigned sflags[2] = { DMA_SET_EV1,  DMA_SET_EV0  };
	struct acq400_dev *adev = (struct acq400_dev *)data;
	struct acq400_dev *adev0 = acq400_devices[0];
	struct HBM** hbm0 = adev0->hb;
	int ic = 0;
	int ib = distributor_first_buffer;
	long dma_timeout = START_TIMEOUT;

	int shot_buffer_count0 = adev->AO_playloop.length/ao_samples_per_hb(adev0);
	int shot_buffer_count = shot_buffer_count0;

	if (ib != 0){
		dev_dbg(DEVP(adev), "xo_data_loop() ib set %d shot_buffer_count:%d",
				ib, shot_buffer_count);
	}
	if (shot_buffer_count*ao_samples_per_hb(adev) < adev->AO_playloop.length){
		shot_buffer_count += 1;
		dev_dbg(DEVP(adev), "ao play data buffer overspill");
	}

	adev->stats.shot++;
	adev->stats.xo.dma_buffers_out =
			adev->stats.xo.dma_buffers_in = 0;

#define DMA_ASYNC_PUSH(adev, chan, hbm) \
	dma_async_memcpy_callback(adev->dma_chan[chan], \
			FIFO_PA(adev0), hbm->pa, adev0->bufferlen, \
			DMA_DS0_FLAGS|wflags[chan]|sflags[chan], \
			acq400_dma_callback, adev)

#define DMA_ASYNC_PUSH_NWFE(adev, chan, hbm) \
	dma_async_memcpy_callback(adev->dma_chan[chan], \
			FIFO_PA(adev0), hbm->pa, adev0->bufferlen, \
			DMA_DS0_FLAGS|sflags[chan], \
			acq400_dma_callback, adev)

#define DMA_ASYNC_PUSH_NOSETEV(adev, chan, hbm) \
	dma_async_memcpy_callback(adev->dma_chan[chan], \
			FIFO_PA(adev0), hbm->pa, adev0->bufferlen, \
			DMA_DS0_FLAGS|wflags[chan], \
			acq400_dma_callback, adev)

#define DMA_ASYNC_ISSUE_PENDING(chan) do { 		\
	dev_dbg(DEVP(adev), "xo_data_loop() dma_async_issue_pending %d", chan->chan_id); \
	dma_async_issue_pending(chan);			\
	++adev->stats.xo.dma_buffers_out; } while(0)

#define DMA_COUNT_IN \
	do { 						\
		++adev->stats.xo.dma_buffers_in;	\
		dev_dbg(DEVP(adev), "DMA_COUNT_IN %d", adev->stats.xo.dma_buffers_in); \
	} while(0)

#define LAST_PUSH(adev)	(adev->stats.xo.dma_buffers_out+1 >=shot_buffer_count)

	/* prime the DMAC with buffers 0 and 1 ready to go.
	 * 0 starts filling right away
	 * */
	adev->dma_cookies[0] = DMA_ASYNC_PUSH_NWFE(adev, 0, hbm0[ib++]);
	if (shot_buffer_count > 1){
		adev->dma_cookies[1] = DMA_ASYNC_PUSH(adev, 1, hbm0[ib++]);
		DMA_ASYNC_ISSUE_PENDING(adev->dma_chan[1]);
	}

	sc_data_engine_reset_enable(DATA_ENGINE_1);
	DMA_ASYNC_ISSUE_PENDING(adev->dma_chan[0]);

	dev_dbg(DEVP(adev), "xo_data_loop() 01 :out:%d in:%d",
			adev->stats.xo.dma_buffers_out, adev->stats.xo.dma_buffers_in);

	go_rt(MAX_RT_PRIO-4);


	adev->task_active = 1;

	for(adev->AO_playloop.cursor = 0;
	    adev->AO_playloop.cursor < adev->AO_playloop.length && !kthread_should_stop();
	    	    	    ic = !ic, dma_timeout = DMA_TIMEOUT){
		if (wait_event_interruptible_timeout(
				adev->DMA_READY,
				adev->dma_callback_done || kthread_should_stop(),
				dma_timeout) <= 0){
			dev_err(DEVP(adev), "TIMEOUT waiting for DMA\n");
			goto quit;
		}


		if (kthread_should_stop()){
			dev_info(DEVP(adev), "kthread_should_stop");
			goto quit;
		}else if (adev->dma_callback_done){
			--adev->dma_callback_done;
			DMA_COUNT_IN;
		}else {
			dev_err(DEVP(adev), "here with no callback int should not happen ..");
		}

		dev_dbg(DEVP(adev), "calling dma_sync_wait() ..");

		if(dma_sync_wait(adev->dma_chan[ic], adev->dma_cookies[ic]) != DMA_SUCCESS){
			dev_err(DEVP(adev), "dma_sync_wait cursor:%d chan:%d timeout",
					adev->AO_playloop.cursor, ic);
			goto quit;
		}
		dev_dbg(DEVP(adev), "back from dma_sync_wait() ..");

		adev->AO_playloop.cursor += ao_samples_per_hb(adev);

		if (adev->stats.xo.dma_buffers_out >= shot_buffer_count){
			if (adev->AO_playloop.one_shot == 0 ||
					adev->AO_playloop.repeats > 0){
				shot_buffer_count += shot_buffer_count0;
				ib = 0;
				adev->AO_playloop.cursor = 0;
				/** @@todo dodgy : output or input dep on mode */
				if (adev->AO_playloop.one_shot == 0){
					adev->AO_playloop.repeats++;
				}else{
					--adev->AO_playloop.repeats;
				}
			}
		}
		if (adev->stats.xo.dma_buffers_out < shot_buffer_count){
			adev->dma_cookies[ic] = LAST_PUSH(adev)?
					DMA_ASYNC_PUSH_NOSETEV(adev, ic, hbm0[ib]):
					DMA_ASYNC_PUSH(adev, ic, hbm0[ib]);
			ib++;
			DMA_ASYNC_ISSUE_PENDING(adev->dma_chan[ic]);
		}
		yield();
	}

quit:
	dev_dbg(DEVP(adev), "xo_data_loop() quit out:%d in:%d",
			adev->stats.xo.dma_buffers_out, adev->stats.xo.dma_buffers_in);

	if (adev->stats.xo.dma_buffers_in < adev->stats.xo.dma_buffers_out){
		if (wait_event_interruptible_timeout(
			adev->DMA_READY,
			adev->dma_callback_done, dma_timeout) <= 0){
			dev_err(DEVP(adev), "TIMEOUT waiting for DMA\n");
		}
	}
	if (adev->dma_callback_done){
		--adev->dma_callback_done;
		DMA_COUNT_IN;
	}
	waitXoFifoEmpty(adev);


	acq400_visit_set(adev0->distributor_set, adev->onStop);

	adev->task_active = 0;
	if (adev->AO_playloop.one_shot == 2){
		kthread_run(ao_auto_rearm, adev, "%s.awgrearm", devname(adev));
	}
	dev_dbg(DEVP(adev), "xo_data_loop() 99 out:%d in:%d",
			adev->stats.xo.dma_buffers_out, adev->stats.xo.dma_buffers_in);
	return 0;
}

void xo400_distributor_feeder_control(struct acq400_dev* adev, int enable)
{
	/* adev is the site (1?) adev. so wtask is dedicated to xo */
	/* @@todo start stop feeder loop */
	if (enable){
		adev->w_task = kthread_run(
			xo_data_loop, adev,
			"%s.xo", devname(adev));
	}else{
		int pollcat = 0;
		if (adev->task_active && adev->w_task != 0){
			kthread_stop(adev->w_task);
		}
		while(adev->task_active){
			msleep(10);
			if (++pollcat > 100){
				dev_warn(DEVP(adev), "waiting for task stop");
			}
		}
		adev->w_task = 0;
	}
}


int xo400_reset_playloop(struct acq400_dev* adev, unsigned playloop_length)
{
	struct acq400_dev *adev0 = acq400_devices[0];
	int first_in_set = xo_use_distributor && (adev == adev0->distributor_set[0]);
	int use_frontside = !xo_use_distributor;

	if (xo_use_distributor && !first_in_set){
		dev_warn(DEVP(adev), "xo400_reset_playloop() xo_use_distributor but not first_in_set");
	}
	dev_dbg(DEVP(adev), "xo400_reset_playloop(%d) => %d",
					adev->AO_playloop.length, playloop_length);

	if (adev->AO_playloop.length && xo_use_distributor && adev->task_active){
		dev_warn(DEVP(adev), "XO AWG is already tee'd up, not possible to abort");
		return -1;
	}

	if (mutex_lock_interruptible(&adev->awg_mutex)) {
		return -1;
	}else{
		_ao420_stop(adev);

		if (first_in_set){
			xo400_distributor_feeder_control(adev, 0);
		}
		mutex_unlock(&adev->awg_mutex);
	}

	while(adev->task_active){
		dev_dbg(DEVP(adev), "xo400_reset_playloop wait task_active -> 0 ");
		msleep(100);
	}

	adev->AO_playloop.length = playloop_length;

	if (playloop_length != 0){
		if (IS_DIO432X(adev)){
			dio432_set_mode(adev, DIO432_CLOCKED, 1);
		}
		if (adev->xo.getFifoSamples(adev) != 0){
			dev_err(DEVP(adev), "ERROR: FIFO not EMPTY at start fill %d",
					adev->xo.getFifoSamples(adev));
		}

		if (first_in_set){
			xo400_getDMA(adev);
			xo400_distributor_feeder_control(adev, 1);
			while(!adev->task_active){
				dev_dbg(DEVP(adev), "xo400_reset_playloop wait task_active -> 0 ");
				msleep(100);
			}
		}else if (use_frontside){
			xo400_getDMA(adev);
			xo400_fill_fifo(adev);
			ao420_clear_fifo_flags(adev);
		}
		/* else do nothing */

		if (first_in_set){
			acq400_visit_set(adev0->distributor_set, adev->onStart);
		}else if (use_frontside){
			adev->onStart(adev);
		}
		/* else do nothing */
	}else{
		if (first_in_set){
			acq400_visit_set(adev0->distributor_set, adev->onStop);
		}else if (use_frontside){
			adev->onStop(adev);
		}
	}

	return 0;
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

#define POISON0 0xc0de0000
#define POISON1 0xc1de0000

#define USZ	sizeof(u32)

void init_one_buffer(struct acq400_dev *adev, struct HBM* hbm)
{
	int ii;
	int maxwords = bufferlen/USZ;
	u32* cursor = hbm->va;

	for (ii = 0; ii < maxwords; ++ii){
		cursor[ii] = ii;
	}
}

unsigned poison_offset(struct acq400_dev *adev)
{
	unsigned bufferlen = adev->bufferlen;

	if (likely(AXI_POISON_OFFSET == 0)){
		return bufferlen;
	}else{
		if (AXI_POISON_OFFSET > bufferlen) AXI_POISON_OFFSET = bufferlen;

		return AXI_POISON_OFFSET;
	}
}

#define FIRST_POISON_WORD(pob) ((pob)/USZ-2)
#define POISON_SZ		(2*USZ)

void poison_one_buffer(struct acq400_dev *adev, struct HBM* hbm)
{
	unsigned po_bytes = poison_offset(adev);
	unsigned first_word = FIRST_POISON_WORD(po_bytes);

	hbm->poison_data[0] = hbm->va[first_word+0];
	hbm->poison_data[1] = hbm->va[first_word+1];
	hbm->va[first_word+0] = POISON0;
	hbm->va[first_word+1] = POISON1;
	dma_sync_single_for_device(DEVP(adev),
				hbm->pa + po_bytes, POISON_SZ, hbm->dir);

	if (adev->rt.axi64_firstups == 0 && hbm->ix == 0){
		dev_dbg(DEVP(adev), "poison_one_buffer() poison applied at +%d bytes", first_word*USZ);
	}
}


void poison_one_buffer_fastidious(struct acq400_dev *adev, struct HBM* hbm)
{
	if (AXI_DEBUG_LOOPBACK_INDEX <= hbm->ix){
		poison_one_buffer(adev, hbm);
	}else{
		dev_dbg(DEVP(adev), "poison_one_buffer_fastidious() refuse to poison %d", hbm->ix);
	}
}



void null_put_empty(struct acq400_dev *adev, struct HBM* hbm)
{

}

int poison_overwritten(struct acq400_dev *adev, struct HBM* hbm)
{
	unsigned po_bytes = poison_offset(adev);
	unsigned first_word = FIRST_POISON_WORD(po_bytes);

	dma_sync_single_for_cpu(DEVP(adev), hbm->pa + po_bytes, POISON_SZ, hbm->dir);
	return hbm->va[first_word+0] != POISON0 &&
	       hbm->va[first_word+1] != POISON1;
}

void clear_poison_from_buffer(struct acq400_dev *adev, struct HBM* hbm)
{
	unsigned po_bytes = poison_offset(adev);
	unsigned first_word = FIRST_POISON_WORD(po_bytes);

	if (!poison_overwritten(adev, hbm)){
		hbm->va[first_word+0] = hbm->poison_data[0];
		hbm->va[first_word+1] = hbm->poison_data[1];
	}
}

int poison_all_buffers(struct acq400_dev *adev)
{
	int ii;
	int nbuffers = move_list_to_stash(adev, &adev->EMPTIES);

	mutex_lock(&adev->list_mutex);
	for (ii = 0; ii < nbuffers; ++ii){
		struct HBM* hbm = adev->axi64_hb[ii];
		if (AXI_INIT_BUFFERS){
			init_one_buffer(adev, hbm);
		}
		poison_one_buffer(adev, hbm);
		list_move_tail(&hbm->list, &adev->EMPTIES);
	}
	mutex_unlock(&adev->list_mutex);
	return nbuffers;
}

void clear_poison_all_buffers(struct acq400_dev *adev, int nbuffers)
{
	int ii;

	mutex_lock(&adev->list_mutex);
	for (ii = 0; ii < nbuffers; ++ii){
		struct HBM* hbm = adev->axi64_hb[ii];
		clear_poison_from_buffer(adev, hbm);
	}
	mutex_unlock(&adev->list_mutex);
}
int check_all_buffers_are_poisoned(struct acq400_dev *adev)
{
	struct HBM *cursor;
	int fails = 0;
	int pass = 0;
	mutex_lock(&adev->list_mutex);
	list_for_each_entry(cursor, &adev->EMPTIES, list){
		if (poison_overwritten(adev, cursor)){
			dev_err(DEVP(adev), "poison missing from %d", cursor->ix);
			++fails;
		}else{
			++pass;
		}
	}
	mutex_unlock(&adev->list_mutex);
	if (fails == 0){
		dev_info(DEVP(adev), "check_all_buffers_are_poisoned %d ALL GOOD", pass);
	}
	return fails;
}
int dma_done(struct acq400_dev *adev, struct HBM* hbm)
{
	return poison_overwritten(adev, hbm);
}

void histo_clear_all(struct acq400_dev *devs[], int maxdev)
{
	int cursor;
	for (cursor = 0; cursor < maxdev; ++cursor){
		acq400_clear_histo(devs[cursor]);
	}
}

void histo_add_all(struct acq400_dev *devs[], int maxdev, int i1)
{
	int cursor;
	for (cursor = i1; cursor < maxdev; ++cursor){
		struct acq400_dev *adev = devs[cursor];
		add_fifo_histo(adev, adev->get_fifo_samples(adev));
	}
	for (cursor = 0; cursor < i1; ++cursor){
		struct acq400_dev *adev = devs[cursor];
		add_fifo_histo(adev, adev->get_fifo_samples(adev));
	}
}

int fifo_monitor(void* data)
{
	struct acq400_dev *devs[MAXSITES+1];
	struct acq400_dev *adev = (struct acq400_dev *)data;
	int idev = 0;
	int cursor;
	int i1 = 0;		/* randomize start time */

	devs[idev++] = adev;
	adev->get_fifo_samples = aggregator_get_fifo_samples;
	for (cursor = 0; cursor < MAXSITES; ++cursor){
		struct acq400_dev* slave = adev->aggregator_set[cursor];
		if (slave){
			devs[idev++] = slave;
			slave->get_fifo_samples = acq420_get_fifo_samples;
		}
	}
	histo_clear_all(devs, idev);

	while(!kthread_should_stop()) {
		if (acq420_convActive(devs[idev>1?1:0])){
			histo_add_all(devs, idev, i1);
			msleep(histo_poll_ms);
		}
//		if (++i1 >= idev) i1 = 0;
	}

	return 0;
}

int axi64_data_loop(void* data)
{
	struct acq400_dev *adev = (struct acq400_dev *)data;
	/* wait for event from OTHER channel */
	int nloop = 0;
	struct HBM* hbm;
	int rc;
	int nbuffers;

	dev_dbg(DEVP(adev), "ai_data_loop() 01");

	adev->onPutEmpty = poison_one_buffer_fastidious;
	nbuffers = poison_all_buffers(adev);

	if (AXI_CALL_HELPER){
		if ((rc = axi64_load_dmac(adev)) != 0){
			dev_err(DEVP(adev), "axi64_load_dmac() failed %d", rc);
			return -1;
		}else{
			dev_info(DEVP(adev), "axi64_load_dmac() helper done");
		}
	}

	if (check_all_buffers_are_poisoned(adev) || kthread_should_stop()){
		goto quit;
	}

	hbm = getEmpty(adev);
	dev_info(DEVP(adev), "axi64_data_loop() holding hbm:%d", hbm->ix);

	yield();
	go_rt(MAX_RT_PRIO-4);
	adev->task_active = 1;

	for(; !kthread_should_stop(); ++nloop){
		int ddone = 0;
		if (wait_event_interruptible_timeout(
			adev->DMA_READY,
			(ddone = dma_done(adev, hbm)) || kthread_should_stop(),
			AXI_BUFFER_CHECK_TICKS) <= 0){
			if (kthread_should_stop()){
				goto quit;
			}
		}
		if (adev->rt.axi64_firstups) adev->rt.axi64_wakeups++;
		if (!ddone){
			continue;
		}

		putFull(adev);
		adev->rt.axi64_firstups++;

		if (AXI_DMA_HAS_CATCHUP){
			while((hbm = getEmpty(adev)) != 0){
				if (poison_overwritten(adev, hbm)){
					putFull(adev);
					adev->rt.axi64_catchups++;
				}else{
					break;	/* now wait for this hbm to complete */
				}
			}
		}else{
			hbm = getEmpty(adev);
		}

		if (check_fifo_statuses(adev)){
			dev_err(DEVP(adev), "ERROR: quit on fifo status check");
			goto quit;
		}

		if (!IS_SC(adev)){
			//acq420_enable_interrupt(adev);
		}
		if (hbm == 0){
			++adev->stats.errors;
			move_list_to_empty(adev, &adev->REFILLS);
			dev_warn(DEVP(adev), "discarded FULL Q\n");
			++adev->rt.buffers_dropped;

			if (quit_on_buffer_exhaustion){
				adev->rt.refill_error = 1;
				wake_up_interruptible_all(&adev->refill_ready);
				dev_warn(DEVP(adev), "quit_on_buffer_exhaustion\n");
				goto quit;
			}else{
				hbm = getEmpty(adev);
				if (hbm == 0){
					dev_err(DEVP(adev), "STILL NO EMPTIES");
					goto quit;
				}else{
					poison_one_buffer(adev, hbm);
				}
			}
		}
	}
quit:
	dev_dbg(DEVP(adev), "ai_data_loop() 98 calling DMA_TERMINATE_ALL");

	clear_poison_all_buffers(adev, nbuffers);
	adev->task_active = 0;
	dev_dbg(DEVP(adev), "ai_data_loop() 99");
	return 0;
}

int ai_data_loop(void *data)
{
	/* wait for event from OTHER channel */
	static const unsigned wflags[2] = { DMA_WAIT_EV0, DMA_WAIT_EV1 };
	static const unsigned sflags[2] = { DMA_SET_EV1,  DMA_SET_EV0  };

	struct acq400_dev *adev = (struct acq400_dev *)data;
	int nloop = 0;
	int ic;
	long dma_timeout = START_TIMEOUT;
	struct HBM* hbm0;
	struct HBM* hbm1;

#define DMA_ASYNC_MEMCPY(adev, chan, hbm) \
	dma_async_memcpy_callback(adev->dma_chan[chan], hbm->pa, \
			FIFO_PA(adev), adev->bufferlen, \
			DMA_SC_FLAGS|wflags[chan]|sflags[chan], \
			acq400_dma_callback, adev)
#define DMA_ASYNC_MEMCPY_NWFE(adev, chan, hbm) \
	dma_async_memcpy_callback(adev->dma_chan[chan], hbm->pa, \
			FIFO_PA(adev), adev->bufferlen, \
			DMA_SC_FLAGS|sflags[chan], \
			acq400_dma_callback, adev)


	dev_dbg(DEVP(adev), "ai_data_loop() 01");

	adev->onPutEmpty = null_put_empty;
	hbm0 = getEmpty(adev);
	hbm1 = getEmpty(adev);


	dev_dbg(DEVP(adev), "hbm0:0x%08x chan:%d\n", hbm0->pa, adev->dma_chan[0]->chan_id);
	dev_dbg(DEVP(adev), "hbm1:0x%08x chan:%d\n", hbm1->pa, adev->dma_chan[1]->chan_id);

	/* prime the DMAC with buffers 0 and 1 ready to go. */
	adev->dma_cookies[1] = DMA_ASYNC_MEMCPY(adev, 1, hbm1);
	dma_async_issue_pending(adev->dma_chan[1]);
	adev->dma_cookies[0] = DMA_ASYNC_MEMCPY_NWFE(adev, 0, hbm0);
	dma_async_issue_pending(adev->dma_chan[0]);

	yield();
	go_rt(MAX_RT_PRIO-4);
	adev->task_active = 1;

	if (adev->fifo_isr_done){
		dev_warn(DEVP(adev), "fifo_isr_done EARLY\n");
	}
#if 0
		/* @@todo pgm: no site 0 interrupt for now */
	/* wait initial hitide interrupt, avoid dma timeout on TRIG */
	if (wait_event_interruptible(adev->w_waitq,
			adev->fifo_isr_done || kthread_should_stop())){
		goto quit;
	}
#endif
	if (kthread_should_stop()){
		goto quit;
	}
#if 0
	dev_dbg(DEVP(adev), "rx initial FIFO interrupt, into the work loop\n");
#endif

	for(; !kthread_should_stop(); ++nloop){
		for (ic = 0; ic < 2 && !kthread_should_stop();
				++ic, dma_timeout = DMA_TIMEOUT){
			struct HBM* hbm;
			int emergency_drain_request = 0;

			dev_dbg(DEVP(adev), "wait for dma_chan[%d] %p %d timeout=%ld\n",
					ic, adev->dma_chan[ic], adev->dma_cookies[ic],
					dma_timeout);

			if (wait_event_interruptible_timeout(
					adev->DMA_READY,
					adev->dma_callback_done || kthread_should_stop(),
					dma_timeout) <= 0){
				dev_err(DEVP(adev), "TIMEOUT waiting for DMA\n");
				goto quit;
			}
			--adev->dma_callback_done;
			if (kthread_should_stop()){
				goto quit;
			}
			if(dma_sync_wait(adev->dma_chan[ic], adev->dma_cookies[ic]) != DMA_SUCCESS){
				dev_err(DEVP(adev), "dma_sync_wait nloop:%d chan:%d timeout", nloop, ic);
				goto quit;
			}
			dev_dbg(DEVP(adev), "DMA_SUCCESS: SYNC %d done\n", ic);

			putFull(adev);
			if ((hbm = getEmpty(adev)) == 0){
				dev_warn(DEVP(adev), "Pulling data from FullQ\n");
				hbm = getEmptyFromRefills(adev);
				if (hbm == 0){
					dev_err(DEVP(adev), "FAILED to pull data from FullQ, quitting\n");
					goto quit;
				}else{
					++adev->stats.errors;
					emergency_drain_request = 1;
				}
			}
			adev->dma_cookies[ic] = DMA_ASYNC_MEMCPY(adev, ic, hbm);
			dma_async_issue_pending(adev->dma_chan[ic]);

			if (check_fifo_statuses(adev)){
				dev_err(DEVP(adev), "ERROR: quit on fifo status check");
				goto quit;
			}

			if (!IS_SC(adev)){
				//acq420_enable_interrupt(adev);
			}
			if (emergency_drain_request){
				move_list_to_empty(adev, &adev->REFILLS);
				dev_warn(DEVP(adev), "discarded FULL Q\n");
				++adev->rt.buffers_dropped;

				if (quit_on_buffer_exhaustion){
					adev->rt.refill_error = 1;
					wake_up_interruptible_all(&adev->refill_ready);
					dev_warn(DEVP(adev), "quit_on_buffer_exhaustion\n");
					goto quit;
				}
			}
		}
	}
quit:
	dev_dbg(DEVP(adev), "ai_data_loop() 98 calling DMA_TERMINATE_ALL");
	for (ic = 0; ic < 2; ++ic){
		struct dma_chan *chan = adev->dma_chan[ic];
		chan->device->device_control(chan, DMA_TERMINATE_ALL, 0);
	}

	adev->task_active = 0;

	if (!kthread_should_stop()){
		dev_dbg(DEVP(adev), "ai_data_loop() wait stop 88");
		do_exit(0);
	}else{
		dev_dbg(DEVP(adev), "ai_data_loop() 99");
		return 0;
	}
#undef DMA_ASYNC_MEMCPY
#undef DMA_ASYNC_MEMCPY_NWFE
}

/* this is the much-vaunted hrtimer - program in nsec,
 * still with 10msec resolution .. or maybe it's really good (second time around)
 */
static enum hrtimer_restart pulse_stretcher_timer_handler(struct hrtimer
					  *handle)
{
	struct acq400_dev *adev;

	adev = container_of(handle, struct acq400_dev, atd.timer);

	acq400_clearDelTrgEvent(adev);
	acq400wr32(adev, ATD_TRIGGERED, adev->atd.event_source);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart pulse_stretcher_display_timer_handler(
		struct hrtimer *handle)
{
	struct acq400_dev *adev;

	adev = container_of(handle, struct acq400_dev, atd_display.timer);
	adev->atd_display.event_source = 0;
	return HRTIMER_NORESTART;
}

int cosco_count=4;
int cosco[4];

module_param_array(cosco, int, &cosco_count, 0644);
MODULE_PARM_DESC(lotide, "histogram of cos isr");

static void xtd_action(struct acq400_dev *adev)
{
	adev->atd.event_source = acq400rd32(adev, ATD_TRIGGERED);

	if (adev->atd.event_source){
			adev->atd_display.event_source = adev->atd.event_source;

		if (HAS_DTD(adev)){
			hrtimer_start(&adev->atd.timer,	ktime_set(0, dtd_pulse_nsec),
					HRTIMER_MODE_REL);
		}

		hrtimer_start(&adev->atd_display.timer,
			ktime_set(0, dtd_display_pulse_nsec), HRTIMER_MODE_REL);
	}
}

/** @@TODO what if two DMA channels busy? */

#define CHANNEL0	0

static void cos_action(struct acq400_dev *adev, u32 status)
{
	struct acq400_dev* adev0 = acq400_devices[0];

	++adev->rt.event_count;

	if ((status&ADC_INT_CSR_EVENT0) != 0){
		if (acq400_event_count_limit &&
		    adev->rt.event_count >= acq400_event_count_limit){
			acq400_enable_event0(adev, 0);
		}
		if (adev0->axi_buffers_after_event) {
			axi64_tie_off_dmac(adev0, CHANNEL0, adev0->axi_buffers_after_event);
			adev0->axi_buffers_after_event = 0;
		}
		adev->rt.samples_at_event = acq400rd32(adev, ADC_SAMPLE_CTR);
		adev->rt.sample_clocks_at_event =
					acq400rd32(adev, ADC_SAMPLE_CLK_CTR);
		if (HAS_XTD(adev)){
			xtd_action(adev);
		}

		wake_up_interruptible(&adev->event_waitq);
	}
	if ((status&ADC_INT_CSR_EVENT1) != 0){
		if (HAS_XTD(adev)){
			xtd_action(adev);
		}
	}
	cosco[EVX_TO_INDEX(status)]++;

	dev_dbg(DEVP(adev), "cos_action %08x\n", status);
}

static void hitide_action(struct acq400_dev *adev, u32 status)
{
	if ((status&ADC_INT_CSR_HITIDE) != 0){
		add_fifo_histo(adev, acq420_get_fifo_samples(adev));
		adev->stats.fifo_interrupts++;
		adev->fifo_isr_done = 1;
		wake_up_interruptible(&adev->w_waitq);
	}
}

static irqreturn_t acq400_isr(int irq, void *dev_id)
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;
	volatile u32 status = x400_get_interrupt(adev);
	cos_action(adev, status);
	hitide_action(adev, status);

	x400_set_interrupt(adev, status);
	dev_dbg(DEVP(adev), "acq400_isr %08x\n", status);
	return IRQ_HANDLED;
}

static irqreturn_t ao400_isr(int irq, void *dev_id)
{
	struct acq400_dev *adev = (struct acq400_dev *)dev_id;
	//volatile u32 status =
	x400_get_interrupt(adev);

	// @@todo check this.
	x400_disable_interrupt(adev);
	//acq420_clear_interrupt(adev, status);

	adev->stats.fifo_interrupts++;

	if (adev->AO_playloop.length){
		return IRQ_WAKE_THREAD;	/* canned */
	}else{
		wake_up_interruptible(&adev->w_waitq);  /* stream_dac */
		return IRQ_HANDLED;
	}
}

struct file_operations acq400_fops = {
        .owner = THIS_MODULE,
        .read = acq400_read,
        .write = acq400_write,
        .open = acq400_open,
        .release = acq400_release,
        .mmap = acq400_mmap_bar
};



#ifdef CONFIG_OF
static struct of_device_id acq400_of_match[] /* __devinitdata */ = {
        { .compatible = "D-TACQ,acq400fmc"  },
        { .compatible = "D-TACQ,acq420fmc"  },
        { .compatible = "D-TACQ,acq430fmc"  },
        { .compatible = "D-TACQ,acq435elf"  },
        { .compatible = "D-TACQ,ao420fmc"   },
        { .compatible = "D-TACQ,acq2006sc"  },
        { .compatible = "D-TACQ,acq1001sc"  },
        { .compatible = "D-TACQ,acq1002sc"  },
        { .compatible = "D-TACQ,acq400pmod" },
        { /* end of table */}
};
MODULE_DEVICE_TABLE(of, acq400_of_match);
#else
#define acq400_of_match NULL
#endif /* CONFIG_OF */


static int acq400_device_tree_init(struct acq400_dev* adev)
{
	struct device_node *of_node = adev->pdev->dev.of_node;

        if (of_node) {
        	u32 irqs[OF_IRQ_COUNT];

        	if (of_property_read_u32(of_node, "site",
        			&adev->of_prams.site) < 0){
        		dev_warn(DEVP(adev), "error: site NOT specified in DT\n");
        		return -1;
        	}else{
        		if (!isGoodSite(adev->of_prams.site)){
        			dev_warn(DEVP(adev),
        					"warning: site %d NOT GOOD\n",
        					adev->of_prams.site);
        			return -1;
        		}else{
        			dev_info(DEVP(adev), "site:%d GOOD\n",
        					adev->of_prams.site);
        		}
        	}
        	/*
                if (of_property_read_u32(of_node, "dma-channel",
                        &adev->of_prams.dma_channel) < 0) {
                        dev_warn(DEVP(adev),
                        	"DMA channel unspecified - assuming 0\n");
                        adev->of_prams.dma_channel = 0;
                }
                dev_info(DEVP(adev),
                        "acq400_device_tree_init() read DMA channel is %d\n",
                        adev->of_prams.dma_channel);

                if (of_property_read_u32(of_node, "fifo-depth",
                        &adev->of_prams.fifo_depth) < 0) {
                        dev_warn(DEVP(adev),
                                "depth unspecified, assuming 0xffffffff\n");
                        adev->of_prams.fifo_depth = 0xffffffff;
                }
                dev_info(DEVP(adev),
                	"acq400_device_tree_init() DMA fifo depth is %d\n",
                	adev->of_prams.fifo_depth);

                if (of_property_read_u32(of_node, "burst-length",
                        &adev->of_prams.burst_length) < 0) {
                        dev_warn(DEVP(adev),
                                "burst length unspecified - assuming 1\n");
                        adev->of_prams.burst_length = 1;
                }

                dev_info(DEVP(adev),
                	"acq400_device_tree_init() DMA burst length is %d\n",
                        adev->of_prams.burst_length);
        	 */
                if (of_property_read_u32_array(
                		of_node, "interrupts", irqs, OF_IRQ_COUNT)){
                	dev_warn(DEVP(adev), "failed to find %d IRQ values", OF_IRQ_COUNT);
                }else{
                	adev->of_prams.irq = irqs[OF_IRQ_HITIDE] + OF_IRQ_MAGIC;
                }

                return 0;
        }
        return -1;
}

void acq400_timer_init(
	struct hrtimer* timer,
	enum hrtimer_restart (*function)(struct hrtimer *))
{
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->function = function;
}
static struct acq400_dev* acq400_allocate_dev(struct platform_device *pdev)
/* Allocate and init a private structure to manage this device */
{
	struct acq400_dev* adev = kzalloc(sizeof(struct acq400_dev), GFP_KERNEL);
        if (adev == NULL) {
                return NULL;
        }
        init_waitqueue_head(&adev->waitq);
        init_waitqueue_head(&adev->DMA_READY);
        init_waitqueue_head(&adev->refill_ready);
        init_waitqueue_head(&adev->hb0_marker);


        adev->pdev = pdev;
        mutex_init(&adev->mutex);
        mutex_init(&adev->awg_mutex);
        mutex_init(&adev->sewFifo[0].sf_mutex);
        mutex_init(&adev->sewFifo[1].sf_mutex);
        adev->fifo_histo = kzalloc(FIFO_HISTO_SZ*sizeof(u32), GFP_KERNEL);

        INIT_LIST_HEAD(&adev->EMPTIES);
        INIT_LIST_HEAD(&adev->INFLIGHT);
        INIT_LIST_HEAD(&adev->REFILLS);
        INIT_LIST_HEAD(&adev->OPENS);
        INIT_LIST_HEAD(&adev->STASH);
        mutex_init(&adev->list_mutex);
        INIT_LIST_HEAD(&adev->bq_clients);
        mutex_init(&adev->bq_clients_mutex);
        init_waitqueue_head(&adev->w_waitq);
        init_waitqueue_head(&adev->event_waitq);
        adev->onStart = acqXXX_onStartNOP;
        adev->onStop = acqXXX_onStopNOP;
        adev->clkdiv_mask = ADC_CLK_DIV_MASK;

        acq400_timer_init(&adev->atd.timer, pulse_stretcher_timer_handler);
        acq400_timer_init(&adev->atd_display.timer,
        		pulse_stretcher_display_timer_handler);

        return adev;
}


static int acq400_remove(struct platform_device *pdev);



static int allocate_hbm(struct acq400_dev* adev, int nb, int bl, int dir)
{
	if (hbm_allocate(DEVP(adev), nb, bl, &adev->EMPTIES, dir)){
		return -1;
	}else{
		struct HBM* cursor;
		int ix = 0;
		adev->hb = kmalloc(nb*sizeof(struct HBM*), GFP_KERNEL);
		list_for_each_entry(cursor, &adev->EMPTIES, list){
			WARN_ON(cursor->ix != ix);
			adev->hb[cursor->ix] = cursor;
			ix++;
		}
		dev_info(DEVP(adev), "setting nbuffers %d\n", ix);
		adev->nbuffers = ix;
		adev->bufferlen = bl;
		adev->axi64_hb = adev->hb+1;
		return 0;
	}
}

struct acq400_dev* acq400_lookupSite(int site)
{
	int is;
	for (is = 0; is < ndevices; ++is){
		struct acq400_dev* adev = acq400_devices[is];
		if (adev->of_prams.site == site){
			return adev;
		}
	}
	return 0;
}
static int acq400_probe(struct platform_device *pdev)
{
        int rc;
        struct resource *acq400_resource;
        struct acq400_dev* adev = acq400_allocate_dev(pdev);

        if (!adev){
        	dev_err(&pdev->dev, "unable to allocate device structure\n");
        	return -ENOMEM;
        }
        pdev->dev.id = ndevices;
        /* Get our platform device resources */
        dev_dbg(DEVP(adev), "id:%d We have %d resources\n", pdev->id, pdev->num_resources);
        acq400_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (acq400_resource == NULL) {
                dev_err(&pdev->dev, "No resources found\n");
                return -ENODEV;
        }

        if (acq400_device_tree_init(adev)){
        	rc = -ENODEV;
        	goto remove;
        }

        adev->dev_physaddr = acq400_resource->start;
        adev->dev_addrsize = acq400_resource->end -
                acq400_resource->start + 1;
        if (!request_mem_region(adev->dev_physaddr,
                adev->dev_addrsize, acq400_devnames[adev->of_prams.site])) {
                dev_err(&pdev->dev, "can't reserve i/o memory at 0x%08X\n",
                        adev->dev_physaddr);
                rc = -ENODEV;
                goto fail;
        }
        adev->dev_virtaddr =
        	ioremap(adev->dev_physaddr, adev->dev_addrsize);
        dev_dbg(DEVP(adev), "acq400: mapped 0x%0x to 0x%0x\n",
        	adev->dev_physaddr, (unsigned int)adev->dev_virtaddr);

        acq400_devices[ndevices++] = adev;
        acq400_getID(adev);

        if (IS_DUMMY(adev)){
        	acq400_createSysfs(&pdev->dev);
        	dev_info(DEVP(adev), "DUMMY device detected, quitting\n");
        	return 0;
        }

        rc = alloc_chrdev_region(&adev->devno, ACQ420_MINOR_0,
        		ACQ420_MINOR_MAX, acq400_devnames[adev->of_prams.site]);
        //status = register_chrdev_region(acq420_dev->devno, 1, MODULE_NAME);
        if (rc < 0) {
                dev_err(&pdev->dev, "unable to register chrdev\n");
                goto fail;
        }

        /* Register with the kernel as a character device */
        cdev_init(&adev->cdev, &acq400_fops);
        adev->cdev.owner = THIS_MODULE;
        rc = cdev_add(&adev->cdev, adev->devno, ACQ420_MINOR_MAX);
        if (rc < 0){
        	goto fail;
        }

        adev->isFifoError = acq420_isFifoError;	/* REMOVE me: better default wanted */

        if (IS_SC(adev)){
        	adev->is_sc = true;
        	if (allocate_hbm(adev, nbuffers, bufferlen, default_dma_direction)){
        		dev_err(&pdev->dev, "failed to allocate buffers");
        		goto fail;
        	}
        	adev->gpg_base = adev->dev_virtaddr + GPG_MEM_BASE;
        	adev->gpg_buffer = kmalloc(4096, GFP_KERNEL);

        	acq400_createSysfs(&pdev->dev);
        	acq400_init_proc(adev);
        	acq2006_createDebugfs(adev);
        	acq400sc_init_defaults(adev);
          	if (IS_AXI64(adev)){
          		if (nbuffers < AXI_BUFFER_COUNT){
          			AXI_BUFFER_COUNT = nbuffers - 1;
          			dev_warn(DEVP(adev),
          				".. not enough buffers limit to %d", nbuffers);
          		}
          		axi64_init_dmac(adev);
          	}
        	return 0;
        }else{
        	if (IS_XO(adev) && xo_use_bigbuf){
        		adev->hb = acq400_devices[0]->hb;
        		dev_info(DEVP(adev), "site %d using MAIN HB", adev->of_prams.site);
        	}else{
        		if (IS_AO424(adev)){
        			if (allocate_hbm(adev, AO420_NBUFFERS,
        					ao424_buffer_length, DMA_TO_DEVICE)){
        				dev_err(&pdev->dev, "failed to allocate buffers");
        				goto fail;
        			}
        		}else if (IS_AO42X(adev) || IS_DIO432X(adev)){
        			if (allocate_hbm(adev, AO420_NBUFFERS,
        				ao420_buffer_length, DMA_TO_DEVICE)){
        				dev_err(&pdev->dev, "failed to allocate buffers");
        				goto fail;
        			}
        		}
        	}
        }
        if (IS_AO42X(adev)||IS_DIO432X(adev)){
        	rc = devm_request_threaded_irq(
        	          	DEVP(adev), adev->of_prams.irq,
        	          	ao400_isr, xo400_dma,
        	          	IRQF_SHARED, acq400_devnames[adev->of_prams.site],
        	          	adev);
        }else{
        	rc = devm_request_irq(
        			DEVP(adev), adev->of_prams.irq, acq400_isr,
        			IRQF_SHARED, acq400_devnames[adev->of_prams.site],
        			adev);
        }

  	if (rc){
  		dev_err(DEVP(adev),"unable to get IRQ%d\n",adev->of_prams.irq);
  		goto fail;
  	}


  	if (IS_ACQ42X(adev)){
  		acq420_init_defaults(adev);
  	}else if (IS_DIO432X(adev)){
  		dio432_init_defaults(adev);
  	}else{
  		switch(GET_MOD_ID(adev)){
  		case MOD_ID_ACQ430FMC:
  		case MOD_ID_ACQ435ELF:
  		case MOD_ID_ACQ437ELF:
  			acq43X_init_defaults(adev);
  			break;
  		case MOD_ID_AO420FMC:
  			ao420_init_defaults(adev);
  			break;
  		case MOD_ID_AO424ELF:
  			ao424_init_defaults(adev);
  			break;
  		case MOD_ID_BOLO8:
  		case MOD_ID_BOLO8B:
  			bolo8_init_defaults(adev);
  			break;
  		case MOD_ID_PMODADC1:
  			pmodadc1_init_defaults(adev);
  			break;
  		case MOD_ID_ACQ480FMC:
  			acq480_init_defaults(adev);
  			break;
  		case MOD_ID_DIO_BISCUIT:
			switch(GET_MOD_IDV(adev)){
  			case MOD_IDV_V2F:
  				v2f_init_defaults(adev);
  				break;
  			case MOD_IDV_DIO:
  			case MOD_IDV_QEN:
  				qen_init_defaults(adev);
  				break;
  			}
  			break;
  		case MOD_ID_PIG_CELF:
  			pig_celf_init_defaults(adev);
  		case MOD_ID_RAD_CELF:
  			rad_celf_init_defaults(adev);
  		case MOD_ID_DAC_CELF:
  			ao428_init_defaults(adev);
  			break;
  		default:
  			dev_warn(DEVP(adev), "no custom init for module type %x",
  		        		(adev)->mod_id>>MOD_ID_TYPE_SHL);
  		}
  	}

        acq400_createSysfs(&pdev->dev);
        acq400_init_proc(adev);
        acq400_createDebugfs(adev);

        return 0;

 fail:
 	--ndevices;
       	dev_err(&pdev->dev, "Bailout!\n");
 remove:
        acq400_remove(pdev);
        return rc;
}

static int acq400_remove(struct platform_device *pdev)
/* undo all the probe things in reverse */
{
	if (pdev->id == -1){
		return -1;
	}else{
		struct acq400_dev* adev = acq400_devices[pdev->id];

		if (adev == 0){
			return -1;
		}
		axi64_free_dmac(adev);
		acq400_removeDebugfs(adev);
		hbm_free(&pdev->dev, &adev->EMPTIES);
		hbm_free(&pdev->dev, &adev->REFILLS);
		acq400_delSysfs(&adev->pdev->dev);
		acq400_del_proc(adev);
		dev_dbg(&pdev->dev, "cdev_del %p\n", &adev->cdev);
		cdev_del(&adev->cdev);
		unregister_chrdev_region(adev->devno, 1);

		/* Unmap the I/O memory */
		if (adev->dev_virtaddr) {
			iounmap(adev->dev_virtaddr);
			release_mem_region(adev->dev_physaddr,
					adev->dev_addrsize);
		}
		/* Free the PL330 buffer client data descriptors */
		if (adev->client_data) {
			kfree(adev->client_data);
		}
		if (adev->gpg_buffer){
			kfree(adev->gpg_buffer);
		}

		kfree(adev->fifo_histo);
		kfree(adev);

		return 0;
	}
}
static struct platform_driver acq400_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = acq400_of_match,
        },
        .probe = acq400_probe,
        .remove = acq400_remove,
};

static void __exit acq400_exit(void)
{
	platform_driver_unregister(&acq400_driver);
	acq400_module_remove_proc();

}

static int __init acq400_init(void)
{
        int status;

	printk("D-TACQ ACQ400 FMC Driver %s\n", REVID);
	DMA_NS_TEST;
	acq400_module_init_proc();
	a400fs_init();
        status = platform_driver_register(&acq400_driver);

        return status;
}

module_init(acq400_init);
module_exit(acq400_exit);

void acq400_set_peripheral_SPI_CS(unsigned csword)
{
	struct acq400_dev* adev = acq400_devices[0];

	dev_dbg(DEVP(adev), "acq400_set_peripheral_SPI_CS() %08x\n", csword);
	acq400wr32(adev, SPI_PERIPHERAL_CS, csword);
}

EXPORT_SYMBOL_GPL(acq400_set_peripheral_SPI_CS);
EXPORT_SYMBOL_GPL(acq400_devices);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ ACQ400_FMC Driver");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);




