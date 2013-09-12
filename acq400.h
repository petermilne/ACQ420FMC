/*
 * acq420FMC.h
 *
 *  Created on: Mar 11, 2013
 *      Author: pgm
 */

#ifndef ACQ420FMC_H_
#define ACQ420FMC_H_

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/sizes.h>
#include <asm/dma.h>
#include <asm/mach/dma.h>
#include <asm/io.h>
//#include <mach/pl330.h>
#include <linux/amba/pl330.h>
#include <linux/of.h>

/* Offsets for control registers in the AXI MM2S FIFO */
#define AXI_FIFO              0x1000
#define AXI_FIFO_LEN          0x1000

#define OF_IRQ_HITIDE		1	/* index of HITIDE in dtb */
#define OF_IRQ_STATUS		4	/* index of STATUS in dtb */
#define OF_IRQ_COUNT		6	/* number of items */
#define OF_IRQ_MAGIC		32	/* add to the OF number to get actual */

#define ADC_BASE		0x0000
#define MOD_ID			(ADC_BASE+0x00)
#define ADC_CTRL		(ADC_BASE+0x04)
#define DAC_CTRL		ADC_CTRL
#define TIM_CTRL		(ADC_BASE+0x08)
#define ADC_HITIDE		(ADC_BASE+0x0C)
#define ADC_FIFO_SAMPLES	(ADC_BASE+0x10)
#define ADC_FIFO_STA		(ADC_BASE+0x14)
#define ADC_INT_CSR		(ADC_BASE+0x18)
#define ADC_CLK_CTR		(ADC_BASE+0x1C)
#define ADC_SAMPLE_CTR		(ADC_BASE+0x20)

#define ADC_CLKDIV		(ADC_BASE+0x40)
#define ADC_GAIN		(ADC_BASE+0x44)
/* obsolete R3
#define ADC_FORMAT 		(ADC_BASE+0x48)
*/
#define ADC_CONV_TIME 		(ADC_BASE+0x4C) /*(mask 0x000000FF)*/

#define ACQ435_MODE		(ADC_BASE+0x44)
#define AO420_RANGE		(ADC_BASE+0x44)

#define ADC_FIFO_SAMPLE_MASK	((1<<14)-1)

#define FIFO_HISTO_SZ	      	(1<<8)
#define STATUS_TO_HISTO(stat)	(((stat)&ADC_FIFO_SAMPLE_MASK)>>(14-8))


#define MOD_ID_TYPE_SHL		24
#define MOD_ID_VERSION_SHL	16
#define MOD_ID_REV_SHL		0
#define MOD_ID_REV_MASK		0x0000ffff

#define MOD_ID_ACQ420FMC	1
#define MOD_ID_ACQ435ELF	2
#define MOD_ID_ACQ430FMC	3
#define MOD_ID_ACQ440FMC	4
#define MOD_ID_ACQ425ELF	5

#define MOD_ID_AO420FMC		0x40
#define MOD_ID_AO421FMC		0x41

#define DAC_CTRL_LL		(1 << 8)	/* AO420FMC */
#define ADC_CTRL32B_data	(1 << 7)
#define ADC_CTRL_18B		(1 << 6)
#define ADC_CTRL_RAMP_EN 	(1 << 5)
#define ADC_CTRL_ADC_EN		(1 << 4)
#define ADC_CTRL_ADC_RST	(1 << 3)
#define ADC_CTRL_FIFO_EN	(1 << 2)
#define ADC_CTRL_FIFO_RST	(1 << 1)
#define ADC_CTRL_MODULE_EN	(1 << 0)

#define ADC_CTRL_RST_ALL 	(ADC_CTRL_ADC_RST | ADC_CTRL_FIFO_RST)
#define ADC_CTRL_ENABLE_ALL	(ADC_CTRL_ADC_EN | ADC_CTRL_FIFO_EN|ADC_CTRL_MODULE_EN)


#define TIM_CTRL_EVENT1_SHL	28
#define TIM_CTRL_EVENT0_SHL	24
#define TIM_CTRL_TRIG_SHL	20
#define TIM_CTRL_SYNC_SHL	16
#define TIM_CTRL_CLK_SHL	12
#define TIM_CTRL_MODE_SHL	0

#define TIM_CTRL_SIG_MASK	0xf
#define TIM_CTRL_SIG_RISING	0x8
#define TIM_CTRL_SIG_SEL	0x7

#define TIM_CTRL_MODE_EV1_EN	(1 << (4+TIM_CTRL_MODE_SHL))
#define TIM_CTRL_MODE_EV0_EN	(1 << (3+TIM_CTRL_MODE_SHL))
#define TIM_CTRL_MODE_HW_TRG	(1 << (2+TIM_CTRL_MODE_SHL))
#define TIM_CTRL_MODE_SYNC	(1 << (1+TIM_CTRL_MODE_SHL))
#define TIM_CTRL_MODE_HW_CLK	(1 << (0+TIM_CTRL_MODE_SHL))



#define ADC_FIFO_STA_CLK	(1<<7)
#define ADC_FIFO_STA_TRG	(1<<6)
#define ADC_FIFO_STA_ACC	(1<<5)
#define ADC_FIFO_STA_ACTIVE	(1<<4)
#define ADC_FIFO_STA_FULL	(1<<3)
#define ADC_FIFO_STA_EMPTY	(1<<2)
#define ADC_FIFO_STA_OVER	(1<<1)
#define ADC_FIFO_STA_UNDER	(1<<0)

#define ADC_FIFO_STA_ERR \
	(ADC_FIFO_STA_UNDER|ADC_FIFO_STA_OVER|ADC_FIFO_STA_FULL)

#define ADC_FIFO_FLAGS 		(ADC_FIFO_STA_ERR|ADC_FIFO_STA_EMPTY)

#define ADC_INT_CSR_COS		(1<<9)
#define ADC_INT_CSR_HITIDE	(1<<8)

#define ADC_INT_CSR_COS_EN	(1<<1)
#define ADC_INT_CSR_HITIDE_EN	(1<<0)


#define ADC_CLK_CTR_SRC_SHL	28
#define ADC_CLK_CTR_SRC_MASK	0xf		/* 1:16 sources */
#define ADC_CLK_CTR_MASK	0x0fffffff	/* 28 bit count */

#define ADC_SAMPLE_CTR_MASK	0x0fffffff

#define ADC_CLK_DIV_MASK	0x0000ffff

#define ADC_CONV_TIME_500	0x96
#define ADC_CONV_TIME_1000	0x36

#define ADC_HT_INT		91
#define HITIDE			2048


#define ACQ435_MODE_HIRES_512	(1<<4)
#define ACQ435_MODE_B3DIS	(1<<3)
#define ACQ435_MODE_B2DIS	(1<<2)
#define ACQ435_MODE_B1DIS	(1<<1)
#define ACQ435_MODE_B0DIS	(1<<0)

#define ACQ435_MODE_BXDIS	0xf

#define MODULE_NAME             "acq420"


/* AO420FMC */

/*
 *  Minor encoding
 *  0 : the original device
 *  100..164 : buffers
 *  200..231 : channels when available
 */
#define ACQ420_MINOR_0	        0
#define ACQ420_MINOR_CONTINUOUS	1
#define ACQ420_MINOR_HISTO	2
#define ACQ420_MINOR_HB0	3	// block on HB0 fill
#define ACQ420_MINOR_MAX	240
#define ACQ420_MINOR_BUF	100
#define ACQ420_MINOR_BUF2	199
#define ACQ420_MINOR_CHAN	200
#define ACQ420_MINOR_CHAN2	232	// in reality 203 of course, but looking ahead ..

#define IS_BUFFER(minor) \
	((minor) >= ACQ420_MINOR_BUF && (minor) <= ACQ420_MINOR_BUF2)
#define BUFFER(minor) 		((minor) - ACQ420_MINOR_BUF)

#define AO_CHAN	4
/** acq400_dev one descriptor per device */
struct acq400_dev {
	dev_t devno;
	struct mutex mutex;
	struct cdev cdev;
	struct platform_device *pdev;
	struct dentry* debug_dir;
	char *debug_names;

	int mod_id;
	wait_queue_head_t waitq;

	struct pl330_client_data *client_data;

	struct OF_PRAMS {
		u32 site;
		u32 dma_channel;
		u32 fifo_depth;
		u32 burst_length;
		u32 irq;
		u32 fake;
	} of_prams;

	u32 DMA_READY;

	struct dma_chan* dma_chan;

	/* Current DMA buffer information */
	/*dma_addr_t buffer_d_addr;
	void *buffer_v_addr;*/
	size_t count;
	size_t this_count;
	int busy;

	/* Hardware device constants */
	u32 dev_physaddr;
	void *dev_virtaddr;
	u32 dev_addrsize;

	/* Driver reference counts */
	u32 writers;

	struct STATS {
	/* Driver statistics */
		u32 bytes_written;
		u32 writes;
		u32 reads;
		u32 opens;
		u32 closes;
		u32 errors;

		u32 fifo_interrupts;
		u32 dma_transactions;
		int shot;
		int run;
	} stats;

	int ramp_en;
	int data32;
	int adc_18b;			/* @@todo set on probe() */
	int nchan_enabled;
	int word_size;
	int is_slave;			/** @@todo how does this get set? */

	struct mutex list_mutex;
	struct list_head EMPTIES;	/* empties waiting isr       */
	struct list_head REFILLS;	/* full buffers waiting app  */
	struct list_head OPENS;		/* buffers in use by app (1) */
	struct HBM** hb;
	int nbuffers;			/* number of buffers available */

	int oneshot;
	struct proc_dir_entry *proc_entry;
	struct CURSOR {
		struct HBM* hb;
		int offset;
	} cursor;
	wait_queue_head_t refill_ready;
	wait_queue_head_t hb0_marker;
	unsigned long hb0_last;

	unsigned *fifo_histo;

	struct RUN_TIME {
		int refill_error;
		int please_stop;
		unsigned nget;
		unsigned nput;
		unsigned hb0_count;
		unsigned hb0_ix;
	} rt;

	struct AO_Immediate {
		union {
			short ch[AO_CHAN];
			unsigned lw[AO_CHAN/2];
		} _u;
	} AO_immediate;
};


/** acq400_path_descriptor - one per open path */
struct acq400_path_descriptor {
	struct acq400_dev* dev;
	int minor;
	struct HBM *hbm;
};

#define PD(filp)		((struct acq400_path_descriptor*)filp->private_data)
#define PDSZ			(sizeof (struct acq400_path_descriptor))
#define ACQ400_DEV(filp)	(PD(filp)->dev)
#define DEVP(adev)		(&(adev)->pdev->dev)

extern struct acq400_dev* acq400_devices[];
extern const char* acq400_names[];

void acq400_createSysfs(struct device *dev);
void acq400_delSysfs(struct device *dev);

void acq400_module_init_proc(void);
void acq400_module_remove_proc(void);
void acq400_init_proc(struct acq400_dev* acq420_dev);
void acq400_del_proc(struct acq400_dev* acq420_dev);

void acq400wr32(struct acq400_dev *adev, int offset, u32 value);
u32 acq400rd32(struct acq400_dev *adev, int offset);

int getHeadroom(struct acq400_dev *adev);

#define MAXDMA	0x4000

#define GET_FULL_OK		0
#define GET_FULL_DONE 		1
#define GET_FULL_REFILL_ERR	2

#define FIFO_PA(adev)  ((adev)->dev_physaddr + AXI_FIFO)

#define NCHAN	4
#define BYTES_PER_CHANNEL(adev) ((adev)->data32? 4: 2)

#define IS_ACQ420(adev) ((adev)->mod_id>>MOD_ID_TYPE_SHL == MOD_ID_ACQ420FMC)
#define IS_ACQ435(adev) ((adev)->mod_id>>MOD_ID_TYPE_SHL == MOD_ID_ACQ435ELF)
#define IS_AO420(adev)  ((adev)->mod_id>>MOD_ID_TYPE_SHL == MOD_ID_AO420FMC)
#define IS_AO421(adev)  ((adev)->mod_id>>MOD_ID_TYPE_SHL == MOD_ID_AO421FMC)


#endif /* ACQ420FMC_H_ */
