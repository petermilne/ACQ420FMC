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
#include <mach/pl330.h>
#include <linux/of.h>

/* Offsets for control registers in the AXI MM2S FIFO */
#define AXI_FIFO              0x0
#define AXI_FIFO_LEN          0x1000



#define ALG_BASE		0x1000
#define ALG_CTRL		(ALG_BASE+0x00)
#define ALG_HITIDE		(ALG_BASE+0x04)
#define ALG_SAMPLES		(ALG_BASE+0x08)
#define ALG_STATUS		(ALG_BASE+0x0C)
#define ALG_INT_CTRL		(ALG_BASE+0x10)
#define ALG_INT_FORCE		(ALG_BASE+0x14)
#define ALG_INT_STAT		(ALG_BASE+0x18)

#define ALG_CLKDIV		(ALG_BASE+0x40)
#define ALG_GAIN		(ALG_BASE+0x44)

#define ALG_ADC_OPTS 		(ALG_BASE+0x48)

#define ALG_ADC_CONV_TIME 	(ALG_BASE+0x4C) /*(mask 0x000000FF)*/

#define ALG_HITIDE_MASK		0x0000fff

#define DATA_FIFO_SZ	      	255
#define STATUS_TO_HISTO(stat)	(((stat)&ALG_HITIDE_MASK)>>4)

#define ALG_CTRL_RAMP_ENABLE 	(1 << 5)
#define ALG_CTRL_ADC_ENABLE	(1 << 4)
#define ALG_CTRL_ADC_RESET	(1 << 3)
#define ALG_CTRL_FIFO_ENABLE	(1 << 2)
#define ALG_CTRL_FIFO_RESET	(1 << 1)
#define ALG_CTRL_ALG_ENABLE	(1 << 0)

#define ALG_CTRL_RESETALL 	(ALG_CTRL_ADC_RESET | ALG_CTRL_FIFO_RESET)
#define ALG_CTRL_ENABLE_ALL	(ALG_CTRL_ADC_ENABLE | ALG_CTRL_FIFO_ENABLE | ALG_CTRL_ALG_ENABLE)


#define ALG_ADC_OPTS_32B_data 	(1 << 1)
#define ALG_ADC_OPTS_IS_18B 	(1 << 0)

#define ALG_ADC_CONV_TIME_DEF	0x36

#define ADC_HT_INT		91
#define HITIDE			2048

#define MODULE_NAME             "acq420"


/*
 *  Minor encoding
 *  0 : the original device
 *  100..164 : buffers
 *  200..231 : channels when available
 */
#define ACQ420_MINOR_0	        0
#define ACQ420_MINOR_CONTINUOUS	1
#define ACQ420_MINOR_HISTO	2
#define ACQ420_MINOR_MAX	240
#define ACQ420_MINOR_BUF	100
#define ACQ420_MINOR_BUF2	199
#define ACQ420_MINOR_CHAN	200
#define ACQ420_MINOR_CHAN2	232	// in reality 203 of course, but looking ahead ..

#define BUFFER(minor) ((minor) > ACQ420_MINOR_BUF)

/** acq420_dev one descriptor per device */
struct acq420_dev {
	dev_t devno;
	struct mutex mutex;
	struct cdev cdev;
	struct platform_device *pdev;

	wait_queue_head_t waitq;

	struct pl330_client_data *client_data;

	u32 dma_channel;
	u32 fifo_depth;
	u32 burst_length;
	u32 irq;		/* device IRQ number	*/
	u32 DMA_READY;

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

	/* Driver statistics */
	u32 bytes_written;
	u32 writes;
	u32 reads;
	u32 opens;
	u32 closes;
	u32 errors;

	int ramp_en;

	struct list_head buffers;
	struct HBM** hb;
	int nbuffers;

	int oneshot;
	struct proc_dir_entry *proc_entry;
	struct {
		struct HBM* hb;
		int offset;
	} cursor;

	unsigned *fifo_histo;
};

#define EMPTIES	buffers
#define REFILLS buffers

/** acq420_path_descriptor - one per open path */
struct acq420_path_descriptor {
	struct acq420_dev* dev;
	int minor;
};

#define PD(filp)		((struct acq420_path_descriptor*)filp->private_data)
#define PDSZ			(sizeof (struct acq420_path_descriptor))
#define ACQ420_DEV(filp)	(PD(filp)->dev)
#define DEVP(adev)		(&(adev)->pdev->dev)

extern struct acq420_dev* acq420_devices[];
extern const char* acq420_names[];

void acq420_createSysfs(struct device *dev);
void acq420_delSysfs(struct device *dev);

void acq420_module_init_proc(void);
void acq420_module_remove_proc(void);
void acq420_init_proc(struct acq420_dev* acq420_dev, int idev);
void acq420_del_proc(struct acq420_dev* acq420_dev);

void acq420wr32(struct acq420_dev *adev, int offset, u32 value);
u32 acq420rd32(struct acq420_dev *adev, int offset);

int getHeadroom(struct acq420_dev *adev);

#define MAXDMA	0x1000
#endif /* ACQ420FMC_H_ */
