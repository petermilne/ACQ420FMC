/*
 * acq420FMC.h
 *
 *  Created on: Mar 11, 2013
 *      Author: pgm
 */

#ifndef ACQ420FMC_H_
#define ACQ420FMC_H_


/* Offsets for control registers in the AXI MM2S FIFO */
#define AXI_FIFO              0x0
#define AXI_FIFO_LEN          0x1000

#define ALG_CTRL		0x1000
#define ALG_HITIDE		0x1004
#define ALG_SAMPLES		0x1008
#define ALG_STATUS		0x100C
#define ALG_INT_CTRL		0x1010
#define ALG_INT_FORCE		0x1014
#define ALG_INT_STAT		0x1018


#define ALG_CTRL_ADC_ENABLE	(1 << 4)
#define ALG_CTRL_ADC_RESET	(1 << 3)
#define ALG_CTRL_FIFO_ENABLE	(1 << 2)
#define ALG_CTRL_FIFO_RESET	(1 << 1)
#define ALG_CTRL_ALG_ENABLE	(1 << 0)

#define ALG_CTRL_RESETALL 	(ALG_CTRL_ADC_RESET | ALG_CTRL_FIFO_RESET)
#define ALG_CTRL_ENABLE_ALL	(ALG_CTRL_ADC_ENABLE | ALG_CTRL_FIFO_ENABLE | ALG_CTRL_ALG_ENABLE)

#define ADC_HT_INT		91
#define HITIDE			0x40

#define MODULE_NAME             "acq420"
#define XFIFO_DMA_MINOR         0



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
	dma_addr_t buffer_d_addr;
	void *buffer_v_addr;
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
};


#endif /* ACQ420FMC_H_ */
