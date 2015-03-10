/*
 * mgt400.h
 *
 *  Created on: 12 Jan 2015
 *      Author: pgm
 */

#ifndef MGT400_H_
#define MGT400_H_

#define DESC_HISTOLEN	128
#define DATA_HISTOLEN	128
#define DESC_HMASK  	(DESC_HISTOLEN-1)
#define DATA_HMASK	(DATA_HISTOLEN-1)

struct mgt400_dev {
	dev_t devno;
	struct of_prams {
		int site;
		int irq;
		int sn;
	} of_prams;
	char devname[16];

	struct platform_device *pdev;
	struct resource *mem;
	void *dev_virtaddr;
	struct cdev cdev;
	char* debug_names;
	struct dentry* debug_dir;
	int RW32_debug;

	struct DMA_CHANNEL {
		unsigned long buffer_count;
		unsigned long desc_histo[DESC_HISTOLEN];
		unsigned long data_histo[DATA_HISTOLEN];
		unsigned long first_descriptors[DESC_HISTOLEN];
		int fd_ix;
	} push, pull;

	struct hrtimer buffer_counter_timer;
};

struct mgt400_path_descriptor {
	struct mgt400_dev* dev;
	int minor;
};

#undef PD
#undef PDSZ
#define PD(filp)	((struct mgt400_path_descriptor*)filp->private_data)
#define PDSZ		(sizeof (struct mgt400_path_descriptor))

extern struct mgt400_dev* mgt400_devices[];

void mgt400wr32(struct mgt400_dev *adev, int offset, u32 value);
u32 mgt400rd32(struct mgt400_dev *adev, int offset);
void mgt400_createDebugfs(struct mgt400_dev* adev);
void mgt400_createSysfs(struct device *dev);
void mgt400_createDebugfs(struct mgt400_dev* adev);
void mgt400_removeDebugfs(struct mgt400_dev* adev);
void mgt400_clear_counters(struct mgt400_dev* mdev);

int mgt400_clear_histo(struct mgt400_dev *mdev, int minor);

/* ZYNQ : RW    HOST: RO (at start only) */
#undef MOD_ID
#define MOD_ID		(0x0000)
#define ZDMA_CR		(0x0004)
#define HEART		(0x0008)
#define AURORA_CR	(0x000C)
#define AURORA_SR	(0x0010)
#define ZIDENT		(0x0014)
#define COMMS_TXB_FSR	(0x0020)
#define COMMS_TXB_FCR	(0x0024)
#define COMMS_RXB_FSR   (0x0040)
#define COMMS_RXB_FCR	(0x0044)

#define ZDMA_CR_ENABLE		(1<<0)

#define AURORA_CR_ENA		(1<<31)
#define AURORA_CR_CLR		(1<<7)
#define AURORA_CR_PWR_DWN	(1<<4)
#define AURORA_CR_LOOPBACK	(0x7)

#define AURORA_SR_HARD_ERR	(1<<6)
#define AURORA_SR_SOFT_ERR	(1<<5)
#define AURORA_SR_FRAME_ERR	(1<<4)
#define AURORA_SR_CHANNEL_UP	(1<<1)
#define AURORA_SR_LANE_UP	(1<<0)

#define AURORA_SR_ERR_XX \
	(AURORA_SR_HARD_ERR|AURORA_SR_SOFT_ERR|AURORA_SR_FRAME_ERR)

#define AURORA_SR_UP_MASK 	(AURORA_SR_CHANNEL_UP|AURORA_SR_LANE_UP)

#define AURORA_SR_ERR(xx) (((xx)&AURORA_SR_ERR_XX)>>4)
#define AURORA_SR_UP(xx)  (((xx)&AURORA_SR_UP_MASK)>>0)

/** PCIE REGS : ZYNQ:RO HOST: RW */
#define PCIE_CTRL 	(0x1004)
#define PCIE_INTR	(0x1008)
#define PCI_CSR		(0x100C)
#define PCIE_DEV_CSR	(0x1010)
#define PCIE_LINK_CSR	(0x1014)
#define PCIE_CONF	(0x1018)
#define PCIE_BUF_CTRL	(0x101C)

/** DMA REGS : ZYNQ:RO HOST:RW */
#define DMA_TEST		(0x2000)
#define DMA_CTRL		(0x2004)
#define DMA_FIFO_SR		(0x2008)
#define DESC_FIFO_SR		(0x200C)
#define DMA_PUSH_DESC_SR	(0x2010)
#define DMA_PULL_DESC_SR	(0x2014)
#define DMA_PUSH_DESC_FIFO	(0x2040)
#define DMA_PULL_DESC_FIFO	(0x2080)

#define DMA_DATA_PULL_SHL		16
#define DMA_DATA_PUSH_SHL		0

#define DMA_DATA_FIFO_COUNT		0xfff0
#define DMA_DATA_FIFO_COUNT_SHL		4
#define DMA_DATA_FIFO_FLAGS		0x000f

#define GET_DMA_DATA_FIFO_COUNT(sta) \
	(((sta)&DMA_DATA_FIFO_COUNT)>>DMA_DATA_FIFO_COUNT_SHL)

#define DESCR_ADDR	0xffffffc00
#define DESCR_INTEN	0x000000100
#define DESCR_LEN	0x0000000f0
#define DESCR_ID	0x00000000f

#define MINOR_PUSH_DATA_HISTO	0
#define MINOR_PUSH_DESC_HISTO 	1
#define MINOR_PULL_DATA_HISTO	2
#define MINOR_PULL_DESC_HISTO	3
#define MINOR_PUSH_DESC_LIST	4
#define MINOR_PULL_DESC_LIST	5
#define MINOR_COUNT		6

#endif /* MGT400_H_ */
