/*
 * acq400_axi_dma_test_harness.cpp
 *
 *  Created on: 29 Sep 2015
 *      Author: pgm
 */


#include <sched.h>

#include <stdio.h>
#include <string.h>

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>
#include <vector>

#include <syslog.h>

//#define USE_DUMMY_START_DESCR

using namespace std;

int BUFFER_LEN = 1048576;

typedef unsigned u32;

struct buffer {
	char id[4];
	u32 pa;
	u32* va;
	int len;

	buffer() : pa(0), va(0), len(0) {
		strcpy(id, "-");
	}
};


struct xilinx_dma_desc_hw {
	u32 next_desc;	/* 0x00 */
	u32 pad1;	/* 0x04 */
	u32 buf_addr;	/* 0x08 */
	u32 pad2;	/* 0x0C */
	u32 pad3;	/* 0x10 */
	u32 pad4;	/* 0x14 */
	u32 control;	/* 0x18 */
	u32 status;	/* 0x1C */
	u32 app_0;	/* 0x20 */
	u32 app_1;	/* 0x24 */
	u32 app_2;	/* 0x28 */
	u32 app_3;	/* 0x2C */
	u32 app_4;	/* 0x30 */
	u32 endpad[3];
};

#define DSZ	(sizeof(struct xilinx_dma_desc_hw))

void getBuffers(vector<buffer*> &buffers)
{
	FILE* fp = fopen("/proc/driver/acq400/0/buffers", "r");
	char def[80];

	if (!fp){
		perror("/proc/driver/acq400/0/buffers");
		exit(1);
	}
	while(fgets(def, 80, fp)){
		char id[4] = {};
		unsigned pa, gash1, gash2;
		int nscan;
		int reject_count = 0;

		//printf("consider %s\n", def);

		if ((nscan = sscanf(def, "%3s,%x,%x,%d", id, &gash1, &pa, &gash2)) == 4){
			buffer *b = new buffer;
			strncpy(b->id, id, 4);
			b->pa = pa;
			b->len = BUFFER_LEN;
			buffers.push_back(b);
		}else{
			if (reject_count++){
				printf("## reject %s nscan:%d\n", def, nscan);
			}
		}
	}

	fclose(fp);
}

class DescriptorMapping {
	int fd;
	buffer* descriptors;
public:
	DescriptorMapping(vector<buffer*> &buffers);
	~DescriptorMapping();

	buffer* operator() (void){
		return descriptors;
	}
};

DescriptorMapping::DescriptorMapping(vector<buffer*> &buffers)
{
	descriptors = buffers.back();
	buffers.pop_back();
	char fname[80];
	sprintf(fname, "%s/%s", "/dev/acq400.0.hb/", descriptors->id);
	fd = open(fname, O_RDWR);
	if (fd < 0){
		perror(fname);
		exit(1);
	}
	descriptors->va = (unsigned*)mmap(0, BUFFER_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	if (descriptors->va == MAP_FAILED){
		perror("mmap hb");
		exit(1);
	}
}

DescriptorMapping::~DescriptorMapping(void)
{
	munmap(descriptors->va, BUFFER_LEN);
	close(fd);
}

u32 makeChain(vector<buffer*> &buffers, int ndesc)
{
	DescriptorMapping descriptorMapping(buffers);
	buffer* descriptors = descriptorMapping();
	xilinx_dma_desc_hw* hw_desc = (xilinx_dma_desc_hw*)descriptors->va;
	int ii = 0;
	unsigned pa;

	return descriptors->pa;
	// destroy mapping on scope exit
}

#define S2MM_DMACR	0x30
#define S2MM_DMASR	0x34
#define S2MM_CURDESC	0x38
#define S2MM_CURDESCM	0x3c
#define S2MM_TAILDESC	0x40
#define S2MM_TAILDESCM	0x44

#define S2MM_DMACR_STOP 0
#define S2MM_DMACR_RUN	(1<<0)
#define S2MM_DMACR_RST	(1<<2)
#define S2MM_DMACR_CYC	(1<<4)

void writePram(const char* param, unsigned value)
{
	char fname[80];
	snprintf(fname, 80, "%s/%s", "/sys/module/acq420fmc/parameters/", param);
	FILE* fp = fopen(fname, "w");
	if (fp != 0){
		fprintf(fp, "0x%x\n", value);
		fclose(fp);
	}else{
		perror(fname);
		exit(1);
	}
}

void dmacInit(unsigned chain_pa, int ndesc, int oneshot)
{
	// for cyclic, MUST NEVER reach TDESC, but for ONESHOT, MUST reach TDESC ..
	unsigned tail_pa = chain_pa + (ndesc-oneshot)*DSZ;

	writePram("AXI_HEAD_DESCR_PA", chain_pa);
	writePram("AXI_TAIL_DESCR_PA", tail_pa);
}


#define BL "/sys/module/acq420fmc/parameters/bufferlen"


int main(int argc, char* argv[])
{
	int ndesc = argc>1? atoi(argv[1]): 5;
	int oneshot = 0;

	if (argc > 2){
		BUFFER_LEN = strtoul(argv[2],0, 0);
	}
	if (argc > 3){
		oneshot = atoi(argv[3]);
	}
	openlog("acq400_axi_dma_test_harness", LOG_PID, LOG_USER);
	syslog(LOG_DEBUG, "%d buffers each length: %d", ndesc, BUFFER_LEN);

	//printf("size: %d\n", sizeof(struct xilinx_dma_desc_hw));
	assert(sizeof(struct xilinx_dma_desc_hw) == 64);
	vector<buffer*> buffers;
	getBuffers(buffers);

	std::vector<buffer*>::const_iterator i;
/*
	for(i=buffers.begin(); i!=buffers.end(); ++i){
		printf("%s pa 0x%08x\n", (*i)->id, (*i)->pa);
	}
*/
	u32 chain_pa = makeChain(buffers, ndesc);

	dmacInit(chain_pa, ndesc, oneshot);

	return 0;
}
