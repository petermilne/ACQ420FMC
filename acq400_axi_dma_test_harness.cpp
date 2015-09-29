/*
 * acq400_axi_dma_test_harness.cpp
 *
 *  Created on: 29 Sep 2015
 *      Author: pgm
 */




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

using namespace std;

int BUFFER_LEN = 1048576;

struct buffer {
	char id[4];
	unsigned pa;
	unsigned* va;
	int len;

	buffer() : pa(0), va(0), len(0) {
		strcpy(id, "-");
	}
};

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

buffer* makeDescriptorMapping(vector<buffer*> &buffers)
{
	buffer * descriptors = buffers.back();
	buffers.pop_back();
	char fname[80];
	sprintf(fname, "%s/%s", "/dev/acq400.0.hb/", descriptors->id);
	int fd = open(fname, O_RDWR);
	if (fd < 0){
		perror(fname);
		exit(1);
	}
	descriptors->va = (unsigned*)mmap(0, BUFFER_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	if (descriptors->va == MAP_FAILED){
		perror("mmap hb");
		exit(1);
	}
	return descriptors;
}

unsigned* makeDmacMapping()
{
	const char* fname = "/dev/mem";
	unsigned *va;
	int fd = open(fname, O_RDWR);
	if (fd < 0){
		perror(fname);
		exit(1);
	}
	va = (unsigned*)mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x801f0000);
	if (va == MAP_FAILED){
		perror("mmap /dev/mem");
		exit(1);
	}
	return va;
}
int main(int argc, char* argv[])
{
	vector<buffer*> buffers;
	getBuffers(buffers);

	std::vector<buffer*>::const_iterator i;
	/*
	for(i=buffers.begin(); i!=buffers.end(); ++i){
		printf("%s pa 0x%08x\n", (*i)->id, (*i)->pa);
	}
	*/
	buffer* descriptors = makeDescriptorMapping(buffers);
	volatile unsigned *dmac = makeDmacMapping();
	return 0;
}
