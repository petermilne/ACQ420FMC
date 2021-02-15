/* streaming DAC */


#include <stdio.h>
#include <unistd.h>

inline int inc(int cursor, int max)
{
	if (++cursor == max){
		return 0;
	}else{
		return cursor;
	}
}


void read_buffers(unsigned descriptors[], int ndesc)
/* fake it */
{
	usleep(100000);
}
int main(int argc, char* argv[]) {
	unsigned descriptors[128];

	FILE* fp = fopen("/dev/acq400.0.dac", "a+");
	if (fp == 0){
		fprintf(stderr, "ERROR failed to open file\n");
		return -1;
	}
	int nbuf = fread(descriptors, sizeof(unsigned), 30, fp);
	printf("nbuf:%d\n", nbuf);

	int put = 0;
	int get = 0;
	while(1){
		read_buffers(descriptors+get, 2);

		int nw = fwrite(descriptors+get, sizeof(unsigned), 2, fp);
		if (nw == 2){
			printf("wr %08x %08x\n", descriptors[get], descriptors[get+1]);
		}else{
			printf("write %d returned %d\n", get, nw);
		}
		get = inc(get, nbuf);
		get = inc(get, nbuf);

		int nr = fread(descriptors+put, sizeof(unsigned), 2, fp);
		if (nr == 2){
			printf("rd %08x %08x\n", descriptors[put], descriptors[put+1]);
		}else{
			printf("read %d returned %d\n", put, nr);
		}
		put = inc(put, nbuf);
		put = inc(put, nbuf);
	}
	return 0;
}
