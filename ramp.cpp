#include <stdio.h>
#include <stdlib.h>
#include <string.h>


template <class T>
int ramp(int nsam, int step, int ssize)
{
	int nwords = nsam * ssize;
	T xx = 0;
	int ch = 0;
	for (; nwords; nwords--){
		fwrite(&xx, sizeof(xx), 1, stdout);
		if (++ch >= ssize){
			xx += step;
			ch = 0;
		}
	}
	return 0;
}

int main(int argc, char* argv[])
{
	if (argc == 1 || strstr(argv[1], "h") != 0){
		printf("ramp [nsam=1024 [step=1 [wsize=4 [ssize=1]]]\n");
		printf("eg ramp 64000 1 4 32\n");
		exit(1);
	}
	int length = argc>1? strtoul(argv[1], 0, 0): 1024;
	int step = argc>2? atoi(argv[2]): 1;
	int wsize = argc>3?  atoi(argv[3]): 4;
	int ssize = argc>4?  atoi(argv[4]): 1;

	switch(wsize){
	case 2:
		return ramp<short>(length, step, ssize);
	case 4:
		return ramp<int>(length, step, ssize);
	default:
		fprintf(stderr, "ERROR: only size 4 supported\n");
		return -1;
	}
}
