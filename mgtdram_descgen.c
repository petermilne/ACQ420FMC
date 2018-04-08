#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>


#define DESCLEN	0x400000
#define SCALE	12
#define ID	 8
#define ID_MASK	0x00ff

int fd_out = 1;

int descgen1(int dx)
{
	unsigned block = DESCLEN >> SCALE;
	unsigned addr = block * dx;
	unsigned descr = addr << ID | (dx&ID_MASK);
	if (write(fd_out, &descr, sizeof(descr)) != sizeof(descr)){
		perror("write");
		exit(1);
	}
	return 0;
}
int descgen(int d0, int d1)
{
	int dx;
	for (dx = d0; dx <= d1; ++dx){
		descgen1(dx);
	}
	return 0;
}
int main(int argc, char* argv[])
{
	int d0 = argc>1? atoi(argv[1]): 0;
	int d1 = argc>2? atoi(argv[argc-1]): d0;
	return descgen(d0, d1);
}
