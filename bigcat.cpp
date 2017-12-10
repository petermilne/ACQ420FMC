#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

int BUFLEN=0x10000;
FILE* FOUT;


void cat(const char* fn, void* buf)
{
	FILE *fp = fopen(fn, "r");
	int nread;
	if (fp == 0){
		fprintf(stderr, "ERROR, failed to open \"%s\"\n", fn);
		exit(1);
	}
	while((nread = fread(buf, 1, BUFLEN, fp)) > 0){
		fwrite(buf, 1, nread, FOUT);
	}
}

void mapcat(const char* fn, void* buf_notused)
{
        FILE *fp = fopen(fn, "r");
	void *pv;
        if (fp == 0){
                fprintf(stderr, "ERROR, failed to open \"%s\"\n", fn);
                exit(1);
        }
	pv = mmap(0, BUFLEN, PROT_READ, MAP_PRIVATE, fileno(fp), 0);
	write(fileno(FOUT), pv, BUFLEN);
}

int main(int argc, char* argv[]){
	void* buf;
	int ii;
	void (*mycat)(const char*, void*) = cat;

	if (getenv("BUFLEN")){
		BUFLEN = strtoul(getenv("BUFLEN"),0,0);
	}
	if (getenv("MAPCAT")){
		mycat = mapcat;
	}
	if (getenv("FOUT")){
		FOUT = fopen(getenv("FOUT"), "w");
		if (FOUT == 0){
			fprintf(stderr, "ERROR failed to open \"%s\"\n", 
				getenv("FOUT"));
			exit(1);
		}
	}else{
		FOUT = stdout;
	}

	buf = malloc(BUFLEN);
	for (ii = 1; ii < argc; ++ii){
		mycat(argv[ii], buf);
	}
	return 0;
}
