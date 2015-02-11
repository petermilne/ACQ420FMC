/*
 * acq400_stream_disk.cpp
 *
 *  Created on: 10 Feb 2015
 *      Author: pgm
 *
 *     usage: scq400_stream_disk NBUFFERS dest1 [dest2]
 *     Read full rate data on stdin
 *     farm to one or more file trees  destX/%03d/%03d.dat
 *     Write data in 1MB files
 *     STOP after NBUFFERS x 1MB files
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>


#define BUFFERLEN	0x100000
#define FILESDIR	100

int bufferlen = BUFFERLEN;
int verbose = 0;

static void processBuffer(const char* outroot, int ibuf, short* buf, int nbuf){
	if (verbose){
		fprintf(stderr, "%02d\n", ibuf);
	}

	char fname[80];
	int cycle = ibuf/FILESDIR;


	sprintf(fname, "%s/%03d/", outroot, cycle);
	mkdir(fname, 0777);
	sprintf(fname, "%s/%03d/%02d.dat", outroot, cycle, ibuf);

	FILE* fp = fopen(fname, "w");

	if (fp == 0){
		perror(fname);
		_exit(errno);
	}
	fwrite(buf, sizeof(short), nbuf, fp);
	fclose(fp);
}

void process(int nbuffers, int ndest, const char* dests[])
{

	int nshorts = BUFFERLEN/sizeof(short);
	short* buf = new short[nshorts];
	long ibuf = 0;

	while(fread(buf, sizeof(short), nshorts, stdin) == nshorts){
		for (int id = 0; id < ndest; ++id){
			processBuffer(dests[id], ibuf, buf, nshorts);
		}
		++ibuf;
	}
}

int main(int argc, const char* argv[])
{
	int nbuffers = 1000;
	int idest;
	if (argc < 3){
		fprintf(stderr, "USAGE: acq400_stream_disk NBUFFERS dest1 [dest2]\n");
		return -1;
	}else{
		nbuffers = atoi(argv[1]);
	}

	process(nbuffers, argc - 2, &argv[2]);
}

