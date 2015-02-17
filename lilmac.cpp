/* ------------------------------------------------------------------------- *
 * lilmac.cpp : same external interface as bigmac, but uses hardware multiplier
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2013 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 11 Sep 2013  
 *    Author: pgm                                                         
 *                                                                           *
 *  This program is free software; you can redistribute it and/or modify     *
 *  it under the terms of Version 2 of the GNU General Public License        *
 *  as published by the Free Software Foundation;                            *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program; if not, write to the Free Software              *
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "popt.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

/*
 *  benchtesting: set a long buffer and time with time(1).
 *
 *
zb_012> time /tmp/bigmac -L 0x300000 -T null
real 0m 0.06s
user 0m 0.00s
sys 0m 0.05s
zb_012> time /tmp/bigmac -L 0x300000 -T cmac
real 0m 1.10s
user 0m 0.97s
sys 0m 0.12s
zb_012> time /tmp/bigmac -L 0x300000 -T nmac
real 0m 0.54s
user 0m 0.43s
sys 0m 0.09s
zb_012> time /tmp/bigmac -L 0x300000 -T nmac2
real 0m 0.44s
user 0m 0.32s
sys 0m 0.10s

SO: cmac : nmac : nmac2 :: 1.04 : 0.48 : 0.38

[pgm@hoy3 ACQ420_FMC]$ time ./bigmac -L 0×300000 -T cmac

real 0m0.078s
user 0m0.070s
sys 0m0.007s

=> 5x faster.

iop321:
time /tmp/bigmac -L 0x300000 -T cmac
real 0m 3.43s
user 0m 1.43s
sys 0m 0.99s

 Compile
  arm-xilinx-linux-gnueabi-g++ \
  	  -mcpu=cortex-a9 -mfloat-abi=softfp -mfpu=neon -DHASNEON \
	  -O3 -o bigmac bigmac.o -L../lib -lpopt
 */

#define NCHAN 	4
/* mac testbench */

#define MACSCALE	15		/** >>15 32768=100% */
#define PC100		32767		/** 100% */

int bufferlen = 0x10000;
const char *test = "null-test";
char *outfile;
char *infile;

/* for testing: use constant gains, offsets. @@todo These need to be set from the UI */
short gains[NCHAN] = { PC100, PC100, PC100, PC100 };
short offsets[NCHAN] = { 0, 0, 0, 0 };


char* gainslist;
char* offsetslist;

int mmap_in;
int mmap_out;
int monitor;
int playloop_site = -1;

struct poptOption opt_table[] = {
	{ "buflen", 'L', POPT_ARG_INT, &bufferlen, 0, 	"buffer length SAMPLES" },
	{ "test",   'T', POPT_ARG_STRING, &test, 0, "test mode" },
	{ "out",    'o', POPT_ARG_STRING, &outfile, 0, "output to file" },
	{ "in",     'i', POPT_ARG_STRING, &infile, 0, "input from file" },
	{ "gains",   'G', POPT_ARG_STRING, &gainslist, 'G', "gains"},
	{ "dc-offsets", 'D', POPT_ARG_STRING, &offsetslist, 'D', "DC offsets"},
	{ "mmap-in", 'I', POPT_ARG_STRING, &infile, 'I', "use mmap to access infile"},
	{ "mmap-out", 'O', POPT_ARG_STRING, &outfile, 'O', "use mmap to access outfile"},
	{ "monitor",  'M', POPT_ARG_INT, &monitor, 0, "monitor mode daemon" },
	{ "playloop_site", 'P', POPT_ARG_INT, &playloop_site, 0, "force reset" },
	POPT_AUTOHELP
	POPT_TABLEEND
};


void ui(int argc, const char** argv)
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int gx[4];
	int ox[4];
	int rc;
	while ( (rc = poptGetNextOpt( opt_context )) > 0 ){
		switch(rc){
		case 'G':
			if (sscanf(gainslist, "%d,%d,%d,%d",
				gx+0, gx+1, gx+2, gx+3) != 4){
				fprintf(stderr, "gainslist must be d,d,d,d\n");
				exit(-1);
			}else{
				for (int ii = 0; ii < 4; ++ii){
					gains[ii] = gx[ii];
				}
			}
			break;
		case 'D':
			if (sscanf(offsetslist, "%d,%d,%d,%d",
				ox+0, ox+1, ox+2, ox+3) != 4){
				fprintf(stderr, "offsetslist must be d,d,d,d\n");
				exit(-1);
			}else{
				for (int ii = 0; ii < 4; ++ii){
					offsets[ii] = ox[ii];
				}
			}
			break;
		case 'I':
			mmap_in = 1; break;
		case 'O':
			mmap_out = 1; break;
		default:
			;
		}
	}
}

// s6              /dev/mem 0x40060000 0x10000   0x40060000

volatile unsigned* go_regs;

void mapRegs(int playloop_site)
{
	FILE* fp = fopen("/dev/mem", "r+");
	unsigned offset = 0x40000000 + playloop_site*0x10000;
	void* region = mmap(NULL, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, fileno(fp), offset);

	go_regs = reinterpret_cast<volatile unsigned*>(
			static_cast<char*>(region)+0x80);
}


// NCHAN, gains, offsets
int exec_mac(short * src_notused, short* dst_notused)
{
	for (int cc = 0; cc < NCHAN; ++cc){
		short gain = gains[cc];
		short offset = offsets[cc];

		if (gain < -32767) gain = -32767;

		unsigned go = static_cast<unsigned>(gain << 16) |
			      (static_cast<unsigned>(offset)&0x0000ffff);

		go_regs[cc] = go;
	}
}

#define BIGMAC_KROOT	"BIGMAC_KROOT"
const char* kroot = "/dev/shm/awg";

class Knob {

public:
	int value;

	Knob(const char* root, const char* fn) : value(0) {
		char path[132];
		sprintf(path, "%s/%s", root, fn);
		FILE *fp = fopen(path, "r");
		if (fp != 0){
			int rc = fscanf(fp, "%d", &value);
			if (rc != 1){
				fprintf(stderr, "ERROR in fread : \"%s\", %d\n",
						path, rc);
			}
			fclose(fp);
		}else{
			perror(path);
			exit(1);
		}
	}
	virtual ~Knob() {
		;
	}
};

#define MAPPING	"/sys/module/acq420fmc/parameters/ao420_mapping"


class KnobGroup {
	Knob* knobs[4];


	static bool done_mapping;

	static void makeMapping() {
		for (int ii = 0; ii < 4; ++ii){
			mapping[ii] = ii;
		}
		FILE *fp = fopen(MAPPING, "r");
		int ii = 0;
		if (fp) {
			char map[4];
			int imax = fscanf(fp, "%d,%d,%d,%d", map+0, map+1,
					map+2, map+3);
			for (int ii = 0; ii < imax; ++ii){
				mapping[ii] = map[ii] - 1;
			}
		}
		fprintf(stderr, "mapping: %d %d %d %d\n",
			mapping[0], mapping[1], mapping[2], mapping[3]);
	}


public:
	static int mapping[4];

	KnobGroup(const char* n0, const char* n1, const char* n2, const char* n3)
	{
		if (!done_mapping){
			makeMapping();
			done_mapping = true;
		}
		knobs[0] = new Knob(kroot, n0);
		knobs[1] = new Knob(kroot, n1);
		knobs[2] = new Knob(kroot, n2);
		knobs[3] = new Knob(kroot, n3);

	}
	~KnobGroup(){
		for (int ii = 0; ii < 4; ++ii){
			delete knobs[ii];
		}
	}
	bool isDifferent(KnobGroup& rhs){
		for (int ii = 0; ii < 4; ++ii){
			if (knobs[ii]->value != rhs.knobs[ii]->value){
				return true;
			}
		}
		return false;
	}
	void toValues(short* values){
		for (int ii = 0; ii < 4; ++ii){
			int ival = knobs[ii]->value;
			short xx;
			if (ival > 32767){
				xx = 32767;
			}else if (ival < -32767){
				xx = -32767;
			}else{
				xx = ival;
			}
			values[mapping[ii]] = xx;
		}

	}
	void copyValues(KnobGroup& rhs){
		for (int ii = 0; ii < 4; ++ii){
			knobs[ii]->value = rhs.knobs[ii]->value;
		}
	}
};

int KnobGroup::mapping[4];
bool KnobGroup::done_mapping;

#define GAINS 	"G1", "G2", "G3", "G4"
#define OFFSETS "D1", "D2", "D3", "D4"

//inotifywait -e close_write -r /var/run/awg/

class Inotify {
	pid_t cpid;
public:
	Inotify() : cpid(0) {}
	void spawn() {
		if (cpid == 0){
			cpid = fork();
			if (cpid == 0){
				execlp("inotifywait", "inotifywait",
					"-q",
					"-e", "close_write",
					"-r", kroot,
					(char *) NULL);
			}
		}
	}
	void wait() {
		if (!cpid){
			spawn();
		}
		if (cpid){
			int status;
			waitpid(cpid, &status, 0);
			cpid = 0;
		}
	}
};

void init_knob(const char* root, const char* kb, int value)
{
	char fname[132];
	struct stat buf;

	sprintf(fname, "%s/%s", root, kb);
	if (stat(fname, &buf) != 0){
		FILE *fp = fopen(fname, "w");
		if (!fp){
			perror(fname);
			exit(1);
		}
		fprintf(fp, "%d\n", value);
		fclose(fp);
	}
}

void update_knob(const char* root, const char* kb, int value)
{
	char fname[132];

	sprintf(fname, "%s/%s", root, kb);

	FILE *fp = fopen(fname, "w");
	if (!fp){
		perror(fname);
		exit(1);
	}
	fprintf(fp, "%d\n", value);
	fclose(fp);
}

int read_int_knob(const char* root, const char* kb)
{
	char fname[132];


	sprintf(fname, "%s/%s", root, kb);

	FILE *fp = fopen(fname, "r");
	if (!fp){
		perror(fname);
		exit(1);
	}
	int value;
	int nscan = fscanf(fp, "%d\n", &value);
	fclose(fp);

	if (nscan){
		return value;
	}else{
		return 0;
	}
}
void init_knobs()
{
	char command[132];
	struct stat buf;
	if (stat(kroot, &buf) != 0){
		sprintf(command, "mkdir -p %s", kroot);
		system(command);
	}
	const char* gx[] = { GAINS };
	const char* dx[] = { OFFSETS };

	for (int ii = 0; ii < 4; ++ii){
		init_knob(kroot, gx[ii], 1024);
		init_knob(kroot, dx[ii], 0);
	}
}

typedef unsigned int u32;
#include "../AO421_ELF/ScratchPad.h"


static void spSet(Scratchpad& sp, int ii, short offsets[], short gains[])
{
	sp.set(Scratchpad::SP_AWG_G1+KnobGroup::mapping[ii],
					(offsets[ii]<<16) | gains[ii]);
}

void update_scratchpad(short offsets[], short gains[])
{
	Scratchpad sp = Scratchpad::instance();

	spSet(sp, 0, offsets, gains);
	spSet(sp, 1, offsets, gains);
	spSet(sp, 2, offsets, gains);
	spSet(sp, 3, offsets, gains);
}
int run_monitor(short *src, short *dst)
{
	Inotify dmon;
	int updates = 0;
	if (getenv(BIGMAC_KROOT)){
		kroot = getenv(BIGMAC_KROOT);
	}
	init_knobs();
	KnobGroup gains(GAINS);
	KnobGroup offsets(OFFSETS);

	gains.toValues(::gains);
	offsets.toValues(::offsets);
	exec_mac(src, dst);

	dmon.spawn();

	while(1){
		KnobGroup gains2(GAINS);
		KnobGroup offsets2(OFFSETS);

		if (gains2.isDifferent(gains) || offsets2.isDifferent(offsets)){
			gains.copyValues(gains2);
			offsets.copyValues(offsets2);
			gains.toValues(::gains);
			offsets.toValues(::offsets);
			exec_mac(src, dst);
			update_scratchpad(::offsets, ::gains);
			update_knob(kroot, "update", ++updates);
			dmon.spawn();
		}else{
			dmon.wait();
		}
	}
	return 0;	// doesn't happen ..
}
int main(int argc, const char** argv)
{
	short *src = 0;
	short *dst = 0;

	ui(argc, argv);
	mapRegs(playloop_site);

	if (monitor){
		return run_monitor(src, dst);
	}
}
