/* ------------------------------------------------------------------------- *
 * bigmac.cpp  		                     	                    
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

#define MACSCALE	10		/** >>10 : 1024=100% */

int bufferlen = 0x10000;
const char *test = "null-test";
char *outfile;
char *infile;

/* for testing: use constant gains, offsets. @@todo These need to be set from the UI */
short gains[NCHAN] = { 5, 6, 7, 8 };
short offsets[NCHAN] = { 100, 200, 300, 400 };


char* gainslist;
char* offsetslist;

int mmap_in;
int mmap_out;

struct poptOption opt_table[] = {
	{ "buflen", 'L', POPT_ARG_INT, &bufferlen, 0, 	"buffer length SAMPLES" },
	{ "test",   'T', POPT_ARG_STRING, &test, 0, "test mode" },
	{ "out",    'o', POPT_ARG_STRING, &outfile, 0, "output to file" },
	{ "in",     'i', POPT_ARG_STRING, &infile, 0, "input from file" },
	{ "gains",   'G', POPT_ARG_STRING, &gainslist, 'G', "gains"},
	{ "dc-offsets", 'D', POPT_ARG_STRING, &offsetslist, 'D', "DC offsets"},
	{ "mmap-in", 'I', POPT_ARG_STRING, &infile, 'I', "use mmap to access infile"},
	{ "mmap-out", 'O', POPT_ARG_STRING, &outfile, 'O', "use mmap to access outfile"},
	POPT_AUTOHELP
	POPT_TABLEEND
};

void cmac(short *dst, const short* src, const int nsam, const int nchan,
		const short* gains, const short* offsets)
{
	int tt, cc;
	short offs[NCHAN];
	for (cc = 0; cc < nchan; ++cc){
		offs[cc] = offsets[cc] << MACSCALE;
	}

	for (tt = 0; tt < nsam; ++tt, dst += nchan, src += nchan){
		for (cc = 0; cc < nchan; ++cc){
			int xx = src[cc];
			xx *= gains[cc] + offs[cc];
			dst[cc] = xx >> MACSCALE;
		}
	}
}

#ifdef HASNEON
#include <arm_neon.h>
void nmac(short * dst, const short * src, const int nsam, const int nchan,
		const short* gains, const short* offsets)
/* replace inner loop with a 4-element vector math call */
{
	int tt;
	int16x4_t GAIN = vld1_s16(gains);
	int16x4_t OFFSET = vld1_s16(offsets);

	for (tt = 0; tt < nsam; ++tt, dst += nchan, src +=nchan){
		int16x4_t SRC = vld1_s16(src);
		int16x4_t DST = vmla_s16(OFFSET, GAIN, SRC);
		vst1_s16(dst, DST);
	}
}

void nmac2(short * dst, const short * src, const int nsam, const int nchan,
		const short* gains, const short* offsets)
/* replace inner loop with a 8-element vector math call */
{
	int tt;
	/* unfortunately we need 8-element gain, offset vectors */
	short *g2 = new short[NCHAN*2];
	short *o2 = new short[NCHAN*2];
	for (int ii = 0; ii < NCHAN; ++ii){
		g2[ii+NCHAN] = g2[ii] = gains[ii];
		o2[ii+NCHAN] = o2[ii] = offsets[ii];
	}
	int16x8_t GAIN = vld1q_s16(g2);
	int16x8_t OFFSET = vld1q_s16(o2);

	for (tt = 0; tt < nsam; tt += 2, dst += nchan*2, src +=nchan*2){
		int16x8_t SRC = vld1q_s16(src);
		int16x8_t DST = vmlaq_s16(OFFSET, GAIN, SRC);
		vst1q_s16(dst, DST);
	}
}
#endif


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


void getBuffers(short **psrc, short **pdst)
{
	short *src = 0;
	short *dst = 0;

	int maplen = bufferlen*NCHAN*sizeof(short);

	if (infile){
		FILE* fp = fopen(infile, "r");
		if (fp == 0){
			perror(infile);
			exit(1);
		}
		if (mmap_in){
			void* region =
			 mmap(NULL, maplen, PROT_READ, MAP_SHARED, fileno(fp),0);

			if ( region == (caddr_t)-1 ){
				perror( "mmap infile" );
				exit(-1);
			}else{
				src = (short*)region;
			}
		}else{
			src = new short[bufferlen*NCHAN];
			int nread = fread(src, sizeof(short), bufferlen*NCHAN, fp);
			if (nread != bufferlen*NCHAN){
				fprintf(stderr, "WARNING: short read :d\n", nread);
			}
			fclose(fp);
		}
	}else{
		memset(src, 0, sizeof(short)*bufferlen*NCHAN);
	}

	if (outfile && mmap_out){
		FILE* fp = fopen(outfile, "r+");
		if (fp == 0){
			perror(infile);
			exit(1);
		}
		void* region =
			 mmap(NULL, maplen, PROT_WRITE, MAP_SHARED, fileno(fp),0);

		if ( region == (caddr_t)-1 ){
			perror( "mmap outfile" );
			exit(-1);
		}else{
			dst = (short*)region;
		}
	}else{
		dst = new short[bufferlen*NCHAN];
	}

	*psrc = src;
	*pdst = dst;
}

void putBuffer(short *dst)
{
	if (outfile && !mmap_out){
		FILE *fp = fopen(outfile, "w");
		if (!fp){
			perror(outfile);
			exit(1);
		}
		fwrite(dst, sizeof(short), bufferlen*NCHAN, fp);
		fclose(fp);
	}
}


int main(int argc, const char** argv)
{
	ui(argc, argv);


	short *src;
	short *dst;

	getBuffers(&src, &dst);


	if (strcmp(test, "memcpy") == 0){
		memcpy(dst, src, bufferlen*NCHAN*sizeof(short));
	}
	if (strcmp(test, "cmac") == 0){
		cmac(dst, src, bufferlen, NCHAN, gains, offsets);
	}
#ifdef HASNEON
	if (strcmp(test, "nmac") == 0){
		nmac(dst, src, bufferlen, NCHAN, gains, offsets);
	}
	if (strcmp(test, "nmac2") == 0){
		nmac2(dst, src, bufferlen, NCHAN, gains, offsets);
	}
#endif

	putBuffer(dst);
	return 0;
}
