/* ------------------------------------------------------------------------- */
/* clocks_to_first_edge.cpp  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 9 Mar 2017  			/ User: pgm
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2017 Peter Milne, D-TACQ Solutions Ltd         *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                *
 *
 * TODO 
 * TODO
 * ------------------------------------------------------------------------- */


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#define NBITS	32

#define DFMT "/dev/acq400/data/%d/01"

#define FINISHED 0xffffffff

namespace G {
	FILE *fp_in;
	unsigned skip;
	unsigned finished = FINISHED;
	unsigned timeout = 0;
};


#define PAGEW (4096/sizeof(unsigned))

int readw(unsigned *xx)
{
	static unsigned buf[PAGEW];
	static int cursor;
	static int len;

	if (cursor >= len){
		len = fread(buf, sizeof(xx), PAGEW, G::fp_in);
		if (len <= 0){
			perror("input terminated");
			exit(1);
		}
		cursor = 0;
	}
	*xx = buf[cursor++];
	return 1;
}

unsigned long long clocks[NBITS] = {};

void print_clocks(void) {
	for (int ib = 0; ib < NBITS; ++ib){
		printf("%llu%c", clocks[ib], ib+1 == NBITS? '\n': ',');
	}
}


unsigned fin = 0;
unsigned timed_out;

void alarm_handler(int sig)
{
	timed_out = 1;
	fin = FINISHED;
}
void count_clocks(void)
{
	unsigned x0 = 0;
	unsigned x1 = 0;

	if (readw(&x0) != 1){
		perror("failed to read first word");
		exit(1);
	}

	unsigned clk = 1;

	for (; clk < G::skip; ++clk){
		if (readw(&x0) != 1){
			perror("run out of road with skip");
			exit(1);
		}
	}
	for (; !timed_out && fin != FINISHED && readw(&x1) == 1; ++clk, x0 = x1){
		unsigned change = (x1 ^ x0);
		if (change & ~fin){
			for (unsigned ib = 0; ib < NBITS; ++ib){
				unsigned bit = 1<<ib;
				if ((fin&bit) == 0 && (change&bit) != 0){
					clocks[ib] = clk;
					fin |= bit;
				}
			}
		}
	}
	fclose(G::fp_in);

}

unsigned long long clk;
void onChange(unsigned x0, unsigned x1)
{
	unsigned change = (x1 ^ x0);
	if (change & ~fin){
		for (unsigned ib = 0; ib < NBITS; ++ib){
			unsigned bit = 1<<ib;
			if ((fin&bit) == 0 && (change&bit) != 0){
				clocks[ib] = clk;
				fin |= bit;
			}
		}
	}
}

void count_clocks_live() {
	unsigned x0;
	unsigned x1;

	if (readw(&x0) != 1){
		perror("failed to read first word");
		exit(1);
	}

	if (G::timeout){
		signal(SIGALRM, alarm_handler);
		alarm(G::timeout);
	}

	for (clk = 1; fin != FINISHED; ++clk){
		readw(&x1);
		if (x1 != x0){
			onChange(x0, x1);
		}else{
			x0 = x1;
		}
	}
}

int ui(int argc, const char** argv)
{
	int site = 1;
	if (getenv("SITE") != 0){
		site = atoi(getenv("SITE"));
	}
	if (getenv("SKIP") != 0){
		G::skip = atoi(getenv("SKIP"));
	}
	if (getenv("FINISHED")){
		G::finished = strtoul(getenv("FINISHED"), 0, 16);
	}
	if (getenv("TIMEOUT")){
		G::timeout = atoi(getenv("TIMEOUT"));
	}
	if (argc > 0 && argv[1][0] == '-'){
		G::fp_in = stdin;
		return 1;
	}
	char fname[80];
	snprintf(fname, 80, DFMT, site);
	G::fp_in = fopen(fname, "r");
	if (!G::fp_in){
		perror(fname);
		exit(1);
	}
	return 0;
}

int main(int argc, const char** argv)
{
	if (ui(argc, argv)){
		count_clocks_live();
	}else{
		count_clocks();
	}
	print_clocks();
	return 0;
}

