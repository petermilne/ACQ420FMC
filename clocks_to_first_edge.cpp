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

#define NBITS	32

#define DFMT "/dev/acq400/data/%d/01"

#define FINISHED 0xffffffff

namespace G {
	FILE *fp_in;
	unsigned skip;
};

int readw(unsigned *xx)
{
	return fread(xx, sizeof(xx), 1, G::fp_in);
}

void count_clocks(void)
{
	unsigned x0 = 0;
	unsigned x1 = 0;
	unsigned fin = 0;
	unsigned clocks[NBITS] = {};

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
	for (; fin != FINISHED && readw(&x1) == 1; ++clk){
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
	for (int ib = 0; ib < NBITS; ++ib){
		printf("%d%c", clocks[ib], ib+1 == NBITS? '\n': ',');
	}
}
void ui(void)
{
	int site = 1;
	if (getenv("SITE") != 0){
		site = atoi(getenv("SITE"));
	}
	if (getenv("SKIP") != 0){
		G::skip = atoi(getenv("SKIP"));
	}
	char fname[80];
	snprintf(fname, 80, DFMT, site);
	G::fp_in = fopen(fname, "r");
	if (!G::fp_in){
		perror(fname);
		exit(1);
	}
}

int main(int argc, char* argv[])
{
	ui();
	count_clocks();
}

