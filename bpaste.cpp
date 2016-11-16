
/* bpaste.cpp  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 16 Nov 2016  			/ User: pgm
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2016 Peter Milne, D-TACQ Solutions Ltd         *
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

#define MAXBUF	0x10000

int bpaste(FILE *fp[])
{
	short *s1 = new short[MAXBUF];
	short *s2 = new short[MAXBUF];
	unsigned long *ll = new unsigned long[MAXBUF];

	int n1; int n2;

	while((n1 = fread(s1, sizeof(short), MAXBUF, fp[0])) &&
	      (n2 = fread(s2, sizeof(short), MAXBUF, fp[1]))    ){
		int nx = n1;
		if (n1 != n2){
			fprintf(stderr, "WARNING: unequal lengths");
			if (n1 < n2){
				nx = n1;
			}else{
				nx = n2;
			}
		}
		for (int ii = 0; ii != nx; ++ii){
			ll[ii] = s1[ii] + (s2[ii]<<16);
		}
		fwrite(ll, sizeof(long), nx, stdout);
		if (n1 != n2){
			return -1;
		}
	}
	return 0;
}

int main(int argc, char* argv[]) {
	if (argc != 3){
		fprintf(stderr, "USAGE: bpaste a b\n");
		return -1;
	}
	FILE **fp = new FILE*[2];
	for (int ii = 1; ii < argc; ++ii){
		fp[ii-1] = fopen(argv[ii], "r");
		if (fp[ii-1] == 0){
			fprintf(stderr, "ERROR: failed to open %s\n", argv[ii]);
		}
	}

	return bpaste(fp);
}

