/* ------------------------------------------------------------------------- *
 * permute.c  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2013 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 4 Nov 2013  
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

/**
 usage: permute < src-file >dst-file MAPPING
 permutes a set of shorts according to mapping
 MAPPING: logical channels 1..N, space separated.
 eg
 permute 4 3 2 1   # reverse order
 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>



int permute(unsigned nchan, char** _mapping){
	unsigned *mapping = new unsigned[nchan];

	for (unsigned ic = 0; ic < nchan; ++ic){
		mapping[ic] = atoi(_mapping[ic]) - 1;
		assert(mapping[ic] >= 0 && mapping[ic] < nchan);
	}

	short *src = new short[nchan];
	short *dst = new short[nchan];

	while(fread(src, sizeof(short), nchan, stdin) == nchan){
		for (unsigned ic = 0; ic < nchan; ++ic){
			dst[ic] = src[mapping[ic]];
		}
		fwrite(dst, sizeof(short), nchan, stdout);
	}
	return 0;
}
int main(int argc, char* argv[])
{
	int nchan = argc - 1;

	if (nchan == 0){
		short sample;
		while(fread(&sample, sizeof(short), 1, stdin) == 1){
			fwrite(&sample, sizeof(short), 1, stdout);
		}
		return 0;
	}else{
		return permute(nchan, argv+1);
	}
}

