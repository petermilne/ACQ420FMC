/* ------------------------------------------------------------------------- */
/* acq400_stream.cpp  		                     	 */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2012 pgm, D-TACQ Solutions Ltd                    *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
 *   Created on: Mar 31, 2013
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

/*
 * acq400_stream.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: pgm
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define PROOT	"/sys/module/acq420FMC/parameters"

static int getKnob(const char* knob, unsigned* value)
{
	FILE *fp = fopen(knob, "r");
	int rc = fscanf(fp, "%u", value);
	fclose(fp);
	return rc;
}

unsigned int nbuffers;
unsigned int bufferlen;
int devnum = 0;

int control_handle;

struct Buffer {
	int fd;
	void *pdata;
	const char* fname;
};

Buffer* buffers;
const char* root;

void init() {
	assert(getKnob(PROOT"/nbuffers", &nbuffers) > 0);
	assert(getKnob(PROOT"/bufferlen", &bufferlen) > 0);

	buffers = new Buffer[nbuffers];

	char *_root = new char [128];
	sprintf(_root, "/dev/acq420.%d", devnum);
	root = _root;

	for (int ii = 0; ii < nbuffers; ++ii){
		char* fname = new char[128];
		sprintf(fname, "%s.hb/%02d", root, ii);
		buffers[ii].fd = open(fname, O_RDONLY);
		assert(buffers[ii].fd > 0);
		buffers[ii].pdata = mmap(0, bufferlen, PROT_READ,
				MAP_SHARED, buffers[ii].fd, 0);
		assert(buffers[ii].pdata != MAP_FAILED);
		buffers[ii].fname = fname;
	}
}

void stream()
{
	char fname[128];
	sprintf(fname, "%s.c", root);
	int fc = open(fname, O_RDONLY);
	assert(fc > 0);

	char buf[80];

	while(read(fc, buf, 80) > 0){
		int ib = atoi(buf);
		assert(ib >= 0);
		assert(ib < nbuffers);
//		fprintf(stderr, "%d %p\n", ib, buffers[ib].pdata);
		write(1, buffers[ib].pdata, bufferlen);
	}
}

int main(int argc, char* argv[])
{
	if (getenv("ACQ400_DEVNUM")){
		devnum = atol(getenv("ACQ400_DEVNUM"));
	}
	if (argc > 1){
		devnum = atoi(argv[1]);
	}
	init();

	if (getenv("ACQ400_WRBUFLEN")){
		bufferlen = atol(getenv("ACQ400_WRBUFLEN"));
	}
	stream();
	return 0;
}
