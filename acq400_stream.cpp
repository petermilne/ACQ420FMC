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
#include <sys/sendfile.h>
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
int use_sendfile;

struct Buffer {
	int fd;
	void *pdata;
	const char* fname;
	int buffer_len;

	virtual int writeBuffer(int out_fd) {
		return write(out_fd, pdata, buffer_len);
	}

	Buffer(const char* _fname, int _buffer_len, bool mapme = true) :
		fname(_fname),
		buffer_len(_buffer_len)
	{
		fd = open(fname, O_RDONLY);
		assert(fd > 0);

		if (mapme){
			pdata = mmap(0, bufferlen, PROT_READ, MAP_SHARED, fd, 0);
			assert(pdata != MAP_FAILED);
		} else {
			pdata = 0;
		}
	}

	static Buffer* create(const char* root, int ibuf, int _buffer_len);
};

struct TurboBuffer : public Buffer {
	virtual int writeBuffer(int out_fd) {
		return sendfile(out_fd, fd, 0, buffer_len);
	}
	TurboBuffer(const char* _fname, int _buffer_len) :
		Buffer(_fname, _buffer_len, false)
	{}
};


Buffer* Buffer::create(const char* root, int ibuf, int _buffer_len)
{
	char* fname = new char[128];
	sprintf(fname, "%s.hb/%02d", root, ibuf);

	if (use_sendfile){
		return new TurboBuffer(fname, _buffer_len);
	} else {
		return new Buffer(fname, _buffer_len);
	}
}

Buffer** buffers;
const char* root;

void init() {
	assert(getKnob(PROOT"/nbuffers", &nbuffers) > 0);
	assert(getKnob(PROOT"/bufferlen", &bufferlen) > 0);

	if (getenv("ACQ400_WRBUFLEN")){
		bufferlen = atol(getenv("ACQ400_WRBUFLEN"));
	}

	buffers = new Buffer* [nbuffers];

	char *_root = new char [128];
	sprintf(_root, "/dev/acq420.%d", devnum);
	root = _root;

	for (int ii = 0; ii < nbuffers; ++ii){
		buffers[ii] = Buffer::create(root, ii, bufferlen);
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
		buffers[ib]->writeBuffer(1);
	}
}

int main(int argc, char* argv[])
{
	if (getenv("ACQ400_DEVNUM")){
		devnum = atol(getenv("ACQ400_DEVNUM"));
	}
	if (getenv("ACQ400_SENDFILE")){
		use_sendfile = atol(getenv("ACQ400_SENDFILE"));
	}
	if (argc > 1){
		devnum = atoi(argv[1]);
	}
	init();
	stream();
	return 0;
}
