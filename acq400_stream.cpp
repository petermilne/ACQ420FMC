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
#include "popt.h"

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

#define BM_NULL	'n'
#define BM_SENDFILE 's'
int buffer_mode;

int verbose;

class Buffer {

protected:
	int fd;
	const char* fname;
	int buffer_len;

public:
	virtual int writeBuffer(int out_fd) = 0;

	Buffer(const char* _fname, int _buffer_len):
		fname(_fname),
		buffer_len(_buffer_len)
	{
		fd = open(fname, O_RDONLY);
		assert(fd > 0);
	}
	static Buffer* create(const char* root, int ibuf, int _buffer_len);
};

class NullBuffer: public Buffer {
public:
	NullBuffer(const char* _fname, int _buffer_len):
		Buffer(_fname, _buffer_len)
	{}
	virtual int writeBuffer(int out_fd) {
		return buffer_len;
	}
};
class MapBuffer: public Buffer {
private:
	void *pdata;

public:
	virtual int writeBuffer(int out_fd) {
		return write(out_fd, pdata, buffer_len);
	}

	MapBuffer(const char* _fname, int _buffer_len) :
		Buffer(_fname, _buffer_len)
	{
		pdata = mmap(0, bufferlen, PROT_READ, MAP_SHARED, fd, 0);
		assert(pdata != MAP_FAILED);
	}
};

class TurboBuffer : public Buffer {
private:
	static int fake_fd;
public:
	virtual int writeBuffer(int out_fd) {
		// driver doesn't do sendfile yet .. fake it.
		lseek(fake_fd, 0, SEEK_SET);
		return sendfile(out_fd, fake_fd, 0, buffer_len);
	}
	TurboBuffer(const char* _fname, int _buffer_len) :
		Buffer(_fname, _buffer_len)
	{
		if (!fake_fd){
			FILE* fp = fopen("/tmp/fakeit", "w");
			assert(fp != 0);

			int imax = bufferlen/sizeof(unsigned);

			for (int cursor = 0; cursor < imax; ++cursor){
				fwrite(&cursor, sizeof(cursor), 1, fp);
			}
			fclose(fp);
			fake_fd = open("/tmp/fakeit", O_RDONLY);
			assert(fake_fd >= 0);
		}
	}
};

int TurboBuffer::fake_fd;


Buffer* Buffer::create(const char* root, int ibuf, int _buffer_len)
{
	char* fname = new char[128];
	sprintf(fname, "%s.hb/%02d", root, ibuf);

	switch(buffer_mode){
	case BM_NULL:
		return new NullBuffer(fname, _buffer_len);
	case BM_SENDFILE:
		return new TurboBuffer(fname, _buffer_len);
	default:
		return new MapBuffer(fname, _buffer_len);
	}
}

Buffer** buffers;
const char* root;

struct poptOption opt_table[] = {
	{ "wrbuflen", 0, POPT_ARG_INT, &bufferlen, 0, 	"reduce buffer len"	},
	{ "null-copy", 0, POPT_ARG_NONE, 0, 'n', 	"no output copy"	},
	{ "sendfile",  0, POPT_ARG_NONE, 0, 's',
			"use sendfile to transmit (fake)"			},
	{ "verbose",   0, POPT_ARG_INT, &verbose, 0,  "set verbosity"		},
	POPT_AUTOHELP
	POPT_TABLEEND
};

void init(int argc, const char** argv) {
	assert(getKnob(PROOT"/nbuffers", &nbuffers) > 0);
	assert(getKnob(PROOT"/bufferlen", &bufferlen) > 0);

	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;
	while ( (rc = poptGetNextOpt( opt_context )) > 0 ){
		switch(rc){
		case 'n':
			buffer_mode = BM_NULL;
			break;
		case 's':
			buffer_mode = BM_SENDFILE;
			break;
		}
	}
	const char* devc = poptGetArg(opt_context);
	if (devc){
		devnum = atoi(devc);
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
	int rc;

	while((rc = read(fc, buf, 80)) > 0){
		buf[rc] = '\0';
		int ib = atoi(buf);
		assert(ib >= 0);
		assert(ib < nbuffers);
		if (verbose){
			fprintf(stderr, "%s", buf);
		}
		buffers[ib]->writeBuffer(1);
	}
}

int main(int argc, const char** argv)
{
	init(argc, argv);
	stream();
	return 0;
}
