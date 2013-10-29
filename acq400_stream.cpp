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
#include <string.h>
#include "popt.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#define PROOT	"/sys/module/acq420fmc/parameters"

#define NCHAN	4

int nchan = NCHAN;

static int getKnob(const char* knob, unsigned* value)
{
	FILE *fp = fopen(knob, "r");
	if (fp){
		int rc = fscanf(fp, "%u", value);
		fclose(fp);
		return rc;
	} else {
		return -1;
	}
}

unsigned int nbuffers = 16;
unsigned int bufferlen = 0x40000;
int wordsize = 2;		/** choice sizeof(short) or sizeof(int) */
unsigned mask = 0xffffffff;	/** mask data with this value */
int oversampling = 1;
int devnum = 0;

int control_handle;

#define BM_NULL	'n'
#define BM_SENDFILE 's'
#define BM_DEMUX 'd'
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

	Buffer(const char* _fname, int _buffer_len, bool nofile):
		fname(_fname),
		buffer_len(_buffer_len)
	{
		if (!nofile){
			fd = open(fname, O_RDONLY);
			assert(fd > 0);
		}
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
protected:
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

#define FMT_OUT_ROOT		"/dev/shm/AI.%d.wf"
#define FMT_OUT_ROOT_NEW	"/dev/shm/AI.%d.new"
#define FMT_OUT_ROOT_OLD	"/dev/shm/AI.%d.old"



template <class T>
class DemuxBuffer: public MapBuffer {
private:
	const unsigned mask;
	int nchan;
	int nsam;
	T* ddata;
	char** fnames;
	char OUT_ROOT[128];
	char OUT_ROOT_NEW[128];
	char OUT_ROOT_OLD[128];
	char CLEAN_COMMAND[128];
	char FIN_COMMAND[128];

	void make_names() {
		sprintf(OUT_ROOT, FMT_OUT_ROOT, devnum);
		sprintf(OUT_ROOT_NEW, FMT_OUT_ROOT_NEW, devnum);
		sprintf(OUT_ROOT_OLD, FMT_OUT_ROOT_OLD, devnum);
		sprintf(CLEAN_COMMAND, "rm -Rf %s", OUT_ROOT_OLD);
		sprintf(FIN_COMMAND,   "date >%s.fin", OUT_ROOT);

		fnames = new char* [nchan];
		char buf[132];

		for (int ic = 0; ic < nchan; ++ic){
			int len = sprintf(buf, "%s/CH%02d", OUT_ROOT_NEW, ic+1);
			fnames[ic] = new char[len+1];
			strcpy(fnames[ic], buf);
		}
	}
	void start() {
		mkdir(OUT_ROOT_NEW, 0777);
	}
	void finish() {
		system(CLEAN_COMMAND);
		rename(OUT_ROOT, OUT_ROOT_OLD);
		rename(OUT_ROOT_NEW, OUT_ROOT);
		system(FIN_COMMAND);
	}
	void demux() {
		T* src = static_cast<T*>(pdata);

		for (int isam = 0; isam < nsam; ++isam){
			for (int ichan = 0; ichan < nchan; ++ichan){
				ddata[ichan*nsam + isam] = (*src++)&mask;
			}
		}
	}
	void writeChan(int ic, T* src, int nsam){
		FILE* fp = fopen(fnames[ic], "w");
		assert(fp);
		fwrite(src, sizeof(T), nsam, fp);
		fclose(fp);
	}
public:
	virtual int writeBuffer(int out_fd) {
		start();
		demux();
		for (int ic = 0; ic < nchan; ++ic){
			writeChan(ic, &ddata[ic*nsam], nsam);
		}
		finish();
		return buffer_len;
	}
	DemuxBuffer(const char* _fname, int _buffer_len, unsigned _mask) :
		MapBuffer(_fname, _buffer_len),
		mask(_mask),
		nchan(::nchan),
		nsam(_buffer_len/sizeof(short)/nchan)
	{
		ddata = new T[nsam*nchan];
		make_names();
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
		Buffer(_fname, _buffer_len, true)
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


template <class T>
class OversamplingMapBuffer: public MapBuffer {
	const int over, asr;
	const int nsam;
	T *outbuf;
public:
	OversamplingMapBuffer(const char* _fname, int _buffer_len,
			int _oversampling, int _asr) :
		MapBuffer(_fname, _buffer_len),
		over(_oversampling),
		asr(_asr),
		nsam(_buffer_len/sizeof(T)/::nchan)
	{
		outbuf = new T[nsam*::nchan/over];
	}
	virtual ~OversamplingMapBuffer() {
		delete [] outbuf;
	}
	virtual int writeBuffer(int out_fd) {
		T* src = static_cast<T*>(pdata);
		int sums[::nchan];
		int nsum = 0;
		int osam = 0;

		for (int ic = 0; ic < ::nchan; ++ic){
			sums[ic] = 0;
		}

		for (int isam = 0; isam < nsam; ++isam){
			for (int ic = 0; ic < ::nchan; ++ic){
				sums[ic] += src[isam*::nchan+ic];
			}
			if (++nsum >= over){
				for (int ic = 0; ic < ::nchan; ++ic){
					outbuf[osam*::nchan+ic] = sums[ic] >> asr;
					sums[ic] = 0;
				}
				++osam;
				nsum = 0;
			}
		}

		return write(out_fd, outbuf, osam*::nchan*sizeof(T));
	}
};

int ASR(int os)
/** calculate Arith Shift Right needed to ensure no overflow for os */
{
	int asr = 0;
	for (; 1<<asr < os; ++asr){
		;
	}
	return asr;
}

Buffer* Buffer::create(const char* root, int ibuf, int _buffer_len)
{
	char* fname = new char[128];
	sprintf(fname, "%s.hb/%02d", root, ibuf);

	switch(buffer_mode){
	case BM_NULL:
		return new NullBuffer(fname, _buffer_len);
	case BM_SENDFILE:
		return new TurboBuffer(fname, _buffer_len);
	case BM_DEMUX:
		switch(wordsize){
		case 2:
			return new DemuxBuffer<short>(fname, _buffer_len, mask);
		case 4:
			return new DemuxBuffer<int>(fname, _buffer_len, mask);
		default:
			fprintf(stderr, "ERROR: wordsize must be 2 or 4");
			exit(1);
		}
	default:
		if (oversampling == 1){
			return new MapBuffer(fname, _buffer_len);
		}else if (wordsize == 2){
			return new OversamplingMapBuffer<short>(
			fname, _buffer_len, oversampling, ASR(oversampling));
		}else{
			return new OversamplingMapBuffer<int> (
			fname, _buffer_len, oversampling, ASR(oversampling));
		}
	}
}

Buffer** buffers;

const char* stream_fmt = "%s.c";

struct poptOption opt_table[] = {
	{ "wrbuflen", 0, POPT_ARG_INT, &bufferlen, 0, 	"reduce buffer len" },
	{ "null-copy", 0, POPT_ARG_NONE, 0, 'n', 	"no output copy"    },
	{ "sendfile",  0, POPT_ARG_NONE, 0, 's',
			"use sendfile to transmit (fake)"		    },
	{ "verbose",   0, POPT_ARG_INT, &verbose, 0,  "set verbosity"	    },
	{ "hb0",       0, POPT_ARG_NONE, 0, 'h' },
	{ "nchan",    'N', POPT_ARG_INT, &::nchan, 0 },
	{ "wordsize", 'w', POPT_ARG_INT, &wordsize, 0, "data word size 2|4" },
	{ "oversampling", 'O', POPT_ARG_INT, &oversampling, 0, "set oversampling"},
	{ "mask",      'M', POPT_ARG_INT, &mask, 0, "set data mask"},
	POPT_AUTOHELP
	POPT_TABLEEND
};


char *getRoot(int devnum)
{
	char *_root = new char [128];
	struct stat sb;

	sprintf(_root, "/dev/acq420.%d", devnum);
	if (stat(_root, &sb) == 0){
		return _root;
	}

	sprintf(_root, "/dev/acq400.%d", devnum);
	if (stat(_root, &sb) == 0){
		return _root;
	}

	fprintf(stderr, "ERROR: /dev/acq4x0.%d NOT FOUND\n", devnum);
	exit(1);
}
const char* root;
void init(int argc, const char** argv) {


	getKnob(PROOT"/nbuffers", &nbuffers);
	getKnob(PROOT"/bufferlen", &bufferlen);

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
		case 'h':
			stream_fmt = "%s.hb0";
			buffer_mode = BM_DEMUX;
			break;
		}
	}
	const char* devc = poptGetArg(opt_context);
	if (devc){
		devnum = atoi(devc);
	}
	buffers = new Buffer* [nbuffers];

	root = getRoot(devnum);


	for (int ii = 0; ii < nbuffers; ++ii){
		buffers[ii] = Buffer::create(root, ii, bufferlen);
	}
}

void stream()
{
	char fname[128];
	sprintf(fname, stream_fmt, root);
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
