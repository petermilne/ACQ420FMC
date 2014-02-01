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

#define NCHAN	4

int nchan = NCHAN;

static int getKnob(int idev, const char* knob, unsigned* value)
{
	char kpath[128];
	sprintf(kpath, "/dev/acq400.%d.knobs/%s", idev, knob);
	FILE *fp = fopen(kpath, "r");
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
#define FULL_MASK 0xffffffff
unsigned mask = FULL_MASK;	/** mask data with this value */
int m1 = 0;			/** start masking from here */
int oversampling = 1;
int devnum = 0;
const char* script_runs_on_completion = 0;
int G_nsam = 4096;

int control_handle;

#define BM_NULL	'n'
#define BM_SENDFILE 's'
#define BM_DEMUX 'h'
#define BM_TEST	't'

int buffer_mode;

int verbose;
int nb_cat =1;	/* number of buffers to concatenate */

class Buffer {

protected:
	int fd;
	const char* fname;
	const int ibuf;
	int buffer_len;

public:
	enum BUFFER_OPTS { BO_NONE, BO_START=0x1, BO_FINISH=0x2 };


	virtual int writeBuffer(int out_fd, int b_opts) = 0;

	Buffer(const char* _fname, int _ibuf, int _buffer_len):
		fname(_fname),
		ibuf(_ibuf),
		buffer_len(_buffer_len)
	{
		fd = open(fname, O_RDONLY);
		if (fd < 0){
			perror(fname);
			exit(1);
		}
	}

	Buffer(const char* _fname, int _ibuf, int _buffer_len, bool nofile):
		fname(_fname),
		ibuf(_ibuf),
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
	NullBuffer(const char* _fname, int _ibuf, int _buffer_len):
		Buffer(_fname, _ibuf, _buffer_len)
	{}
	virtual int writeBuffer(int out_fd, int b_opts) {
		return buffer_len;
	}
};
class MapBuffer: public Buffer {
protected:
	void *pdata;

public:
	virtual int writeBuffer(int out_fd, int b_opts) {
		return write(out_fd, pdata, buffer_len);
	}

	MapBuffer(const char* _fname, int _ibuf, int _buffer_len) :
		Buffer(_fname, _ibuf, _buffer_len)
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
	unsigned* mask;
	int nchan;
	int nsam;
	static T** dddata;
	static T** ddcursors;

	bool data_fits_buffer;
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
		for (int ic = 0; ic < nchan; ++ic){
			ddcursors[ic] =	dddata[ic];
		}
	}
	void finish() {
		system(CLEAN_COMMAND);
		rename(OUT_ROOT, OUT_ROOT_OLD);
		rename(OUT_ROOT_NEW, OUT_ROOT);
		system(FIN_COMMAND);
		if (script_runs_on_completion != 0){
			system(script_runs_on_completion);
		}
	}
	static unsigned ch_id(T data)
	{
		return data&0x000000ff;
	}

	void dump(T* src, int nwords){
		char cmd[80];
		sprintf(cmd, "hexdump -ve '%d/%d \"%%08x \" \"\\n\" '",
							nchan, sizeof(T));
		FILE *pp = popen(cmd, "w");
		fwrite(src, sizeof(T), nwords, pp);
		pclose(pp);
	}
	bool demux(bool start) {
		T* src1 = static_cast<T*>(pdata);
		T* src = static_cast<T*>(pdata);
		int isam = 0;

		if (start && !data_fits_buffer){
			/* search for start point - site 1 */
			while (ch_id(src[0]) != 0x20 ||
			       ch_id(src[1]) != 0x21 ||
			       ch_id(src[2]) != 0x22 ||
			       ch_id(src[3]) != 0x23    ){
				if (++src - src1 > nsam/100){
					if (verbose > 1){
						dump(src1, nchan*4);
					}
					fprintf(stderr, "failed to channel align\n");
					return true;
				}
			}
		}
		int startoff = (src - src1)/sizeof(T)/nchan;
		for (isam = startoff; isam < nsam; ++isam){
			for (int ichan = 0; ichan < nchan; ++ichan){
				*ddcursors[ichan]++ = (*src++)&mask[ichan];
			}
		}
		return false;
	}
	bool writeChan(int ic, T* src, int nsam){
		FILE* fp = fopen(fnames[ic], "w");
		if (fp ==0){
			perror(fnames[ic]);
			return true;
		}
		fwrite(dddata[ic], sizeof(T), ddcursors[ic]-dddata[ic], fp);
		fclose(fp);
		return false;
	}
public:
	virtual int writeBuffer(int out_fd, int b_opts) {
		if ((b_opts&BO_START) != 0){
			start();
		}
		demux((b_opts&BO_START));
		if ((b_opts&BO_FINISH) != 0){
			for (int ic = 0; ic < nchan; ++ic){
				if (writeChan(ic, dddata[ic], nsam)){
					// links take out from under
					return -1;
				}
			}
			finish();
		}
		return buffer_len;
	}
	DemuxBuffer(const char* _fname, int _ibuf, int _buffer_len, unsigned _mask) :
		MapBuffer(_fname, _ibuf, _buffer_len),
		nchan(::nchan),
		nsam(_buffer_len/sizeof(T)/nchan)
	{
		mask = new unsigned[nchan];
		for (int ic = 0; ic < nchan; ++ic){
			mask[ic] = ic < m1? FULL_MASK: _mask;
		}
		if (dddata == 0){
			dddata = new T*[nchan];
			ddcursors = new T*[nchan];
			for (int ic = 0; ic < nchan; ++ic){
				ddcursors[ic] =	dddata[ic] = new T[nsam*nb_cat];
			}
		}

		make_names();

		data_fits_buffer = nsam*sizeof(T)*nchan == _buffer_len;
	}
};

template<class T> T** DemuxBuffer<T>::dddata;
template<class T> T** DemuxBuffer<T>::ddcursors;

class TurboBuffer : public Buffer {
private:
	static int fake_fd;
public:
	virtual int writeBuffer(int out_fd, int b_opts) {
		// driver doesn't do sendfile yet .. fake it.
		lseek(fake_fd, 0, SEEK_SET);
		return sendfile(out_fd, fake_fd, 0, buffer_len);
	}
	TurboBuffer(const char* _fname, int _ibuf, int _buffer_len) :
		Buffer(_fname, _ibuf, _buffer_len, true)
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
	OversamplingMapBuffer(const char* _fname, int _ibuf, int _buffer_len,
			int _oversampling, int _asr) :
		MapBuffer(_fname, _ibuf, _buffer_len),
		over(_oversampling),
		asr(_asr),
		nsam(_buffer_len/sizeof(T)/::nchan)
	{
		outbuf = new T[nsam*::nchan/over];
	}
	virtual ~OversamplingMapBuffer() {
		delete [] outbuf;
	}
	virtual int writeBuffer(int out_fd, int b_opts) {
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

#define SCOFF	24		/* sample count offset */
#define EFMT	"ERROR [%02d] %10llu at offset:%d %08x -> %s:%08x\n"

int u32toB(int nu32) {
	return nu32*sizeof(unsigned);
}
class ScratchpadTestBuffer: public MapBuffer {
	static bool report_done;
	const unsigned sample_size;
	const unsigned samples_per_buffer;

	static unsigned sample_count;
	static bool sample_count_valid;
	static unsigned long long samples_elapsed;
	static unsigned shim;
	static unsigned buffer_count;

	int last_sample_offset()
	/* offset in u32 */
	{
		return ((samples_per_buffer-1)*sample_size)/sizeof(unsigned);
	}

	bool isValidQuad(unsigned *pbuf, unsigned startoff){
		return (pbuf[startoff+0]&0x0000001f) == 0x0 &&
		       (pbuf[startoff+1]&0x0000001f) == 0x1 &&
		       (pbuf[startoff+2]&0x0000001f) == 0x2 &&
		       (pbuf[startoff+3]&0x0000001f) == 0x3;
	}
	int checkAlignment(unsigned *src) {
		if (isValidQuad(src, shim)){
			return 0;
		}else{

			for (shim = 0; shim < 32; ++shim){
				if (isValidQuad(src, shim)){
					fprintf(stderr, "ERROR [%02d] misalign shim set %d\n", ibuf, shim);
					exit(1);
					return 1;
				}
			}
			fprintf(stderr, "ERROR buffer %5d failed to realign, try next buffer\n");
			return -1;
		}
	}
	int scoff(int offset = 0)
	/** return offset in u32 of Sample Count, compensated by shim */
	{
		return offset + (shim+SCOFF)%32;
	}
	int byteoff(int sample){
		return sample*sample_size;
	}

	void searchPointOfError(unsigned *src)
	{
		fprintf(stderr, ".. searching exact location\n");
		unsigned sc1 = src[scoff()];

		for (int ii = 0; ii < samples_per_buffer; ++ii, src += ::nchan){
			if (!isValidQuad(src, shim)){
				fprintf(stderr, "Channel Jump at sample %d offset 0x%08x\n",
							ii, byteoff(ii));
				break;
			}else{
				if (ii){
					unsigned sc2 = src[scoff()];
					if (sc2 != sc1+ii){
						fprintf(stderr,
						"Missed samples at sample %d offset 0x%08x %u -> %u\n",
							ii, byteoff(ii), sc1+ii, sc2);
						break;
					}
				}
			}
		}
	}
public:
	ScratchpadTestBuffer(const char* _fname, int _ibuf, int _buffer_len):
		MapBuffer(_fname, _ibuf, _buffer_len),
		sample_size(sizeof(unsigned) * ::nchan),
		samples_per_buffer(_buffer_len/sample_size)
	{
		if (!report_done){
			fprintf(stderr, "%20s : %u\n", "samples_per_buffer", samples_per_buffer);
			fprintf(stderr, "%20s : %u\n", "sample_size", sample_size);
			fprintf(stderr, "%20s : %u\n", "last_sample_offset()", last_sample_offset());
			report_done = true;
		}
	}
	virtual ~ScratchpadTestBuffer() {

	}
	virtual int writeBuffer(int out_fd, int b_opts) {
		++buffer_count;
		unsigned *src = static_cast<unsigned*>(pdata);

		if (checkAlignment(src) < 0){
			return 0;
		}
		unsigned sc1 = src[scoff()];
		unsigned sc2 = src[scoff(last_sample_offset())];

		fprintf(stderr, "[%02d] %d %d  %08x %08x\n", ibuf, sc1, sc2, sc1, sc2);

		if (sample_count_valid){
			if (sc1 != sample_count + 1){
				fprintf(stderr, EFMT, ibuf, samples_elapsed,
						u32toB(SCOFF), sample_count, "sc1", sc1);
			}
		}
		if (sc2 != sc1 + samples_per_buffer-1){
			fprintf(stderr, EFMT, ibuf, samples_elapsed,
					u32toB(last_sample_offset() + SCOFF),
					sc1+samples_per_buffer-2, "sc2", sc2);
			searchPointOfError(src);
			sample_count_valid = false;
			exit(1);
			return 0;
		}else{
			sample_count = sc2;
			sample_count_valid = true;
			samples_elapsed += samples_per_buffer;
			return buffer_len;
		}
	}
};

bool ScratchpadTestBuffer::report_done;
unsigned long long ScratchpadTestBuffer::samples_elapsed;
unsigned ScratchpadTestBuffer::sample_count;
bool ScratchpadTestBuffer::sample_count_valid;
unsigned ScratchpadTestBuffer::shim;
unsigned ScratchpadTestBuffer::buffer_count;

int ASR(int os)
/** calculate Arith Shift Right needed to ensure no overflow for os */
{
	int asr = 0;
	for (; 1<<asr < os; ++asr){
		;
	}
	return asr;
}

Buffer* Buffer::create(const char* root, int _ibuf, int _buffer_len)
{
	char* fname = new char[128];
	sprintf(fname, "%s.hb/%02d", root, _ibuf);

	switch(buffer_mode){
	case BM_NULL:
		return new NullBuffer(fname, _ibuf, _buffer_len);
	case BM_SENDFILE:
		return new TurboBuffer(fname, _ibuf, _buffer_len);
	case BM_DEMUX:
		switch(wordsize){
		case 2:
			return new DemuxBuffer<short>(fname, _ibuf, _buffer_len, mask);
		case 4:
			return new DemuxBuffer<int>(fname, _ibuf, _buffer_len, mask);
		default:
			fprintf(stderr, "ERROR: wordsize must be 2 or 4");
			exit(1);
		}
	case BM_TEST:
		return new ScratchpadTestBuffer(fname, _ibuf, _buffer_len);
	default:
		if (oversampling == 1){
			return new MapBuffer(fname, _ibuf, _buffer_len);
		}else if (wordsize == 2){
			return new OversamplingMapBuffer<short>(
			fname, _ibuf, _buffer_len, oversampling, ASR(oversampling));
		}else{
			return new OversamplingMapBuffer<int> (
			fname, _ibuf, _buffer_len, oversampling, ASR(oversampling));
		}
	}
}

Buffer** buffers;



const char* stream_fmt = "%s.c";

struct poptOption opt_table[] = {
	{ "wrbuflen", 0, POPT_ARG_INT, &bufferlen, 0, 	"reduce buffer len" },
	{ "null-copy", 0, POPT_ARG_NONE, 0, BM_NULL, 	"no output copy"    },
	{ "sendfile",  0, POPT_ARG_NONE, 0, BM_SENDFILE,
			"use sendfile to transmit (fake)"		    },
	{ "verbose",   0, POPT_ARG_INT, &verbose, 0,  "set verbosity"	    },
	{ "hb0",       0, POPT_ARG_NONE, 0, BM_DEMUX },
	{ "test-scratchpad", 0, POPT_ARG_NONE, 0, BM_TEST,
			"minimal overhead data test buffer top/tail for sample count"},
	{ "nchan",    'N', POPT_ARG_INT, &::nchan, 0 },
	{ "wordsize", 'w', POPT_ARG_INT, &wordsize, 0,
			"data word size 2|4" },
	{ "oversampling", 'O', POPT_ARG_INT, &oversampling, 0,
			"set oversampling"},
	{ "mask",      'M', POPT_ARG_INT, &mask, 0,
			"set data mask"},
	{ "m1",        'm', POPT_ARG_INT, &m1, 0,
			"index of first masked channel"},
	{ "nsam",       0,  POPT_ARG_INT, &G_nsam, 0,
			"desired output samples" },
	{ "oncompletion", 0, POPT_ARG_STRING, &script_runs_on_completion, 0,
			"run this script on completion of buffer"
	},
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



	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;
	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
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
		case BM_TEST:
			buffer_mode = BM_TEST;
			break;
		}
	}
	const char* devc = poptGetArg(opt_context);
	if (devc){
		devnum = atoi(devc);
	}



	getKnob(devnum, "nbuffers",  &nbuffers);
	getKnob(devnum, "bufferlen", &bufferlen);

	if (buffer_mode == BM_DEMUX){
		for (nb_cat = 1;
		     nb_cat*bufferlen/(nchan*wordsize) < G_nsam; ++nb_cat){
			;
		}
		if (verbose){
			fprintf(stderr, "BM_DEMUX bufsam:%d\n",
					bufferlen/(nchan*wordsize));
			fprintf(stderr, "BM_DEMUX nb_cat set %d\n", nb_cat);
		}
	}
	buffers = new Buffer* [nbuffers];

	root = getRoot(devnum);

	for (int ii = 0; ii < nbuffers; ++ii){
		buffers[ii] = Buffer::create(root, ii, bufferlen);
	}
}

class StreamHead {
protected:
	virtual void onBuffer(int ib) {
		buffers[ib]->writeBuffer(1, Buffer::BO_NONE);
	}
public:
	void stream() {
		char fname[128];
		sprintf(fname, stream_fmt, root);
		int fc = open(fname, O_RDONLY);
		assert(fc > 0);

		char buf[80];
		int rc;
		int icat = 0;

		while((rc = read(fc, buf, 80)) > 0){
			buf[rc] = '\0';
			int ib = atoi(buf);
			assert(ib >= 0);
			assert(ib <= nbuffers);

			onBuffer(ib);
		}
	}
	static StreamHead& instance();
};

class StreamHeadHB0: public StreamHead  {
protected:
	virtual void onBuffer(int ib) {
		for (int icat = 0; icat < nb_cat; ++icat){
			int buffer_opts = Buffer::BO_NONE;
			if (icat == 0){
				buffer_opts |= Buffer::BO_START;
			}
			if (icat+1 >= nb_cat){
				buffer_opts |= Buffer::BO_FINISH;
			}
			if (verbose){
				fprintf(stderr, "buffer:%d/%d opts:%d\n",
						ib, nb_cat, buffer_opts);
			}
			buffers[ib+icat]->writeBuffer(1, buffer_opts);
		}
	}
};

StreamHead& StreamHead::instance() {
	static StreamHead* _instance =
		buffer_mode==BM_DEMUX? new StreamHeadHB0(): new StreamHead();
	return *_instance;
}


int main(int argc, const char** argv)
{
	init(argc, argv);
	StreamHead::instance().stream();
	return 0;
}
