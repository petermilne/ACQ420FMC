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
#include <vector>

#include <libgen.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <errno.h>
#define NCHAN	4


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

/* all globals in one namespace : G */
namespace G {
int nchan = NCHAN;
unsigned int nbuffers = 16;
unsigned int bufferlen = 0x40000;
int wordsize = 2;		/** choice sizeof(short) or sizeof(int) */
#define FULL_MASK 0xffffffff
unsigned mask = FULL_MASK;	/** mask data with this value */
int m1 = 0;			/** start masking from here */
int oversampling = 1;
int devnum = 0;
const char* script_runs_on_completion = 0;
const char* aggregator_sites = 0;
int nsam = 4096;

int control_handle;

int pre;
int post;
bool no_demux;

#define BM_NOT_SPECIFIED	'\0'
#define BM_NULL			'n'
#define BM_SENDFILE 		's'
#define BM_DEMUX 		'h'
#define BM_TEST			't'
#define BM_PREPOST 		'P'

#define SM_STREAM    		'S'
#define SM_TRANSIENT 		'T'

int buffer_mode = BM_NOT_SPECIFIED;
int stream_mode = SM_STREAM;
};



int verbose;
int nb_cat =1;	/* number of buffers to concatenate */


int sample_size() {
	return G::nchan * G::wordsize;
}

class Buffer {

protected:
	int fd;
	const char* fname;
	const int ibuf;
	int buffer_len;

public:
	enum BUFFER_OPTS { BO_NONE, BO_START=0x1, BO_FINISH=0x2 };

	int getLen() { return buffer_len; }

	virtual unsigned getItem(int ii) { return 0; }
	virtual unsigned getSizeofItem() { return 1; }

	virtual int writeBuffer(int out_fd, int b_opts) = 0;
	virtual int copyBuffer(void* dest) { return -1; }

	Buffer(const char* _fname, int _ibuf, int _buffer_len, bool writeable = false):
		fname(_fname),
		ibuf(_ibuf),
		buffer_len(_buffer_len)
	{
		if (writeable){
			fd = open(fname, O_RDWR|O_CREAT, 0777);
		}else{
			fd = open(fname, O_RDONLY);
		}
		if (fd < 0){
			perror(fname);
			exit(1);
		}
	}
	virtual ~Buffer() {
		close(fd);
	}
/*
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
*/
	static Buffer* create(const char* root, int ibuf, int _buffer_len);

	const char* getName() {
		return fname;
	}
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
	virtual int copyBuffer(void* dest) {
		memcpy(dest, pdata, buffer_len);
		return buffer_len;
	}

	void* getBase() { return pdata; }

	int includes(void *cursor)
	/** returns remain length if buffer includes cursor */
	{

		char* cc = static_cast<char *>(cursor);
		char* bp = static_cast<char *>(pdata);
		char *be = bp + buffer_len;
		int remain = 0;
		if (cc >= bp && cc <= be){
			remain =  buffer_len - (cc-bp);
		}

		if (verbose){
			printf("%s %s ret %d\n", __func__, fname, remain);
		}
		return remain;
	}

	MapBuffer(const char* _fname, int _ibuf, int _buffer_len,
			bool writeable = false, void* addr_hint = 0) :
		Buffer(_fname, _ibuf, _buffer_len, writeable)
	{
		if (writeable){
			char* buffer = new char[buffer_len];
			memset(buffer, 0, buffer_len);
			write(fd, buffer, buffer_len);
		}
		pdata = mmap(addr_hint, G::bufferlen,
			PROT_READ|(writeable? PROT_WRITE:0), MAP_SHARED, fd, 0);
		assert(pdata != MAP_FAILED);
	}
	virtual ~MapBuffer() {
		munmap(pdata, G::bufferlen);
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

	static int startchan;

	void make_names() {
		sprintf(OUT_ROOT, FMT_OUT_ROOT, G::devnum);
		sprintf(OUT_ROOT_NEW, FMT_OUT_ROOT_NEW, G::devnum);
		sprintf(OUT_ROOT_OLD, FMT_OUT_ROOT_OLD, G::devnum);
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
		startchan = 0;
	}
	void finish() {
		system(CLEAN_COMMAND);
		rename(OUT_ROOT, OUT_ROOT_OLD);
		rename(OUT_ROOT_NEW, OUT_ROOT);
		system(FIN_COMMAND);
		if (G::script_runs_on_completion != 0){
			system(G::script_runs_on_completion);
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

		if (verbose > 1 && start && ch_id(src[0]) != 0x20){
			fprintf(stderr, "handling misalign at [0] %08x\n", src[0]);
		}
		if (start && !data_fits_buffer){
			/* search for start point - site 1 */
			for (; !(ch_id(src[0]) == 0x20 &&
			         ch_id(src[1]) == 0x21 &&
			         ch_id(src[2]) == 0x22 &&
			         ch_id(src[3]) == 0x23    ); ++src){
				if (src - src1 > 256){
					if (verbose > 1){
						dump(src1, src-src1);
					}
					fprintf(stderr, "failed to channel align\n");
					return true;
				}
			}
		}

		int startoff = (src - src1);

		if (verbose && startoff){
			fprintf(stderr, "realign: %p -> %p %d %d\n",
					src1, src, src-src1, startoff);
		}
		if (verbose && startchan != 0){
			fprintf(stderr, "start:%d startchan:%d data[0]:%08x\n",
					start, startchan, *src);
		}
		int ichan = startchan;
		/* run to the end of buffer. nsam could be rounded down,
		 * so do not use it.
		 */
		for (isam = startoff/nchan; true; ++isam, ichan = 0){
			for (; ichan < nchan; ++ichan){
				T last = (*src++)&mask[ichan];
				*ddcursors[ichan]++ = last;
				if ((src-src1)*sizeof(T) >= buffer_len){
					if (verbose){
						fprintf(stderr,
						"END buf ch:%d last:%08x\n",
						ichan, last);
					}
					if (++ichan >= nchan) ichan = 0;
					startchan = ichan;

					if (verbose){
						fprintf(stderr,
						"END buf startchan:%d\n", startchan);
					}
					return false;
				}
			}
		}
		/* does not happen */
		return true;
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
		nchan(G::nchan),
		nsam(_buffer_len/sizeof(T)/G::nchan)
	{
		mask = new unsigned[nchan];
		for (int ic = 0; ic < nchan; ++ic){
			mask[ic] = ic < G::m1? FULL_MASK: _mask;
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
	virtual unsigned getItem(int ii) {
		T* src = static_cast<T*>(pdata);
		return src[ii];
	}
	virtual unsigned getSizeofItem() { return sizeof(T); }
};

template<class T> int DemuxBuffer<T>::startchan;
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

			int imax = G::bufferlen/sizeof(unsigned);

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
		nsam(_buffer_len/sizeof(T)/G::nchan)
	{
		outbuf = new T[nsam*G::nchan/over];
	}
	virtual ~OversamplingMapBuffer() {
		delete [] outbuf;
	}
	virtual int writeBuffer(int out_fd, int b_opts) {
		T* src = static_cast<T*>(pdata);
		int sums[G::nchan];
		int nsum = 0;
		int osam = 0;

		for (int ic = 0; ic < G::nchan; ++ic){
			sums[ic] = 0;
		}

		for (int isam = 0; isam < nsam; ++isam){
			for (int ic = 0; ic < G::nchan; ++ic){
				sums[ic] += src[isam*G::nchan+ic];
			}
			if (++nsum >= over){
				for (int ic = 0; ic < G::nchan; ++ic){
					outbuf[osam*G::nchan+ic] = sums[ic] >> asr;
					sums[ic] = 0;
				}
				++osam;
				nsum = 0;
			}
		}

		return write(out_fd, outbuf, osam*G::nchan*sizeof(T));
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

		for (int ii = 0; ii < samples_per_buffer; ++ii, src += G::nchan){
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
		sample_size(sizeof(unsigned) * G::nchan),
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
	sprintf(fname, "%s.hb/%03d", root, _ibuf);

	switch(G::buffer_mode){
	case BM_NULL:
		return new NullBuffer(fname, _ibuf, _buffer_len);
	case BM_SENDFILE:
		return new TurboBuffer(fname, _ibuf, _buffer_len);
	case BM_DEMUX:
		switch(G::wordsize){
		case 2:
			return new DemuxBuffer<short>(fname, _ibuf, _buffer_len, G::mask);
		case 4:
			return new DemuxBuffer<int>(fname, _ibuf, _buffer_len, G::mask);
		default:
			fprintf(stderr, "ERROR: wordsize must be 2 or 4");
			exit(1);
		}
	case BM_TEST:
		return new ScratchpadTestBuffer(fname, _ibuf, _buffer_len);
	default:
		if (G::oversampling == 1){
			return new MapBuffer(fname, _ibuf, _buffer_len);
		}else if (G::wordsize == 2){
			return new OversamplingMapBuffer<short>(
			fname, _ibuf, _buffer_len, G::oversampling, ASR(G::oversampling));
		}else{
			return new OversamplingMapBuffer<int> (
			fname, _ibuf, _buffer_len, G::oversampling, ASR(G::oversampling));
		}
	}
}

Buffer** buffers;



const char* stream_fmt = "%s.c";

struct poptOption opt_table[] = {
	{ "wrbuflen", 0, POPT_ARG_INT, &G::bufferlen, 0, 	"reduce buffer len" },
	{ "null-copy", 0, POPT_ARG_NONE, 0, BM_NULL, 	"no output copy"    },
	{ "sendfile",  0, POPT_ARG_NONE, 0, BM_SENDFILE,
			"use sendfile to transmit (fake)"		    },
	{ "verbose",   0, POPT_ARG_INT, &verbose, 0,  "set verbosity"	    },
	{ "hb0",       0, POPT_ARG_NONE, 0, BM_DEMUX },
	{ "test-scratchpad", 0, POPT_ARG_NONE, 0, BM_TEST,
			"minimal overhead data test buffer top/tail for sample count"},
	{ "nchan",    'N', POPT_ARG_INT, &G::nchan, 0 },
	{ "wordsize", 'w', POPT_ARG_INT, &G::wordsize, 0,
			"data word size 2|4" },
	{ "oversampling", 'O', POPT_ARG_INT, &G::oversampling, 0,
			"set oversampling"},
	{ "mask",      'M', POPT_ARG_INT, &G::mask, 0,
			"set data mask"},
	{ "m1",        'm', POPT_ARG_INT, &G::m1, 0,
			"index of first masked channel"},
	{ "nsam",       0,  POPT_ARG_INT, &G::nsam, 0,
			"desired output samples" },
	{ "oncompletion", 0, POPT_ARG_STRING, &G::script_runs_on_completion, 0,
			"run this script on completion of buffer"
	},
	{ "pre",	0, POPT_ARG_INT, &G::pre, 'P',
			"transient capture, pre-length"
	},
	{ "post",       0, POPT_ARG_INT, &G::post, 'O',
			"transient capture, post-length"
	},
	{ "no_demux",   0, POPT_ARG_NONE, 0, 'D',
			"no demux after transient"
	},
	{ "sites",      0, POPT_ARG_STRING, &G::aggregator_sites, 0,
			"group of aggregated sites to lock"
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

void acq400_stream_getstate(void);

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/select.h>

static void wait_and_cleanup_sighandler(int signo)
{
	printf("wait_and_cleanup_sighandler(%d)\n", signo);
	kill(0, SIGTERM);
	exit(0);
}


static void wait_and_cleanup(pid_t child)
{
	sigset_t  emptyset, blockset;

	printf("wait_and_cleanup 01 pid %d\n", getpid());
	sigemptyset(&blockset);
	sigaddset(&blockset, SIGHUP);
	sigaddset(&blockset, SIGTERM);
	sigaddset(&blockset, SIGINT);
	sigaddset(&blockset, SIGCHLD);
	sigprocmask(SIG_BLOCK, &blockset, NULL);

	struct sigaction sa;
	sa.sa_handler = wait_and_cleanup_sighandler;
	sa.sa_flags = 0;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGCHLD, &sa, NULL);

	sigemptyset(&emptyset);
	fd_set exceptfds;
	FD_ZERO(&exceptfds);
	FD_SET(0, &exceptfds);
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(0, &readfds);

	for(bool finished = false; !finished;){
		int ready = pselect(1, &readfds, NULL, &exceptfds, NULL, &emptyset);

		if (ready == -1 && errno != EINTR){
			perror("pselect");
			finished = true;
			break;
		}else if (FD_ISSET(0, &exceptfds)){
			printf("exception on stdin\n");
			finished = true;
		}else if (FD_ISSET(0, &readfds)){
			if (feof(stdin)){
				printf("EOF\n");
				finished = true;
			}else if (ferror(stdin)){
				printf("ERROR\n");
				finished = true;
			}else{
				char stuff[80];
				fgets(stuff, 80, stdin);

				printf("data on stdin %s\n", stuff);
				if (strncmp(stuff, "quit", 4) == 0){
					finished = true;
				}
			}
		}else{
			printf("out of pselect, not sure why\n");
		}
	}

        printf("wait_and_cleanup exterminate\n");
        kill(0, SIGTERM);
        exit(0);
}

static void hold_open(int site)
{
	char devname[80];
	sprintf(devname, "/dev/acq400.%d.cs", site);
	FILE *fp = fopen(devname, "r");
	if (fp == 0){
		perror(devname);
		exit(1);
	}
	while(fread(devname, 1, 80, fp)){
		;
	}
	exit(1);
}
static void hold_open(const char* sites)
{
	int the_sites[6];
	int nsites = sscanf(sites, "%d,%d,%d,%d,%d,%d",
		the_sites+0, the_sites+1, the_sites+2,
		the_sites+3, the_sites+4, the_sites+5);
	if (nsites){
		setpgid(0, 0);
		pid_t child = fork();
		if (child != 0){
			/* original becomes reaper */
			wait_and_cleanup(child);
		}else{
			for (int isite = 0; isite < nsites; ++isite){
				child = fork();

		                if (child == 0) {
		                	hold_open(the_sites[isite]);
		                	assert(1);
		                }
			}
		}
	}
	/* newly forked main program continues to do its stuff */
}
void init(int argc, const char** argv) {
	char* progname = new char(strlen(argv[0]));
	if (strcmp(progname, "acq400_stream_getstate") == 0){
		acq400_stream_getstate();
	}
	delete [] progname;

	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;
	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		case 'n':
			G::buffer_mode = BM_NULL;
			break;
		case 's':
			G::buffer_mode = BM_SENDFILE;
			break;
		case 'h':
			stream_fmt = "%s.hb0";
			G::buffer_mode = BM_DEMUX;
			break;
		case 'D':
			G::no_demux = true;
			break;
		case 'P':
		case 'O':
			G::stream_mode = SM_TRANSIENT;
			break;
		case BM_TEST:
			G::buffer_mode = BM_TEST;
			break;
		}
	}


	const char* devc = poptGetArg(opt_context);
	if (devc){
		G::devnum = atoi(devc);
	}
	/* else .. defaults to 0 */

	if (G::stream_mode == SM_TRANSIENT){
		if (G::buffer_mode == BM_NOT_SPECIFIED){
			G::buffer_mode = BM_PREPOST;
		}
	}
	if (G::aggregator_sites != 0){
		hold_open(G::aggregator_sites);
	}

	getKnob(G::devnum, "nbuffers",  &G::nbuffers);
	getKnob(G::devnum, "bufferlen", &G::bufferlen);

	if (G::buffer_mode == BM_DEMUX){
		for (nb_cat = 1;
		     nb_cat*G::bufferlen/(G::nchan*G::wordsize) < G::nsam; ++nb_cat){
			;
		}
		if (verbose){
			fprintf(stderr, "BM_DEMUX bufsam:%d\n",
					G::bufferlen/(G::nchan*G::wordsize));
			fprintf(stderr, "BM_DEMUX nb_cat set %d\n", nb_cat);
		}
	}
	buffers = new Buffer* [G::nbuffers];

	root = getRoot(G::devnum);

	for (int ii = 0; ii < G::nbuffers; ++ii){
		buffers[ii] = Buffer::create(root, ii, G::bufferlen);
	}
}

class StreamHead {
protected:
	int fc;

	StreamHead() {
		char fname[128];
		sprintf(fname, stream_fmt, root);
		fc = open(fname, O_RDONLY);
		assert(fc > 0);
	}

	int getBufferId() {
		char buf[80];
		int rc = read(fc, buf, 80);

		if (rc > 0){
			buf[rc] = '\0';
			int ib = atoi(buf);
			assert(ib >= 0);
			assert(ib <= G::nbuffers);
			return ib;
		}else{
			return rc;
		}
	}
	void close() {
		if (fc){
			::close(fc);
			fc = 0;
		}
	}
	virtual ~StreamHead() {
		close();
	}
public:
	virtual void stream();
	static StreamHead& instance();
};


void StreamHead::stream() {
	int ib;
	while((ib = getBufferId()) >= 0){
		buffers[ib]->writeBuffer(1, Buffer::BO_NONE);
	}
}

class NullStreamHead: public StreamHead {

public:
	virtual void stream() {
		int ib;
		while((ib = getBufferId()) >= 0){
			printf("%d\n", ib);
		}
	}
};

class StreamHeadHB0: public StreamHead  {
protected:
	virtual void stream();
};

void StreamHeadHB0::stream() {
	char buf[80];
	int rc;
	int icat = 0;
	int nb = 0;

	while((rc = read(fc, buf, 80)) > 0){
		buf[rc] = '\0';

		int ib[2];
		int nscan = sscanf(buf, "%d %d", ib, ib+1);
		assert(nscan >= 1);
		if (nscan > 0) assert(ib[0] >= 0 && ib[0] < G::nbuffers);
		if (nscan > 1) assert(ib[1] >= 0 && ib[1] < G::nbuffers);

		if (verbose) fprintf(stderr, "\n\n\nUPDATE:%4d nscan:%d read: %s",
				++nb, nscan, buf);

		if (nb_cat > 1 && nscan == 2){
			Buffer* b0 = buffers[ib[0]];
			Buffer* b1 = buffers[ib[1]];
			if (verbose){
				fprintf(stderr, "b0:%p b1:%p\n", b0, b1);
				int last = b0->getLen()/b0->getSizeofItem() - 1;

				fprintf(stderr,
				"buffer end %3d %08x %08x %08x %08x init %3d\n",
				ib[0], 	b0->getItem(last-1), b0->getItem(last),
				b1->getItem(0),	b1->getItem(1), ib[1]);
				fprintf(stderr,
				"buffer end %3d %08x %08x %08x %08x init %3d\n",
				ib[0],
				b0->getItem(last-1)&0x0ff,
				b0->getItem(last)&0x0ff,
				b1->getItem(0)&0x0ff,
				b1->getItem(1)&0x0ff, ib[1]);
			}
			if (verbose > 10){
				fprintf(stderr, "no write\n"); continue;
			}
			b0->writeBuffer(1, Buffer::BO_START);
			b1->writeBuffer(1, Buffer::BO_FINISH);
		}else if (nscan == 2){
			Buffer* b1 = buffers[ib[1]];
			b1->writeBuffer(1, Buffer::BO_START|Buffer::BO_FINISH);
		}else if (nscan == 1){
			Buffer* b0 = buffers[ib[0]];
			b0->writeBuffer(1, Buffer::BO_START|Buffer::BO_FINISH);
		}
		if (verbose) fprintf(stderr, "UPDATE:%4d finished\n", nb);
	}
}

#define OBS_ROOT "/dev/shm/transient"
#define TOTAL_BUFFER_LIMIT	(G::nbuffers * G::bufferlen)



enum STATE {
	ST_STOP,
	ST_RUN_PRE,
	ST_RUN_POST,
	ST_POSTPROCESS
};

int event_received;		/* @@todo : on signal */

typedef std::vector<MapBuffer*> MBUFV;
typedef std::vector<MapBuffer*>::iterator MBUFV_IT;
typedef std::vector<FILE *> FPV;
typedef std::vector<FILE *>::iterator FPV_IT;


#define TRANSOUT	OBS_ROOT"/ch"

unsigned total_buffer_limit() {

}

class Demuxer {
public:
	virtual void demux(void *start, int nbytes) {}
};
template <class T>
class DemuxerImpl : public Demuxer {
	FPV ch;
	int ichan;

	void build_ch() {
		char fname[80];
		sprintf(fname, "mkdir -p %s", TRANSOUT);
		system(fname);

		const char* fmt = G::nchan > 99? "%s/%03d": "%s/%02d";
		ch.push_back(0);	// [0]
		for (int ich = 1; ich <= G::nchan; ++ich){
			sprintf(fname, fmt, TRANSOUT, ich);
			FILE *fp = fopen(fname, "w");
			assert(fp);
			ch.push_back(fp);
		}
	}
	int nextChan() {
		if (++ichan > G::nchan){
			ichan = 1;
		}
		return ichan;
	}
public:
	DemuxerImpl() : ichan(1) {

	}
	virtual ~DemuxerImpl() {
		for (FPV_IT it = ch.begin(); it != ch.end(); ++it){
			fclose(*it);
		}
	}

	virtual void demux(void *start, int nsamples);
};

template <class T>
void DemuxerImpl<T>::demux(void* start, int nbytes)
{
	if (ch.size() == 0){
		build_ch();
	}
	T* startp = (T*)start;
	T* bp = (T*)start;

	if (verbose) {
		printf("%s start:%p nbytes:%d nchan:%d\n",
			__func__, start, nbytes, G::nchan);
		printf("%s start[0:] = %08x,%08x,%08x,%08x\n",
				__func__, bp[0], bp[1], bp[2], bp[3]);
		if (verbose > 1){
			FILE* pf = popen("hexdump -e '8/4 \"%08x \" \"\\n\"'", "w");
			fwrite(bp, sizeof(T), 512, pf);
			pclose(pf);
		}

	}
	for (; (bp-startp)*sizeof(T) < nbytes;){
		for (; (bp-startp)*sizeof(T) < nbytes; nextChan()){
			fwrite(bp++, sizeof(T), 1, ch[ichan]);
		}
	}
	if (verbose){
		printf("%s 99\n", __func__);
	}
}

/* @@todo .. map to shm area for external monitoring. */
struct Progress {
	enum STATE state;
	int pre;
	int post;
	unsigned long long elapsed;
	Progress() {
		memset(this, 0, sizeof(Progress));
	}
	static Progress& instance();

	void print() {
		printf("%d %d %d %llu\n", state, pre, post, elapsed);
		fflush(stdout);
	}
};

Progress& Progress::instance() {
	static Progress* _instance;

	if (_instance == 0){
		int rc = shmget(0xdeadbeef, 128, IPC_CREAT|0666);
		if (rc == -1){
			perror("shmget()");
			exit(1);
		}else{
			void *shm = shmat(rc, 0, 0);
			if (shm == (void*)-1){
				perror("shmat()");
				exit(1);
			}else{
				_instance = (Progress*)shm;
				memset(_instance, 0, sizeof(Progress));
			}
		}
	}

	return *_instance;
}

void acq400_stream_getstate(void)
{
	Progress::instance().print();
	exit(0);
}

class StreamHeadPrePost: public StreamHead  {
protected:
	int pre;
	int post;
	Progress& actual;
	int total_bs;
	int nobufs;

	int samples_buffer;

	void setState(enum STATE _state){
		actual.state = _state;
	}

	virtual void onStreamStart() 		 {}
	virtual void onStreamBufferStart(int ib) {}
	virtual void onStreamEnd() 		 {}

	void streamCore() {
		int ib;

		while((ib = getBufferId()) >= 0){
			onStreamBufferStart(ib);

			switch(actual.state){
			case ST_RUN_PRE:
				if (event_received){
					actual.state = ST_RUN_POST;
				}
				break;
			case ST_RUN_POST:
				actual.post += samples_buffer;
				if (actual.post > post){
					actual.post = post;
					actual.state = ST_POSTPROCESS;
					actual.elapsed += samples_buffer;
					actual.print();
					return;
				}
			}
			actual.elapsed += samples_buffer;
			actual.print();
		}
	}
	unsigned findEvent() {
		assert(0);
		return 0;
	}
	unsigned workbackfrom(unsigned evp) {
		assert(0);
		return 0;
	}
public:
	StreamHeadPrePost(int _pre, int _post) :
			pre(_pre), post(_post),
			samples_buffer(0),
			actual(Progress::instance())
		{
		setState(ST_STOP);
		int total_bs = (pre+post)*sample_size() + G::bufferlen;
		samples_buffer = G::bufferlen/sample_size();

		while((pre+post)*sample_size() > TOTAL_BUFFER_LIMIT){
			if (post) post -= samples_buffer/2;
			if (pre)  pre  -= samples_buffer/2;
		}

		/* round total buffer up to multiple of sample size */
		/* @@todo .. nobody understands this WARNING
		if (total_bs > (pre+post)*sample_size()){
			fprintf(stderr,
			"WARNING reducing to fit memory pre:%d post:%d\n",
					pre, post);
		}
		*/


		nobufs = total_bs/G::bufferlen;
		while (nobufs*G::bufferlen < total_bs || nobufs < 2){
			++nobufs;
		}

		if (verbose){
			fprintf(stderr, "StreamHeadPrePost pre:%d post:%d\n",
				pre, post);
			fprintf(stderr, "StreamHeadPrePost sample_size:%d\n",
				sample_size());
			fprintf(stderr, "StreamHeadPrePost total buffer:%d obs:%d nob:%d\n",
				total_bs, G::bufferlen, nobufs);
		}
	}

	void stream() {
		setState(pre? ST_RUN_PRE: ST_RUN_POST);
		onStreamStart();
		streamCore();
		close();
		onStreamEnd();
		setState(ST_STOP);
		actual.print();
	}
};
class PostprocessingStreamHeadPrePost: public StreamHeadPrePost  {

	void *ba0;
	void *ba1;

	char *cursor;
	void *event_cursor;	/* mark where event was detected */

	std::vector <MapBuffer*> outbuffers;
	Demuxer& demuxer;

	void buildOutputBuffers() {
		system("mkdir -p " OBS_ROOT);
		ba0 = (void*)0x4000000;
		char * ba = static_cast<char*>(ba0);
		for (int ii = 0; ii < nobufs; ++ii, ba += G::bufferlen){
			char *fname = new char[strlen(OBS_ROOT)+6];
			snprintf(fname, strlen(OBS_ROOT)+6, "%s/%03d",
					OBS_ROOT, ii);
			outbuffers.push_back(
				new MapBuffer(fname, ii, G::bufferlen, true, ba));
		}
		ba1 = ba;

		for (MBUFV_IT obit = outbuffers.begin();
				obit != outbuffers.end(); ++obit){
			if (verbose > 1){
				fprintf(stderr, "buffer: %s %p %p\n",
					(*obit)->getName(),
					(*obit)->getBase(),
					(char*)(*obit)->getBase()+G::bufferlen);
			}
		}
	}


	int _demux(int nbytes) {
		if (verbose){
			printf("%s cursor:%p nbytes:%d\n", __func__, cursor, nbytes);
		}
		for (MBUFV_IT it = outbuffers.begin(); it != outbuffers.end(); ++it){
			int remain = (*it)->includes(cursor);
			if (remain){
				if (remain > nbytes){
					remain = nbytes;
				}
				demuxer.demux(cursor, remain);
				char fname[128];
				sprintf(fname, "%s", (*it)->getName());
				delete (*it);
				outbuffers.erase(it);
				unlink(fname);
				nbytes -= remain;
				cursor += remain;
				return remain;
			}
		}
		return 0;
	}
	void demux(int nsamples){
		int nbytes = nsamples*sample_size();

		while(nbytes > 0){
			int rembytes = _demux(nbytes);
			if (rembytes){
				nbytes -= rembytes;
			}
		}
	}
	void postProcess() {
		setState(ST_POSTPROCESS); actual.print();
		if (pre){
			unsigned epos = findEvent();
			unsigned prestart = workbackfrom(epos);
			cursor = static_cast<char*>(ba0)+prestart;
			demux(pre+post);
		}else{
			cursor = static_cast<char*>(ba0);
			demux(post);
		}
	}
	virtual void onStreamStart() {
		cursor = static_cast<char*>(ba0);
	}
	virtual void onStreamBufferStart(int ib) {
		buffers[ib]->copyBuffer(cursor);
		cursor += G::bufferlen;
		if (cursor >= static_cast<char*>(ba1)){
			cursor = static_cast<char*>(ba0);
		}
		switch(actual.state){
		case ST_RUN_PRE:
			if (event_received){
				event_cursor = cursor;
			}
			break;
		}
	}
	virtual void onStreamEnd() {
		if (!G::no_demux){
			postProcess();
		}
	}
public:
	PostprocessingStreamHeadPrePost(Demuxer& _demuxer, int _pre, int _post) :
		StreamHeadPrePost(_pre, _post),
		demuxer(_demuxer) {
		buildOutputBuffers();
	}
};

StreamHead& StreamHead::instance() {
	static StreamHead* _instance;

	if (_instance == 0){
		if (G::stream_mode == SM_TRANSIENT){
			if (G::buffer_mode == BM_PREPOST){
				Demuxer *demuxer;
				if (G::wordsize == 4){
					demuxer = new DemuxerImpl<int>;
				}else{
					demuxer = new DemuxerImpl<short>;
				}
				_instance = new PostprocessingStreamHeadPrePost(*demuxer, G::pre, G::post);
			}else{
				_instance = new StreamHeadPrePost(G::pre, G::post);
			}
		}else{
			switch(G::buffer_mode){
			case BM_DEMUX:
				_instance = new StreamHeadHB0();
				break;
			case BM_NULL:
				_instance = new NullStreamHead();
				break;
			default:
				_instance = new StreamHead();
				break;
			}
		}
	}
	return *_instance;
}



int main(int argc, const char** argv)
{

	init(argc, argv);
	StreamHead::instance().stream();
	return 0;
}
