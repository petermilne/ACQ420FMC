/* ------------------------------------------------------------------------- */
/* acq400_stream.cpp  		                     	 		     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2012 pgm, D-TACQ Solutions Ltd                            *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
 *   Created on: Mar 31, 2013                                                *
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
 *
 * - Usage
 * - No args: stream and copy to stdout eg the 4210 service
 * - --null-copy : no copy
 * - --pre/--post : pre/post transient BOS - data left in KBUFS
 * -- --pre=0 : easy, first N KBUFS
 * -- --pre!=0 : need to find the cursor.
 * - --demux : controls post shot demux
 * - --oversampling : controls sub-rate streaming (EPICS feed).
 *
 *
 * Dropped: copy/demux (SOS mode).
 *
 * Demux: has to be done in-situ in the kbufs.
 * Then allow an overlay VFS to extract the data.
 * Demux --pre==0 : demux in situ starting at ibuf=0
 * Demux --pre!-0 : demux and write back to ibuf=0
 * Easy to read from linea mapped buffers. Copy to local, DMA back
 * Driver provides a pair of temp buffers?. Allow ping/pong copy back.
 *
 * All the kbufs are mapped to a linear thing: this makes it easy for app-code.
 * Less easy for kernel code ..
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/sendfile.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "popt.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <libgen.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <errno.h>

#include <sched.h>

#define VERID	"B1006"

#define NCHAN	4

#include <semaphore.h>
#include <syslog.h>

#include <vector>

#include "local.h"		/* chomp() hopefully, not a lot of other garbage */

using namespace std;
int timespec_subtract (timespec *result, timespec *x, timespec *y) {
	const unsigned nsps = 1000000000;
	/* Perform the carry for the later subtraction by updating y. */
	if (x->tv_nsec < y->tv_nsec) {
		int nsec = (y->tv_nsec - x->tv_nsec) / 1000000 + 1;
		y->tv_nsec -= nsps * nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_nsec - y->tv_nsec > nsps) {
		int nsec = (x->tv_nsec - y->tv_nsec) / nsps;
		y->tv_nsec += nsps * nsec;
		y->tv_sec -= nsec;
	}

	/* Compute the time remaining to wait. tv_nsec is certainly positive. */
	if (result){
		result->tv_sec = x->tv_sec - y->tv_sec;
		result->tv_nsec = x->tv_nsec - y->tv_nsec;
	}
	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
}

static int getKnob(int idev, const char* knob, unsigned* value)
{
	char kpath[128];
	if (knob[0] == '/'){
		strncpy(kpath, knob, 128);
	}else{
		snprintf(kpath, 128, "/dev/acq400.%d.knobs/%s", idev, knob);
	}
	FILE *fp = fopen(kpath, "r");
	if (fp){
		int rc = fscanf(fp, "%u", value);
		fclose(fp);
		return rc;
	} else {
		return -1;
	}
}

static int getKnob(int idev, const char* knob, char* value)
{
	char kpath[128];
	if (knob[0] == '/'){
		strncpy(kpath, knob, 128);
	}else{
		snprintf(kpath, 128, "/dev/acq400.%d.knobs/%s", idev, knob);
	}
	FILE *fp = fopen(kpath, "r");
	if (fp){
		int rc = fscanf(fp, "%s", value);
		fclose(fp);
		return rc;
	} else {
		return -1;
	}
}

static int setKnob(int idev, const char* knob, const char* value)
{
	char kpath[128];
	if (knob[0] == '/'){
		strncpy(kpath, knob, 128);
	}else{
		snprintf(kpath, 128, "/dev/acq400.%d.knobs/%s", idev, knob);
	}
	FILE *fp = fopen(kpath, "w");
	if (fp){
		int rc = fprintf(fp, "%s\n", value);
		fclose(fp);
		return rc;
	} else {
		return -1;
	}
}

static int setKnob(int idev, const char* knob, int value)
{
	char vx[32]; snprintf(vx, 32, "%d", value);
	return setKnob(idev, knob, vx);
}

#define BM_NOT_SPECIFIED	'\0'
#define BM_NULL			'n'
#define BM_RAW			'r'	/* no demux */
#define BM_DEMUX 		'h'
#define BM_TEST			't'
#define BM_PREPOST 		'P'

#define SM_STREAM    		'S'
#define SM_TRANSIENT 		'T'

/* all globals in one namespace : G */
namespace G {
	unsigned int nchan = NCHAN;
	unsigned int nbuffers = 16;
	unsigned int bufferlen = 0x40000;
	int wordsize = 2;		/** choice sizeof(short) or sizeof(int) */
	#define FULL_MASK 0xffffffff
	unsigned mask = FULL_MASK;	/** mask data with this value */
	int m1 = 0;			/** start masking from here */
	int oversampling = 0;
	int devnum = 0;
	const char* script_runs_on_completion = 0;
	const char* aggregator_sites = 0;
	int nsam = 4096;

	int control_handle;

	int pre;
	int post;
	int demux;

	int buffer_mode = BM_NOT_SPECIFIED;
	int stream_mode = SM_STREAM;

	bool soft_trigger;
	sem_t* aggsem;
	char* aggsem_name;
	char* state_file;
	bool show_es;
	FILE* state_fp;
	char* pre_demux_script;
	int show_first_sample;
	bool es_diagnostic;
	int report_es;
	int nsites;
	int the_sites[6];
	char* progress;
	int corner_turn_delay;	/* test pre/post caps on corner turn, buffers */
	int exit_on_trigger_fail;

	char* subset;
	char* sum;

	bool null_copy;
};



int verbose;
int nb_cat =1;	/* number of buffers to concatenate */


int sample_size() {
	return G::nchan * G::wordsize;
}

unsigned s2b(unsigned samples) {
	return samples*sample_size();
}
unsigned b2s(unsigned bytes) {
	return bytes/sample_size();
}



template <int MASK, int PAT>
class ES {
	bool is_es_word(unsigned word) {
		return (word&MASK) == PAT;
	}
public:
	bool isES(unsigned *cursor){
		return is_es_word(cursor[0]) && is_es_word(cursor[1]) &&
			is_es_word(cursor[2]) && is_es_word(cursor[3]);
	}
};

ES<0xfffffff0, 0xaa55f150> evX;
ES<0xffffffff, 0xaa55f151> ev0;

static int createOutfile(const char* fname) {
	int fd = open(fname,
			O_WRONLY|O_CREAT|O_TRUNC, S_IRWXU|S_IRGRP|S_IROTH);
	if (fd < 0){
		perror(fname);
		exit(1);
	}
	return fd;
}

class Buffer {

protected:
	int fd;
	const char* fname;	
	int buffer_len;
	const int ibuf;
	static int last_buf;
protected:
	char *pdata;
public:
	enum BUFFER_OPTS { BO_NONE, BO_START=0x1, BO_FINISH=0x2 };

	int getLen() { return buffer_len; }

	virtual unsigned getItem(int ii) { return 0; }
	virtual unsigned getSizeofItem() { return 1; }

	virtual int writeBuffer(int out_fd, int b_opts) = 0;
	virtual int writeBuffer(int out_fd, int b_opts, int start_off, int len)
	{
		return -1;
	}

	virtual int copyBuffer(void* dest) { return -1; }

	Buffer(const char* _fname, int _buffer_len):
		fname(_fname),
		ibuf(last_buf++),
		buffer_len(_buffer_len)
	{
		fd = open(fname, O_RDWR, 0777);

		if (fd < 0){
			perror(fname);
			exit(1);
		}

		the_buffers.push_back(this);
	}
	Buffer(Buffer* cpy) :
		fd(cpy->fd),
		fname(cpy->fname),
		buffer_len(cpy->buffer_len),
		ibuf(cpy->ibuf),
		pdata(cpy->pdata)
	{}
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
	static Buffer* create(const char* root, int _buffer_len);
	static vector<Buffer*> the_buffers;

	const char* getName() {
		return fname;
	}

	virtual int pred() {
		return ibuf == 0? last_buf-1: ibuf-1;
	}
	virtual int succ() {
		return ibuf >= last_buf-1? 0: ibuf+1;
	}
	int ib() {
		return ibuf;
	}
	virtual char* getBase() { return pdata; }

	char *getEnd() {
		return getBase() + getLen();
	}

	static int samples_per_buffer() {
		return G::bufferlen/sample_size();
	}
};

int Buffer::last_buf;
vector<Buffer*> Buffer::the_buffers;

class NullBuffer: public Buffer {
public:
	NullBuffer(const char* _fname, int _buffer_len):
		Buffer(_fname, _buffer_len)
	{
		static int report;
		if (!report &&verbose){
			fprintf(stderr, "NullBuffer()\n");
			++report;
		}
	}
	virtual int writeBuffer(int out_fd, int b_opts) {
		return buffer_len;
	}
};
class MapBuffer: public Buffer {
protected:
	static char* ba0;		/* low end of mapping */
	static char* ba1;		/* hi end of mapping */
	static char* ba_lo;		/* lo end of data set */
	static char* ba_hi;		/* hi end of data set likely == ba1 */
	static bool buffer_0_reserved;

public:
	virtual int writeBuffer(int out_fd, int b_opts, int start_off, int len)
	/**< all offsets in bytes */
	{
		if (start_off > buffer_len) {
			return 0;
		}else{
			if (buffer_len - start_off < len){
				len = buffer_len - start_off;
			}
		}
		return write(out_fd, pdata+start_off, len);
	}

	virtual int writeBuffer(int out_fd, int b_opts) {
		return write(out_fd, pdata, buffer_len);
	}
	virtual int copyBuffer(void* dest) {
		memcpy(dest, pdata, buffer_len);
		return buffer_len;
	}

	static char* get_ba0()	 { return ba0;   }
	static char* get_ba_lo() { return ba_lo; }
	static char* get_ba_hi() { return ba_hi; }


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
			fprintf(stderr, "%s %s ret %d\n", __func__, fname, remain);
		}
		return remain;
	}

	MapBuffer(const char* _fname, int _buffer_len) :
		Buffer(_fname, _buffer_len)
	{

		pdata = static_cast<char*>(mmap(static_cast<void*>(ba1), G::bufferlen,
			PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, fd, 0));

		if (pdata != ba1){
			fprintf(stderr, "mmap() failed to get the hint:%p actual %p\n", ba1, pdata);
			exit(1);
		}
		if (verbose > 2) fprintf(stderr, "MapBuffer[%d] %s, %p\n",
				the_buffers.size()-1, fname, pdata);

		ba_hi = ba1 += G::bufferlen;
		assert(pdata != MAP_FAILED);
	}
	virtual ~MapBuffer() {
		munmap(pdata, G::bufferlen);
	}
	static int getBuffer(char* pb) {
		if (pb < ba0){
			return -1;
		}else if (pb > ba1){
			return -2;
		}else{
			return (pb-ba0)/G::bufferlen;
		}
	}
	static const char* listBuffers(char* p0, char* p1, bool show_ba = false){
		char report[512];
		report[0] = '\0';
		int ibuf1 = -1;

		for(char* pn = p0; pn <= p1; pn += G::bufferlen){
			int ibuf = getBuffer(pn);
			if (ibuf != ibuf1){
				if (!show_ba){
					sprintf(report+strlen(report), strlen(report)? ",%d":"%d", ibuf);
				}else{
					char* ba = reinterpret_cast<MapBuffer*>(the_buffers[ibuf])->pdata;
					sprintf(report+strlen(report), strlen(report)? ",%p":"%p", ba);
				}
				ibuf1 = ibuf;
			}
		}
		char* ret = new char[strlen(report)+1]; // yes, this is a leak
		strcpy(ret, report);
		return ret;
	}
	static void reserve() {
		ba_lo = ba0 + G::bufferlen;
		buffer_0_reserved = true;
	}
	static bool hasReserved() {
		return buffer_0_reserved;
	}
	virtual int pred() {
		int first_buf = buffer_0_reserved? 1: 0;
		return ibuf == first_buf? last_buf-1: ibuf-1;
	}
	virtual int succ() {
		int first_buf = buffer_0_reserved? 1: 0;
		return ibuf >= last_buf-1? first_buf: ibuf+1;
	}

	static char* ba(int ibuf){
		return reinterpret_cast<MapBuffer*>(Buffer::the_buffers[ibuf])->pdata;
	}
};

#define MAPBUFFER_BA0     (char*)(0x40000000)
char* MapBuffer::ba0   = MAPBUFFER_BA0;		/* const */
char* MapBuffer::ba1   = MAPBUFFER_BA0;		/* initial value increments to top */
char* MapBuffer::ba_lo = MAPBUFFER_BA0;		/* initial value, may move by reserve count */
char* MapBuffer::ba_hi;				/* tracks ba1 @@todo may be redundant */
bool MapBuffer::buffer_0_reserved;

#define FMT_OUT_ROOT		"/dev/shm/AI.%d.wf"
#define FMT_OUT_ROOT_NEW	"/dev/shm/AI.%d.new"
#define FMT_OUT_ROOT_OLD	"/dev/shm/AI.%d.old"



template <class T>
class DemuxBuffer: public Buffer {
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

	static u32 ID_MASK;

	static int startchan;

	static void init() {
		if (ID_MASK) return;

		if (getenv("NOSID")){
			ID_MASK = 0x1f;
			fprintf(stderr, "ID_MASK set %02x\n", ID_MASK);
		}else{
			ID_MASK = 0xff;
		}
	}
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
		return data&ID_MASK;
	}

	void dump(T* src, int nwords){
		char cmd[80];
		sprintf(cmd, "hexdump -ve '%d/%d \"%%08x \" \"\\n\" '",
							nchan, sizeof(T));
		FILE *pp = popen(cmd, "w");
		fwrite(src, sizeof(T), nwords, pp);
		pclose(pp);
	}
	/* NB: this is an out-of-place demux, source is unchanged */
	bool demux(bool start, int start_off, int len) {
		T* src1 = reinterpret_cast<T*>(pdata+start_off);
		T* src = reinterpret_cast<T*>(pdata+start_off);
		int Tlen = len/sizeof(T);
		int isam = 0;

		if (verbose > 1) fprintf(stderr, "demux() start_off:%08x src:%p\n",
				start_off, src);

		if (verbose > 1 && start && ch_id(src[0]) != 0x00){
			fprintf(stderr, "handling misalign at [0] %08x data_fits_buffer:%d\n",
					src[0], data_fits_buffer);
		}

		if (start && !data_fits_buffer){
			/* search for start point - site 1 */
			for (; !(ch_id(src[0]) == ch_id(0x20) &&
			         ch_id(src[1]) == ch_id(0x21) &&
			         ch_id(src[2]) == ch_id(0x22) &&
			         ch_id(src[3]) == ch_id(0x23)    ); ++src){
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
		if (verbose) fprintf(stderr, "can skip ES");

		for (isam = startoff/nchan; true; ++isam, ichan = 0){
			while (evX.isES(reinterpret_cast<unsigned*>(src))){
				if (verbose) fprintf(stderr, "skip ES\n");
				src += nchan;
			}
			for (; ichan < nchan; ++ichan){
				T last = (*src++)&mask[ichan];
				*ddcursors[ichan]++ = last;
				if (src-src1 >= Tlen){
					if (verbose){
						fprintf(stderr,
						"demux() END buf ch:%d src:%p len:%d\n",
						ichan, src,  ddcursors[ichan]-dddata[ichan]);
					}
					if (++ichan >= nchan) ichan = 0;
					startchan = ichan;

					if (verbose && startchan != 0){
						fprintf(stderr,
						"demux() END buf startchan:%d\n", startchan);
					}
					return false;
				}
			}
		}
		/* does not happen */
		return true;
	}
	bool writeChan(int ic){
		FILE* fp = fopen(fnames[ic], "w");
		if (fp ==0){
			perror(fnames[ic]);
			return true;
		}
		int nelems = ddcursors[ic]-dddata[ic];
		int nwrite = fwrite(dddata[ic], sizeof(T), nelems, fp);
		fclose(fp);
		if (nwrite != nelems || verbose&&ic==0){
			fprintf(stderr, "DemuxBuffer::writeChan(%s) %d %d %s\n",
					fnames[ic], nwrite, nelems, nwrite!=nelems? "ERROR":"");
		}
		return false;
	}
public:
	virtual int writeBuffer(int out_fd, int b_opts) {
		if ((b_opts&BO_START) != 0){
			start();
		}
		demux((b_opts&BO_START),0, buffer_len);
		if ((b_opts&BO_FINISH) != 0){
			for (int ic = 0; ic < nchan; ++ic){
				if (writeChan(ic)){
					// links take out from under
					return -1;
				}
			}
			finish();
		}
		return buffer_len;
	}
	virtual int writeBuffer(int out_fd, int b_opts, int start_off, int len)
	{
		if ((b_opts&BO_START) != 0){
			start();
		}
		if (verbose) fprintf(stderr, "DemuxBuffer::writeBuffer %s%s %p + 0x%08x + %d %p..%p\n",
				(b_opts&BO_START)? "S":"", (b_opts&BO_FINISH)? "F":"",
				pdata, start_off, len, pdata+start_off, pdata+start_off+len);

		demux((b_opts&BO_START), start_off, len);
		if ((b_opts&BO_FINISH) != 0){
			for (int ic = 0; ic < nchan; ++ic){
				if (writeChan(ic)){
					if (verbose) fprintf(stderr, "writeChan() fail\n");
					// links take out from under
					return -1;
				}
			}
			finish();
		}
		return buffer_len;
	}

	DemuxBuffer(Buffer* cpy, unsigned _mask) :
		Buffer(cpy),
		nchan(G::nchan)
	{
		nsam = buffer_len/sizeof(T)/G::nchan;
		init();
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

		data_fits_buffer = nsam*sizeof(T)*nchan == buffer_len;
		if (verbose>1){
			fprintf(stderr, "nsam:%d _buffer_len:%d data_fits_buffer? %s\n",
					nsam, buffer_len, data_fits_buffer? "YES": "NO");
		}
	}

	virtual unsigned getItem(int ii) {
		T* src = reinterpret_cast<T*>(pdata);
		return src[ii];
	}
	virtual unsigned getSizeofItem() { return sizeof(T); }
};

template<class T> unsigned DemuxBuffer<T>::ID_MASK;
template<class T> int DemuxBuffer<T>::startchan;
template<class T> T** DemuxBuffer<T>::dddata;
template<class T> T** DemuxBuffer<T>::ddcursors;


template <class T>
class OversamplingMapBuffer: public Buffer {
	const int over, asr;
	const int asr1;
	const int nsam;
	T *outbuf;
	int* sums;
public:
	OversamplingMapBuffer(Buffer* cpy, int _oversampling, int _asr) :
		Buffer(cpy),
		over(_oversampling),
		asr(_asr),
		asr1(sizeof(T)==4?8:0),
		nsam(buffer_len/sizeof(T)/G::nchan)
	{
		static int report;
		if (!report &&verbose){
			fprintf(stderr, "OversamplingMapBuffer()\n");
			++report;
		}
		outbuf = new T[nsam*G::nchan/over];
		sums = new int[G::nchan];
	}
	virtual ~OversamplingMapBuffer() {
		delete [] outbuf;
		delete [] sums;
	}
	virtual int writeBuffer(int out_fd, int b_opts) {
		T* src = reinterpret_cast<T*>(pdata);
		int nsum = 0;
		int osam = 0;

		fprintf(stderr, "OversamplingMapBuffer::writeBuffer(<%d>) out_fd:%d\n", sizeof(T), out_fd);
		memset(sums, 0, G::nchan*sizeof(int));

		for (int isam = 0; isam < nsam; ++isam){
			for (int ic = 0; ic < G::nchan; ++ic){
				sums[ic] += src[isam*G::nchan+ic] >> asr1;
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

template <class T>
class OversamplingMapBufferSingleSample: public Buffer {
	const int over, asr;
	const int asr1;
	const int nsam;
	int* sums;

public:
	OversamplingMapBufferSingleSample(Buffer* cpy,
			int _oversampling, int _asr) :
		Buffer(cpy),
		over(_oversampling),
		asr(_asr),
		asr1(sizeof(T)==4?8:0),
		nsam(buffer_len/sizeof(T)/G::nchan)
		{
		static int report;
		if (!report &&verbose){
			fprintf(stderr, "OversamplingMapBufferSingleSample()\n");
			++report;
		}
		sums = new int[G::nchan];
	}
	virtual ~OversamplingMapBufferSingleSample() {
		delete [] sums;
	}
	virtual int writeBuffer(int out_fd, int b_opts) {
		T* src = reinterpret_cast<T*>(pdata);
		int stride = nsam/over;


		if (G::show_first_sample){
			memset(sums, 0, G::nchan*sizeof(int));

			for (int ic = 0; ic < G::nchan; ++ic){
				sums[ic] += src[ic];
			}
			int fd = createOutfile("/dev/shm/first_sample");
			write(fd, sums, G::nchan*sizeof(int));
			close(fd);
		}
		memset(sums, 0, G::nchan*sizeof(int));

		for (int isam = 0; isam < nsam; isam += stride){
			/* runs of samples are bad - could be ES, could be strange 0x0000 .. either way, REJECT */
			T checkit = src[isam*G::nchan+0] >> asr1;
			bool checkit_ok = false;
			for (int ic = 1; ic < 4; ++ic){
				T checkit2 = src[isam*G::nchan+ic] >> asr1;
				if (checkit2 != checkit){
					checkit_ok = true;
					break;
				}
			}
			if (!checkit_ok){
				fprintf(stderr, "checkit_ok reject\n");
				return 0;
			}
			for (int ic = 0; ic < G::nchan; ++ic){
				sums[ic] += src[isam*G::nchan+ic] >> asr1;
			}
		}

		if (asr){
			for (int ic = 0; ic < G::nchan; ++ic){
				sums[ic] >>= asr;
			}
		}
		if (verbose) fprintf(stderr, "writeBuffer() 99\n");

		return write(out_fd, sums, G::nchan*sizeof(int));
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
	ScratchpadTestBuffer(const char* _fname, int _buffer_len):
		MapBuffer(_fname, _buffer_len),
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
		unsigned* src = reinterpret_cast<unsigned*>(pdata);

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

Buffer* Buffer::create(const char* root, int _buffer_len)
{
	char* fname = new char[128];
	sprintf(fname, "%s.hb/%03d", root, Buffer::last_buf);

	static int nreport;
	if (nreport == 0 && verbose){
		++nreport;
		if (verbose) fprintf(stderr, "Buffer::create() G::buffer_mode: %c\n", G::buffer_mode);
	}

	switch(G::buffer_mode){
	case BM_NULL:
		return new NullBuffer(fname, _buffer_len);
	case BM_TEST:
		return new ScratchpadTestBuffer(fname, _buffer_len);
	default:
		return new MapBuffer(fname, _buffer_len);
	}
}


class BufferCloner {
	static void cloneBuffers(BufferCloner& cloner)
	{
		vector<Buffer*> cpyBuffers = Buffer::the_buffers;

		Buffer::the_buffers.clear();
		for (int ii = 0; ii < cpyBuffers.size(); ++ii){
			Buffer::the_buffers[ii] = cloner(cpyBuffers[ii]);
		}
	}
public:
	virtual Buffer* operator() (Buffer* cpy) = 0;

	template <class T>
	static void cloneBuffers()
	{
		T cloner;

		BufferCloner::cloneBuffers(cloner);
	}
};

class OversamplingBufferCloner: public BufferCloner {
	static Buffer* createOversamplingBuffer(Buffer* cpy)
	{
		int os = abs(G::oversampling);
		bool single = G::oversampling < 0;

		if (G::wordsize == 2){
			if (single){
				return new OversamplingMapBufferSingleSample<short>(
						cpy, os, 0);
			}else{
				return new OversamplingMapBuffer<short>(
						cpy, os, ASR(os));
			}
		}else{
			if (single){
				return new OversamplingMapBufferSingleSample<int>(
						cpy, os, ASR(os));
			}else{
				return new OversamplingMapBuffer<int> (
						cpy, os, ASR(os));
			}
		}
	}
public:
	OversamplingBufferCloner() {}
	virtual Buffer* operator() (Buffer* cpy) {
		return createOversamplingBuffer(cpy);
	}
};

class DemuxBufferCloner: public BufferCloner {
	static Buffer* createDemuxBuffer(Buffer *cpy)
	{
		switch(G::wordsize){
		case 2:
			return new DemuxBuffer<short>(cpy, G::mask);
		case 4:
			return new DemuxBuffer<int>(cpy, G::mask);
		default:
			fprintf(stderr, "ERROR: wordsize must be 2 or 4");
			exit(1);
		}
	}

public:
	virtual Buffer* operator() (Buffer* cpy) {
		return createDemuxBuffer(cpy);
	}
};


enum STATE {
	ST_STOP,
	ST_ARM,
	ST_RUN_PRE,
	ST_RUN_POST,
	ST_POSTPROCESS
};

#define NSinMS	1000000
#define MSinS	1000

long diffmsec(struct timespec* t0, struct timespec *t1){
	long msec;

	if (t1->tv_nsec > t0->tv_nsec){
		msec = (t1->tv_nsec - t0->tv_nsec)/NSinMS;
		msec += (t1->tv_sec - t0->tv_sec)*MSinS;
	}else{
		msec  = (NSinMS + t1->tv_nsec - t0->tv_nsec)/NSinMS;
		msec += (t1->tv_sec - t0->tv_sec - 1)*MSinS;
	}
	return msec;
}

#define MIN_REPORT_INTERVAL_MS	200


#define 	PRINT_WHEN_YOU_CAN	false

struct Progress {
	enum STATE state;
	int pre;
	int post;
	unsigned long long elapsed;
	struct timespec last_time;
	static long min_report_interval;
	const char* name;

	char previous[80];

	FILE* status_fp;

	bool isRateLimited() {
		struct timespec time_now;
		clock_gettime(CLOCK_REALTIME_COARSE, &time_now);

		if (min_report_interval == 0 || diffmsec(&last_time, &time_now) >= min_report_interval){
			last_time = time_now;
			return false;
		}else{
			return true;
		}
	}
	Progress(FILE *_fp, const char* _name = "Progress") :
			status_fp(_fp) {
		memset(this, 0, sizeof(Progress));
		name = _name;
		status_fp = stderr;
		if (getenv("MIN_REPORT_INTERVAL_MS")){
			min_report_interval = atoi(getenv("MIN_REPORT_INTERVAL_MS"));
			if (verbose){
				fprintf(status_fp, "min_report_interval set %d\n",
						min_report_interval);
			}
		}
		clock_gettime(CLOCK_REALTIME_COARSE, &last_time);

		if (verbose) fprintf(stderr, "Progress %s\n", name);
	}
	static Progress& instance(FILE* fp = 0);

	virtual void printState(char current[]) {

	}
	virtual void print(bool ignore_ratelimit = true, int extra = 0) {

	}
	virtual void setState(enum STATE _state){

	}

	static Progress null_progress;
};

Progress Progress::null_progress(stderr);


struct ProgressImpl: public Progress {

	ProgressImpl(FILE *_fp) : Progress(_fp, "ProgressImpl") {
	}
	virtual void printState(char current[]) {
		if (G::state_fp){
			rewind(G::state_fp);
			fputs(current, G::state_fp);
			fflush(G::state_fp);
		}
	}
	virtual void print(bool ignore_ratelimit = true, int extra = 0) {
		char current[80];
		snprintf(current, 80, "%d %d %d %llu %d\n", state, pre, post, elapsed, extra);

		if ((ignore_ratelimit || !isRateLimited()) && strcmp(current, previous)){
			fputs(current, status_fp);
			fflush(status_fp);
			strcpy(previous, current);
		}
		printState(current);
	}
	virtual void setState(enum STATE _state){
		state = _state;
		print();
	}
};

const char* stream_fmt = "%s.c";

int shuffle_test;
int fill_ramp_incr;

struct poptOption opt_table[] = {
	{ "wrbuflen", 0, POPT_ARG_INT, &G::bufferlen, 0,
			"reduce buffer len" },
	{ "null-copy", 0, POPT_ARG_NONE, 0, BM_NULL,
			"no output copy"    },
	{ "verbose",   'v', POPT_ARG_INT, &verbose, 0,
			"set verbosity"	    },
	{ "report-es", 'r', POPT_ARG_INT, &G::report_es, 0,
			"report es position every find" },
	{ "hb0",       0, POPT_ARG_NONE, 0, BM_DEMUX },
	{ "test-scratchpad", 0, POPT_ARG_NONE, 0, BM_TEST,
			"minimal overhead data test buffer top/tail for sample count"},
	{ "nchan",    'N', POPT_ARG_INT, &G::nchan, 0 },
	{ "wordsize", 'w', POPT_ARG_INT, &G::wordsize, 0,
			"data word size 2|4" },
	{ "oversampling", 'O', POPT_ARG_INT, &G::oversampling, 0,
			"set oversampling (negative: single output sample per buffer)"},
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
	{ "demux",   'D', POPT_ARG_INT, &G::demux, 'D',
			"force demux / no demux after transient"
	},
	{ "sites",      0, POPT_ARG_STRING, &G::aggregator_sites, 0,
			"group of aggregated sites to lock"
	},
	{ "soft-trigger", 0, POPT_ARG_INT, &G::soft_trigger, 0,
			"assert soft-trigger after start"
	},
	{ "state-file",   0, POPT_ARG_STRING, &G::state_file, 'S',
			"write changing state string to this file"
	},
	{ "shuffle_test", 0, POPT_ARG_INT, &shuffle_test, 0,
			"time buffer shuffle"
	},
	{ "fill_ramp", 0, POPT_ARG_INT, &fill_ramp_incr, 'R', "fill with test data"},
	{ "pre-demux-script", 0, POPT_ARG_STRING, &G::pre_demux_script, 0,
			"breakout before demux"
	},
	{ "progress", 'p', POPT_ARG_STRING, &G::progress, 0,
			"log to this file"
	},
	{ "corner-turn-delay", 'c', POPT_ARG_INT, &G::corner_turn_delay, 0,
			"increase this to test effect of event at corner .. 510 ish"
	},
	{
	  "exit-on-trigger-fail", 0, POPT_ARG_INT, &G::exit_on_trigger_fail, 0,
	  	  	  "force quit on trigger fail"
	},
	{ "subset", 0, POPT_ARG_STRING, &G::subset, 0, "reduce output channel count" },
	{ "sum",    0, POPT_ARG_STRING, &G::sum, 0, "sum N channels and output on another stream" },
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

static void wait_and_cleanup_sighandler(int signo);


void ident(const char* tid = "acq400_stream") {
	char fname[80];
	sprintf(fname, "/var/run/%s.%d.pid", tid, G::devnum);
	FILE* fp = fopen(fname, "w");
	if (fp == 0) {
		perror(fname);
		exit(1);
	}
	fprintf(fp, "%d", getpid());
	fclose(fp);
}

static void wait_and_cleanup(pid_t child)
{
	sigset_t  emptyset, blockset;

	if (verbose) fprintf(stderr, "wait_and_cleanup 01 pid %d\n", getpid());

	ident();
	sigemptyset(&blockset);
	sigaddset(&blockset, SIGHUP);
	sigaddset(&blockset, SIGTERM);
	sigaddset(&blockset, SIGINT);
	sigaddset(&blockset, SIGCHLD);
	sigprocmask(SIG_BLOCK, &blockset, NULL);

	Progress::instance();
	struct sigaction sa;
	sa.sa_handler = wait_and_cleanup_sighandler;
	sa.sa_flags = 0;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGHUP, &sa, NULL);

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGCHLD, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

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
			if (verbose) fprintf(stderr, "exception on stdin\n");
			finished = true;
		}else if (FD_ISSET(0, &readfds)){
			if (feof(stdin)){
				if (verbose) fprintf(stderr,"EOF\n");
				finished = true;
			}else if (ferror(stdin)){
				if (verbose) fprintf(stderr, "ERROR\n");
				finished = true;
			}else{
				char stuff[80];
				fgets(stuff, 80, stdin);

				if (verbose) fprintf(stderr, "data on stdin %s\n", stuff);
				if (strncmp(stuff, "quit", 4) == 0){
					finished = true;
				}else{
					fprintf(stderr, "options> quit\n");
				}
			}
		}else{
			if (verbose) fprintf(stderr, "out of pselect, not sure why\n");
		}
	}

	fprintf(stderr, "wait_and_cleanup exterminate\n");
        Progress::instance().setState(ST_STOP);
        kill(0, SIGTERM);
        exit(0);
}

static void hold_open(int site)
{
	char devname[80];
	char message[80];

	sprintf(devname, "/dev/acq400.%d.cs", site);
	FILE *fp = fopen(devname, "r");
	if (fp == 0){
		perror(devname);
		exit(1);
	}
	while(fread(message, 1, 80, fp)){
		chomp(message);
		bool quit = strstr(message, "ERROR") != 0;
		fprintf(stderr, "%s : \"%s\" %s\n", devname, message, quit? "QUIT ON ERROR": "OK");
		if (quit){
			break;
		}
	}
	exit(1);
}

std::vector<pid_t> holders;

static void make_aggsem() {
	G::aggsem_name = new char[80];
	sprintf(G::aggsem_name, "/acq400_stream.%d", getpid());
	G::aggsem = sem_open(G::aggsem_name, O_CREAT, S_IRWXU, 0);
	if (G::aggsem == SEM_FAILED){
		perror(G::aggsem_name);
		exit(1);
	}
	if (verbose) fprintf(stderr, "G_aggsem:%p\n", G::aggsem);
}

static void holder_wait_and_pass_aggsem() {
	int val;

	sem_getvalue(G::aggsem, &val);
	if (verbose) fprintf(stderr, "%d  %s sem:%d\n", getpid(), "before wait", val);
	if (sem_wait(G::aggsem) != 0){
		perror("sem_wait");
	}

	if (verbose) fprintf(stderr, "%d  %s\n", getpid(), "after wait");
	if (sem_post(G::aggsem) != 0){
		perror("sem_post");
	}

	if (verbose) fprintf(stderr, "%d  %s\n", getpid(), "after post");
	sem_close(G::aggsem);
	sem_unlink(G::aggsem_name);	// sem is only destroyed AFTER all users close()
}

static void hold_open(const char* sites)
{
	int *ss = G::the_sites;
	G::nsites = sscanf(sites, "%d,%d,%d,%d,%d,%d",
				ss+0, ss+1, ss+2, ss+3, ss+4, ss+5);

	if (G::nsites){
		setpgid(0, 0);
		pid_t child = fork();
		if (child != 0){
			/* original becomes reaper */
			wait_and_cleanup(child);
		}else{
			make_aggsem();

			for (int isite = 0; isite < G::nsites; ++isite ){
				child = fork();

		                if (child == 0) {
		                	syslog(LOG_DEBUG, "%d  %10s %d\n", getpid(), "hold_open", isite);
		                	holder_wait_and_pass_aggsem();
		                	hold_open(G::the_sites[isite]);
		                	assert(1);
		                }else{
		                	sched_yield();
		                	holders.push_back(child);
		                }
			}
		}
	}
	/* newly forked main program continues to do its stuff */
}

static void kill_the_holders() {
	std::vector<pid_t>::iterator it;
	for (it = holders.begin(); it != holders.end(); ++it){
		kill(*it, SIGTERM);
		if (verbose) fprintf(stderr, "kill_the_holders %d\n", *it);
	}
}

void shuffle_all_down1() {
	char *to = Buffer::the_buffers[0]->getBase();
	char *from = Buffer::the_buffers[1]->getBase();
	int len = G::bufferlen * (G::nbuffers-1);
	fprintf(stderr, "shuffle_all_down1: to:%p from:%p len:%d\n", to, from, len);
	memcpy(to,from,len);
}

template <class T>
void demux_all_down1(bool use_new) {
	T* to = use_new? new T[G::bufferlen/sizeof(T)] :
			reinterpret_cast<T*>(Buffer::the_buffers[0]->getBase());
	T* from = reinterpret_cast<T*>(Buffer::the_buffers[1]->getBase());
	const unsigned nchan = G::nchan;
	const unsigned nsam = G::bufferlen * (G::nbuffers-1) / sample_size();
	const unsigned tomask = use_new? G::bufferlen/sizeof(T)-1: 0xffffffff;

	fprintf(stderr, "demux_all_down1 nchan:%d nsam:%d ws=%d mask:%08x\n",
			nchan, nsam, sizeof(T), tomask);

	for (unsigned sample = 0; sample < nsam; ++sample){
		for (unsigned chan = 0; chan < nchan; ++chan){
			to[(chan*nsam + sample)&tomask] = from[sample*nchan + chan];
		}
	}
}
void do_shuffle_test(int level)
/* TEST MODE: time iterating all the buffers */
{
	switch(level){
	case 1:
		shuffle_all_down1();
		break;
	case 2:
	case 3:
		if (G::wordsize == 4){
			demux_all_down1<unsigned>(level&1);
		}else{
			demux_all_down1<short>(level&1);
		}
		break;

	default:
		fprintf(stderr, "shuffle_test level %d not supported\n", level);
	}
	exit(0);
}

void do_fill_ramp()
{
	unsigned* cursor = reinterpret_cast<unsigned*>(MapBuffer::get_ba0());
	unsigned* ba99 = reinterpret_cast<unsigned*>(MapBuffer::get_ba_hi());
	unsigned xx = 0;

	while(cursor < ba99){
		*cursor++ = xx += fill_ramp_incr;
	}
	exit(0);
}

static void reserve_block0();

void init_globs(void)
{
	getKnob(0, "/etc/acq400/0/NCHAN", &G::nchan);
	unsigned int data32 = false;
	getKnob(0, "/etc/acq400/0/data32", &data32);
	G::wordsize = data32? sizeof(int): sizeof(short);

	if(getenv("ES_DIAGNOSTIC")){
		G::es_diagnostic = atoi(getenv("ES_DIAGNOSTIC"));
	}
}

void init(int argc, const char** argv) {
	char* progname = new char(strlen(argv[0]));
	if (strcmp(progname, "acq400_stream_getstate") == 0){
		acq400_stream_getstate();
	}
	delete [] progname;

	init_globs();
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;
	bool fill_ramp = false;

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		case 'n':
			G::null_copy = BM_NULL;
			break;
		case BM_DEMUX:
			fprintf(stderr, "--hb0 has been superceded. quitting\n");
			exit(1);
			break;
		case 'D':
			if (G::demux == 0){
				G::buffer_mode = BM_RAW;
			}
			break;
		case 'P':
		case 'O':
			G::stream_mode = SM_TRANSIENT;
			break;
		case 'S':
			G::state_fp = fopen(G::state_file, "w");
			if (G::state_fp == 0){
				perror(G::state_file);
				exit(1);
			}
			break;
		case 'R':
			fill_ramp = true;
			break;
		case BM_TEST:
			G::buffer_mode = BM_TEST;
			break;
		case 'p':
			if (strcmp(G::progress, "-") == 0){
				Progress::instance(stdout);
			}else{
				FILE *fp = fopen(G::progress, "w");
				if (!fp){
					perror(G::progress); exit(1);
				}else{
					Progress::instance(fp);
				}
			}
		}
	}

	getKnob(G::devnum, "nbuffers",  &G::nbuffers);
	getKnob(G::devnum, "bufferlen", &G::bufferlen);

	if (G::pre || G::demux){
		reserve_block0();		// MUST get length first ..
	}

	openlog("acq400_stream", LOG_PID, LOG_USER);
	syslog(LOG_DEBUG, "hello world %s", VERID);
	const char* devc = poptGetArg(opt_context);
	if (devc){
		G::devnum = atoi(devc);
	}

	if (G::devnum == 0 && G::aggregator_sites == 0){
		static char _sites[32];
		getKnob(0, "/etc/acq400/0/sites", _sites);
		G::aggregator_sites = _sites;
		if (verbose) fprintf(stderr, "default sites:%s\n", G::aggregator_sites);
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

	root = getRoot(G::devnum);

	for (int ii = 0; ii < G::nbuffers; ++ii){
		Buffer::create(root, G::bufferlen);
	}
	if (shuffle_test){
		do_shuffle_test(shuffle_test);
	}
	if (fill_ramp){
		do_fill_ramp();
	}
}

class StreamHead {

	static bool has_pre_post_live_demux(void);
	static StreamHead* createLiveDataInstance();
protected:
	int fc;
	int fout;
	StreamHead(int _fc, int _fout = 1) : fc(_fc), fout(_fout) {
		assert(fc >= 0);
		assert(fout >= 0);
	}
	virtual ~StreamHead() {}

	virtual int getBufferId() {
		char buf[80];
		int rc = read(fc, buf, 80);

		if (rc > 0){
			buf[rc] = '\0';
			int ib = strtoul(buf, 0, 10);
			assert(ib >= 0);
			assert(ib <= G::nbuffers);
			return ib;
		}else{
			return rc;
		}
	}


public:
	virtual void stream() {
		int ib;

		if (verbose) fprintf(stderr, "StreamHead::stream()\n");

		while((ib = getBufferId()) >= 0){
			Buffer::the_buffers[ib]->writeBuffer(fout, Buffer::BO_NONE);
		}
	}
	virtual void startStream() {
		stream();
	}
	static StreamHead* instance();

};


class StreamHeadImpl: public StreamHead {
protected:
	Progress& actual;
	const int samples_buffer;

	void close() {
		kill_the_holders();
		if (G::aggsem){
		       	sem_close(G::aggsem);
		       	sem_unlink(G::aggsem_name);
		}
		if (fc){
			::close(fc);
			fc = 0;
		}
	}
	virtual ~StreamHeadImpl() {
		close();
	}

	void setState(enum STATE state) {
		if (verbose) fprintf(stderr, "StreamHeadImpl::setState(%d)\n", state);
		actual.setState(state);
		if (verbose) fprintf(stderr, "StreamHeadImpl::setState(%d) DONE\n", state);
	}

	void do_soft_trigger() {
		setKnob(0, "soft_trig", "0");
		setKnob(0, "soft_trig", "1");
		setKnob(0, "soft_trig", "0");
	}
	bool is_triggered() {
		unsigned trig = 0;
		getKnob(G::the_sites[0], "is_triggered", &trig);
		return trig;
	}
	void soft_trigger_control() {
		int repeat = 0;
		do_soft_trigger();
		while (!is_triggered()){
			if (repeat > 100){
				fprintf(stderr, "ERROR: failed to trigger\n");
				if (G::exit_on_trigger_fail){
					system("kill -9 $(cat /var/run/acq400_stream_main.0.pid)");
					exit(1);
				}
			}
			usleep(10000);
			++repeat;
			if (!is_triggered()) {
				do_soft_trigger();
			}
		}
		if (verbose || repeat){
			fprintf(stderr, "soft_trigger_control() repeat %d\n", repeat);
		}
	}

	void schedule_soft_trigger(void) {
		if (verbose) fprintf(stderr, "schedule_soft_trigger()");
		pid_t child = fork();
		if (child == 0){
			nice(2);
			sched_yield();
			soft_trigger_control();
			exit(0);
		}
	}
	static int open_feed() {
		char fname[128];
		sprintf(fname, stream_fmt, root);
		int _fc = open(fname, O_RDONLY);
		assert(_fc > 0);
		return _fc;
	}

	int f_ev;
	int nfds;
	bool event_received;
	char event_info[80];

	void startEventWatcher() {
		f_ev = open("/dev/acq400.1.ev", O_RDONLY);
		if (f_ev < 0){
			perror("/dev/acq400.1.ev");
			exit(1);
		}
		if (f_ev > fc){
			nfds = f_ev+1;
		}
	}
	void report(const char* id, int ibuf, char *esp);

	void esDiagnostic(Buffer* the_buffer, unsigned *cursor)
	{
		if (G::es_diagnostic == 0) return;

		FILE *fp = fopen("/dev/shm/es", "w");
		if (!fp){
			perror("/dev/shm/es");
		}else{
			fwrite(cursor, G::wordsize, G::nchan, fp);
			fclose(fp);
		}
		fp = fopen("/dev/shm/es5", "w");
		if (!fp){
			perror("/dev/shm/es5");
		}else{
			fwrite(cursor-2*G::nchan, G::wordsize, 5*G::nchan, fp);
			fclose(fp);
		}

		if (G::es_diagnostic > 1){
			char* buffer = new char[0x100000];
			the_buffer->copyBuffer(buffer);
			fp = fopen("/tmp/es_buffer", "w");
			if (fp){
				fwrite(buffer, 1, 0x100000, fp);
				fclose(fp);
				delete [] buffer;
			}else{
				perror("/tmp/es_buffer");
			}
		}
	}
	bool findEvent(int *ibuf, char** espp) {
		/*if (verbose) */ fprintf(stderr, "findEvent \"%s\"\n", event_info);
		unsigned long usecs;
		int b1, b2;
		int nscan = sscanf(event_info, "%lu %d %d", &usecs, &b1, &b2);

		assert(nscan == 3);
		if (b1 == -1 && b2 == -1){
			return false;
		}

		if (verbose) fprintf(stderr, "findEvent b1=%d b2=%d\n", b1, b2);

		Buffer* buffer2 = Buffer::the_buffers[b2];

		if (verbose) fprintf(stderr, "call findEvent buffer2 %d\n", buffer2->ib());
		char* esp = findEvent(buffer2);
		if (esp){
			report("b2", b2, esp);
			if (ibuf) *ibuf = b2;
			if (espp) *espp = esp;
			return true;
		}else{
			if (verbose) fprintf(stderr, "b2 not found\n");
		}

		Buffer* buffer1;
		if (b1 == -1){
			buffer1 = Buffer::the_buffers[b1 = buffer2->pred()];
			if (verbose) fprintf(stderr, "b1==-1 now set %d\n", b1);
		}else{
			buffer1 = Buffer::the_buffers[b1];
		}
		if (verbose) fprintf(stderr, "call findEvent buffer1 %d\n", buffer1->ib());
		esp = findEvent(buffer1);
		if (esp){
			report("b1", b1, esp);
			if (ibuf) *ibuf = b1;
			if (espp) *espp = esp;
			return true;
		}

		if (verbose) fprintf(stderr, "backtrack\n");
		for (int backtrack = 1;
			(buffer1 = Buffer::the_buffers[buffer1->pred()]) != buffer2;
			++backtrack){
			esp = findEvent(buffer1);
			if (esp){
				char bn[8]; sprintf(bn, "bt%d", backtrack);
				report(bn, buffer1->ib(), esp);
				if (ibuf) *ibuf = b1;
				if (espp) *espp = esp;
				return true;
			}
		}
		if (verbose) fprintf(stderr, "ERROR: not found anywhere\n");

		return false;
	}
	int countES(char* start, int len) {
		unsigned stride = G::nchan*G::wordsize/sizeof(unsigned);
		unsigned *cursor = reinterpret_cast<unsigned*>(start);
		unsigned *base = cursor;
		unsigned lenw = len/G::wordsize;
		int escount = 0;

		for (; cursor - base < lenw; cursor += stride){
			if (evX.isES(cursor)){
				if (verbose) fprintf(stderr, "FOUND: %08x %08x\n", cursor[0], cursor[4]);
				++escount;
			}
		}
		return escount;
	}
	char* findEvent(Buffer* the_buffer) {
		unsigned stride = G::nchan*G::wordsize/sizeof(unsigned);
		unsigned *cursor = reinterpret_cast<unsigned*>(the_buffer->getBase());
		unsigned *base = cursor;
		unsigned lenw = the_buffer->getLen()/G::wordsize;
		int sample_offset = 0;

		if (verbose) fprintf(stderr, "findEvent 01 base:%p lenw %d\n", base, lenw);

		for (; cursor - base < lenw; cursor += stride, sample_offset += 1){
			if (verbose > 2) fprintf(stderr, "findEvent cursor:%p\n", cursor);
			if (ev0.isES(cursor)){
				if (verbose) fprintf(stderr, "FOUND: %08x %08x\n", cursor[0], cursor[4]);
				esDiagnostic(the_buffer, cursor);
				return reinterpret_cast<char*>(cursor);
			}
		}
		if (verbose) fprintf(stderr, "findEvent 99 NOT FOUND\n");
		return 0;
	}
public:
	StreamHeadImpl(Progress& progress) : StreamHead(1234),
		actual(progress),
		samples_buffer(G::bufferlen/sample_size()),
		f_ev(0), nfds(0), event_received(0) {
		if (verbose) fprintf(stderr, "StreamHeadImpl() pid %d progress: %s\n", getpid(), actual.name);
	}
	virtual void startStream() {
		fc = open_feed();
		stream();
	}
	virtual void stream() {
		int ib;
		if (verbose) fprintf(stderr, "StreamHeadImpl::stream() :\n");

		ident("acq400_stream_headImpl");
		setState(ST_ARM);
		if (G::soft_trigger){
			schedule_soft_trigger();
		}
		while((ib = getBufferId()) >= 0){
			if (verbose) fprintf(stderr, "StreamHeadImpl::stream() : %d\n", ib);
			Buffer::the_buffers[ib]->writeBuffer(1, Buffer::BO_NONE);
			switch(actual.state){
			case ST_ARM:
				setState(ST_RUN_PRE);
			}
			actual.elapsed += samples_buffer;
		}
		setState(ST_STOP);
	}
};


class NullStreamHead: public StreamHeadImpl {

public:
	virtual void stream() {
		int ib;
		setState(ST_ARM);
		if (G::soft_trigger){
			schedule_soft_trigger();
		}
		while((ib = getBufferId()) >= 0){
			switch(actual.state){
			case ST_ARM:
				setState(ST_RUN_PRE);
			}
			printf("%d\n", ib);
			fflush(stdout);
			actual.elapsed += samples_buffer;
		}
		setState(ST_STOP);
	}
	NullStreamHead(Progress& progress) :
		StreamHeadImpl(progress)
	{}
};

class StreamHeadHB0: public StreamHeadImpl  {
protected:
	virtual void stream();

public:
       StreamHeadHB0(): StreamHeadImpl(Progress::null_progress) {
               if (G::script_runs_on_completion == 0){
                       G::script_runs_on_completion = "/tmp/ondemux_complete";
               }
       }
       virtual ~StreamHeadHB0() {
	       setKnob(0, "/etc/acq400/0/live_mode", "0");
       }
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
			Buffer* b0 = Buffer::the_buffers[ib[0]];
			Buffer* b1 = Buffer::the_buffers[ib[1]];
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
			Buffer* b1 = Buffer::the_buffers[ib[1]];
			b1->writeBuffer(1, Buffer::BO_START|Buffer::BO_FINISH);
		}else if (nscan == 1){
			Buffer* b0 = Buffer::the_buffers[ib[0]];
			b0->writeBuffer(1, Buffer::BO_START|Buffer::BO_FINISH);
		}
		if (verbose) fprintf(stderr, "UPDATE:%4d finished\n", nb);
	}
}

#define OBS_ROOT "/dev/shm/transient"
#define TOTAL_BUFFER_LIMIT	(G::nbuffers * G::bufferlen)

#define LIVE_PRE	"/etc/acq400/0/live_pre"
#define LIVE_POST	"/etc/acq400/0/live_post"

class StreamHeadLivePP : public StreamHeadHB0 {
	int pre;
	int post;
	const int sample_size;
	int* sample_interval_usecs;

	int prelen() { return pre*sample_size; }
	int postlen() { return post*sample_size; }

	void getSampleInterval();
	void startSampleIntervalWatcher();

	static bool getPram(const char* pf, int* pram)
	{
		FILE *fp = fopen(pf, "r");
		if (!fp) {
			return false;
		}
		char aline[32];
		bool rc = false;
		if (fgets(aline, 32, fp)){
			rc = sscanf(aline, "%d", pram) == 1;
		}
		fclose(fp);
		return rc;
	}
	static bool getPP(int *_pre, int* _post)
	{
		int pp[2];
		if (getPram(LIVE_PRE, pp) && getPram(LIVE_POST, pp+1)){
			if (_pre) *_pre = pp[0];
			if (_post) *_post = pp[1];
			return true;
		}else{
			return false;
		}
	}
	bool getPP(void) {
		return getPP(&pre, &post);
	}

	void waitPost() {
		int siu = 100;	/* assume min SR=10kHz, ensure data has arrived before read .. */
		if (sample_interval_usecs){
			siu = *sample_interval_usecs;
		}
		if (siu < 1000000){	/* Live PP should never sleep > 1s */
			usleep(post*siu);
		}
	}
	static bool event0_enabled(int site){
		char event_line[80];
		if (getKnob(site, "event0", event_line) == 1){
			unsigned ena = 0;
			if (sscanf(event_line, "event0=%u", &ena) == 1){
				return ena != 0;
			}
		}
		return 0;
	}

	int  _stream();
public:
	StreamHeadLivePP():
			pre(0), post(4096),
			sample_size(G::nchan*G::wordsize) {
		fprintf(stderr, "StreamHeadLivePP()\n");
		startEventWatcher();
		startSampleIntervalWatcher();

		if (verbose) fprintf(stderr, "StreamHeadLivePP: buffer[0] : %p\n",
				Buffer::the_buffers[0]->getBase());
		if (verbose) fprintf(stderr, "StreamHeadLivePP: buffer[1] : %p\n",
				Buffer::the_buffers[1]->getBase());
	}

	static bool hasPP() {
		int pp[2];

		return event0_enabled(1) && getPP(pp, pp+1) && pp[0]+pp[1] > 0;
	}
	virtual void stream();
};

bool StreamHead::has_pre_post_live_demux(void) {
	return StreamHeadLivePP::hasPP();
}

void StreamHeadLivePP::getSampleInterval()
{

	FILE *pp = popen("get.site 1 SIG:sample_count:FREQ", "r");

	int freq;
	if (fscanf(pp, "SIG:sample_count:FREQ %d", &freq) == 1 && freq > 0){
		int siu = 1000000/freq;
		if (siu != *sample_interval_usecs){
			if (abs(siu - *sample_interval_usecs) > 5){
				syslog(LOG_DEBUG, "SI change %d -> %d\n",
						*sample_interval_usecs, siu);
				*sample_interval_usecs = siu;
			}
		}
	}
	pclose(pp);
}
void StreamHeadLivePP::startSampleIntervalWatcher() {
	int rc = shmget(0xdeadbeee, sizeof(int), IPC_CREAT|0666);
	if (rc == -1){
		perror("shmget()");
		exit(1);
	}else{
		void *shm = shmat(rc, 0, 0);
		if (shm == (void*)-1){
			perror("shmat()");
			exit(1);
		}else{
			sample_interval_usecs = (int*)shm;
			*sample_interval_usecs = 100;

			if (fork() == 0){
				while (1){
					sleep(1);
					getSampleInterval();
				}
			}
		}
	}
}

int StreamHeadLivePP::_stream() {
	int rc;
	int icat = 0;
	int nb = 0;
	char* b0 = MapBuffer::get_ba_lo();
	char* b1 = MapBuffer::get_ba_hi();
	int bo1 = Buffer::BO_NONE;
	int bo2 = Buffer::BO_NONE; ;


	fprintf(stderr, "StreamHeadLivePP::stream(): f_ev %d\n", f_ev);

	// 1637099 -1 201    sample_count, hb0 hb1
	for( getPP(); (rc = read(f_ev, event_info, 80)) > 0; getPP()){
		if (pre){
			bo1 = Buffer::BO_START;
		}else{
			bo2 = Buffer::BO_START;
		}
		if (!post){
			bo1 |= Buffer::BO_FINISH;
		}else{
			bo2 |= Buffer::BO_FINISH;
		}

		event_info[rc] = '\0';
		chomp(event_info);
		if (verbose) fprintf(stderr, "fd_ev read \"%s\"\n", event_info);
		event_received = true;

		int ibuf;
		char* es;

		if (!findEvent(&ibuf, &es)){
			if (verbose) fprintf(stderr, "StreamHeadLivePP::stream() 390\n");
			continue;	// silently drop it. there will be more
		}else if (es+postlen() > Buffer::the_buffers[ibuf]->getEnd()){
			// only data in this buffer is guaranteed to be there .. drop it
			if (verbose) fprintf(stderr, "StreamHeadLivePP::stream() 395\n");
			continue;
		}
		fprintf(stderr, "StreamHeadLivePP::stream() found %d %p\n",
					ibuf, es);
		char *es1 = es + sample_size;

		// all info referenced to buffer 0!
		Buffer* buf = Buffer::the_buffers[0];
		b0 = buf->getBase();

		if (!(es - prelen() > b0 && es1 + postlen() < b1 )){
			if (verbose) fprintf(stderr, "StreamHeadLivePP::stream() 49\n");
			continue;	// silently drop it. there will be more
		}
		if (pre){
			int escount = countES(es-prelen(), prelen());
			int eslen = escount*sample_size;
			if (es - prelen() > b0 + eslen){
				if (verbose) fprintf(stderr,
					"StreamHeadLivePP::stream() 55 escount:%d\n", escount);
			}
			buf->writeBuffer(1, bo1, es - prelen() - b0 - eslen, prelen()+eslen);
		}
		if (post){
			int escount = countES(es1, postlen());
			int eslen = escount*sample_size;
			if (es1 - b0 > postlen() - eslen){
				if (verbose) fprintf(stderr,
					"StreamHeadLivePP::stream() 57 escount:%d\n", escount);
			}
			waitPost();
			buf->writeBuffer(1, bo2, es1 - b0, postlen()+eslen);
		}
		if (verbose) fprintf(stderr, "StreamHeadLivePP::stream() 69\n");
	}

	return rc;
}

void StreamHeadLivePP::stream() {
	while(1){
		int rc = _stream();
		if (rc == 0){
			continue;
		}
		if (rc == -1){
			switch(errno){
			case EINTR:
			case EAGAIN:
			default:
				break;
			}
		}
	}
}

typedef std::vector<MapBuffer*> MBUFV;
typedef std::vector<MapBuffer*>::iterator MBUFV_IT;
typedef std::vector<FILE *> FPV;
typedef std::vector<FILE *>::iterator FPV_IT;


#define TRANSOUT	OBS_ROOT"/ch"

unsigned total_buffer_limit() {

}

class Demuxer {
public:
	virtual int demux(void *start, int nbytes) = 0;
	int operator()(void *start, int nsamples){
		if (verbose) fprintf(stderr, "%s %p %d\n", __FUNCTION__, start, nsamples);
		return demux(start, nsamples);
	}
};
template <class T>
class DemuxerImpl : public Demuxer {

	struct BufferCursor {
		const int channel_buffer_bytes;	/* number of bytes per channel per buffer */
		int ibuf;
		char* base;
		char* cursor;

		int channel_remain_bytes() {
			return channel_buffer_bytes - (cursor - base);
		}
		int total_remain_bytes() {
			return channel_remain_bytes() * G::nchan;
		}
		void init(int _ibuf) {
			ibuf = _ibuf;
			base = cursor = Buffer::the_buffers[ibuf]->getBase();
			if (verbose) fprintf(stderr, "BufferCursor::init(%d) base:%p\n", ibuf, base);
		}
		BufferCursor() :
			channel_buffer_bytes(G::bufferlen/G::nchan)
		{
			init(0);
		}
	} dst;

	const int channel_buffer_sam() {
		return dst.channel_buffer_bytes/sizeof(T);
	}

	/* watch out for end of source buffer ! */
	int _demux(void* start, int nbytes);


public:
	DemuxerImpl() : dst() {
		if (verbose) fprintf(stderr, "%s %d\n", __FUNCTION__, dst.channel_buffer_bytes);
	}
	virtual ~DemuxerImpl() {

	}

	virtual int demux(void *start, int nsamples);
};


void Demuxer_report(int buffer);

template <class T>
int DemuxerImpl<T>::_demux(void* start, int nbytes){
	const int nsam = b2s(nbytes);
	const int cbs = channel_buffer_sam();
	T* pdst = reinterpret_cast<T*>(dst.cursor);
	T* psrc = reinterpret_cast<T*>(start);
	int b1 = MapBuffer::getBuffer(reinterpret_cast<char*>(psrc));

	if (verbose) fprintf(stderr, "%s (%p %d) b:%d %p = %p * nsam:%d cbs:%d\n",
			__FUNCTION__, start, nbytes, b1, pdst, psrc, nsam, cbs);

	msync(MapBuffer::ba(b1), G::bufferlen, MS_SYNC);

	for (int sam = 0; sam < nsam; ++sam){
		for (int chan = 0; chan < G::nchan; ++chan){
#ifdef BUFFER_IDENT
			if (chan == 7){
				pdst[chan*cbs+sam] = b1;
				continue;
			}
#endif
			pdst[chan*cbs+sam] = psrc[chan];
		}
		T* psrc2 = psrc + G::nchan;

		int b2 = MapBuffer::getBuffer(reinterpret_cast<char*>(psrc2));
		if (b1 != b2){
			Progress::instance().print(true, b2);
			msync(MapBuffer::ba(b2), G::bufferlen, MS_SYNC);
		}
		psrc = psrc2;
		b1 = b2;
	}
	pdst += nsam;
	dst.cursor = reinterpret_cast<char*>(pdst);
	return nbytes;
}


template <class T>
int DemuxerImpl<T>::demux(void* start, int nbytes)
/* return bytes demuxed */
{
	char* startp = reinterpret_cast<char*>(start);
	int rc = 0;

	if (verbose) fprintf(stderr, "%s 01 (%p %d)\n", __FUNCTION__, start, nbytes);

	assert(nbytes%sample_size() == 0);

	while(nbytes){
		int trb = dst.total_remain_bytes();

		if (verbose) fprintf(stderr, "%s 10 nbytes:%d trb:%d\n", __FUNCTION__, nbytes, trb);

		if (nbytes < trb){
			rc += _demux(startp, nbytes);
			nbytes = 0;
		}else{
			int nb = _demux(startp, trb);
			// assert(nb == trb);
			dst.init(dst.ibuf+1);
			nbytes -= trb;
			startp += nb;
			rc += nb;
			if (verbose) fprintf(stderr, "%s 50 rc:%d startp:%p\n", __FUNCTION__, rc, startp);
		}
	}
	if (verbose) fprintf(stderr, "%s 99  rc:%d\n", __FUNCTION__, rc);
	return rc;
}

/* @@todo .. map to shm area for external monitoring. */



long Progress::min_report_interval = MIN_REPORT_INTERVAL_MS;

Progress& Progress::instance(FILE *fp) {
	static Progress* _instance;

	/*
	if (!_instance){
		_instance = new Progress;

	}
	*/
	if (_instance == 0){
		if (verbose) fprintf(stderr, "Progress:instance() : size %d\n", sizeof(Progress));

		int rc = shmget(0xdeadbeef, sizeof(Progress), IPC_CREAT|0666);
		if (rc == -1){
			perror("shmget()");
			exit(1);
		}else{
			void *shm = shmat(rc, 0, 0);
			if (shm == (void*)-1){
				perror("shmat()");
				exit(1);
			}else{
				Progress *p = new ProgressImpl(fp? fp: stdout);
				_instance = (Progress*)shm;
				memcpy(_instance, p, sizeof(Progress));

				if (getenv("MIN_REPORT_INTERVAL_MS")){
					min_report_interval = atoi(getenv("MIN_REPORT_INTERVAL_MS"));
					fprintf(stderr,"min_report_interval set %d\n", min_report_interval);
				}
				if (verbose) fprintf(stderr,"min_report_interval set %d\n", min_report_interval);
			}
		}

	}

	if (verbose) fprintf(stderr,"Progress::instance() %p pid:%d\n", _instance, getpid());
	return *_instance;
}


static bool cleanup_done;

static void wait_and_cleanup_sighandler(int signo)
{
	if (verbose) fprintf(stderr,"wait_and_cleanup_sighandler(%d) pid:%d %s\n",
			signo, getpid, cleanup_done? "FRESH": "DONE");
	if (!cleanup_done){
		kill(0, SIGTERM);
		cleanup_done = true;
		Progress::instance().setState(ST_STOP);
		if (verbose) fprintf(stderr,"wait_and_cleanup_sighandler progress done\n");
	}
	exit(0);
}
void acq400_stream_getstate(void)
{
	Progress::instance().print();
	exit(0);
}

#define BLOG	"/etc/acq400/0/full_buffers"

class BufferLog {
	int ib0;
	bool all_full;
	FILE* blog;

public:
	BufferLog() : ib0(-1), all_full(false) {
		blog = fopen(BLOG, "w");
		if (blog == 0){
			perror(BLOG);
			exit(1);
		}
	}
	~BufferLog() {
		fprintf(blog, "\n");
		fclose(blog);
	}
	bool update(int ib){
		if (ib0 == -1){
			ib0 = ib;
		}else if (ib == ib0){
			all_full = true;
		}
		if (!all_full){
			fprintf(blog, "%03d ", ib);
		}
	}
};



class StreamHeadClient {
protected:
	int start;
	int len;

	void getStartLen(const char* def) {
		int args[2];
		switch (sscanf(def, "%d,%d", args, args+1)){
		case 2:
			start = args[0]>=1? args[0]-1: 0;
			len = args[1];
			break;
		case 1:
			start = 0;
			len = args[0];
			break;
		default:
			start = 0;
			len = G::nchan;
		}
	}

public:
	virtual void onStreamStart() = 0;
	virtual void onStreamBufferStart(int ib) = 0;
	virtual void onStreamEnd() = 0;
	virtual ~StreamHeadClient() {}
};
typedef std::vector<StreamHeadClient*>::iterator  IT;

class SumStreamHeadClient: public StreamHeadClient {

protected:
	int outfd;
public:
	SumStreamHeadClient(const char* def) {
		getStartLen(def);
		outfd = open("/dev/shm/sumstreamclient", O_WRONLY);
	}
	virtual void onStreamStart() {}
	virtual void onStreamBufferStart(int ib) {}
	virtual void onStreamEnd() {}
	virtual ~SumStreamHeadClient() {}

	static 	SumStreamHeadClient* instance(const char* def);
};

template <class T>
class SumStreamHeadClientImpl: public SumStreamHeadClient {
	int* sum_buf;
	int nbuf;
public:
	SumStreamHeadClientImpl<T>(const char* def) : SumStreamHeadClient(def)
	{
		nbuf = G::bufferlen/sample_size();
		sum_buf = new int[nbuf];
	}
	virtual void onStreamBufferStart(int ib) {
		Buffer* buffer = Buffer::the_buffers[ib];
		T* data = reinterpret_cast<T*>(buffer->getBase());
		T* end = reinterpret_cast<T*>(buffer->getEnd());
		int totchan = G::nchan;
		int sum;
		int shr = sizeof(T) == 4? 8: 0;


		for (int isam = 0; data < end; data += totchan, ++isam){
			int ic;
			for (ic = start, sum = 0; ic < start+len; ++ic){
				sum += data[ic] >> shr;
			}
			sum_buf[isam] = sum;

		}
		write(outfd, sum_buf, nbuf*sizeof(int));
	}
};

SumStreamHeadClient* SumStreamHeadClient::instance(const char* def) {
	if (G::wordsize == 4){
		return new SumStreamHeadClientImpl<int>(def);
	}else{
		return new SumStreamHeadClientImpl<short>(def);
	}
}

class SubsetStreamHeadClient: public StreamHeadClient {
protected:

public:
	SubsetStreamHeadClient(const char* def) {
		getStartLen(def);
	}
	virtual void onStreamStart() {}
	virtual void onStreamBufferStart(int ib) = 0;
	virtual void onStreamEnd() {}
	virtual ~SubsetStreamHeadClient() {}

	static SubsetStreamHeadClient* instance(const char* def);
};

template <class T>
class SubsetStreamHeadClientImpl: public SubsetStreamHeadClient {
	T* buf;
	int nbuf;
public:
	SubsetStreamHeadClientImpl<T>(const char* def) : SubsetStreamHeadClient(def)
	{
		nbuf = G::bufferlen/sample_size()*len;
		buf = new T [nbuf];
	}
	virtual void onStreamBufferStart(int ib) {
		Buffer* buffer = Buffer::the_buffers[ib];
		T* data = reinterpret_cast<T*>(buffer->getBase());
		T* end = reinterpret_cast<T*>(buffer->getEnd());
		int totchan = G::nchan;
		int nwrite = len*sizeof(T);

		for (int isam = 0; data < end; data += totchan, ++isam){
			memcpy(buf+isam*len, data+start, len*sizeof(T));
		}
		write(1, buf, nbuf*sizeof(T));
	}
};

SubsetStreamHeadClient* SubsetStreamHeadClient::instance(const char* def)
{
	if (G::wordsize == 4){
		return new SubsetStreamHeadClientImpl<int>(def);
	}else{
		return new SubsetStreamHeadClientImpl<short>(def);
	}
}
struct Segment {
	char* base; int len;
	Segment(char* _base, int _len):
		base(_base), len(_len) {}
};

typedef vector<Segment>::iterator SegmentIterator;

template <class T>
void dump(T* src, int nwords){
	char cmd[80];
	sprintf(cmd, "hexdump -ve '%d/%d \"%%08x \" \"\\n\" ' >/tmp/es",
						G::nchan, sizeof(T));
	FILE *pp = popen(cmd, "w");
	fwrite(src, sizeof(T), nwords, pp);
	pclose(pp);
}
class BufferDistribution {

private:
	vector<Segment> segments;

	vector<Segment>& _getSegments();
public:
	int ibuf;
	char* esp;
	char* ba_lo;
	char* ba_hi;
	int tailroom;
	int headroom;
	bool pre_fits;
	bool post_fits;

	BufferDistribution(int _ibuf, char *_esp) :
		ibuf(_ibuf), esp(_esp),
		ba_lo(MapBuffer::get_ba_lo()),
		ba_hi(MapBuffer::get_ba_hi()),
		tailroom(b2s(esp - ba_lo)),
		headroom(b2s(ba_hi - esp)),
		pre_fits(tailroom >= G::pre),
		post_fits(headroom >= G::post)
	{}
	void show() {
		if (pre_fits&&post_fits){
			fprintf(stderr, "linear: %p |%p| %p\n",
				esp - s2b(G::pre), esp, esp + s2b(G::post));
			fprintf(stderr, "buffers:%s | %s | %s\n",
			MapBuffer::listBuffers(esp - s2b(G::pre), esp),
			MapBuffer::listBuffers(esp, esp),
			MapBuffer::listBuffers(esp, esp + s2b(G::post)));
			if (verbose > 1){
				fprintf(stderr, "buffers:%s | %s | %s\n",
				MapBuffer::listBuffers(esp - s2b(G::pre), esp, true),
				MapBuffer::listBuffers(esp, esp, true),
				MapBuffer::listBuffers(esp, esp + s2b(G::post)), true);
			}

			if (G::wordsize == 4){
				dump<unsigned>(reinterpret_cast<unsigned*>(esp), G::nchan);
			}else{
				dump<unsigned>(reinterpret_cast<unsigned*>(esp), G::nchan/2);
			}

		}else if (!pre_fits){
			fprintf(stderr, "precorner: %p-%p, %p |%p| %p\n",
					ba_hi-s2b(G::pre-tailroom), ba_hi,
					ba_lo,
					esp,
					esp + s2b(G::post));
		}else{
			fprintf(stderr, "postcorner: %p |%p| %p %p-%p\n",
					esp - s2b(G::pre),
					esp,
					ba_hi,
					ba_lo, ba_lo + s2b(G::post-headroom));
		}
	}
	void report() {
		FILE *fp = fopen("/dev/shm/estime", "w");
		if (fp == 0){
			perror("/dev/shm/estime");
			return;
		}
		fprintf(fp, "%d\n", reinterpret_cast<unsigned*>(esp)[4]);
		fclose(fp);
		fp = fopen("/dev/shm/esbin", "w");
		if (fp == 0){
			perror("/dev/shm/esbin");
			return;
		}
		fwrite(esp, 1, sample_size(), fp);
		fclose(fp);
	}


	enum BD_MODE { BD_LINEAR, BD_PRECORNER, BD_POSTCORNER };

	enum BD_MODE mode() const {
		if (pre_fits&&post_fits){
			return BD_LINEAR;
		}else if (!pre_fits){
			return BD_PRECORNER;
		}else if (!post_fits){
			return BD_POSTCORNER;
		}else{
			assert(pre_fits||post_fits);
		}
	}

	const char* showMode() {
		switch(mode()){
		case BD_LINEAR: 	return "BD_LINEAR";
		case BD_PRECORNER:	return "BD_PRECORNER";
		case BD_POSTCORNER:	return "BD_POSTCORNER";
		default:
			return "ERROR bad mode";
		}
	}

	void showSegments() {
		int seg = 0;
		for (SegmentIterator it = getSegments().begin();
				it != getSegments().end(); ++it, ++seg){
			fprintf(stderr, "BufferDistribution segments %d %p %d\n",
					seg, (*it).base, (*it).len);
		}
	}
	vector<Segment>& getSegments() {
		if (segments.size()){
			return segments;
		}else{
			return _getSegments();
		}
	}
};

void StreamHeadImpl::report(const char* id, int ibuf, char *esp){
	if (!G::report_es) return;
	fprintf(stderr, "StreamHeadPrePost::report: buffer:%s [%d] esp:%p\n",
			id, ibuf, esp);
	fprintf(stderr, "Buffer length bytes: %d\n", G::bufferlen);
	fprintf(stderr, "Buffer length samples: %d\n", Buffer::samples_per_buffer());
	fprintf(stderr, "Prelen: %d\n", G::pre);
	fprintf(stderr, "Postlen: %d\n", G::post);

	BufferDistribution(ibuf, esp).show();
}

/*
 * Buffer Memory:
 *
 * BD_LINEAR:
 * Before:
 *            R R R R S S S
 * |-|-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
 * 0-L-----------------------------------------------H1
 * After:
 *  R R R R S S S
 *
 * BD_PRECORNER:
 * Before:
 *    R R S S S                                     R
 * |-|-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
 * 0-L-----------------------------------------------H1
 * After:
 *  R R R R S S S
 *  That works, but this will fail:
  * Before:
 *    R R S S S                                   R R
 * |-|-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
 * 0-L-----------------------------------------------H1
 * After:
 *  R R R R S S S
 *  Ouch!. OK, doesn't matter if it's slow, because it's an oddball.
 */

vector<Segment>& BufferDistribution::_getSegments() {
	int prebytes = s2b(G::pre);
	int postbytes = s2b(G::post);
	int nb;

	if (verbose) fprintf(stderr, "BufferDistribution::_getSegments() mode %s\n", showMode());
	switch (mode()){
	case BD_LINEAR:
		segments.push_back(Segment(esp-prebytes, prebytes));
		if (G::show_es){
			segments.push_back(Segment(esp, sample_size()));
		}
		segments.push_back(Segment(esp+sample_size(), postbytes));
		break;
	case BD_PRECORNER:
		nb = s2b(G::pre-tailroom);
		segments.push_back(Segment(ba_hi-nb, nb));
		nb = s2b(tailroom);
		segments.push_back(Segment(esp-nb, nb));
		if (G::show_es){
			segments.push_back(Segment(esp, sample_size()));
		}
		segments.push_back(Segment(esp+sample_size(), postbytes));
		break;
	case BD_POSTCORNER:
		segments.push_back(Segment(esp-prebytes, prebytes));
		if (G::show_es){
			segments.push_back(Segment(esp, sample_size()));
		}
		nb = s2b(headroom-1);
		segments.push_back(Segment(esp+sample_size(), nb));
		segments.push_back(Segment(ba_lo, postbytes-nb));
		break;
	default:
		assert(0);
	}

	if (verbose) showSegments();

	return segments;
}
class BLT {
	const char* base;
	char* cursor;
public:
	BLT(char* _cursor) :
		base(_cursor),
		cursor(_cursor)
	{}
	virtual int operator() (char* from, int nbytes){
		if (verbose) fprintf(stderr, "blt() %p %p %d\n", cursor, from, nbytes);
		memcpy(cursor, from, nbytes);
		cursor += nbytes;
		return cursor - base;
	}
};



/*
 *  @@todo buffer_start
 *  pre==0, nodemux : ba0
 *  pre==0, demux   : ba1
 *  pre!=0, nodemux : epos - pre
 *  pre!=0, nodemux : epos - pre
 */

#define NOTIFY_HOOK "/dev/acq400/data/.control"


class Event0 {
	char event[80];
	bool disabled;

	void setEvent0(const char* triplet){
		char cmd[80];
		sprintf(cmd, "set.site 1 event0=%s\n", triplet);
		if (verbose) fprintf(stderr, cmd);
		FILE *pp = popen(cmd, "r");
		fgets(cmd, 80, pp);
		pclose(pp);
	}
public:
	Event0() : disabled(false) {
		char cmd[80];
		sprintf(cmd, "get.site 1 event0");
		FILE *pp = popen(cmd, "r");
		if (fscanf(pp, "event0=%s", event) != 1){
			fprintf(stderr, "fscanf failed\n");
		}
		pclose(pp);
	}
	void disable() {
		if (!disabled){
			setEvent0("0,0,0");
			disabled = true;
		}
	}
	void enable() {
		if (disabled){
			setEvent0(event);
			disabled = false;
		}
	}
};

class StreamHeadWithClients: public StreamHeadImpl {
protected:
	vector <StreamHeadClient*> peers;

public:
	StreamHeadWithClients(Progress& progress) :
			StreamHeadImpl(progress)
	{}
	void addClient(StreamHeadClient* pp){
		if (verbose) fprintf(stderr, "append %p\n", pp);
		peers.push_back(pp);
	}
	virtual void stream() {
		int ib;
		if (verbose) fprintf(stderr, "StreamHeadImpl::stream() :\n");

		ident("acq400_stream_headImpl");
		setState(ST_ARM);

		for (IT it = peers.begin(); it != peers.end(); ++it){
			(*it)->onStreamStart();
		}

		if (G::soft_trigger){
			schedule_soft_trigger();
		}
		while((ib = getBufferId()) >= 0){
			if (verbose) fprintf(stderr, "StreamHeadImpl::stream() : %d\n", ib);

			for (IT it = peers.begin(); it != peers.end(); ++it){
				if (verbose > 1) fprintf(stderr, "call peer %p %p\n", *it, this);
				(*it)->onStreamBufferStart(ib);
			}
			switch(actual.state){
			case ST_ARM:
				setState(ST_RUN_PRE);
			}
			actual.elapsed += samples_buffer;
		}
		setState(ST_STOP);

		for (IT it = peers.begin(); it != peers.end(); ++it){
			(*it)->onStreamEnd();
		}
	}
};
class StreamHeadPrePost: public StreamHeadWithClients, StreamHeadClient  {
protected:
	int pre;
	int post;

	int total_bs;
	int nobufs;

	bool cooked;
	char* typestring;
	Event0 event0;


	/* COOKED=1 NSAMPLES=1999 NCHAN=128 >/dev/acq400/data/.control */

	void notify_result() {
		char resbuf[128];
		sprintf(resbuf, "COOKED=%d NSAMPLES=%d NCHAN=%d TYPE=%s\n",
				cooked? 1:0, G::pre+G::post, G::nchan,
				G::wordsize==2? "SHORT": "LONG");
		if (verbose) fprintf(stderr, "notify_result() \"%s\"\n", resbuf);
		FILE* fp = fopen(NOTIFY_HOOK, "w");
		if (fp == 0) {
			perror(NOTIFY_HOOK);
			exit(1);
		}
		fprintf(fp, resbuf);
		fclose(fp);
	}
	virtual void postProcess(int ibuf, char* es) {
		BLT blt(MapBuffer::get_ba0());
		if (pre){
			if (verbose) fprintf(stderr,
					"StreamHeadPrePost::postProcess() pre\n");
			BufferDistribution bd(ibuf, es);

			bd.report();
			if (verbose) bd.show();

			for (SegmentIterator it = bd.getSegments().begin();
				it != bd.getSegments().end(); ++it   ){
				blt((*it).base, (*it).len);
			}
		}else{
			if (verbose) fprintf(stderr,
					"StreamHeadPrePost::postProcess() pre==0\n");
			/* probably redundant. pre==0, no reserve */
			if (es != 0 && es != MapBuffer::get_ba0()){
				blt(es, s2b(G::post));
			}
		}
	}
	virtual void onStreamStart() 		 {}
	virtual void onStreamBufferStart(int ib) {
		if (verbose>1) fprintf(stderr, "yo: onStreamBufferStart %d\n", ib);
	}
	virtual void onStreamEnd() {
		if (getenv("ACQ400_STREAM_DEBUG_STREAM_END")){
			verbose = atoi(getenv("ACQ400_STREAM_DEBUG_STREAM_END"));
		}
		setState(ST_POSTPROCESS); actual.print();

		if (pre){
			int ibuf;
			char* es;

			if (findEvent(&ibuf, &es)){
				postProcess(ibuf, es);
			}else{
				fprintf(stderr, "ERROR EVENT NOT FOUND, DATA NOT VALID\n");
			}
		}else{
			postProcess(0, MapBuffer::get_ba_lo());
		}
		notify_result();
	}

	virtual int getBufferId() {
		char buf[80];
		fd_set readfds;
		fd_set readfds0;
		int rc;

		FD_ZERO(&readfds);
		FD_SET(fc, &readfds);
		FD_SET(f_ev, &readfds);

		if (verbose>1) fprintf(stderr, "fc %d f_ev %d nfds %d\n", fc, f_ev, nfds);

		for(readfds0 = readfds;
			(rc = select(nfds, &readfds, 0, 0, 0)) > 0; readfds = readfds0){
			if (verbose>1) fprintf(stderr, "select returns  %d\n", rc);

			if (FD_ISSET(f_ev, &readfds)){
				if (verbose>1) fprintf(stderr, "FD_ISSET f_ev %d\n", f_ev);
				/* we can only handle ONE EVENT */
				if (!event_received){
					rc = read(f_ev, event_info, 80);
					if (rc <= 0){
						syslog(LOG_DEBUG, "read error  %d\n", rc);
					}else{
						event_info[rc] = '\0';
						chomp(event_info);
						if (verbose) fprintf(stderr, "fd_ev read \"%s\"\n", event_info);
						event_received = true;
						FD_CLR(f_ev, &readfds0);
					}
				}
			}
			if (FD_ISSET(fc, &readfds)){
				if (verbose>2) fprintf(stderr, "FD_ISSET fc %d\n", fc);
				rc = read(fc, buf, 80);

				if (rc > 0){
					buf[rc] = '\0';
					int ib = atoi(buf);
					assert(ib >= 0);
					assert(ib <= G::nbuffers);
					return ib;
				}else{
					return -1;
				}
			}
		}

		if (rc < 0){
			syslog(LOG_DEBUG, "select error  %d\n", rc);
		}
		return -1;
	}
	void streamCore() {
		BufferLog blog;
		int ib;
		int post_run_over_one = 0;
		struct timespec finish_time;
		int buffers_over_pre = 0;

		while((ib = getBufferId()) >= 0){
			blog.update(ib);

			if (verbose > 1) fprintf(stderr, "streamCore %d 01\n", ib);

			switch(actual.state){
			case ST_ARM:
				setState(pre? ST_RUN_PRE: ST_RUN_POST);
			}

			for (IT it = peers.begin(); it != peers.end(); ++it){
				if (verbose > 1) fprintf(stderr, "call peer %p %p\n", *it, this);
				(*it)->onStreamBufferStart(ib);
			}

			if (verbose > 1) fprintf(stderr, "done with peers\n");
			switch(actual.state){
			case ST_RUN_PRE:
				if (verbose>1) fprintf(stderr, "ST_RUN_PRE\n");
				if (actual.pre < pre){
					if (actual.pre + samples_buffer < pre){
						actual.pre += samples_buffer;
					}else{
						actual.pre = pre;
					}
				}
				if (actual.pre >= pre){
					if (event_received){
						clock_gettime(CLOCK_REALTIME, &finish_time);
						finish_time.tv_sec += 1;
						setState(ST_RUN_POST);
					}else{
						if (++buffers_over_pre > G::corner_turn_delay){
							event0.enable();
						}
					}
				}
				break;
			case ST_RUN_POST:
				if (verbose>1) fprintf(stderr, "ST_RUN_POST\n");
				actual.post += samples_buffer;
				if (actual.post > post){
					actual.post = post;
					actual.elapsed += samples_buffer;
					/* always overrun by one buffer to ensure good data
					 * when ES is adjusted
					 */
					if (post_run_over_one++){
						struct timespec now;
						clock_gettime(CLOCK_REALTIME, &now);
						if (timespec_subtract(0, &finish_time, &now)){
							setState(ST_POSTPROCESS);
							if (verbose) fprintf(stderr,
								"streamCore() quits, extra %d\n",
								post_run_over_one);
							return;
						}
					}
				}
				break;
			default:
				if (verbose) fprintf(stderr, "dodgy default %d\n", actual.state);
			}
			actual.elapsed += samples_buffer;
			actual.print(PRINT_WHEN_YOU_CAN);

			if (verbose>1) fprintf(stderr, "streamCore() %d 99\n", ib);
		}

		if (verbose) fprintf(stderr, "streamCore() ERROR bad bufferId %d\n", ib);
	}






	unsigned workbackfrom(unsigned evp) {
		assert(0);
		return 0;
	}

#define RSV	"/dev/acq400.0.rsv"

	void estop() {
		setKnob(0, "estop", "1");
	}
public:
	StreamHeadPrePost(Progress& progress, int _pre, int _post) :
		StreamHeadWithClients(progress),
		pre(_pre), post(_post),
		cooked(false)
		{
		actual.status_fp = stdout;
		if (verbose) fprintf(stderr, "StreamHeadPrePost()\n");
		setState(ST_STOP);
		int total_bs = (pre+post)*sample_size() + G::bufferlen;
		nfds = fc+1;

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
		peers.push_back(this);

		if (pre != 0){
			reserve_block0();
		}
		startEventWatcher();
	}



	void stream() {
		setState(ST_ARM);
		for (IT it = peers.begin(); it != peers.end(); ++it){
			(*it)->onStreamStart();
		}

		if (G::soft_trigger){
			event0.disable();
			schedule_soft_trigger();
		}
		streamCore();
		estop();
		for (IT it = peers.begin(); it != peers.end(); ++it){
			(*it)->onStreamEnd();
		}
		setState(ST_STOP);
		close();
	}



	static void reserve_block0 () {

		if (verbose) fprintf(stderr, "%s : 01 %s\n", __FUNCTION__, RSV);
		if (!MapBuffer::hasReserved()){
			int fd = open(RSV, O_RDONLY);
			MapBuffer::reserve();
			/* leak! : open, hold, release on quit */

			if (verbose) fprintf(stderr, "%s : %d\n", __FUNCTION__, fd);
			if (fd < 0){
				perror(RSV);
			}
			assert(fd >= 0);
		}

	}
};

static void reserve_block0() {
	StreamHeadPrePost::reserve_block0();
}


class SubrateStreamHead: public StreamHead {
	static int createOutfile() {
		return ::createOutfile("/dev/shm/subrate");
	}
public:
	SubrateStreamHead():
		StreamHead(open("/dev/acq400.0.bq", O_RDONLY), 1) {
		FILE *fp = fopen("/var/run/acq400_stream.bq.pid", "w");
		fprintf(fp, "%d\n", getpid());
		fclose(fp);
		close(fout);		/* we open it every time .. */
		BufferCloner::cloneBuffers<OversamplingBufferCloner>();
	}
	virtual ~SubrateStreamHead() {
	}
	virtual void stream() {
		for (int ib; (ib = getBufferId()) >= 0; ){
			if (verbose){
				fprintf(stderr,
				  "SubrateStreamHead::stream:[%d] pid:%d\n",
				ib, getpid());
			}
			fout = createOutfile();
			Buffer::the_buffers[ib]->writeBuffer(fout, Buffer::BO_NONE);
			close(fout);
			if (verbose){
				fprintf(stderr, "SubrateStreamHead::stream:%d 99\n",
						ib);
			}
		}
	}
};


class DemuxingStreamHeadPrePost: public StreamHeadPrePost  {
	char *cursor;
	void *event_cursor;	/* mark where event was detected */

	std::vector <MapBuffer*> outbuffers;
	Demuxer& demuxer;

	virtual void postProcess(int ibuf, char* es) {
		if (G::pre_demux_script){
			system(G::pre_demux_script);
		}
		if (verbose) fprintf(stderr, "**********************\n");
		if (verbose) fprintf(stderr, "%s %d %p\n", __FUNCTION__, ibuf, es);
		if (pre){
			BufferDistribution bd(ibuf, es);
			bd.report();

			for (SegmentIterator it = bd.getSegments().begin();
				it != bd.getSegments().end(); ++it   ){
				demuxer((*it).base, (*it).len);
			}
		}else{
			demuxer(es, s2b(G::post));
		}
		cooked = true;
	}

public:
	DemuxingStreamHeadPrePost(Progress& progress, Demuxer& _demuxer, int _pre, int _post) :
		StreamHeadPrePost(progress, _pre, _post),
		demuxer(_demuxer) {
		if (verbose) fprintf(stderr, "%s\n", __FUNCTION__);
		reserve_block0();
	}
};

void waitHolders() {
	for (int isite = 0; isite < G::nsites; ++isite){
		unsigned holder_ready = 0;
		for(int pollcat = 0;; ++pollcat, usleep(10000)){
			if (getKnob(G::the_sites[isite], "continuous_reader",
					&holder_ready) != 1){
				fprintf(stderr, "ERROR: getKnob continuous_reader\n");
				exit(1);
			}
			if (holder_ready){
				break;
			}

			if (pollcat > 100){
				fprintf(stderr, "ERROR:waitHolders: timeout\n");
				exit(1);
			}
		}
	}
}

StreamHead* StreamHead::createLiveDataInstance()
{
	ident("acq400_stream_hb0");

	for (nb_cat = 1;
	     nb_cat*G::bufferlen/(G::nchan*G::wordsize) < G::nsam; ++nb_cat){
		;
	}
	BufferCloner::cloneBuffers<DemuxBufferCloner>();
	stream_fmt = "%s.hb0";

	bool live_pp = has_pre_post_live_demux();
	setKnob(0, "/etc/acq400/0/live_mode", live_pp? "2": "1");
	if (live_pp){
		return new StreamHeadLivePP;
	}else{
		return new StreamHeadHB0;
	}
}

void setEventCountLimit(int limit)
{
	setKnob(0,
	"/sys/module/acq420fmc/parameters/acq400_event_count_limit", limit);
}
StreamHead* StreamHead::instance() {
	static StreamHead* _instance;

	if (_instance == 0){

		setEventCountLimit(G::stream_mode == SM_TRANSIENT);

		if (G::oversampling && fork() == 0){
			_instance = new SubrateStreamHead;
			return _instance;
		}

		if (fork() == 0){
			_instance = createLiveDataInstance();
			return _instance;
		}

		ident("acq400_stream_main");
		if (G::stream_mode == SM_TRANSIENT){
			if (G::buffer_mode == BM_PREPOST && G::demux){
				Demuxer *demuxer;
				if (G::wordsize == 4){
					demuxer = new DemuxerImpl<int>;
				}else{
					demuxer = new DemuxerImpl<short>;
				}
				_instance = new DemuxingStreamHeadPrePost(
					Progress::instance(), *demuxer, G::pre, G::post);
			}else{
				_instance = new StreamHeadPrePost(
					Progress::instance(), G::pre, G::post);
			}
		}else{
			if (G::null_copy){
				G::buffer_mode = BM_NULL;
			}
			switch(G::buffer_mode){
			case BM_NULL:
				_instance = new NullStreamHead(Progress::instance());
				break;
			default:
				if (G::sum || G::subset){
					StreamHeadWithClients * sh = new StreamHeadWithClients(Progress::instance());
					if (G::sum){
						sh->addClient(SumStreamHeadClient::instance(G::sum));
					}
					if (G::subset){
						sh->addClient(SubsetStreamHeadClient::instance(G::subset));
					}
					_instance = sh;
				}else{
					_instance = new StreamHeadImpl(Progress::instance());
				}
				break;
			}
		}
	}
	if (G::aggsem){
		if (sem_post(G::aggsem) != 0){
			perror("sem_post");
		}
	}
	waitHolders();
	if (verbose) fprintf(stderr, "StreamHead::instance() %p\n", _instance);
	return _instance;
}

int main(int argc, const char** argv)
{
	init(argc, argv);
	StreamHead::instance()->startStream();
	return 0;
}
