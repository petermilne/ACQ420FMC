/* ------------------------------------------------------------------------- */
/* multivent.cpp  D-TACQ ACQ400 FMC  DRIVER handle multiple events.
 * field the event, locate in memory, extract and save
 * Project: ACQ420_FMC
 * Created: 10 August 2019  			/ User: pgm
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2019 Peter Milne, D-TACQ Solutions Ltd         *
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
\* ------------------------------------------------------------------------- */

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

#include <syslog.h>

#define VERID	"B1000"

#define NCHAN	4

#include <semaphore.h>
#include <syslog.h>


#include "local.h"		/* chomp() hopefully, not a lot of other garbage */
#include "knobs.h"

#include "File.h"
#include "Knob.h"


using namespace std;


namespace G {
	unsigned sample_size = sizeof(unsigned);	// bytes per sample
	unsigned sample_count_offset;
	unsigned offset = 0;
	int devnum = 0;					// full system.
	int verbose;
	unsigned fill_len;					// buffer not always fill to the top0
	unsigned spb;					// samples per buffer
	FILE* fp_out = stdout;
	FILE* fp_in = stdin;
	int stub;
	int max_events;					// quit after this many events.

	unsigned pre_samples;				// user spec pre samples
	unsigned post_samples;

	unsigned buffers_pre = 1;
	unsigned buffers_post;
	int show_events;
};

unsigned SCIX() {
	return G::sample_count_offset/sizeof(unsigned);
}

#include "Buffer.h"

struct poptOption opt_table[] = {
	{ "sample-size", 'S', POPT_ARG_INT, &G::sample_size, 0,
			"bytes per sample [deprecated]"
	},
	{ "max-events", 'M', POPT_ARG_INT, &G::max_events, 0,
			"maximum number of events to record"
	},
	{ "stub", 0, POPT_ARG_INT, &G::stub, 0, "stub work action" },
	{
	  "verbose", 'v', POPT_ARG_INT, &G::verbose, 0, "debug"
	},
	{
	  "show-events", 0, POPT_ARG_INT, &G::show_events, 0, "show events"
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};

#define PRED(ib) ((ib)==0? Buffer::nbuffers: (ib)-1)

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

int Buffer::create(const char* root, int _buffer_len)
{
	char* fname = new char[128];
	sprintf(fname, "%s.hb/%03d", root, Buffer::last_buf);

	the_buffers.push_back(new MapBuffer(fname, _buffer_len));
	return 0;
}

int G_nsamples;

/*
acq2106_112> grep ^050, /proc/driver/acq400/0/buffers
050,e1800000,0x21800000,0x400000,0
*/
unsigned getSpecificBufferlen(int ibuf)
{
	char cmd[128];
	sprintf(cmd, "grep ^%03d, /proc/driver/acq400/0/buffers", ibuf);
	unsigned bl = 0;
	FILE * pp = popen(cmd, "r");
	if (fscanf(pp, "%*d,%*x,%*x,%x,%*d", &bl) == 1){
		fprintf(stderr, "success bl:0x%08x\n", bl);
	}else{
		fprintf(stderr, "FAIL: %s\n", cmd);
	}
	pclose(pp);
	return bl;
}


class Stopwatch {
	const char* name;
	clock_t t0;
public:
	Stopwatch(const char* _name): name(_name), t0(clock()) {}
	~Stopwatch() {
		clock_t t1 = clock();
		float dt = (t1 - t0);
		printf("%s %.1f usec\n", name, dt/CLOCKS_PER_SEC*1000000);
	}
};
class EventInfo {
//	864 269 270 OK 0x00008000 546717752
public:
	int id;
	int b0;
	int b1;
	unsigned status;
	unsigned count;
	int nf;
	const char* def;
	EventInfo(const char* _def): id(0), b0(0), b1(0), status(0), count(0), def(_def){
		nf = sscanf(def, "%d %d %d OK %x %u", &id, &b0, &b1, &status, &count);
		if (b0 == -1) b0 = b1;
	}
	EventInfo(void) {
		memset(this, 0, sizeof(EventInfo));
	}
	void print(void){
		printf("id=%d b0=%d b1=%d count=%u\n", id, b0, b1, count);
	}
};
class EventHandler {
	int site;
	int offset;
	int fd;
	char* fnameb;
	const char *fname;
	static vector<EventHandler*> handlers;	
	typedef vector<EventHandler*>::iterator EHI;
	int recursion;
	static unsigned evt_sc;
	EventInfo pe;

	static void store_latch(void) {
		struct stat statbuf;
		if (stat("/etc/acq400/1/evt_sc_latch", &statbuf) == 0){
			Knob evt_sc_latch("/etc/acq400/1/evt_sc_latch");
			evt_sc = 0;
			evt_sc_latch.get(&evt_sc);

		}
	}
protected:
	EventHandler(int _site, int _offset): site(_site), offset(_offset)
	{
		fnameb = new char[80];
		sprintf(fnameb, "%s.ev", getRoot(site));
		fname = fnameb;
		fd = open(fname, O_RDONLY);
	}
	EventHandler(int _site, const char* _fname, int _offset): site(_site), offset(_offset)
	{
		fname = _fname;
		fd = open(fname, O_RDONLY);
	}
	virtual ~EventHandler() {
		close(fd);
		delete [] fname;
	}
	void stash(EventInfo ei) {
		char fname[80];

		sprintf(fname, "/tmp/event-%d-%u-0x%08x.dat", site, ei.count, ei.status);
		FILE *fp = fopen(fname, "w");
		Buffer* b0 = Buffer::the_buffers[ei.b0];

		for (unsigned ib_pre = 0; ib_pre < G::buffers_pre; ++ib_pre){
			Buffer* bpred = Buffer::the_buffers[b0->pred()];
			for (unsigned ii = ib_pre+1; ii < G::buffers_pre; ++ii){
				bpred = Buffer::the_buffers[bpred->pred()];
			}
			fwrite(dynamic_cast<MapBuffer*>(bpred)->getBase(), 1, G::fill_len, fp);
		}

		for (unsigned ib_post = 0; ib_post <= G::buffers_post; ++ib_post){
			if (ib_post){
				usleep(40000);			/* 4MB/96 = 40msec WORKTODO */
			}
			fwrite(dynamic_cast<MapBuffer*>(b0)->getBase(), 1, G::fill_len, fp);
			b0 = Buffer::the_buffers[b0->succ()];
		}
		fclose(fp);
	}
	virtual int action(EventInfo ei) {
		if (G::stub){
			printf("STUB [%d]: %s: b0=%d def=%s\n", recursion, fname, ei.b0, ei.def);
			return 0;
		}
		if (ei.nf < 5){
			fprintf(stderr, "ERROR: bad eventInfo\n");
			return -1;
		}
		if (ei.count == 4294967295){
			fprintf(stderr, "ERROR: overflow count, reject %s\n", ei.def);
			return -1;
		}
		if (++recursion > 5){
			fprintf(stderr, "ERROR: maxrecursion, let it get away\n");
			recursion = 0;
			return -1;
		}
		if (ei.nf != 5){
			fprintf(stderr, "ERROR: eventInfo INVALID\n");
			return -1;
		}else{
			ei.print();
		}
		if (pe.nf){
			if ((ei.b0 != -1 && (pe.b1 == ei.b0 || pe.b0 == ei.b0)) ||
			    (ei.b1 != -1 && (pe.b1 == ei.b1 || pe.b0 == ei.b1))    ){
				fprintf(stderr, "WARNING: new event, old buffer: %u discard\n", ei.b0);
				return -1;
			}
		}
		if (G::verbose) printf("previous:%d current:%d\n", pe.b0, ei.b0);

		char* b0_base = Buffer::the_buffers[ei.b0]->getBase();

		unsigned* b0_counts = (unsigned*)b0_base;
		unsigned c0 = b0_counts[SCIX()];

		if (G::verbose) printf("we have base %p count:%u c0:%u\n", b0_base, ei.count, c0);

		if (ei.count < c0){
			ei.b0 -= 1; if (ei.b0 == -1) ei.b0 = Buffer::nbuffers-1;
			if (G::verbose) printf("go back %d\n", ei.b0);
			return action(ei);
		}

		unsigned isam = ei.count - c0;
		if (isam >= G::spb){
			ei.b0 += 1; if (ei.b0 >= (int)Buffer::nbuffers) ei.b0 = 0;
			if (G::verbose) printf("go forward %d\n", ei.b0);
			return action(ei);
		}


		printf("DO IT [%d]: %s: %d %s\n", recursion, fname, ei.b0, ei.def);
#if 0
		FILE *pp = popen("hexdump -e '16/2 \"%04x,\" 1/4 \"%08x,\" 1/4 \"%u\\n\"'", "w");
		fwrite(b0_base+ (isam-4)*G::sample_size, 1, G::sample_size*9, pp);
		pclose(pp);
#else
		stash(ei);

		fprintf(stderr, "evt_sc_latch:%u %s isr:%u delta:%u\n",
			evt_sc, evt_sc>ei.count? ">": "<",
			ei.count, evt_sc>ei.count? evt_sc-ei.count: ei.count-evt_sc);
#endif
		pe = ei;
		recursion = 0;
		return 0;
	}

public:
	static bool create(int _site);
	static bool create(int _site, const char* _fname);

       	static int poll() {
		sigset_t emptyset;
		struct timespec pto = {};
		fd_set exceptfds, readfds;
		char event_info[128];
		int fd_max = -1;
		int rc;
		
		sigemptyset(&emptyset);
		pto.tv_sec = 5;
		FD_ZERO(&exceptfds);
		FD_ZERO(&readfds);

                for (EHI it = handlers.begin(); it != handlers.end(); ++it){
			int fd = (*it)->fd;
			FD_SET(fd, &exceptfds);
			FD_SET(fd, &readfds);
			if (fd > fd_max){
				fd_max = fd;
			}
		}

		rc = pselect(fd_max+1, &readfds, NULL, &exceptfds, &pto, &emptyset);
		if (rc < 0){
			syslog(LOG_ERR, "ERROR: pselect() fail %d\n", errno);
		}else if (rc > 0){
			store_latch();

			Stopwatch w("ActionTimer");
			for (EHI it = handlers.begin(); it != handlers.end(); ++it){
				int fd = (*it)->fd;
				if (FD_ISSET(fd, &readfds)){
					if ((rc = read(fd, event_info, 80)) <= 0){
						syslog(LOG_ERR, "ERROR read returned %d\n", rc);
					}else{
						event_info[rc] = '\0';
						chomp(event_info);
						if (G::show_events){
							printf("%s:%s\n", (*it)->fname, event_info);
						}
					}
					rc = (*it)->action(event_info);
				}
				if (FD_ISSET(fd, &exceptfds)){
					syslog(LOG_ERR, "ERROR: fail on %s %d\n", (*it)->fname, errno);
				}
			}
		}else{
			printf("Timeout\n");
		}

		return rc;
       }
};

vector<EventHandler*> EventHandler::handlers;
unsigned EventHandler::evt_sc;


class COSEventHandler: public EventHandler {
public:
	COSEventHandler(int _site, int _offset) : EventHandler(_site, _offset) 
	{}
};

class ATDEventHandler: public EventHandler {
public:
        ATDEventHandler(int _site, int _offset) : EventHandler(_site, _offset)
        {}
};


bool EventHandler::create(int _site) {
	/* WORDTODO : replace hard coded functions */
	switch(_site){
		case 1:
			handlers.push_back(new EventHandler(_site, 0));
			return true;
		case 2:
			handlers.push_back(new COSEventHandler(_site, 32));
			return true;	
		case 14:
			handlers.push_back(new ATDEventHandler(_site, 0));
			return true;
		default:
			return false;
	}
}
bool EventHandler::create(int _site, const char* _fname) {
	handlers.push_back(new EventHandler(_site, _fname, 0));
	return true;
}


#define MODPRAMS "/sys/module/acq420fmc/parameters/"
#define NBUF	 MODPRAMS "nbuffers"
#define BUFLEN	 MODPRAMS "bufferlen"

#define FILL_LEN "/dev/acq400.0.knobs/bufferlen"

BufferManager* bm;

int wait_for_active(void)
{
	unsigned state = 0;
	int retry = 0;

	getKnob(0, "/etc/acq400/0/state", &state);

	while(state == 0){
		if (retry++ != 0 ){
			usleep(1000);
			if ((retry % 1000) == 0){
				fprintf(stderr, "wait_for_active %u\n", retry);
			}
		}
		getKnob(0, "/etc/acq400/0/state", &state);
	}
	return 0;
}

void ui(int argc, const char** argv)
{
	const char* evar;

	if ((evar = getenv("VERBOSE"))){
		G::verbose = atoi(evar);
	}

        poptContext opt_context =
                        poptGetContext(argv[0], argc, argv, opt_table, 0);

        int rc;
        getKnob(0, "/etc/acq400/0/ssb", &G::sample_size);
        getKnob(0, "/etc/acq400/0/spadstart", &G::sample_count_offset);
	getKnob(-1, NBUF,  &Buffer::nbuffers);
	getKnob(-1, BUFLEN, &Buffer::bufferlen);
	getKnob(-1, FILL_LEN, &G::fill_len);

	getKnob(14, "/etc/acq400/PRE", &G::pre_samples);
	getKnob(14, "/etc/acq400/POST", &G::post_samples);

	G::spb = G::fill_len/G::sample_size;

	if (G::verbose){
		fprintf(stderr, "ssb:%d sample_count_offset:%d spb:%d\n",
				G::sample_size, G::sample_count_offset, G::spb);
	}


        while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
                switch(rc){
                default:
                        ;
                }
        }

	if (G::pre_samples || G::post_samples){
		int pre_buffers = G::pre_samples/G::spb;
		if (pre_buffers > 1){
			G::buffers_pre = pre_buffers;
		}
		int post_buffers = G::post_samples/G::spb;
		if (post_buffers > 1){
			G::buffers_post = post_buffers;
		}
	}

        bm = new BufferManager(getRoot(G::devnum));

        // opening event stream BEFORE run doesn't work (run start action overrides) so ..
        wait_for_active();

	const char* ssite;
	while ((ssite = poptGetArg(opt_context)) != 0){
		if (ssite[0] == '/'){
			int site;

			if (sscanf(ssite, "/dev/acq400.%d.", &site) == 1){
				EventHandler::create(site, ssite);
			}
		}else{
			EventHandler::create(atoi(ssite));
		}
	}
}



int run(void)
{
	int event_count = 0;


	while(G::max_events == 0 || event_count < G::max_events){
		if (EventHandler::poll() == 0){
			++event_count;
		}
	}
	return 0;
}

int main(int argc, const char** argv)
{
	ui(argc, argv);
	return run();
}


