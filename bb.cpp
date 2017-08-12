/* ------------------------------------------------------------------------- */
/* bb.cpp  D-TACQ ACQ400 FMC  DRIVER    "big buffer" : read or write
 * Project: ACQ420_FMC
 * Created: 16 Jul 2016  			/ User: pgm
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

#define VERID	"B1007"

#define NCHAN	4

#include <semaphore.h>
#include <syslog.h>

#include <vector>

#include "local.h"		/* chomp() hopefully, not a lot of other garbage */
#include "knobs.h"

#include "File.h"


using namespace std;

/* copy from driver .. */
enum AO_playloop_oneshot { AO_continuous, AO_oneshot, AO_oneshot_rearm };

namespace G {
	unsigned sample_size = sizeof(unsigned);	// bytes per sample
	int play_site = 1;
	unsigned offset = 0;
	int devnum = 0;					// full system.
	FILE* fp_out = stdout;
	FILE* fp_in = stdin;

	int mode = AO_oneshot;
	int verbose;
	unsigned buffer0;				// index from here
};

#include "Buffer.h"

struct poptOption opt_table[] = {
	{ "sample-size", 'S', POPT_ARG_INT, &G::sample_size, 0,
			"bytes per sample [deprecated]"
	},
	{ "play", 'P', POPT_ARG_INT, &G::play_site, 0,
			"AWG site [deprecated]"
	},
	{ "offset", 'o', POPT_ARG_INT, &G::offset, 0,
			"offset in buffer (for multi-site ops)"
	},
	{ "mode",  'm', POPT_ARG_INT, &G::mode, 0,
			"play mode 0: continuous, 1:oneshot 2:oneshot_rearm"
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};

enum RUN_MODE { M_FILL, M_LOAD, M_DUMP };

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

void set_playloop_length(int nsamples)
{
	char cmd[128];
	sprintf(cmd, "set.site %d playloop_length %d %d",
			G::play_site, G_nsamples = nsamples, G::mode);
	system(cmd);
}

int pad(int nsamples, int pad_samples)
{
	char* base = Buffer::the_buffers[0]->getBase();
	char* end = base + nsamples*G::sample_size;
	char* last = end - G::sample_size;

	while(pad_samples--){
		memcpy(end, last, G::sample_size);
		end += G::sample_size;
		nsamples++;
	}
	return nsamples;
}

int _load() {
	int maxbuf = Buffer::nbuffers*Buffer::bufferlen/G::sample_size;

	unsigned nsamples = fread(Buffer::the_buffers[0]->getBase(),
			G::sample_size, maxbuf, G::fp_in);
	int residue = (nsamples*G::sample_size)%Buffer::bufferlen;

	if (residue){
		nsamples = pad(nsamples, (Buffer::bufferlen - residue)/G::sample_size);
	}

	return nsamples;
}

int fill() {
	_load();
	return 0;
}
int load() {
	set_playloop_length(0);

	set_playloop_length(_load());

	return 0;
}
int dump() {
	unsigned nsamples;
	getKnob(G::play_site, "playloop_length", &nsamples);

	fwrite(Buffer::the_buffers[0]->getBase(),
			G::sample_size, nsamples, G::fp_out);
	return 0;
}

#define DFB	"/sys/module/acq420fmc/parameters/distributor_first_buffer"
RUN_MODE ui(int argc, const char** argv)
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);

	if (getenv("VERBOSE")){
		G::verbose = atoi(getenv("VERBOSE"));
	}
	getKnob(G::devnum, "nbuffers",  &Buffer::nbuffers);
	getKnob(G::devnum, "bufferlen", &Buffer::bufferlen);
	getKnob(G::devnum, DFB, &G::buffer0);
	unsigned dist_s1;
	getKnob(0, "dist_s1", &dist_s1);
	if (dist_s1){
		G::play_site = dist_s1;
		getKnob(0, "/etc/acq400/0/dssb", &G::sample_size);
		fprintf(stderr, "s1:%d size:%d\n", dist_s1, G::sample_size);
	}
	int rc;

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		default:
			;
		}
	}

	const char* mode = poptGetArg(opt_context);
	if (mode != 0){
		if (strcmp(mode, "load") == 0){
			return M_LOAD;
		}else if (strcmp(mode, "fill") == 0){
			return M_FILL;
		}
	}
	return M_DUMP;
}

class BufferManager {
	void init_buffers()
	{
		const char* root = getRoot(G::devnum);
		Buffer::last_buf = G::buffer0;
		for (unsigned ii = 0; ii < Buffer::nbuffers-G::buffer0; ++ii){
			Buffer::create(root, Buffer::bufferlen);
		}
	}
	void delete_buffers()
	{
		/* this _should_ be automatic. But it's not! */
		for (unsigned ii = 0; ii < Buffer::the_buffers.size(); ++ii){
			delete Buffer::the_buffers[ii];
		}
	}
public:
	BufferManager() {
		init_buffers();
	}
	~BufferManager() {
		delete_buffers();
		printf("DONE %d\n", G_nsamples);
	}
};
int main(int argc, const char** argv)
{
	RUN_MODE rm = ui(argc, argv);
	BufferManager bm;
	switch(rm){
	case M_FILL:
		return fill();
	case M_LOAD:
		return load();
	default:
		return dump();
	}
}


