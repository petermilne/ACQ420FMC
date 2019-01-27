/*
 * bbq_send_ai.cpp : transmit ai buffer data from buffers once reported full by Buffer Q.
 *
 *  Created on: 27 Jan 2019
 *      Author: pgm
 */




/* ------------------------------------------------------------------------- */
/* bbq_send_ao.cpp  D-TACQ ACQ400 FMC  DRIVER    "big buffer" : data sender
 * Project: ACQ420_FMC
 * Created: 27 Jan 2019  			/ User: pgm
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



#include "local.h"		/* chomp() hopefully, not a lot of other garbage */
#include "knobs.h"

#include "File.h"


using namespace std;
#include "Buffer.h"


#define SAMPLE_FOREVER	0

namespace G {
	int samples_per_packet  = 1;
	int use_udp = 0;
	int verbose = 0;
	int devnum = 0;
	unsigned buffer_data_bytes;
	unsigned buffer_sample_size;
	unsigned max_samples = SAMPLE_FOREVER;
	const char* rhost = 0;
	const char* rport = 0;
};
struct poptOption opt_table[] = {
	{ "samples_per_packet", 'S', POPT_ARG_INT, &G::samples_per_packet, 0,
			"samples per packet (message in UDP, write in TCP)"
	},
	{ "use_udp", 'u', POPT_ARG_INT, &G::use_udp, 0,
			"send udp messages"
	},
	{ "max_samples", 'm', POPT_ARG_INT, &G::max_samples, 0,
			"max samples to send (0: infinity)"
	},
	{
	  "verbose", 'v', POPT_ARG_INT, &G::verbose, 0, "debug"
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

int Buffer::create(const char* root, int _buffer_len)
{
	char* fname = new char[128];
	sprintf(fname, "%s.hb/%03d", root, Buffer::last_buf);

	the_buffers.push_back(new MapBuffer(fname, _buffer_len));
	return 0;
}

class BufferManager {
	void init_buffers()
	{
		const char* root = getRoot(G::devnum);

		for (unsigned ii = 0; ii < Buffer::nbuffers; ++ii){
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
	BufferManager(unsigned start_buf = 0) {
		Buffer::last_buf = start_buf;
		init_buffers();
	}
	~BufferManager() {
		delete_buffers();
	}
};

#define MODPRAMS "/sys/module/acq420fmc/parameters/"
#define DFB	 MODPRAMS "distributor_first_buffer"
#define BUFLEN	 MODPRAMS "bufferlen"
#define NBUF	 MODPRAMS "nbuffers"
#define SSB	 "/etc/acq400/0/ssb"
void ui(int argc, const char** argv)
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	unsigned distributor_first_buffer = 0;


	getKnob(-1, NBUF,  &Buffer::nbuffers);
	getKnob(-1, BUFLEN, &Buffer::bufferlen);
	getKnob(-1, DFB, 	&distributor_first_buffer);
	if (distributor_first_buffer){
		Buffer::nbuffers -= distributor_first_buffer;
	}
	getKnob(G::devnum, "bufferlen", &G::buffer_data_bytes);
	getKnob(-1, SSB, &G::buffer_sample_size);

	int rc;

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		default:
			;
		}
	}

	G::rhost = poptGetArg(opt_context);
	G::rport = poptGetArg(opt_context);

	if (G::verbose){
		fprintf(stderr, "bbq_send_ai\n");
		if (G::max_samples == SAMPLE_FOREVER){
			fprintf(stderr, " send samples FOREVER\n");
		}else{
			fprintf(stderr, " send samples %u\n", G::max_samples? G::max_samples:0xffffffff);
		}
		fprintf(stderr, " sample size: %d, samples_per_packet:%d, message size: %d\n",
				G::buffer_sample_size, G::samples_per_packet,
				G::buffer_sample_size*G::samples_per_packet);
		fprintf(stderr, " bytes per buffer:%d, messages per buffer:%d\n",
				G::buffer_data_bytes,
				G::buffer_data_bytes/(G::buffer_sample_size*G::samples_per_packet));
		fprintf(stderr, " send using %s to %s:%s\n", G::use_udp? "UDP": "TCP", G::rhost, G::rport);
	}

	if (G::rhost == 0 || G::rport == 0){
		fprintf(stderr, "usage bbq_send_ai [opts] HOST PORT\n");
		exit(1);
	}
}

int run(void)
{
	return 0;
}
int main(int argc, const char** argv)
{
	ui(argc, argv);
	BufferManager bm;
	return run();
}

