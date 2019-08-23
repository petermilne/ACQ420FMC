/* ------------------------------------------------------------------------- */
/* atd.cpp  D-TACQ ACQ400 comb memory for ATD result
 * field the event, locate in memory, extract and save
 * Project: ACQ420_FMC
 * Created: 17 August 2019  			/ User: pgm
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


using namespace std;


namespace G {
	unsigned sample_size = sizeof(unsigned);	// bytes per sample
	unsigned sample_count_offset;
	unsigned offset = 15;				// ch16
	unsigned atd_code = 0x1e1d7f80;			// rising 2.4V
	int verbose;
	int nchan = 16;
	int atd_site = 14;
};

#define NO_THRESHOLD 0x7f807f80

int get_atd_code(void)
{
	int fd = open("/dev/acq400.14.atd", O_RDONLY);
	unsigned *va = (unsigned*)mmap(0, 0x1000, PROT_READ, MAP_SHARED, fd, 0);
	unsigned *cursor;

	if (va == MAP_FAILED){
		perror("MAP_FAILED");
		exit(1);
	}

	for (cursor = va; cursor - va < G::nchan; ++cursor){
		if (*cursor != NO_THRESHOLD){
			G::offset = cursor -va;
			G::atd_code = *cursor;
			fprintf(stderr, "found atd_code %u 0x%08x\n", G::offset, G::atd_code);
			return 0;
		}
	}
	fprintf(stderr, "atd_code not found\n");
	exit(1);
	return -1;
}
int map_file(const char* fn, int perms, void **va)
{
	struct stat statbuf;
	int rc;
	int fd = open(fn, perms);
	if (fd == -1) return fd;
	if ((rc = fstat(fd, &statbuf)) != 0){
		fprintf(stderr, "stat failed\n");
		return rc;
	}
	*va = mmap(0, statbuf.st_size, PROT_READ, MAP_SHARED, fd, 0);
	if (*va == MAP_FAILED){
		fprintf(stderr, "MAP_FAILED");
		return -1;
	}
	return statbuf.st_size;
}

void dump_sam(short* cursor)
{
	FILE *pp = popen("hexdump -e '16/2 \"%04x,\" 1/4 \"%08x,\" 3/4 \"%10u \" \"\\n\"'", "w");
	fwrite(cursor, 1, G::sample_size, pp);
	pclose(pp);
}
int search_rising(short* data, const int nsam, const unsigned offset, const int stride, const unsigned ABCD)
{
	const short AA = ((ABCD&0xff000000)>>24) << 8;
	const short BB = ((ABCD&0x00ff0000)>>16) << 8;

	for (short *cursor = data; cursor - data < nsam*stride; cursor += stride){
		if (cursor[offset] > AA && cursor[offset] > BB ){
			int isam = (cursor-data) / stride;
			printf("search_rising: crossing at %d\n", isam);
			dump_sam(cursor);
			return isam;
		}
	}
	return -1;
}

#define SEARCH_BACK	5000
int search(const char* fn)
{
	void* va = 0;
	int blen = map_file(fn, O_RDONLY, &va);

	if (blen < 0){
		fprintf(stderr, "mapfile %s failed\n", fn);
		exit(1);
	}
	printf("searching %s len :%d\n", fn, blen);
	int isam = search_rising((short*)va, blen/G::sample_size, G::offset, G::sample_size/sizeof(short), G::atd_code);
	if (isam == -1){
		fprintf(stderr, "ERROR: search failed\n");
		return -1;
	}
	if (isam == 0){
		fprintf(stderr, "ERROR: search failed at zero\n");
		return -1;
	}
	char* cursor = ((char*) va) + isam * G::sample_size;
	unsigned ucount = ((unsigned*)cursor)[16/2 + 1];
	unsigned fncount;
	int rc = sscanf(fn, "/tmp/event-14-%u.dat", &fncount);
	if (rc == 1){
		printf("count at isr %u %s measured %u delta %u\n\n",
				fncount, fncount>ucount? ">": "<", ucount,
					 fncount>ucount? fncount-ucount: ucount-fncount);
	}

	return 0;
}



struct poptOption opt_table[] = {
	{ "sample-size", 'S', POPT_ARG_INT, &G::sample_size, 0,
			"bytes per sample [deprecated]"
	},
	{
	  "verbose", 'v', POPT_ARG_INT, &G::verbose, 0, "debug"
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};

int ui(int argc, const char** argv)
{
        getKnob(0, "/etc/acq400/0/ssb", &G::sample_size);
        get_atd_code();
        poptContext opt_context =
                        poptGetContext(argv[0], argc, argv, opt_table, 0);
        int rc;
        while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
                switch(rc){
                default:
                        ;
                }
        }
	const char* fn;
	while ((fn = poptGetArg(opt_context)) != 0){
		search(fn);
	}
	return 0;
}


int main(int argc, const char** argv)
{
	return ui(argc, argv);
}

