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

int get_mac(char* mac, int max_mac)
{
	FILE *pp = popen("ifconfig eth1 | grep HWa | awk '{ print $5 }'", "r");
	fgets(mac, max_mac, pp);
	pclose(pp);
	return 0;
}

void* mmap_wr(void)
{
        int fd = open("/dev/mem", O_RDWR);
        unsigned *va = (unsigned*)mmap(0, 0x40000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40080000);

        if (va == MAP_FAILED){
                perror("MAP_FAILED");
                exit(1);
        }
	return va;
}

void wr_load(char* wrbase, const char* fname)
{
	FILE* fp = fopen(fname, "r");
	if (fp == 0){
		perror(fname);
		exit(1);
	}
	char* buf = new char[0x20000];
	int nr = fread(buf, 1, 0x20000, fp);
	memcpy(wrbase, buf, nr);
	fclose(fp);
}

int main(int argc, const char** argv)
{
	char mac_buf[80];
	strcpy(mac_buf, "ac:ac:ac:ac:");
	get_mac(mac_buf+strlen(mac_buf), 80-strlen(mac_buf));
	printf("mac_buf:%s\n", mac_buf);

	char* token;
	char* rest = mac_buf;

	char* wrbase = (char*)mmap_wr();
	char* wrprams = wrbase + 0x1f000;
	unsigned * wr_reset = (unsigned*)(wrbase+0x20400);
	unsigned * wr_hwid  = (unsigned*)(wrbase+0x20414);

	*wr_reset = 0x1deadbee;
	if (argc > 1) {
		wr_load(wrbase, argv[1]);
	}
	*wr_hwid  = 0x41435134;				// ACQ4
	/* fit wierd backwards load in LM32 */
	int ii = 0; int jj = 3;
	while((token = strtok_r(rest, ":", &rest))){
		wrprams[ii+jj] = strtoul(token, 0, 16);
		if (--jj < 0){
			ii += 4;
			jj = 3;
		}
	}
	*wr_reset = 0x0deadbee;
	return 0;
}

