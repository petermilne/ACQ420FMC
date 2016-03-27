/* ------------------------------------------------------------------------- */
/* acq400_ssl.c ACQ420_FMC						     */
/* ------------------------------------------------------------------------- */
/* acq400_ssl.c  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 27 Mar 2016  			/ User: pgm
 * acq400_ssl --ssl [start,stride,length] [CH-DEF]
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
/* ------------------------------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h> // sigaction(), sigsuspend(), sig*()
#include <unistd.h> // alarm()
#include <pthread.h>
#include <sys/ioctl.h>

#include "popt.h"
#include "acq-util.h"
#include "acq400_fs_ioctl.h"

#include "File.h"

using namespace std;

#define MAXCHAN	32	// @@todo get this from the system
#define MAXBUF	0x100000

namespace G {
	int site = 1;
	int verbose = 0;
	const char* ssl = "0,1,:";
	int start = 0;
	int stride = 1;
	int length = 99999999;
	const char* channels = ":";
	int *the_channels;
	int ws = sizeof(short);
	char* buffer = new char[MAXBUF];
	FILE * fout = stdout;
};
struct poptOption opt_table[] = {
	{ "site", 	's', POPT_ARG_INT, &G::site, 0, "site" },
	{ "ssl",        'T', POPT_ARG_STRING,  &G::ssl, 0, "start,stride,length" },
	{ "ws", 	'w', POPT_ARG_INT, &G::ws, 0, "wordsize" },
	{ "verbose", 	'v', POPT_ARG_INT, &G::verbose, 0, "" },
	POPT_AUTOHELP
	POPT_TABLEEND
};

int getSSL() {
	sscanf(G::ssl, "%d,%d,%d", &G::start, &G::stride, &G::length);
}

int getChannelList()
{
	acqMakeChannelRange(G::the_channels, MAXCHAN, G::channels);
}

int process(int ic)
{
	char fname[80];
	sprintf(fname, "/dev/acq400/data/%d/%02d", G::site, ic);

	File ff(fname, "r");

	if (G::start){
		if (fseek(ff.fp(), G::start*G::ws, SEEK_SET) != 0){
			perror("fseek");
			exit(1);
		}
	}
	if (G::stride != 1){
		if (ioctl(ff.fd(), ACQ400_FS_STRIDE, G::stride) != 0){
			perror("ioctl");
		}
	}

	int nread;
	while((nread = fread(G::buffer, 1, MAXBUF, ff.fp())) > 0){
		fwrite(G::buffer, 1, nread, G::fout);
	}
	if (ferror(ff.fp())){
		perror("fread");
		exit(1);
	}
	return 0;
}
int process() {
	for (int ic = 1; ic <= MAXCHAN; ++ic){
		if (G::the_channels[ic]){
			int rc = process(ic);
			if (rc) return rc;
		}
	}

	return 0;
}

void cli(int argc, const char* argv[])
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;

	G::the_channels = new int[MAXCHAN+1];
	memset(G::the_channels, 0, MAXCHAN+1*sizeof(int));

	while ((rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		default:
			;
		}
	}
	const char* cdef = poptGetArg(opt_context);
	if (cdef != 0) G::channels = cdef;
}

int main(int argc, const char* argv[])
{
	cli(argc, argv);
	getChannelList();
	getSSL();

	return process();
}

