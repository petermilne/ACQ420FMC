/*
 * slowmon.cpp : read slow mon data, aggregate and output
 *
 *  Created on: 18 Mar 2022
 *      Author: pgm
 */

#include "popt.h"

#include "local.h"
#include "Env.h"
#include "File.h"
#include "knobs.h"


#include <unistd.h>
#include <signal.h>


/* meta data front and back. no math! */
#define META1	sizeof(long)
#define META2	sizeof(long)

namespace G {
	unsigned usec = 250000;
	unsigned nacc = 1;
	FILE* fp;
	unsigned ndata = 64+4;
	short *data;
	long *sums;
	unsigned isam = 0;
}

void clear_sums()
{
	memset(G::sums, 0, G::ndata*sizeof(long));
}
void onSample(int sig)
{
	ssize_t len = G::ndata*sizeof(short);
	ssize_t nread = read(fileno(G::fp), G::data, len);
	if (nread != len){
		perror("read fail");
		exit(1);
	}

	if (G::nacc == 1){
		write(1, G::data, len);
	}else{
		const int last = G::ndata-META2/sizeof(short);
		const int first = G::ndata-META1/sizeof(short);

		for (int ii = first; ii < last; ++ii){
			G::sums[ii] += G::data[ii];
		}

		if (++G::isam == G::nacc){
			for (int ii = first; ii < last; ++ii){
				G::data[ii] = G::sums[ii] / G::nacc;
			}
			write(1, G::data, len);
			clear_sums();
			G::isam = 0;
		}
	}
}

void init(int argc, const char* argv[])
{
	G::data = new short[G::ndata];
	G::sums = new long[G::ndata];
	clear_sums();
	G::fp = fopen("/dev/acq400.0.subr", "r");
}

void ui()
{
	unsigned usec;
	unsigned fs;
	unsigned fin;

	getKnob(0, "/etc/acq400/0/slowmon_fs", &fs);
	getKnob(0, "/etc/acq400/0/slowmon_fin", &fin);
	G::nacc = fs/fin;
	usec = 1000000/fs;

	if (usec != G::usec){
		G::usec = usec;
		ualarm(G::usec, G::usec);
	}
}

int main(int argc, const char* argv[])
{
	init(argc, argv);
	signal(SIGALRM, onSample);
	ualarm(G::usec, G::usec); //alarm in a second, and every second after that.

	for(;;){
		pause();
		ui();
	}
	return 0;
}


