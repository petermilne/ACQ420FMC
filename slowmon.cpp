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



namespace G {
	unsigned usec = 250000;
	unsigned nacc = 1;
	FILE* fp;
	unsigned ndata = 64+4;
	short *data;
}
void onSample(int sig)
{
	ssize_t len = G::ndata*sizeof(short);
	ssize_t nread = read(fileno(G::fp), G::data, len);
	if (nread != len){
		perror("read fail");
		exit(1);
	}
	write(1, G::data, len);
}


void init(int argc, const char* argv[])
{
	G::data = new short[G::ndata];
	G::fp = fopen("/dev/acq400.0.subr", "r");
}

void ui()
{
	unsigned usec;
	unsigned fs;
	unsigned fin;
	unsigned nacc;
	getKnob(0, "/etc/acq400/0/slowmon_fs", &fs);
	getKnob(0, "/etc/acq400/0/slowmon_fin", &fin);
	nacc = fs/fin;
	usec = 1000000/fs;
	if (nacc > 1){
		setKnob(0, "/var/log/slomonerr.log", "@@todo nacc set to 1");
		nacc = 1;
	}
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


