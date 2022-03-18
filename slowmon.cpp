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
#include "Knob.h"


#include <unistd.h>
#include <signal.h>



namespace G {
	unsigned usec = 250000;
	FILE* fp;
	unsigned ndata = 32;
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
	G::fp = fopen("/dev/acq400.1.subr", "r");
}

int main(int argc, const char* argv[])
{
	init(argc, argv);
	signal(SIGALRM, onSample);
	ualarm(G::usec, G::usec); //alarm in a second, and every second after that.

	for(;;){
		pause();
	}
	return 0;
}


