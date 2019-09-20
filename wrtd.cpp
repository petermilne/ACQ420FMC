/*
 * wrtd.cpp : White Rabbit Time Distribution
 *
 *  Created on: 19 Sep 2019
 *      Author: pgm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "popt.h"

#include "Multicast.h"


namespace G {
	const char* group = "224.0.23.159";
	int port = 5044;
	const char* fname = "/dev/acq400.0.wr_ts";
	int delta_ticks = 800000;			// 10 msec at 80MHz
        unsigned max_ticks = 80000000;
        int verbose = 0;
}


#define SECONDS_SHL	28
#define SECONDS_MASK	0x7
#define TICKS_MASK	0x0fffffff
#define TS_EN		(1<<31)

struct TS {
	unsigned raw;
	char repr[16];

	TS(unsigned _raw = 0): raw(_raw) {}
	TS(unsigned _secs, unsigned _ticks, bool en=true) {
		raw = (_secs&SECONDS_MASK) << SECONDS_SHL | (_ticks&TICKS_MASK) | (en?TS_EN:0);
	}

	unsigned secs() const { return (raw& ~TS_EN) >> SECONDS_SHL; }
	unsigned ticks() const { return raw&TICKS_MASK; }

	TS add (unsigned dsecs, unsigned dticks = 0) {
		unsigned ss = secs();
		unsigned tt = ticks();
		tt += dticks;
		if (tt > G::max_ticks){
			tt -= G::max_ticks;
			ss += 1;
		}
		ss += dsecs;
		return TS(ss, tt);
	}
	const char* toStr(void) {
		sprintf(repr, "%d:%07d", secs(), ticks());
		return repr;
	}
};
struct poptOption opt_table[] = {
	{
	  "verbose", 'v', POPT_ARG_INT, &G::verbose, 0, "debug"
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};

const char* ui(int argc, const char** argv)
{
        poptContext opt_context =
                        poptGetContext(argv[0], argc, argv, opt_table, 0);
        int rc;
        while ((rc = poptGetNextOpt( opt_context )) >= 0 ){
                switch(rc){
                case 's':

                default:
                        ;
                }
        }
        return poptGetArg(opt_context);
}


int sender(MultiCast& mc)
{
	FILE *fp = fopen(G::fname, "r");
	TS ts;
	while(fread(&ts.raw, sizeof(unsigned), 1, fp) == 1){
		if (G::verbose) fprintf(stderr, "sender:ts:%s\n", ts.toStr());
		ts = ts.add(1);
		mc.sendto(&ts, sizeof(unsigned));
	}
	return 0;
}

int receiver(MultiCast& mc)
{
	FILE *fp = fopen(G::fname, "w");
	TS ts;
	while(mc.recvfrom(&ts.raw, sizeof(ts.raw)) == sizeof(ts.raw)){
		if (G::verbose) fprintf(stderr, "receiver:ts:%s\n", ts.toStr());
		int rc = fwrite(&ts.raw, sizeof(unsigned), 1, fp);
		if (rc < 1){
			perror("fwrite");
		}
		fflush(fp);
	}
	return 0;
}
int main(int argc, const char* argv[])
{
	const char* mode = ui(argc, argv);

	if (strcmp(mode, "tx") == 0){
		return sender(MultiCast::factory(G::group, G::port, MultiCast::MC_SENDER));
	}else{
		return receiver(MultiCast::factory(G::group, G::port, MultiCast::MC_RECEIVER));
	}
}

