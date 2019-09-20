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

#define M1 1000000
#define NSPS	(1000*M1)		// nanoseconds per second

#define REPORT_THRESHOLD (3*M1)		// 3 msec warning

namespace G {
	const char* group = "224.0.23.159";
	int port = 5044;
	const char* fname = "/dev/acq400.0.wr_ts";
	const char* current = "/dev/acq400.0.wr_cur";
	int delta_ticks = 800000;			// 10 msec at 80MHz
        unsigned ticks_per_sec = 80000000;
        int verbose = 0;
        unsigned dns = 100000;		// delta nsec
        unsigned ticks_per_ns = 50;		// ticks per nsec
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
		if (tt > G::ticks_per_sec){
			tt -= G::ticks_per_sec;
			ss += 1;
		}
		ss += dsecs;
		return TS(ss, tt);
	}
	TS operator+ (unsigned dticks) {
		return add(0, dticks);
	}
	long diff(TS& ts2){
		unsigned _ticks = ticks();
		unsigned _secs = secs();
		if (ts2.ticks() > ticks()){
			_ticks += G::ticks_per_sec;
			_ticks -= ts2.ticks();
			_secs -= 1;
		}else{
			_ticks -= ts2.ticks();
		}
		return (_secs - ts2.secs())*NSPS + _ticks*G::ticks_per_ns;
	}

	const char* toStr(void) {
		sprintf(repr, "%d:%07d", secs(), ticks());
		return repr;
	}
};
struct poptOption opt_table[] = {
	{
	  "tickns", 0, POPT_ARG_INT, &G::ticks_per_ns, 0, "tick size nsec"
	},
	{
	  "dns", 'd', POPT_ARG_INT, &G::dns, 0, "nsec to add to current time"
	},
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
        G::ticks_per_sec = NSPS / G::ticks_per_ns;
        G::delta_ticks = G::dns / G::ticks_per_ns;

        if (G::verbose) fprintf(stderr, "ns per tick: %u ticks per s: %u delta_ticks %u\n",
        		G::ticks_per_ns, G::ticks_per_sec, G::delta_ticks);

        return poptGetArg(opt_context);
}


int sender(MultiCast& mc)
{
	FILE *fp = fopen(G::fname, "r");
	TS ts;
	while(fread(&ts.raw, sizeof(unsigned), 1, fp) == 1){
		if (G::verbose) fprintf(stderr, "sender:ts:%s\n", ts.toStr());
		ts = ts + G::delta_ticks;
		mc.sendto(&ts, sizeof(unsigned));
	}
	return 0;
}

int receiver(MultiCast& mc)
{
	FILE *fp = fopen(G::fname, "w");
	FILE *fp_cur = fopen(G::current, "r");

	TS ts;
	TS ts_cur;
	while(mc.recvfrom(&ts.raw, sizeof(ts.raw)) == sizeof(ts.raw)){
		if (G::verbose > 1) fprintf(stderr, "receiver:ts:%s\n", ts.toStr());
		int rc = fwrite(&ts.raw, sizeof(unsigned), 1, fp);
		if (rc < 1){
			perror("fwrite");
		}
		fflush(fp);
		fread(&ts_cur.raw, sizeof(unsigned), 1, fp_cur);

		TS ts_delay = ts_cur + G::delta_ticks;
		if (G::verbose) fprintf(stderr, "wrtd rx demand:%s cur:%s %ld usec\n", ts.toStr(), ts_cur.toStr(), ts.diff(ts_delay)/1000);
		if(ts_delay.diff(ts) > REPORT_THRESHOLD){
			fprintf(stderr, "wrtd rc WARNING threshold ");
		}
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

