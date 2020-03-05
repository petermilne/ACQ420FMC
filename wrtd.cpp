/* wrtd.cpp : White Rabbit Time Distribution                  	 	     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2019 pgm, D-TACQ Solutions Ltd                            *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
 *   Created on: Created on: 19 Sep 2019                                     *
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/*
 * wrtd.cpp : White Rabbit Time Distribution
 *
 *  Created on: 19 Sep 2019
 *      Author: pgm
 *
 * - Usage
 *   wrtd tx
 *   	waits for incoming external trigger, sends wrtd trigger packet set for a [near] future time
 *   wrtx tx_immediate
 *   	sends trigger packet immediately (test mode) .. this is a soft trigger, really
 *   wrtd rx
 *   	receives network triggers and configures WRTT to fire at specified time
 *
 * -command line options
 *   try --help for full list
 * - environment
 *   WRTD_RX_MATCHES=m1[,m2,m3...]   # receiver matches on multiple strings, not just default ["acq2106"[7]]
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "split2.h"

#include "popt.h"

#include "Knob.h"
#include "Multicast.h"

#define M1 1000000
#define NSPS	(1000*M1)		// nanoseconds per second

#define MAX_TX_INF	0xFFFFFFFF

namespace G {
	const char* group = "224.0.23.159";
	int port = 5044;
	const char* fname = "/dev/acq400.0.wr_ts";
	const char* current = "/dev/acq400.0.wr_cur";
	int delta_ticks;
        unsigned ticks_per_sec = 80000000;
        int verbose = 0;
        unsigned dns = 40*M1;				// delta nsec
        unsigned ns_per_tick = 50;			// ticks per nsec
        unsigned local_clkdiv;				// Site 1 clock divider, set at start
        unsigned local_clkoffset;			// local_clk_offset eg 2 x 50nsec for ACQ42x
        unsigned max_tx = MAX_TX_INF;			// send max this many trigs
        const char* tx_id;				// transmit id
}

#define REPORT_THRESHOLD (G::dns/4)

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
	unsigned nsec() const { return ticks() * G::ns_per_tick; }

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
		return (_secs - ts2.secs())*NSPS + _ticks*G::ns_per_tick;
	}

	const char* toStr(void) {
		sprintf(repr, "%d:%07d", secs(), ticks());
		return repr;
	}
};
struct poptOption opt_table[] = {
	{
	  "tickns", 0, POPT_ARG_INT, &G::ns_per_tick, 0, "tick size nsec"
	},
	{
	  "dns", 'd', POPT_ARG_INT, &G::dns, 0, "nsec to add to current time"
	},
	{
	  "verbose", 'v', POPT_ARG_INT, &G::verbose, 0, "debug"
	},
	{
	  "local_clkdiv", 'l', POPT_ARG_INT, &G::local_clkdiv, 0, "local clock divider"
	},
	{
	  "local_clkoffset", 'L', POPT_ARG_INT, &G::local_clkoffset, 0, "local clock offset"
	},
	{
	  "max_tx", 0, POPT_ARG_INT, &G::max_tx, 0, "maximum transmit count"
	},
	{
          "tx_id", 0, POPT_ARG_STRING, &G::tx_id, 0, "txid: default is $(hostname)"
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};

const char* ui(int argc, const char** argv)
{
        poptContext opt_context =
                        poptGetContext(argv[0], argc, argv, opt_table, 0);
        int rc;

        Knob clkdiv(1, "clkdiv");
        Knob modname(1, "module_name");

        if (strstr(modname(), "acq48")){
        	G::local_clkdiv = 1;
        	G::local_clkoffset = 0;
        }else{
        	clkdiv.get((unsigned*)&G::local_clkdiv);
        	G::local_clkoffset = 2;
        }
        while ((rc = poptGetNextOpt( opt_context )) >= 0 ){
                switch(rc){
                case 's':
                default:
                        ;
                }
        }
        G::ticks_per_sec = NSPS / G::ns_per_tick;
        G::delta_ticks = G::dns / G::ns_per_tick;

        if (G::verbose) fprintf(stderr, "ns per tick: %u ticks per s: %u delta_ticks %u\n",
        		G::ns_per_tick, G::ticks_per_sec, G::delta_ticks);

        const char* mode = poptGetArg(opt_context);
        if (!mode){
        	fprintf(stderr, "ERROR: please specify mode tx|tx_immediate|rx");
        	exit(1);
        }
        return mode;
}



class TSCaster {
protected:
	MultiCast& mc;
	TSCaster(MultiCast& _mc) : mc(_mc)
	{}
	TS ts;
public:
	virtual void sendto(TS& ts) {
		mc.sendto(&ts, sizeof(unsigned));
	}
	virtual TS recvfrom() {
		if (mc.recvfrom(&ts.raw, sizeof(ts.raw)) != sizeof(ts.raw)){
			perror("closedown");
			exit(1);
		}
		return ts;
	}
	virtual int printLast() {
		return printf("%08x\n", ts.raw);
	}
	static TSCaster& factory(MultiCast& _mc);
};

#include <stdint.h>
#include "wrtd-common.h"
#include <unistd.h>

class MessageFilter {
public:
	virtual bool operator () (struct wrtd_message& msg) = 0;

	static MessageFilter& factory();
};

class Acq2106DefaultMessageFilter : public MessageFilter {
public:
	virtual bool operator() (struct wrtd_message& msg) {
		if (strncmp((char*)msg.event_id, "acq2106", 7) == 0){
			return true;
		}else{
			fprintf(stderr, "HELP! non acq2106 message received\n");
			return false;
		}
	}
};




typedef std::vector<std::string> VS;

class MultipleMatchFilter : public MessageFilter {
	VS matches;
public:
	MultipleMatchFilter(const char* _matches){

		split2<VS>(_matches, matches, ',');

	}
	virtual bool operator() (struct wrtd_message& msg) {
		for (std::string ss : matches){
			if (strncmp(ss.c_str(), (char*)msg.event_id, WRTD_ID_LEN) == 0){
				return true;
			}
		}
		return false;
	}
};


MessageFilter& MessageFilter::factory() {
	const char* matches = getenv("WRTD_RX_MATCHES");
	if (matches){
		return * new MultipleMatchFilter(matches);
	}
	return * new Acq2106DefaultMessageFilter();
}

class WrtdCaster : public TSCaster {
	int seq;
	char hn[13];			// acq2106_[n]nnn

	MessageFilter& is_for_us;
	struct wrtd_message msg;
protected:
	WrtdCaster(MultiCast& _mc, MessageFilter& filter) : TSCaster(_mc), seq(0), is_for_us(filter)
	{
		memset(&msg, 0, sizeof(msg));
		if (G::tx_id){
			strncpy((char*)msg.event_id, G::tx_id, WRTD_ID_LEN);
		}else{
			gethostname(hn, sizeof(hn));
			snprintf((char*)msg.event_id, WRTD_ID_LEN, "%s.%c", hn, '0');
		}
	}
	friend class TSCaster;

	virtual void sendto(TS& ts) {
	        msg.hw_detect[0] = 'L';
	        msg.hw_detect[1] = 'X';
	        msg.hw_detect[2] = 'I';
	        msg.seq = seq++;
	        msg.ts_sec = ts.secs();			// truncated at 7.. worktodo use TAI
	        msg.ts_ns = ts.nsec();
	        //msg.event_id is pre-cooked, all other fields are zero
	        mc.sendto(&msg, sizeof(msg));
	}
	virtual int printLast() {
		return printf("%s %16s %u %u %u\n", msg.hw_detect, msg.event_id, msg.seq, msg.ts_sec, msg.ts_ns);
	}
	virtual TS recvfrom() {
		while(true){
			if (mc.recvfrom(&msg, sizeof(msg)) != sizeof(msg)){
				perror("closedown");
				exit(1);
			}
			if (is_for_us(msg)){
				TS ts(msg.ts_sec, msg.ts_ns/G::ns_per_tick);
				return ts;
			}
		}
	}
};
TSCaster& TSCaster::factory(MultiCast& _mc) {
	int use_wrtd_fullmessage = 1;
	const char* value = getenv("WRTD_FULLMESSAGE");
	if (value){
		use_wrtd_fullmessage = atoi(value);
	}
	if (use_wrtd_fullmessage){
		return * new WrtdCaster(_mc, MessageFilter::factory());
	}else{
		return * new TSCaster(_mc);
	}
}



int sender(TSCaster& comms)
{
	if (G::max_tx == 0){
		return 0;
	}
	FILE *fp = fopen(G::fname, "r");
	TS ts;

	for (unsigned ntx = 0; fread(&ts.raw, sizeof(unsigned), 1, fp) == 1; ++ntx){
		TS ts_tx = ts + G::delta_ticks;
		comms.sendto(ts_tx);
		if (G::verbose > 1) fprintf(stderr, "sender:ntx:%u ts:%s ts_tx:%s\n", ntx, ts.toStr(), ts_tx.toStr());
		if (G::max_tx != MAX_TX_INF && ntx >= G::max_tx){
			break;
		}
	}
	fclose(fp);
	return 0;
}

int tx_immediate(TSCaster& comms)
{
	if (G::max_tx == 0){
		return 0;
	}

	FILE *fp_cur = fopen(G::current, "r");
	TS ts;

	unsigned ntx = 0;
	while(fread(&ts.raw, sizeof(unsigned), 1, fp_cur) == 1){
		if (G::verbose > 1) fprintf(stderr, "sender:ts:%s\n", ts.toStr());
		ts = ts + G::delta_ticks;
		comms.sendto(ts);
		++ntx;
		if (G::max_tx == MAX_TX_INF || ntx >= G::max_tx){
			break;
		}else{
			usleep(2*G::dns/1000);		// don't overrun previous message
		}
	}
	fclose(fp_cur);
	return 0;
}
TS _adjust_ts(TS ts0)
{
	int rem = ts0.ticks() % G::local_clkdiv;
	unsigned ticks = ts0.ticks();


	if (rem != 0){
		ticks += G::local_clkdiv - rem;
	}
	if (ticks > G::local_clkoffset){
		ticks -= G::local_clkoffset;
	}

	if (G::verbose > 1) fprintf(stderr, "adjust_ts: ts0 %u div %u rem %u off %u adj %u\n",
			ts0.ticks(), G::local_clkdiv, rem, G::local_clkoffset, ticks);

	return TS(ts0.secs(), ticks);
}
TS adjust_ts(TS ts0)
{
	if (G::local_clkdiv > 1 || G::local_clkoffset != 0){
		return _adjust_ts(ts0);
	}else{
		return ts0;
	}
}
int receiver(TSCaster& comms)
{
	FILE *fp = fopen(G::fname, "w");
	FILE *fp_cur = fopen(G::current, "r");
	long dms = G::dns/M1;

	for (unsigned nrx = 0;; ++nrx){
		TS ts = comms.recvfrom();
		TS ts_adj = adjust_ts(ts);

		int rc = fwrite(&ts_adj.raw, sizeof(unsigned), 1, fp);
		if (rc < 1){
			perror("fwrite");
		}
		fflush(fp);

		TS ts_cur;
		fread(&ts_cur.raw, sizeof(unsigned), 1, fp_cur);

		if (G::verbose > 1) fprintf(stderr, "receiver:nrx:%u cur:%s ts:%s adj:%s\n", nrx, ts_cur.toStr(), ts.toStr(), ts_adj.toStr());
		if (G::verbose) comms.printLast();

		long dt = ts.diff(ts_cur);
		if (dt < 0){
			fprintf(stderr, "wrtd rx ERROR missed ts by %ld msec\n", dt/M1);
		}else if (dt < (long)REPORT_THRESHOLD){
			fprintf(stderr, "wrtd rx WARNING threshold %ld msec under limit %ld\n", dt/M1, dms);
		}
	}
	fclose(fp);
	fclose(fp_cur);
	return 0;
}
int main(int argc, const char* argv[])
{
	const char* mode = ui(argc, argv);

	if (strcmp(mode, "tx_immediate") == 0){
		return tx_immediate(TSCaster::factory(MultiCast::factory(G::group, G::port, MultiCast::MC_SENDER)));
	}else if (strcmp(mode, "tx") == 0){
		return sender(TSCaster::factory(MultiCast::factory(G::group, G::port, MultiCast::MC_SENDER)));
	}else{
		return receiver(TSCaster::factory(MultiCast::factory(G::group, G::port, MultiCast::MC_RECEIVER)));
	}
}

