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
 *   WRTD_RX_MATCHES=m1[,m2,m3...]   # receiver matches on multiple strings, not just default ["acq2106"[7]], operates WRTT[0]
 *   WRTD_RX_MATCHES1=mx[my..]       # match strings operate WRTT1. WRTT0 has priority, strings MUST be unique.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <glob.h>
#include <libgen.h>

#include <assert.h>

#include "split2.h"

#include "popt.h"

#include "local.h"
#include "Env.h"
#include "File.h"
#include "Knob.h"
#include "Multicast.h"


#define M1 1000000
#define NSPS	(1000*M1)		// nanoseconds per second

#define MAX_TX_INF	0xFFFFFFFF


typedef std::vector<std::string> VS;


#include <sched.h>
void goRealTime(int sched_fifo_priority)
{
        struct sched_param p = {};
        p.sched_priority = sched_fifo_priority;

        int rc = sched_setscheduler(0, SCHED_FIFO, &p);

        if (rc){
                perror("failed to set RT priority");
        }
}


#define DEV_TS		"/dev/acq400.0.wr_ts"	// blocking device returns TIMESTAMP
#define DEV_CUR		"/dev/acq400.0.wr_cur"	// noblock device returns current TAI
#define DEV_TRG0	"/dev/acq400.0.wr_trg0" // write trigger0 definition here
#define DEV_TRG1	"/dev/acq400.0.wr_trg1" // write trigger1 definition here

FILE *fopen_safe(const char* fname, const char* mode = "r")
{
	FILE *fp = fopen(fname, mode);
	if (fp == 0){
		perror(fname);
		exit(errno);
	}
	return fp;
}

namespace G {
	const char* group = "224.0.23.159";
	int port = 5044;
	int delta_ticks;
        unsigned ticks_per_sec = 80000000;
        int verbose = 0;
        unsigned dns = 40*M1;				// delta nsec
        double ns_per_tick = 50.0;			// ticks per nsec
        unsigned local_clkdiv;				// Site 1 clock divider, set at start
        unsigned local_clkoffset;			// local_clk_offset eg 2 x 50nsec for ACQ42x
        unsigned max_tx = 1;				// send max this many trigs
        bool max_tx_specified;				// TRUE if UI changed max_tx
        const char* tx_id;				// transmit id
        int rt_prio = 0;
        int trg = 0;					// trg 0 or 1, 2, decoded from message
        int delay01;					// tr==2? trg0 at time t, trg1 at t+delay01
        unsigned tx_mask;
        const char* dev_ts = DEV_TS;
        unsigned site;
}

#define REPORT_THRESHOLD (G::dns/4)

#define SECONDS_SHL	28
#define SECONDS_MASK	0x7
#define TICKS_MASK	0x0fffffff
#define TS_EN		(1<<31)

#define TS_QUICK	0xffffffffU			// trigger right away, no timing ..
#define TS_QUICK_TICKS	0x0fffffffU			// trigger right away, no timing ..

struct TS {

private:
	void make_raw(unsigned _secs, unsigned _ticks, bool en=true) {
		mask = 0;
		raw = (_secs&SECONDS_MASK) << SECONDS_SHL | (_ticks&TICKS_MASK) | (en?TS_EN:0);
	}
	unsigned inc(unsigned s){
		return ++s&SECONDS_MASK;
	}
	unsigned dec(unsigned s){
		return --s&SECONDS_MASK;
	}
public:
	unsigned raw;
	char repr[16];
	unsigned char mask;		/* possible, multiple receivers */

	TS(unsigned _raw = 0): raw(_raw), mask(0) {}
	TS(unsigned _secs, unsigned _ticks, bool en=true) {
		make_raw(_secs, _ticks, en);
	}
	TS(const char* def){
		 VS tsx;
		 split2<VS>(def, tsx, ':');
		 assert(tsx.size() == 2);
		 make_raw(strtoul(tsx[0].c_str(), 0, 10), strtoul(tsx[1].c_str(), 0, 10));
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
	bool operator== (TS& ts2) const {
		return ticks() == ts2.ticks() && secs() == ts2.secs();
	}
	bool operator!= (TS& ts2) const {
		return ticks() != ts2.ticks() || secs() != ts2.secs();
	}
	long diff(TS& ts2){
		unsigned _ticks = ticks();
		unsigned _secs = secs();
		if (ts2.ticks() > ticks()){
			_ticks += G::ticks_per_sec;
			_ticks -= ts2.ticks();
			_secs = dec(_secs);
		}else{
			_ticks -= ts2.ticks();
		}
		return (_secs - ts2.secs())*NSPS + _ticks*G::ns_per_tick;
	}

	const char* toStr(void) {
		sprintf(repr, "%d:%07d", secs(), ticks());
		return repr;
	}
	static int do_ts_diff(const char* _ts1, const char* _ts2){
		TS ts1(_ts1);
		TS ts2(_ts2);
		fprintf(stderr, "ts1:%s ts2:%s diff:%ld\n", ts1.toStr(), ts2.toStr(), ts1.diff(ts2));
		return 0;
	}

	static TS ts_quick;
};

TS TS::ts_quick = TS_QUICK;

struct poptOption opt_table[] = {
	{
	  "tickns", 0, POPT_ARG_INT, &G::ns_per_tick, 0, "tick size nsec"
	},
	{
	  "dns", 'd', POPT_ARG_INT, &G::dns, 0, "nsec to add to current time"
	},
	{
	  "delta_ns", 'd', POPT_ARG_INT, &G::dns, 0, "nsec to add to current time"
	},
	{
	  "rt_prio", 'p', POPT_ARG_INT, &G::rt_prio, 0, "real time priority"
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
	  "max_tx", 0, POPT_ARG_INT, &G::max_tx, 'm', "maximum transmit count"
	},
	{
          "tx_id", 0, POPT_ARG_STRING, &G::tx_id, 0, "txid: default is $(hostname)"
	},
	{
	  "delay01", 0, POPT_ARG_INT, &G::delay01, 0, "in double tap, delay to second trigger"
	},
	{
	  "tx_mask", 0, POPT_ARG_INT, &G::tx_mask, 0, "mask for TIGA trigger tx"
	},
	{
	  "dev_ts", 0, POPT_ARG_STRING, &G::dev_ts, 0, "timestamp device eg may be a TIGA site.."
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};


bool is_tiga()
{
	Knob k(0, "wr_tai_trg_s1");
	return k.exists();
}

const char* ui_get_cmd_name(const char* path)
{
	char* cmd_name = new char[strlen(path)+1];
	strcpy(cmd_name, path);
	return basename(cmd_name);
}

const char* ui(int argc, const char** argv)
{
	const char* cmd_name = ui_get_cmd_name(argv[0]);

        poptContext opt_context =
                        poptGetContext(argv[0], argc, argv, opt_table, 0);
        int rc;

        G::ns_per_tick 	= 	Env::getenv("WRTD_TICKNS", 	50.0	);
        G::dns 		= 	Env::getenv("WRTD_DELTA_NS", 	50000000);
        G::tx_id 	= 	Env::getenv("WRTD_ID", 	"WRTD0"	);
        G::verbose 	= 	Env::getenv("WRTD_VERBOSE", 	0	);
        G::rt_prio	= 	Env::getenv("WRTD_RTPRIO", 	0	);
        G::delay01	= 	Env::getenv("WRTD_DELAY01", 	1000000	);
        G::tx_mask	= 	Env::getenv("WRTD_TX_MASK", 	0	);
        G::dev_ts	= 	Env::getenv("WRTD_DEV_TS",    DEV_TS	);
        G::local_clkoffset = 	Env::getenv("WRTD_LOCAL_CLKOFFSET",	0);
        G::local_clkdiv = 	Env::getenv("WRTD_LOCAL_CLKDIV",    	1);

        const char* ip_multicast_if = ::getenv("WRTD_MULTICAST_IF");
        if (ip_multicast_if){
        	MultiCast::set_IP_MULTICAST_IF(ip_multicast_if);
        }
        if (!is_tiga()){
        	Knob clkdiv(1, "clkdiv");
        	Knob modname(1, "module_name");
        	if (strstr(modname(), "acq48")){
        		G::local_clkdiv = 1;
        		G::local_clkoffset = 0;
        	}else{
        		clkdiv.get((unsigned*)&G::local_clkdiv);
        		G::local_clkoffset = 2;
        	}
        }
        while ((rc = poptGetNextOpt( opt_context )) >= 0 ){
                switch(rc){
                case 'm':
                	G::max_tx_specified = true;
                	break;
                default:
                        ;
                }
        }
        G::ticks_per_sec = NSPS / G::ns_per_tick;
        G::delta_ticks = G::dns / G::ns_per_tick;

        if (G::verbose) fprintf(stderr, "ns per tick: %.3f ticks per s: %u delta_ticks %u\n",
        		G::ns_per_tick, G::ticks_per_sec, G::delta_ticks);

        const char* mode = "wrtd_rx";

        if (strcmp(cmd_name, "wrtd") == 0){
        	 mode = poptGetArg(opt_context);
        }
        if (strcmp(mode, "ts_diff") == 0){
        	TS::do_ts_diff(poptGetArg(opt_context), poptGetArg(opt_context));
        	exit(0);
        }

        const char* tx_id = poptGetArg(opt_context);
        if (tx_id){
        	if (isdigit(tx_id[0])){
        		G::max_tx = atoi(tx_id);		// args N TXID
        		G::max_tx_specified = true;
        		tx_id = poptGetArg(opt_context);
        		if (tx_id && !isdigit(tx_id[0])){
        			G::tx_id = tx_id;
        		}
        	}else{
        		G::tx_id = tx_id;			// args TXID
        	}
        }
        							// else use defaults
        G::delay01 /= G::ns_per_tick;

	if (G::rt_prio){
		goRealTime(G::rt_prio);
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
	virtual void sendto(const TS& ts) {
		mc.sendto(&ts, sizeof(unsigned));
	}
	virtual void sendraw(unsigned raw) {
		mc.sendto(&raw, sizeof(unsigned));
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






class MultipleMatchFilter : public MessageFilter {
	std::vector<VS> matches;

	void append_match(const char* mx)
	{
		VS* _mx = new VS;
		if (mx){
			split2<VS>(mx, *_mx, ',');

		}
		matches.push_back(*_mx);
	}
public:
	MultipleMatchFilter(const char* m0, const char* m1, const char* m2){
		append_match(m0);
		append_match(m1);
		append_match(m2);
	}
	virtual bool operator() (struct wrtd_message& msg) {
		for (unsigned ii = 0; ii < matches.size(); ++ii){
			for (std::string ss : matches[ii]){
				if (strncmp(ss.c_str(), (char*)msg.event_id, WRTD_ID_LEN) == 0){
					G::trg = ii;
					return true;
				}
			}
		}
		return false;
	}
};


MessageFilter& MessageFilter::factory() {
	const char* matches = getenv("WRTD_RX_MATCHES");
	if (matches){
		return * new MultipleMatchFilter(matches, getenv("WRTD_RX_MATCHES1"), getenv("WRTD_RX_DOUBLETAP"));
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

	virtual void sendraw(unsigned raw) {
		msg.hw_detect[0] = 'L';
		msg.hw_detect[1] = 'X';
		msg.hw_detect[2] = 'I';
		msg.seq = seq++;
		msg.ts_sec = 0;			// truncated at 7.. worktodo use TAI
		msg.ts_ns = raw;
		//msg.event_id is pre-cooked, all other fields are zero
		msg.event_id[IMASK()] = G::tx_mask;	// use global default, NOT member ts ..
		mc.sendto(&msg, sizeof(msg));
		if (G::verbose) printLast();
	}
	virtual void sendto(const TS& ts) {
	        msg.hw_detect[0] = 'L';
	        msg.hw_detect[1] = 'X';
	        msg.hw_detect[2] = 'I';
	        msg.seq = seq++;
	        msg.ts_sec = ts.secs();			// truncated at 7.. worktodo use TAI
	        msg.ts_ns = ts.nsec();
	        //msg.event_id is pre-cooked, all other fields are zero
	        msg.event_id[IMASK()] = ts.mask;
	        mc.sendto(&msg, sizeof(msg));
	        if (G::verbose) printLast();
	}
	virtual int printLast() {
		return printf("%s %16s mask=%x seq=%u sec=%u ns=%u\n", msg.hw_detect, msg.event_id,
					msg.event_id[IMASK()], msg.seq, msg.ts_sec, msg.ts_ns);
	}
	virtual TS recvfrom() {
		while(true){
			if (mc.recvfrom(&msg, sizeof(msg)) != sizeof(msg)){
				perror("closedown");
				exit(1);
			}
			if (is_for_us(msg)){
				if (G::verbose){
					fprintf(stderr, "%s FOR US %08x %08x\n", PFN, msg.ts_ns, TS_QUICK);
					printLast();
				}
				if (msg.ts_ns == TS_QUICK){
					TS ts(TS_QUICK);
					ts.mask = msg.event_id[IMASK()];
					if (G::verbose){
						fprintf(stderr, "%s TS_QUICK ts:%s mask:%x\n", PFN, ts.toStr(), ts.mask);
					}
					return ts;
				}else{
					if (G::verbose){
						fprintf(stderr, "%s TS TIME ts:%s mask:%x\n", PFN, ts.toStr(), ts.mask);
					}
					TS ts(msg.ts_sec, msg.ts_ns/G::ns_per_tick);
					ts.mask = msg.event_id[IMASK()];
					return ts;
				}
			}
		}
	}
public:
	static const int IMASK(){
		return WRTD_ID_LEN-1;
	}
};
TSCaster& TSCaster::factory(MultiCast& _mc) {
	if (Env::getenv("WRTD_FULLMESSAGE", 1)){
		return * new WrtdCaster(_mc, MessageFilter::factory());
	}else{
		return * new TSCaster(_mc);
	}
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
	if (ts0 != TS::ts_quick && (G::local_clkdiv > 1 || G::local_clkoffset != 0)){
		return _adjust_ts(ts0);
	}else{
		return ts0;
	}
}

void _write_trg(FILE* fp, TS ts)
{
	int rc = fwrite(&ts.raw, sizeof(unsigned), 1, fp);
	if (rc < 1){
		perror("fwrite");
	}
	fflush(fp);
}


class Receiver {

protected:
	const int ntriggers;
	const long dms;

	FILE **fp_trg;
	FILE *fp_cur;

	char* report_fname;
	char* report;

	Receiver(int _ntriggers = 2) :  ntriggers(_ntriggers), dms(G::dns/M1), report_fname(new char[80]), report(new char[128]) {
		sprintf(report_fname, "/etc/acq400/%d/WRTD_REPORT", G::site);
		fp_trg = new FILE* [ntriggers];
		memset(fp_trg, 0, ntriggers*sizeof(FILE*));
		fp_trg[0] = fopen_safe(DEV_TRG0, "w");
		fp_trg[1] = fopen_safe(DEV_TRG1, "w");
		fp_cur = fopen_safe(DEV_CUR, "r");
	}
	virtual void onAction(TS ts, TS ts_adj){
		if (G::verbose){
			fprintf(stderr, "%s ts:%s ts_adj:%s mask:%x\n", PFN, ts.toStr(), ts_adj.toStr(), ts.mask);
		}
		if (ts.mask != 0){
			unsigned char mask = ts.mask;
			FILE *fp;

			for (int ii = 0; (ii < ntriggers) && mask; mask >>= 1, ++ii){
				if ((mask&1) && (fp = fp_trg[ii])){
					_write_trg(fp, ts_adj);
				}
			}
		}else if (G::trg < 2){
			_write_trg(fp_trg[G::trg], ts_adj);
		}else{							/* DOUBLE TAP */
			if (ts_adj != TS::ts_quick){
				_write_trg(fp_trg[0], ts_adj);
				_write_trg(fp_trg[1], adjust_ts( ts + G::delay01));
			}else{
				_write_trg(fp_trg[0], TS_QUICK);
				usleep(G::delay01*G::ns_per_tick/1000);
				_write_trg(fp_trg[1], TS_QUICK);
			}
		}

	}
public:
	virtual ~Receiver() {
		fclose(fp_trg[0]);
		fclose(fp_trg[1]);
		fclose(fp_cur);
		delete [] report;
		delete [] report_fname;
	}
	void action(TS ts, int nrx = 0){
		TS ts_adj = adjust_ts(ts);
		onAction(ts, ts_adj);
		TS ts_cur;
		fread(&ts_cur.raw, sizeof(unsigned), 1, fp_cur);

		long dt = ts.diff(ts_cur);

		snprintf(report, 128, "Receiver:%d nrx:%u cur:%s ts:%s adj:%s diff:%ld %s\n",
				G::trg, nrx, ts_cur.toStr(), ts.toStr(), ts_adj.toStr(), dt, dt<0? "ERROR": "OK");

		FILE *fp_report = fopen(report_fname, "w");
		fprintf(fp_report, report);
		fclose(fp_report);
		if (G::verbose > 1){
			fprintf(stderr, report);
		}


		if (dt < 0){
			fprintf(stderr, "wrtd rx ERROR missed ts by %ld msec\n", dt/M1);
		}else if (dt < (long)REPORT_THRESHOLD){
			fprintf(stderr, "wrtd rx WARNING threshold %ld msec under limit %ld\n", dt/M1, dms);
		}
	}
	int event_loop(TSCaster& comms) {
		for (unsigned nrx = 0;; ++nrx){
			TS ts = comms.recvfrom();
			if (G::verbose) comms.printLast();
			if (G::verbose) fprintf(stderr, "%s() TS:%s %08x\n", PFN, ts.toStr(), ts.raw);
			action(ts, nrx);
			if (G::verbose) comms.printLast();
		}
		return 0;
	}
	static Receiver* instance();
};


class TIGA_Receiver: public Receiver {

protected:
	TIGA_Receiver() : Receiver(8)
	{
		G::local_clkdiv = G::local_clkoffset = 0;		// stub clock adjust
		if (G::verbose){
			fprintf(stderr, "TIGA_Receiver()\n");
		}

		glob_t globbuf;
		glob("/dev/acq400.0.wr_tiga_tt_s?", 0, NULL, &globbuf);
		for (unsigned ii = 0; ii < globbuf.gl_pathc; ++ii){
			const char* fn = globbuf.gl_pathv[ii];
			int site = fn[strlen(fn)-1]-'0';

			if (G::verbose){
				fprintf(stderr, "TIGA_Receiver() fn:\"%s\" site:%d\n", fn, site);
			}

			if (site >= 1 && site <= 6){
				fp_trg[site+1] = fopen_safe(fn, "w");		/* site1 => [2] */
			}
		}
		globfree(&globbuf);
	}

friend class Receiver;
};

Receiver* Receiver::instance()
{
	static Receiver* _instance;

	if (!_instance){
		if (Env::getenv("WRTD_TIGA", 0)){
			_instance = new TIGA_Receiver;
		}else{
			_instance = new Receiver;
		}
	}
	return _instance;
}
class Transmitter {
	FILE* fp;
	const int sleep_us;
public:
	Transmitter(const char* dev, int _sleep_us = 0) :
		fp(::fopen_safe(dev)), sleep_us(_sleep_us)
	{}
	virtual ~Transmitter(){
		fclose(fp);
	}
	int event_loop(TSCaster& comms, Receiver* local_rx) {
		if (G::max_tx == 0){
			return 0;
		}
		TS ts;
		for (unsigned ntx = 0; fread(&ts.raw, sizeof(unsigned), 1, fp) == 1; ++ntx){
			TS ts_tx = ts + G::delta_ticks;
			ts_tx.mask = G::tx_mask;
			comms.sendto(ts_tx);
			if (local_rx){
				local_rx->action(ts_tx, ntx);
			}
			++ntx;
			if (G::verbose > 1) fprintf(stderr, "sender:ntx:%u ts:%s ts_tx:%s\n", ntx, ts.toStr(), ts_tx.toStr());
			if (G::max_tx != MAX_TX_INF && ntx >= G::max_tx){
				break;
			}else if (sleep_us){
				usleep(sleep_us);
			}
		}
		return 0;
	}
};

void get_local_env(void)
{
	G::verbose = Env::getenv("WRTD_VERBOSE", 0);
	G::site = Env::getenv("SITE", 11);
	char envname[80];
	sprintf(envname, "/dev/shm/wr%d.sh", G::site);
	get_local_env(envname, G::verbose);
}

int sleep_if_notenabled(const char* key)
{
	if (Env::getenv(key, 0) == 0){
		if (G::verbose){
			fprintf(stderr, "%s==0, sleep(9999)\n",key);
		}
		sleep(9999);
		return 1;
	}else{
		return 0;
	}
}


int txi() {
	if (G::verbose){
		fprintf(stderr, "%s\n", PFN);
	}
	Transmitter t(DEV_CUR, 2*G::dns/1000);
	Receiver* r = Env::getenv("WRTD_LOCAL_RX_ACTION", 0)? Receiver::instance(): 0;
	return t.event_loop(TSCaster::factory(MultiCast::factory(G::group, G::port, MultiCast::MC_SENDER)), r);
}
int tx() {
	if (!G::max_tx_specified){
		G::max_tx = MAX_TX_INF;
	}
	if (G::verbose){
		fprintf(stderr, "%s\n", PFN);
	}
	Transmitter t(G::dev_ts);
	Receiver* r = Env::getenv("WRTD_LOCAL_RX_ACTION", 0)? Receiver::instance(): 0;
	return t.event_loop(TSCaster::factory(MultiCast::factory(G::group, G::port, MultiCast::MC_SENDER)), r);
}

int txq() {
	if (G::verbose){
		fprintf(stderr, "%s\n", PFN);
	}
	TSCaster& comms = TSCaster::factory(MultiCast::factory(G::group, G::port, MultiCast::MC_SENDER));
	comms.sendraw(TS_QUICK);
	return 0;
}
int rx() {
	return Receiver::instance()->event_loop(
			TSCaster::factory(MultiCast::factory(G::group, G::port, MultiCast::MC_RECEIVER)));
}

int main(int argc, const char* argv[])
{
	get_local_env();
	const char* mode = ui(argc, argv);

	if (strcmp(basename((char*)argv[0]), "wrtd_txq") == 0 || strcmp(mode, "txq") == 0){
		return txq();
	}else if (strcmp(mode, "tx_immediate") == 0 || strcmp(mode, "txi") == 0){
		return txi();
	}else if (strcmp(mode, "tx") == 0){
		return sleep_if_notenabled("WRTD_TX") || tx();
	}else{
		return sleep_if_notenabled("WRTD_RX") || rx();
	}
}

