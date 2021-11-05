/*
 * wrtd_message.h
 *
 *  Created on: 5 Nov 2021
 *      Author: pgm
 */

#ifndef WRTD_MESSAGE_H_
#define WRTD_MESSAGE_H_


namespace G {
	int verbose = 0;
        int trg = 0;					// trg 0 or 1, 2, decoded from message
        const char* tx_id;				// transmit id
        unsigned tx_mask;
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


	void sendcommon(){
		msg.hw_detect[0] = 'L';
		msg.hw_detect[1] = 'X';
		msg.hw_detect[2] = 'I';
		msg.seq = seq++;

	        mc.sendto(&msg, sizeof(msg));
	        if (G::verbose) printLast();
	}
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
		msg.ts_sec = 0;
		msg.ts_ns = raw;
		//msg.event_id is pre-cooked, all other fields are zero
		msg.event_id[IMASK()] = G::tx_mask;	// use global default, NOT member ts ..

		sendcommon();
	}
	virtual void sendto(const TS& ts) {
	        msg.ts_sec = ts.secs();			// truncated at 7.. worktodo use TAI
	        msg.ts_ns = ts.nsec();
	        //msg.event_id is pre-cooked, all other fields are zero
	        msg.event_id[IMASK()] = ts.mask;

	        sendcommon();
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


class Receiver {
public:
	Receiver()
	{}
	virtual ~Receiver()
	{}
	virtual void onAction(TS ts, TS ts_adj)
	{}

	virtual void action(TS ts, int nrx = 0) {
		fprintf(stderr, "%s ts:%s mask:%x\n", PFN, ts.toStr(), ts.mask);
	}
	virtual int event_loop(TSCaster& comms) {
		for (unsigned nrx = 0;; ++nrx){
			TS ts = comms.recvfrom();
			if (G::verbose > 1) fprintf(stderr, "%s() TS:%s %08x\n", PFN, ts.toStr(), ts.raw);
			action(ts, nrx);
			if (G::verbose) comms.printLast();
		}
		return 0;
	}

	static Receiver* instance();
};







#endif /* WRTD_MESSAGE_H_ */
