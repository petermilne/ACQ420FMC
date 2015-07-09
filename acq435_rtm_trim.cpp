/* ------------------------------------------------------------------------- *
 * acq435_rtm_trim.cpp
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2013 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 20 Nov 2013  
 *    Author: pgm                                                         
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h> // sigaction(), sigsuspend(), sig*()
#include <unistd.h> // alarm()
#include <pthread.h>
#include "popt.h"

int nchan = 32;
int ntransient = 16;
const char* decoder_type = "pass_thru";

#define MAGIC_MASK	0xfffff000
#define MAGIC		0xaa55f000

class Decoder {


protected:
	unsigned sample;
	int itran;
	bool waiting_es;

	virtual int onData(unsigned *data) {
		return 1;
	}

	bool magic(unsigned data) {
		return (data&MAGIC_MASK) == MAGIC;
	}
	bool is_es(unsigned *data) {
		return magic(data[0]) && magic(data[1]) &&
			magic(data[2]) && magic(data[3]);
	}

	Decoder(const char* name = "Decoder") : sample(0), itran(0), waiting_es(true)
	{
		//fprintf(stderr, "%s nchan:%d ntransient:%d\n", name, nchan, ntransient);
	}

public:
	/** return tru if data is an OUTPUT */
	int operator() (unsigned *data)
	{
		if (waiting_es){
			if (is_es(data)){
				waiting_es = false;
				itran = 0;
			}
			return 0;
		} else {
			if (itran++ < ntransient){
				return onData(data);
			}else{
				waiting_es = true;
				return 0;
			}
		}
	}

	static Decoder& instance();

	static int sample_size() {
		return nchan * sizeof(unsigned);
	}
};

class SummingDecoder : public Decoder {
	unsigned *sums;
	int ntbits;


	void clear(unsigned *data) {
		memset(data, 0, sample_size());
	}
protected:
	virtual int onData(unsigned *data) {
		int rc = 0;
		if (itran == 1){
			clear(sums);
		}
		for (int ii = 0; ii < nchan; ++ii){
			sums[ii] += data[ii] >> 8;
		}

		if (itran == ntransient){
			int shl = 8 - ntbits;
			for (int ii = 0; ii < nchan; ++ii){
				data[ii] = (sums[ii] << shl) | (data[ii]&0x00ff);
			}
			rc = 1;
		}
		return rc;
	}
public:
	SummingDecoder() : Decoder("SummingDecoder") {
		for (ntbits = 8; ntransient < (1<<ntbits); --ntbits)
			;
		sums = new unsigned[nchan];
		clear(sums);
	}
};

Decoder& Decoder::instance() {
	if (strcmp(decoder_type, "Summing") == 0){
		return *new SummingDecoder;
	}else{
		return *new Decoder;
	}
}
struct poptOption opt_table[] = {
	{ "nchan", 'n', POPT_ARG_INT, &nchan, 0, "channel count [32]" },
	{ "transient-length", 't', POPT_ARG_INT, &ntransient, 0, "transient to process [16]" },
	{ "processor", 'p', POPT_ARG_STRING, &decoder_type, 0, "processor [pass thru] / {Summing}" },
	POPT_AUTOHELP
	POPT_TABLEEND
};

void cli(int argc, const char** argv)
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
			;
		}
	}
	if (ntransient < 1) ntransient = 1;
}




int main(int argc, const char** argv)
{
	cli(argc, argv);

	Decoder& decoder(Decoder::instance());

	unsigned *data = new unsigned[nchan];

	while(fread(data, sizeof(unsigned), nchan, stdin) == nchan){
		if (decoder(data)){
			fwrite(data, sizeof(unsigned), nchan, stdout);
		}
	}
}
