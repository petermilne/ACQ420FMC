/* ------------------------------------------------------------------------- *
 * acq435_decode.cpp  		                     	                    
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

/*
 *  reads 435 data and spots problems.
 *
 */

unsigned long long nb;
unsigned long long nb0;
int ecount;


int progrep;
int monitor_interval;	/* seconds between monitor reports, 0: disable */

class Decoder {
	const int NC;
protected:
	unsigned imatch;

	Decoder() : NC(32), imatch(0) {}
	Decoder(int nc) : NC(nc), imatch(0) {}

	unsigned incr(unsigned ii) {
		return ++ii & 0x1f;
	}
public:
	static Decoder* create(const char* key);

	virtual bool analyse(unsigned *data){
		for (int ii = 0; ii < NC; ++ii, imatch = incr(imatch), ++nb){
			if ((data[ii]&0x1f) == imatch){
				continue;
			}else{
				fprintf(stderr, "Error:%4d %10lld %10lld %d %08x %x expected %x\n",
						++ecount, nb, nb-nb0, ii,
						data[ii], data[ii]&0x1f, imatch);
				imatch = data[ii]&0x1f;
				nb0 = nb;
				return false;
			}
		}
		return true;
	}
	virtual void monitor() {
		printf("monitor:\n");
	}
};

class ScratchpadDecoder : public Decoder {

	unsigned prev_sample;
	bool has_prev_sample;

	unsigned long long sample_count;
	unsigned decoder_errors;
	unsigned missed_sample_errors;

public:
	ScratchpadDecoder() :
		Decoder(24), prev_sample(0), has_prev_sample(false),
			sample_count(0), decoder_errors(0), missed_sample_errors(0)
	{
		printf("ScratchpadDecoder\n");
	}

	virtual bool analyse(unsigned *data){
		++sample_count;
		if (Decoder::analyse(data)){
			unsigned new_sample = data[24];
			if (has_prev_sample){
				if (new_sample != prev_sample+1){
					++missed_sample_errors;
					fprintf(stderr, "ERROR: sample not in sequence %u -> %u  %u\n",
						prev_sample, new_sample, new_sample-prev_sample);
				}
			}
			prev_sample = new_sample;
			has_prev_sample = true;
			imatch = 0;

			if (progrep && new_sample%progrep == 0){
				printf("INFO: %d\n", new_sample);
			}
			return true;
		}else{
			++decoder_errors;
			return false;
		}
	}
	virtual void monitor() {
		char message[256];
		int len = snprintf(message, 256, "samples:%10lld ES:{%08x %10d}",
				sample_count, prev_sample, prev_sample);
		if (decoder_errors || missed_sample_errors){
			snprintf(message+len, 256-len, "ERRORS: rot:%u skip:%u",
					decoder_errors, missed_sample_errors);
		}
		puts(message);
	}
};

Decoder* Decoder::create(const char* key){
	if (strcmp(key, "scratchpad") == 0){
		return new ScratchpadDecoder;
	}else{
		return new Decoder;
	}
}

void resync(FILE *fin)
{
	unsigned data;
	int ic = 0;
	int resync_count = 0;

	for (; fread(&data, sizeof(unsigned), 1, fin) == 1; ++resync_count){
		unsigned test_id = data&0x1f;
		switch(ic){
		case 0:
			ic = test_id == 0? 1: 0;
			break;
		case 1:
			ic = test_id == 1? 2: 0;
			break;
		case 2:
			ic = test_id == 2? 3: 0;
			break;
		case 3:
			ic = test_id == 3? 4: 0;
			break;
		default:
			if (++ic == 32){
				printf("resync count %d\n", resync_count);
				return;
			}
		}
	}
}

const char* decoder_type = "";
struct poptOption opt_table[] = {
	{ "monitor", 'm', POPT_ARG_INT, &monitor_interval, 0, "status monitor" },
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
	const char* _decoder_type = poptGetArg(opt_context);
	if (_decoder_type){
		decoder_type = _decoder_type;
	}
}



void *monitor(void *data)
{
	Decoder *decoder = static_cast<Decoder*>(data);
	for (unsigned runtime = 0; ; ++runtime){
		sleep(monitor_interval);
		printf("monitor:%5ds ", runtime);
		decoder->monitor();
	}
	return 0;
}

void start_monitor(void *data) {
	pthread_t mt;

	if(pthread_create(&mt, NULL, monitor, data)) {
		fprintf(stderr, "Error creating thread\n");
		exit(1);
	}
}
int main(int argc, const char** argv)
{
	cli(argc, argv);
	unsigned data[32];
	Decoder* decoder = Decoder::create(decoder_type);

	if (monitor_interval){
		start_monitor(decoder);
	}
	if (getenv("PROGREP")){
		progrep = atoi(getenv("PROGREP"));
	}
	while(fread(data, sizeof(unsigned), 32, stdin) == 32){
		if (!decoder->analyse(data)){
			resync(stdin);
		}
	}
}
