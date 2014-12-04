/* ------------------------------------------------------------------------- *
 * wavegen.cpp  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 29 Apr 2014  
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

/*
 * create a waveform in site $SITE. Data specified by channel

wavegen cdef=wavespec

cdef: 1,2 .. N  also a range eg :
wavespec:  filename[*N][+/-V]

 * --srcdir[=/usr/local/awgdata
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h> // sigaction(), sigsuspend(), sig*()
#include <unistd.h> // alarm()
#include <pthread.h>
#include "popt.h"
#include <vector>
#include <string>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <ctime>
#include <fstream>

#include "acq-util.h"

using namespace std;

namespace Globs {
	int site = -1;
	const char* src_dir = "/usr/local/awgdata/ch";
	int loop = false;
	int autorearm = false;
	int ch_offset = 0;
	bool has_gain;
	double ch_gain = 1;
	int verbose;
	int append;
};

#define MINBYTES	0x20000


class ChanDef {

public:
	char chan_def[132];
	char expr[132];

	static vector<int> mapping;
	static vector<ChanDef*> channels;
	static char mapdef[256];
	int ichan;
	const char* fname;
	int ndata;
	int cursor;
	static bool raw_data_unsigned;
	static bool raw_data_signage_check_done;

	static void check_signage() {
		if (raw_data_signage_check_done) return;

		char knob[128];
		sprintf(knob, "/dev/acq400.%d.knobs/dac_encoding", Globs::site);
		FILE* fp = fopen(knob, "r");
		if (fp == 0){
			perror(knob);
		}else{
			fgets(knob, 128, fp);
			if (strstr(knob, "unsigned") != 0){
				cerr << "dac uses unsigned data" << endl;
				raw_data_unsigned = true;
			}
			raw_data_signage_check_done = true;
		}
		fclose(fp);
	}
	ChanDef(int _ichan): ichan(_ichan), cursor(0)
	{
		check_signage();
	}
public:
	virtual bool fwrite1(FILE* fp) = 0;
	static int word_size;

	ChanDef(const char* spec)
	{
		check_signage();
	}

	static int nchan() {
		return mapping.size();
	}
	static void getMapping();

	void print() {
		cerr << fname << " ndata:" << ndata << endl;
	}
	static void list() {
		cerr << "site:" << Globs::site << " sdir:" << Globs::src_dir << endl;
		cerr << "mapdef:" << mapdef <<endl;
		for (int ii = 0; ii < nchan(); ++ii){
			channels[ii]->print();
		}
	}

	static int output(FILE *fpout) {
		int nwrite = 0;

		for (int nbusy = nchan(), sample = 0; nbusy; ++sample){
			nbusy = 0;

			for (int ii = 0; ii < nchan(); ++ii){
				ChanDef* chan = channels[ii];
				if (sample == 0) chan->cursor = 0;
				if (chan->fwrite1(fpout)){
					++nbusy;
				}
				if (chan->cursor > nwrite){
					nwrite = chan->cursor;
				}
			}
		}

		return nwrite;
	}
	static void output() {
		char fn[256];
		sprintf(fn, "/dev/acq400.%d.%s",
				Globs::site,
				Globs::loop? "awgc":
				Globs::autorearm? "awgr": "awg");
		FILE *fpout = fopen(fn, Globs::append? "r+": "w");
		if (fpout == 0){
			perror(fn);
			exit(1);
		}

		int totwrite = 0;
		int minsam = MINBYTES/nchan()/word_size;

		while (totwrite < minsam){
			totwrite += output(fpout);
		}

		fclose(fpout);
	}

	static void create(const char* spec);
};

int ChanDef::word_size = 2;
bool ChanDef::raw_data_unsigned;
bool ChanDef::raw_data_signage_check_done;

template <class T>
class ChanDefImpl: public ChanDef {
	void convert_to_unsigned() {
		T sbit = 1<<(ChanDef::word_size == 2? 15: 31);

		for (int ii = 0; ii < ndata; ++ii){
			data[ii] ^= sbit;
		}
	}
public:
	T* data;

	virtual bool fwrite1(FILE* fp) {
		bool busy = false;

		T sam = data[cursor];

		sam += Globs::ch_offset * ichan;

		fwrite(&sam, sizeof(T), 1, fp);

		if (cursor+1 < ndata){
			if (++cursor < ndata){
				busy = true;
			}
		}
		return busy;
	}

	void load_chan_def()
	/* @todo: load single channel, single file, no prams. That comes later ..*/
	{
		int lchan = atoi(chan_def);

		if (lchan < 0 || lchan > nchan()) {
			fprintf(stderr, "ERROR: channel not in range 1..%d\n", nchan());
		}else{
			ichan = mapping[lchan-1] - 1;
		}
		fname = expr;
		/* avoid duplicating source data */
		for (int ii = 0; ii < channels.size(); ++ii){
			if (strcmp(channels[ii]->fname, fname) == 0){
				data = dynamic_cast<ChanDefImpl<T> *>(channels[ii])->data;
				ndata = channels[ii]->ndata;
				channels[ichan] = this;
				return;
			}
		}
		char path[256];
		if (fname[0] == '/'){
			sprintf(path, "%s", fname);
		}else{
			sprintf(path, "%s/%s", Globs::src_dir, fname);
		}

		struct stat buf;
		if (stat(path, &buf) != 0){
			perror(path);
			exit(1);
		}

		if (Globs::verbose) cerr << "stat:" << path << " length:" << buf.st_size << endl;
		ndata = buf.st_size/sizeof(T);
		data = new T[ndata];
		FILE *fp = fopen(path, "r");
		if (fread(data, sizeof(T), ndata, fp) != ndata){
			perror("failed to read all the data");
			exit(1);
		}
		fclose(fp);

		if (Globs::has_gain){
			for (int ic = 0; ic < ndata; ++ic){
				double dsam = data[ic];
				dsam *= Globs::ch_gain;
				data[ic] = dsam;
			}
		}
		if (raw_data_unsigned){
			convert_to_unsigned();
		}
		if (Globs::verbose) cerr << "New ChanDef:" <<ndata <<endl;
		channels[ichan] = this;
	}

	ChanDefImpl(const char* spec) : ChanDef(spec) {
		string ss(spec);
		string delim("=");
		size_t pos = 0;

		//cerr << "ChanDef(" << ss << ")" <<endl;
		if ((pos = ss.find(delim)) != string::npos){
			strcpy(chan_def, ss.substr(0, pos).c_str());
			ss.erase(0, pos + delim.length());
			strcpy(expr, ss.c_str());
			load_chan_def();
		}else{
			fprintf(stderr, "ERROR, unable to use arg \"%s\"\n", spec);
			exit(1);
		}
	}
	ChanDefImpl(const char* _chan_def, const char* _expr) : ChanDef(_chan_def){
		strcpy(chan_def, _chan_def);
		strcpy(expr, _expr);
		load_chan_def();
	}
	ChanDefImpl(int _chan_def, const char* _expr) : ChanDef(_chan_def){
			sprintf(chan_def, "%d", _chan_def);
			strcpy(expr, _expr);
			load_chan_def();
	}
	ChanDefImpl(int _ichan): ChanDef(_ichan)
	{
		data = new T[1];
		data[0] = 0;
		ndata = 1;
		fname = "null";
	}
};

void ChanDef::create(const char* spec) {
	char _chan_def[128];
	char _expr[128];
	string ss(spec);
	string delim("=");
	size_t pos = 0;

	int* channels = new int[nchan()+1]; /* index from 1 */
	int ndef = 0;

	//cerr << "ChanDef(" << ss << ")" <<endl;
	if ((pos = ss.find(delim)) != string::npos){
		strncpy(_chan_def, ss.substr(0, pos).c_str(), 127);
		ss.erase(0, pos + delim.length());
		strncpy(_expr, ss.c_str(), 127);
		for (int ichan = 1; ichan <= nchan(); ++ichan){
			channels[ichan] = 0;
		}
		ndef = acqMakeChannelRange(channels, nchan(), _chan_def);
	}else{
		fprintf(stderr, "ERROR, unable to use arg \"%s\"\n", spec);
		exit(1);
	}
	for (int ch = 1; ch <= nchan(); ++ch){
		char expanded_expr[128];
		sprintf(expanded_expr, _expr, ch);

		if (channels[ch]){
			switch(ChanDef::word_size){
			case 2:
				new ChanDefImpl<short>(ch, expanded_expr);
				break;
			case 4:
				new ChanDefImpl<long>(ch, expanded_expr);
				break;
			default:
				fprintf(stderr, "ERROR: word_size MUST be 2 or 4");
				exit(-1);
			}
		}
	}
	delete [] channels;
}

vector<int> ChanDef::mapping;

vector<ChanDef*> ChanDef::channels;
char ChanDef::mapdef[256];

void ChanDef::getMapping()
{
	if (Globs::site == -1){
		fprintf(stderr, "ERROR SITE not defined");
		exit(1);
	}
	char cmapfile[128];
	sprintf(cmapfile, "/proc/driver/acq400/%d/channel_mapping", Globs::site);

	FILE* fp = fopen(cmapfile, "r");
	if (fp == 0) {
		fprintf(stderr, "ERROR, failed to access \"%s\"\n", cmapfile);
	}
	fgets(mapdef, 256, fp);
	fclose(fp);

	std::stringstream ss(mapdef);

	int i;

	while (ss >> i){
	    mapping.push_back(i);

	    if (ss.peek() == ',')
	        ss.ignore();
	}

	for (int ii = 0; ii < nchan(); ++ii){
		switch(ChanDef::word_size){
		case 2:
			channels.push_back(new ChanDefImpl<short>(ii));
			break;
		case 4:
			channels.push_back(new ChanDefImpl<long>(ii));
			break;
		default:
			fprintf(stderr, "ERROR: word_size MUST be 2 or 4");
			exit(-1);
		}
	}
}


struct poptOption opt_table[] = {
	{ "site", 	's', POPT_ARG_INT, &Globs::site, 0, "site" },
	{ "src_dir", 	0, POPT_ARG_STRING, &Globs::src_dir, 0, "data directory" },
	{ "loop",  	'l', POPT_ARG_INT, &Globs::loop, 0, "runs in a loop" },
	{ "rearm", 	'R', POPT_ARG_INT, &Globs::autorearm, 0, " auto rearm after oneshot" },
	{ "word_size", 	'w', POPT_ARG_INT, &ChanDef::word_size, 0, "word size 2 or 4" },
	{ "ch_gain", 	0, POPT_ARG_DOUBLE, &Globs::ch_gain, 'g', "gain up xx *ch_gain" },
	{ "ch_offset", 	0, POPT_ARG_INT, &Globs::ch_offset, 0, "ident offset + ch*ch_offset" },
	{ "append", 	'a', POPT_ARG_INT, &Globs::append, 0, "append data" },
	{ "verbose", 	'v', POPT_ARG_INT, &Globs::verbose, 0, "" },
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
		case 'g':
			Globs::has_gain = true;
			printf("gain set: %f\n", Globs::ch_gain);
			break;
		default:
			;
		}
	}

	ChanDef::getMapping();

	const char* arg;
	while((arg = poptGetArg(opt_context)) != 0){
		ChanDef::create(arg);
	}
}

int main(int argc, const char** argv)
{
	if (getenv("SITE")){
		Globs::site = atoi(getenv("SITE"));
	}
	cli(argc, argv);

	ChanDef::list();
	ChanDef::output();
}
