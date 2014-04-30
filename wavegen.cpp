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

using namespace std;

namespace Globs {
	int site = -1;
	const char* src_dir = "/usr/local/awgdata/ch";
	bool loop = false;
};

class ChanDef {
	char chan_def[132];
	char expr[132];

	void load_chan_def();
	static vector<int> mapping;
	static vector<ChanDef*> channels;
	static char mapdef[256];
	int ichan;

protected:
	const char* fname;
	short* data;
	int ndata;
	int cursor;

	ChanDef(int _ichan): ichan(_ichan), cursor(0)
	{}
public:

	ChanDef(const char* spec)
	{
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
	static void output() {
		char fn[256];
		sprintf(fn, "/dev/acq400.%d.%s",
				Globs::site, Globs::loop? "awgc": "awg");
		FILE *fpout = fopen(fn, "a");
		if (fpout == 0){
			perror(fn);
			exit(1);
		}

		for (int nbusy = nchan(), sample = 0; nbusy; ++sample){
			nbusy = 0;

			for (int ii = 0; ii < nchan(); ++ii){
				ChanDef* chan = channels[ii];

				/*
				if (sample < 3){
					cerr << chan->ichan << " " << chan->expr
					    << "[ " <<chan->cursor << "] = "
					    	    <<chan->data[chan->cursor] << endl;
				}
				*/
				fwrite(chan->data+chan->cursor, sizeof(short), 1, fpout);
				if (chan->cursor+1 < chan->ndata){
					if (++chan->cursor < chan->ndata){
						++nbusy;
					}
				}
			}
		}
		fclose(fpout);
	}
};

class NullChanDef : public ChanDef {

public:
	NullChanDef(int _ichan) : ChanDef(_ichan) {
		data = new short[1];
		data[0] = 0;
		ndata = 1;
		fname = "null";
	}
};
vector<int> ChanDef::mapping;

vector<ChanDef*> ChanDef::channels;
char ChanDef::mapdef[256];

void ChanDef::load_chan_def()
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
			data = channels[ii]->data;
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

	cerr << "stat:" << path << " length:" << buf.st_size << endl;
	ndata = buf.st_size/sizeof(short);
	data = new short[ndata];
	FILE *fp = fopen(path, "r");
	if (fread(data, sizeof(short), ndata, fp) != ndata){
		perror("failed to read all the data");
		exit(1);
	}
	fclose(fp);

	cerr << "New ChanDef:" <<ndata <<endl;
	channels[ichan] = this;
}

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
		channels.push_back(new NullChanDef(ii));
	}
}


struct poptOption opt_table[] = {
	{ "site", 's', POPT_ARG_INT, &Globs::site, 0, "site" },
	{ "src_dir", 0, POPT_ARG_STRING, &Globs::src_dir, 0, "data directory" },
	{ "loop",  'l', POPT_ARG_INT, &Globs::loop, 0, "runs in a loop" },
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

	ChanDef::getMapping();

	const char* arg;
	while((arg = poptGetArg(opt_context)) != 0){
		new ChanDef(arg);
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
