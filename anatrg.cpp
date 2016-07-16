/* ------------------------------------------------------------------------- *
 * anatrg.cpp : set analog trigger thresholds
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
#include <stdio.h>



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
#include <vector>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "acq-util.h"

using namespace std;

namespace G {
	int site = 0;
	int hysteresis = 5;
	int channel = 1;
	int verbose;
	int dummy;
	int nchan = 32;
	void* mapping;
};

struct poptOption opt_table[] = {
	{ "site", 	's', POPT_ARG_INT, &G::site, 0, "site" },
	{ "channel",    'c', POPT_ARG_INT, &G::channel, 0, "channel" },
	{ "nchan",      'n', POPT_ARG_INT, &G::nchan, 0, "number of channels" },
	{ "hysteresis", 'H', POPT_ARG_INT, &G::hysteresis, 0, "set deadband" },
	{ "verbose", 	'v', POPT_ARG_INT, &G::verbose, 0, "" },
	{ "dummy",      'd', POPT_ARG_INT, &G::dummy, 0, "do not map device" },
	POPT_AUTOHELP
	POPT_TABLEEND
};

#define ALL_CHANNELS 0

unsigned clp(int x)
{
	if (x > 127) x = 127;
	if (x < -128) x = -128;

	return x & 0x00ff;
}


unsigned ABCD(int a, int b, int c, int d)
{
	return clp(a) << 24 | clp(b) << 16 | clp(c) << 8 | clp(d);
}

#define DISABLE_HI	127
#define DISABLE_LO	-128

#define DEFAULT_P	-1

int getValue(const char* str)
{
	int xx;

	if (str == 0){
		xx = DEFAULT_P;
	}else if (strstr(str, "%")){
		xx = atoi(str) * 255/100;
	}else{
		xx = atoi(str);
	}

	return xx;
}

void make_mapping()
{
	char fname[80];
	sprintf(fname, "/dev/acq400.%d.atd", G::site);
	int fd = open(fname, O_RDWR);
	if (fd == -1){
		perror(fname);
		exit(1);
	}
	G::mapping = mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	if (G::mapping == MAP_FAILED){
		perror("mmap fail");
		exit(1);
	}
}
const char* cli(int argc, const char** argv, int& p1, int& p2)
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;
	char pname[80];
	strncpy(pname, argv[0], 79);

	sscanf(basename(pname), "anatrg_%d", &G::channel);
	//printf("argv[0] %s channel:%d\n",basename(pname), G::channel);

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		default:
			;
		}
	}

	const char* verb = poptGetArg(opt_context);
	p1 = getValue(poptGetArg(opt_context));
	p2 = getValue(poptGetArg(opt_context));

	if (!G::dummy){
		make_mapping();
	}
	return verb;
}

struct Command {
	const char* key;
	Command(const char* _key) : key(_key) {}
	virtual unsigned operator() (int p1, int p2) = 0;
	bool match(const char *testkey){
		return strcmp(key, testkey) == 0;
	}
};
class AnaTrg {
	vector<Command*> commands;
	AnaTrg();
	void setThreshold(int chan, unsigned abcd){
		unsigned *thblock = reinterpret_cast<unsigned *>(G::mapping);
		thblock[chan-1] = abcd;
	}
	void setThreshold(unsigned abcd) {

		if (G::channel == ALL_CHANNELS){
			for (int ic = 1; ic <= G::nchan; ++ic){
				setThreshold(ic, abcd);
			}
		}else{
			setThreshold(G::channel, abcd);
		}
	}
public:
	static AnaTrg& instance();
	int operator() (const char* key, int p1, int p2) {
		for (vector<Command*>::iterator it = commands.begin();
				it != commands.end(); ++it){
			Command& cursor = **it;
			if (strcmp(key, "help") == 0){
				printf("%s\n", cursor.key);
			}else if (cursor.match(key)){
				setThreshold(cursor(p1, p2));
				return 0;
			}
		}

		return -1;
	}
};

struct NoneCommand : public Command {
	NoneCommand() : Command("none") {}
	virtual unsigned operator() (int p1, int p2){
		return ABCD(DISABLE_HI, DISABLE_LO, DISABLE_HI, DISABLE_LO);
	}
};

struct RisingCommand: public Command {
	RisingCommand() : Command("rising") {}
	virtual unsigned operator() (int p1, int p2) {
		unsigned abcd;

		abcd = ABCD(p1, p1 - G::hysteresis, DISABLE_HI, DISABLE_LO);
		if (G::verbose) printf("%s %d %d ABCD: 0x%08x\n", key, p1, p2, abcd);

		return abcd;
	}
};

struct FallingCommand: public Command {
	FallingCommand() : Command("falling") {}
	virtual unsigned operator() (int p1, int p2) {
		unsigned abcd;

		abcd = ABCD(DISABLE_HI, DISABLE_LO, p1 + G::hysteresis, p1);
		if (G::verbose) printf("%s %d %d ABCD: 0x%08x\n", key, p1, p2, abcd);

		return abcd;
	}
};

struct InsideCommand: public Command {
	InsideCommand() : Command("inside") {}
	virtual unsigned operator() (int p1, int p2) {
		unsigned abcd;

		abcd = ABCD(p1, p1 - G::hysteresis, p2 + G::hysteresis, p2);
		if (G::verbose) printf("%s %d %d ABCD: 0x%08x\n", key, p1, p2, abcd);
		return abcd;
	}
};

struct OutsideCommand: public Command {
	OutsideCommand() : Command("outside") {}
	virtual unsigned operator() (int p1, int p2) {
		unsigned abcd;
		abcd = ABCD(p1, p2 - G::hysteresis, p1 + G::hysteresis, p2);
		if (G::verbose) printf("%s %d %d ABCD: 0x%08x\n", key, p1, p2, abcd);
		return abcd;
	}
};

AnaTrg::AnaTrg()
{
	commands.push_back(new NoneCommand);
	commands.push_back(new RisingCommand);
	commands.push_back(new FallingCommand);
	commands.push_back(new InsideCommand);
	commands.push_back(new OutsideCommand);
}

AnaTrg& AnaTrg::instance()
{
	static AnaTrg* _instance = new AnaTrg;

	return *_instance;
}
int main(int argc, const char** argv)
{
	if (getenv("SITE")){
		G::site = atoi(getenv("SITE"));
	}
	const char* verb;
	int p1;
	int p2;

	if ((verb = cli(argc, argv, p1, p2)) != 0){
		AnaTrg& at = AnaTrg::instance();

		return at(verb, p1, p2);
	}
	return 0;
}
