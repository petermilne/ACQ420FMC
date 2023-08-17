/*
 * awg_composer.cpp
 *
 *  Created on: 15 Aug 2023
 *      Author: pgm
 *
 *
 *
generic awg_composer invocation:
awg_composer --nreps=xx SEG1=NSEG1 SEG2=NSEG2

Generic Binary file interface:
/tmp/AWGCHANS/XXX
/tmp/AWGCHANS/XXX/thing_N.dat   # CH N template

 */

#define _POSIX_SOURCE
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#undef _POSIX_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <assert.h>
#include <vector>

#include "popt.h"

namespace G {
	int nchan = 4;
	int data32;
	const char* root = "/tmp/AWG";
	int verbose;
	int nreps = 1;
};


struct poptOption opt_table[] = {
	{
	  "nreps", 'N', POPT_ARG_INT, &G::nreps, 0, "repetitions of entire pattern"
	},
	{
	  "nchan", 'n', POPT_ARG_INT, &G::nchan, 0, "number of channels"
	},
	{
	 "data32", 0, POPT_ARG_INT, &G::data32, 0, "set 1 for 32 bit data"
	},
	{
	 "root", 0, POPT_ARG_STRING, &G::root, 0, "set file root"
	},
	{
	  "verbose", 'v', POPT_ARG_INT, &G::verbose, 0, "debug"
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};

struct Segment {
	int reps;
	char* name;

	Segment(int _reps, const char* _name): reps(_reps) {
		name = new char[strlen(_name)+1];
		strcpy(name, _name);
	}
	virtual ~Segment() {
		delete [] name;
	}

	static Segment* factory(const char* segdef);
	static std::vector<Segment*> segments;
};
std::vector<Segment*> Segment::segments;

template<class T>
struct ConcreteSegment: public Segment {
	T* data;
	struct ChFile {
		int ch;
		char* fname;
		int len;
	};
	std::vector<ChFile*> files;

	void get_files()
	{

	}
	ConcreteSegment(int _reps, const char* _name): Segment(_reps, _name), data(0)
	{
		get_files();
	}
};

Segment* Segment::factory(const char* segdef){
	char _segname[128];
	int reps = 1;
	const char* segname = _segname;
	if (sscanf(segdef, "%d*%s", &reps, _segname) == 2) {
		;
	}else{
		segname = segdef;
	}
	if (G::data32){
		return new ConcreteSegment<int>(reps, segname);
	}else{
		return new ConcreteSegment<short>(reps, segname);
	}
}


void ui(int argc, const char** argv)
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;

	while ((rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		default:
			;
		}
	}
	const char* segdef;
	while ((segdef = poptGetArg(opt_context)) != 0){
		Segment::segments.push_back(Segment::factory(segdef));
	}
}

int main(int argc, const char* argv[])
{
	ui(argc, argv);


	for (int rep = 0; rep < G::nreps; ++rep){
		for (Segment* segment: Segment::segments){
			for (int segrep = 0; segrep < segment->reps; ++segrep){

			}
		}
	}
}
