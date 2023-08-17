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

/* fixes error stat: Value too large for defined data type */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <assert.h>
#include <string>
#include <vector>

#include "popt.h"
#include "split2.h"

#include <iostream>
#include <fstream>

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

#define MAXNAME 128

struct TemplateFile;
typedef std::vector<TemplateFile*> VTF;
typedef std::vector<std::string> VS;

struct TemplateFile {
	std::string seg;
	int chan;
	int len;
	std::string fname;


	TemplateFile(std::string _seg, int _chan, int _len, std::string _fname):
		seg(_seg), chan(_chan), len(_len), fname(_fname)
	{}
	static TemplateFile* factory(VS fields);

	std::string toString(void){
		return seg + "," + std::to_string(chan) + "," + std::to_string(len) + "," + fname;
	}
};

VTF template_files;

#define MAXLS	65536

/*
 * [pgm@hoy5 acq400_hapi]$ cat /tmp/AWG/MANIFEST
0 /tmp/AWG/AA/cycloid-512-1000-11000-256-1200_1.dat 1536 AA 1 cycloid-512-1000-11000-256-1200_1.dat
1 /tmp/AWG/AA/parabola-512-0-16000-0_2.dat 512 AA 2 parabola-512-0-16000-0_2.dat
 *
 */
#define MAN_IDX 	0
#define MAN_FN  	1
#define MAN_LENW	2
#define MAN_SEG		3
#define MAN_CH		4
#define MAN_BN		5

TemplateFile* TemplateFile::factory(VS fields){
	return new TemplateFile(fields[MAN_SEG], std::stoi(fields[MAN_CH]),
			std::stoi(fields[MAN_LENW]), fields[MAN_FN]);
}
void do_scan()
{
	std::string manifest = std::string(G::root) + "/MANIFEST";
	std::fstream fp;

	fp.open(manifest, std::ios::in);
	assert(fp.is_open());

	std::string manline;
	while(std::getline(fp, manline)){
		//std::cout << "Hello:" << manline << "\n";
		VS fields;
		split2(manline, fields, ' ');
		if (fields[0][0] == '#'){
			continue;
		}else{

		}
		template_files.push_back(TemplateFile::factory(fields));
	}
	fp.close();
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
	do_scan();

	for (auto tt: template_files){
		std::cout << tt->toString() << std::endl;
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
