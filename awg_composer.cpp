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
#include <sys/stat.h>
#include <assert.h>
#include <string>
#include <vector>
#include <errno.h>

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
	FILE* out = stdout;
	const char* outfile;
	const char* awg_mode;
	bool output_is_pipe;

	int macs;
};

/*
record (mbbi, "${UUT}:${SITE}:AWG:MODE") {
        field(DTYP, "Soft Channel")
        field(ZRST, "continuous")
        field(ONST, "oneshot")
        field(TWST, "oneshot_rearm")
        field(DESC, "AWG operating mode")
        field(NOBT, "2")
}

acq1001_590> grep 542 /etc/in*
/etc/inetd.conf:54200 stream tcp nowait root bb.sha1sum bb.sha1sum 1
/etc/inetd.conf:54201 stream tcp nowait root bb bb load --mode 1
/etc/inetd.conf:54202 stream tcp nowait root bb bb load --mode 2
/etc/inetd.conf:54205 stream tcp nowait root bb bb load --mode 0

*/

struct ModeMap {
	const char* mode;
	const int port;
};

ModeMap modeMap[] = {
		{ "continuous",    54205 },
		{ "oneshot",       54201 },
		{ "oneshot_rearm", 54202 },
		{ "0",             54205 },
		{ "1",             54201 },
		{ "2",             54202 },
};
#define NMAP	(sizeof(modeMap)/sizeof(ModeMap))

int awgmode2port(const char* mode){
	for (unsigned ii = 0; ii < NMAP; ++ii){
		if (strcmp(modeMap[ii].mode, mode) == 0){
			return modeMap[ii].port;
		}
	}
	return -1;
}

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
	{
	  "awg_mode", 'a', POPT_ARG_STRING, &G::awg_mode, 'a', "awg mode: continuous|oneshot|oneshot_rearm"
	},
	{
	  "outfile", 'o', POPT_ARG_STRING, &G::outfile, 'o', "set output file, default stdout"
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

	virtual void action (int) = 0;

	static Segment* factory(const char* segdef);
	static std::vector<Segment*> segments;
};
std::vector<Segment*> Segment::segments;


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

template<class T>
struct ConcreteSegment: public Segment {
	T** channels;
	int* ch_len;
	T* data;
	int ndata;
	VTF templates;

	static int lenw(std::string fname){
                struct stat buf;
                const char* path = fname.c_str();
                if (stat(path, &buf) != 0){
                        perror(path);
                        exit(1);
                }
                return buf.st_size/sizeof(T);
	}
	void get_templates()
	{
		int max_lenw = 0;

		for (auto tt: ::template_files){
			if (strcmp(tt->seg.c_str(), name) == 0){
				templates.push_back(tt);
			}
		}

		for (auto tt: templates){
			const char* tt_fname = tt->fname.c_str();
			int tt_len = lenw(tt->fname);
			if (tt_len > max_lenw){
				max_lenw = tt_len;
			}
			FILE* fp = fopen(tt_fname, "r");
			if (fp == 0){
				perror(tt_fname);
				exit(errno);
			}
			assert(tt->chan <= G::nchan);
			assert(tt->chan > 0);
			int ic = tt->chan -1;

			channels[ic] = new T[tt_len];
			int nread = fread(channels[ic], sizeof(T), tt_len, fp);
			if (nread != tt_len){
				fprintf(stderr, "ERROR: read %s returned %d wanted %d\n", tt_fname, nread, tt_len);
				exit(1);
			}
			fclose(fp);
			ch_len[ic] = nread;
		}
		data = new T[ndata = max_lenw*G::nchan];

		// create mux data set for G::nchan, use last value in ch data set if data set is short.
		for (int sam = 0; sam < max_lenw; ++sam){
			for (int ic = 0; ic < G::nchan; ++ic){
				int yy;

				if (ch_len[ic] == 0){
					yy = 0;
				}else if (sam >= ch_len[ic]){
					yy = channels[ic][ch_len[ic]-1];
				}else{
					yy = channels[ic][sam];
				}
				data[sam*G::nchan + ic] = yy;
			}
		}
	}
	virtual std::string toString() {
		return "ConcreteSegment<" + std::to_string(sizeof(T)) + "> " + name +" *" + std::to_string(reps) + " ndata:" + std::to_string(ndata);
	}
	ConcreteSegment(int _reps, const char* _name): Segment(_reps, _name), data(0), ndata(0)
	{
		channels = new T* [G::nchan];		// index from 1
		ch_len = new int [G::nchan];

		memset(channels, 0, sizeof(T)*G::nchan+1);
		memset(ch_len, 0, sizeof(int)*G::nchan+1);

		get_templates();
	}
	virtual ~ConcreteSegment() {
		delete [] ch_len;
		delete [] channels;
		delete [] data;
	}

	virtual void emit_segment() {
		fwrite(data, sizeof(T), ndata, G::out);
	}
	virtual void action (int rep) {
		if (G::verbose > 1){
			std::cerr << toString() << std::endl;
		}
		emit_segment();
	}
};


#include <cmath>



template<class T>
struct Segment2D: public ConcreteSegment<T> {
	const int channel_2d;

	Segment2D(int _reps, const char* _name, int _channel_2d):
		ConcreteSegment<T>(_reps, _name), channel_2d(_channel_2d)
	{}
	virtual std::string toString() {
		return "Segment2D<" + std::to_string(sizeof(T)) + "> " + this->name +" *" + std::to_string(this->reps) + " ndata:" + std::to_string(this->ndata) + " channel_2d:" + std::to_string(channel_2d);
	}
	virtual void action (int rep) {

		int max_sam = this->ch_len[channel_2d];

		if (rep < max_sam){
			if (G::verbose > 1){
				std::cerr << this->toString() << " rep:" << rep << " max_sam:" << max_sam << std::endl;
			}
			long scale = this->channels[channel_2d][rep];

			for (int sam = 0; sam < max_sam; ++sam){

				T yy = (T)std::sqrt(this->channels[channel_2d][sam]*scale);

				this->data[sam*G::nchan + channel_2d] = yy;
				++G::macs;
			}
		}
		this->emit_segment();
	}
};

Segment* Segment::factory(const char* segdef){
	char _segname[128];
	int reps = 1;
	const char* segname = _segname;
	int channel_2d = -1;
	if (sscanf(segdef, "%d*%s", &reps, _segname) == 2) {
		;
	}else{
		segname = segdef;
	}
	if (getenv("CHANNEL_2D") != 0){
		channel_2d = atoi(getenv("CHANNEL_2D"));
	}
	if (G::data32){
		return new ConcreteSegment<int>(reps, segname);
	}else{
		if (channel_2d >= 0){
			return new Segment2D<short>(reps, segname, channel_2d);
		}else{
			return new ConcreteSegment<short>(reps, segname);
		}
	}

}



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
		case 'a': {
			int port = awgmode2port(G::awg_mode);
			if (port == -1){
				fprintf(stderr, "ERROR, mode \"%s\" not supported\n", G::awg_mode);
				exit(1);
			}else{
				char cmd[132];
				snprintf(cmd, 132, "nc localhost %d", port);
				G::out = popen(cmd, "w");
				if (G::out == 0){
					perror(cmd);
					exit(1);
				}
				G::output_is_pipe = true;
			}
			break;
		}
		case 'o':
			if (G::awg_mode){
				fprintf(stderr, "ERROR: output stdout not compatible with output awg\n");
				exit(1);
			}
			G::out = fopen(G::outfile, "w");
			assert(G::out);
			break;
		default:
			;
		}
	}

	do_scan();

	const char* segdef;
	while ((segdef = poptGetArg(opt_context)) != 0){
		Segment::segments.push_back(Segment::factory(segdef));
	}
	if (G::verbose){
		for (auto tt: template_files){
			std::cerr << tt->toString() << std::endl;
		}
	}
}

int main(int argc, const char* argv[])
{
	ui(argc, argv);

	for (int rep = 0; rep < G::nreps; ++rep){
		for (Segment* segment: Segment::segments){
			for (int segrep = 0; segrep < segment->reps; ++segrep){
				segment->action(segrep);
			}
		}
	}

	if (G::output_is_pipe){
		pclose(G::out);
	}
	if (G::verbose){
		std::cerr << "mathops:" << G::macs << std::endl;
	}
}
